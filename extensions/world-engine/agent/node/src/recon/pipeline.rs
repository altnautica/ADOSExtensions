//! Runs a staged job stage by stage, persisting each stage through a
//! [`StageSink`] and skipping stages a resumed job already finished.

use std::fs;
use std::path::{Path, PathBuf};

use ados_capture_contract::{CameraModelModel, RigSensorKind, SessionReader};

use super::mesh::{self, MeshMethod};
use super::preset::{
    stage_weight, ProductSwitches, CALIBRATE_STAGES, CHANGE_STAGES, RECONSTRUCT_STAGES,
};
use super::products_stage::{self, ArtifactOut, ProductsInput, OUTPUTS_FILE};
use super::{
    check_tools, chunks, glim, job_dir, prepare, sfm, train, Preset, StageError, StagedJob,
    ToolRunner,
};
use crate::products::{change, cloud};
use crate::store_stages::{StageRow, StageState};

/// Where a running staged job reports to (the job store, through the engine
/// lock, in the daemon).
pub trait StageSink: Send + Sync {
    /// The job's persisted stage rows (a resumed job has some finished).
    fn stages(&self) -> Vec<StageRow>;
    /// Create the stage rows once.
    fn init(&self, stages: &[&str]);
    fn begin(&self, stage: &str);
    fn progress(&self, stage: &str, stage_progress: f32, job_progress: f32);
    fn finish(&self, stage: &str, state: StageState, detail: &serde_json::Value);
    /// The job was cancelled while running.
    fn cancelled(&self) -> bool;
}

/// What a staged run needs beyond the job itself.
pub struct RunCtx<'a> {
    pub work_root: &'a Path,
    pub runner: &'a dyn ToolRunner,
    pub sink: &'a dyn StageSink,
    /// The built tour viewer the tour bundle copies.
    pub tour_viewer: PathBuf,
    /// Total GPU memory, for the splat cap.
    pub vram_mb: Option<u64>,
    /// An ONNX person detector for dynamic-object masks, when configured.
    pub person_model: Option<PathBuf>,
}

struct Stages<'a> {
    sink: &'a dyn StageSink,
    rows: Vec<StageRow>,
    order: Vec<&'static str>,
}

impl<'a> Stages<'a> {
    fn new(sink: &'a dyn StageSink, order: &[&'static str]) -> Self {
        sink.init(order);
        Self {
            rows: sink.stages(),
            sink,
            order: order.to_vec(),
        }
    }

    fn base(&self, stage: &str) -> f32 {
        let weights: Vec<f32> = self.order.iter().map(|s| stage_weight(s)).collect();
        let total: f32 = weights.iter().sum();
        let before: f32 = self
            .order
            .iter()
            .zip(&weights)
            .take_while(|(s, _)| **s != stage)
            .map(|(_, w)| w)
            .sum();
        before / total
    }

    fn weight(&self, stage: &str) -> f32 {
        let total: f32 = self.order.iter().map(|s| stage_weight(s)).sum();
        stage_weight(stage) / total
    }

    /// Run `f` as `stage`, or return the stored detail of a stage a resumed job
    /// already finished.
    fn run(
        &self,
        stage: &str,
        f: impl FnOnce(&mut dyn FnMut(f32)) -> Result<serde_json::Value, StageError>,
    ) -> Result<serde_json::Value, StageError> {
        if let Some(row) = self
            .rows
            .iter()
            .find(|r| r.id == stage && r.state.is_finished())
        {
            return Ok(row.detail.clone());
        }
        if self.sink.cancelled() {
            return Err(StageError::new("E_CANCELLED", "the job was cancelled"));
        }
        self.sink.begin(stage);
        let (base, w) = (self.base(stage), self.weight(stage));
        let sink = self.sink;
        let mut report = |p: f32| sink.progress(stage, p, base + w * p);
        match f(&mut report) {
            Ok(detail) => {
                self.sink.finish(stage, StageState::Done, &detail);
                Ok(detail)
            }
            Err(e) => {
                self.sink
                    .finish(stage, StageState::Failed, &serde_json::to_value(&e)?);
                Err(e)
            }
        }
    }

    fn skip(&self, stage: &str, reason: &str) {
        if !self
            .rows
            .iter()
            .any(|r| r.id == stage && r.state.is_finished())
        {
            self.sink.finish(
                stage,
                StageState::Skipped,
                &serde_json::json!({ "reason": reason }),
            );
        }
    }
}

/// Run a staged job to its deliverables (`out/outputs.json`).
pub fn run_staged(
    job_id: &str,
    staged: &StagedJob,
    ctx: &RunCtx<'_>,
) -> Result<Vec<ArtifactOut>, StageError> {
    let dir = job_dir(ctx.work_root, job_id)?;
    fs::create_dir_all(&dir)?;
    match staged {
        StagedJob::Reconstruct {
            session_id,
            session_dir,
            preset,
            generation,
            chunk_frames,
            glim_odometry,
            products,
            appearance,
            mesh_method,
        } => reconstruct(
            &dir,
            session_id,
            session_dir,
            *preset,
            *chunk_frames,
            *glim_odometry,
            *products,
            *appearance,
            *mesh_method,
            *generation,
            ctx,
        ),
        StagedJob::Change { a, b, a_job, b_job } => change_job(&dir, ctx, (a, a_job), (b, b_job)),
        StagedJob::Calibrate {
            session_dir,
            request,
            ..
        } => calibrate(&dir, session_dir, request, ctx),
    }
}

/// Whether training takes the monocular depth prior: only for a camera-only
/// session SfM mapped without prior poses. A posed session (VIO, flight
/// controller or LiDAR odometry track) already has its geometry from the
/// track, and on such sessions the prior cost about 3 dB of training PSNR;
/// where it is used the trainer still drops each view whose prior disagrees
/// with the sparse points. `prepare` computes the priors either way (the
/// motion and mirror masks use them).
fn train_depth_prior(preset_wants: bool, has_lidar: bool, sfm_detail: &serde_json::Value) -> bool {
    let posed = matches!(
        sfm_detail["method"].as_str(),
        Some("known_poses" | "pose_prior_mapper")
    );
    preset_wants && !has_lidar && !posed
}

#[allow(clippy::too_many_arguments)]
fn reconstruct(
    dir: &Path,
    session_id: &str,
    session_dir: &Path,
    preset: Preset,
    chunk_frames: Option<usize>,
    glim_odometry: glim::GlimOdometry,
    products: ProductSwitches,
    appearance: train::Appearance,
    mesh_method: MeshMethod,
    generation: u32,
    ctx: &RunCtx<'_>,
) -> Result<Vec<ArtifactOut>, StageError> {
    let stages = Stages::new(ctx.sink, &RECONSTRUCT_STAGES);
    check_tools(ctx.runner, &preset.required_tools())?;
    let spec = products.apply(preset.spec());
    let reader = SessionReader::open(session_dir)?;
    fs::write(
        dir.join("session-rig.json"),
        serde_json::to_vec_pretty(reader.rig())?,
    )?;
    // A LiDAR session's poses come from GLIM, so it must be installed (and,
    // for GPU odometry, docker must have the NVIDIA runtime).
    if glim::rig_has_lidar(reader.rig()) {
        glim::check_available(ctx.runner, dir, glim_odometry)?;
    }
    let dataset = dir.join("dataset");

    let prep = stages.run("prepare", |p| {
        let summary = prepare::run(&reader, &dataset, &spec, &mut |f| p(0.6 * f))?;
        let has_lidar = summary.lidar_points > 0;
        let mut detail = serde_json::to_value(&summary)?;
        // Monocular depth serves two consumers: the training depth prior of a
        // camera-only session, and the mirror/glass mask pass of a LiDAR
        // session (depth that disagrees with LiDAR on a plane is a reflection).
        // It runs before the masks so both find it.
        let want_depth = (spec.depth_priors && !has_lidar) || (spec.masks && has_lidar);
        if want_depth {
            train::depth_priors(ctx.runner, &dataset, &mut |f| p(0.6 + 0.2 * f))?;
            detail["depths"] = true.into();
        }
        if spec.masks {
            let m = train::masks(
                ctx.runner,
                &dataset,
                ctx.person_model.as_deref(),
                &mut |f| p(0.8 + 0.2 * f),
            )?;
            detail["masks"] = serde_json::to_value(m)?;
        }
        Ok(detail)
    })?;
    let prior = prep["pose_prior"].as_str().map(str::to_string);
    let has_lidar = prep["lidar_points"].as_u64().unwrap_or(0) > 0;

    let sfm_dir = dir.join("sfm");
    let sfm_detail = stages.run("sfm", |p| {
        // A LiDAR session takes its poses from the LiDAR-inertial trajectory
        // (aligned onto the recorded pose track) and keeps them fixed; a
        // rejected trajectory leaves the recorded poses as they were. GLIM
        // runs on the rig's LiDAR whether or not a track was recorded.
        let lidar_rig = glim::rig_has_lidar(reader.rig());
        let share = if lidar_rig { 0.3 } else { 0.0 };
        let glim = match lidar_rig {
            true => glim::run(
                ctx.runner,
                &reader,
                dir,
                &dataset,
                glim_odometry,
                &mut |f| p(share * f),
            )?,
            false => None,
        };
        let p = &mut |f: f32| p(share + (1.0 - share) * f);
        let fixed = glim.as_ref().is_some_and(|g| g.used) || prior.as_deref() == Some("lidar");
        let opts = sfm::SfmOptions {
            fixed_poses: fixed,
            refine: preset != Preset::Preview,
            hierarchical: spec.hierarchical,
            prior_source: prior.as_deref(),
        };
        // A posed warehouse session larger than one chunk runs per chunk.
        let plan = match preset {
            Preset::Warehouse => chunks::plan_dataset(&dataset, chunk_frames)?,
            _ => None,
        };
        let mut detail = match plan {
            Some(plan) => chunks::run_sfm(ctx.runner, &dataset, &sfm_dir, &plan, &opts, p)?,
            None => serde_json::to_value(sfm::run(ctx.runner, &dataset, &sfm_dir, &opts, p)?)?,
        };
        if let Some(g) = glim {
            // With no recorded track GLIM's poses sit in its own arbitrary
            // frame, so the model is local (no geo anchor).
            if g.used && g.align_rmse_m.is_none() {
                detail["world_frame"] = false.into();
            }
            detail["glim"] = serde_json::to_value(g)?;
        }
        Ok(detail)
    })?;
    let world_frame = sfm_detail["world_frame"].as_bool().unwrap_or(false);
    let chunk_plan = match sfm_detail["chunks"].is_array() {
        true => Some(chunks::read_plan(&sfm_dir)?),
        false => None,
    };

    let rig = reader.rig();
    let ut = rig.sensors.iter().any(|s| {
        matches!(s.kind, RigSensorKind::Camera)
            && s.camera.as_ref().is_some_and(|c| {
                c.readout_ns > 0
                    || matches!(
                        c.model,
                        CameraModelModel::OpencvFisheye | CameraModelModel::Kb4
                    )
            })
    });
    let max_splats = train::max_splats_for(spec.max_splats_8gb, ctx.vram_mb);
    let train_dir = dir.join("train");
    let depth_prior = train_depth_prior(spec.depth_priors, has_lidar, &sfm_detail);
    let train_detail = stages.run("train", |p| {
        // `Some(reason)`: train with Brush, which stands in at full length
        // (and the stage says why) when the gsplat kernels do not run on this
        // node (no CUDA toolkit, no GPU).
        let brush: Option<Option<String>> = if preset == Preset::Preview {
            Some(None)
        } else if let Err(why) = train::gsplat_available(ctx.runner, &train_dir) {
            tracing::warn!(error = %why, "gsplat unavailable; training with brush");
            ctx.runner.resolve("brush")?;
            Some(Some(format!("gsplat unavailable: {}", why.message)))
        } else {
            None
        };
        let (steps, runner) = (spec.train_steps, ctx.runner);
        let train_one = |ds: &Path, out: &Path, p: &mut dyn FnMut(f32)| match &brush {
            Some(reason) => train::brush(runner, ds, out, steps, max_splats, reason.as_deref(), p),
            None => {
                let depths = ds.join("depths");
                let depths = (depth_prior && depths.is_dir()).then_some(depths);
                let p = &mut |f| p(0.1 + 0.9 * f);
                train::gsplat(
                    runner,
                    ds,
                    out,
                    steps,
                    max_splats,
                    ut,
                    appearance,
                    depths.as_deref(),
                    p,
                )
            }
        };
        let mut detail = match &chunk_plan {
            Some(plan) => {
                chunks::run_train(plan, &dataset, &train_dir, &sfm_detail, &train_one, p)?
            }
            None => serde_json::to_value(train_one(&dataset, &train_dir, p)?)?,
        };
        detail["depth_prior"] = depth_prior.into();
        Ok(detail)
    })?;

    if spec.mesh {
        stages.run("mesh", |p| {
            let steps = spec.train_steps;
            mesh::stage(
                ctx.runner,
                dir,
                session_dir,
                &train_detail,
                mesh_method,
                steps,
                p,
            )
        })?;
    } else {
        stages.skip("mesh", "the preset has no mesh");
    }

    let title = reader
        .manifest()
        .operator
        .site
        .clone()
        .unwrap_or_else(|| session_id.to_string());
    let anchor = if world_frame {
        reader.manifest().anchor.clone()
    } else {
        None
    };
    stages.run("products", |p| {
        let arts = products_stage::run(
            ctx.runner,
            &ProductsInput {
                job_dir: dir,
                session_id,
                title: &title,
                anchor: anchor.clone(),
                generation,
                spec: &spec,
                tour_viewer: &ctx.tour_viewer,
            },
            p,
        )?;
        Ok(serde_json::json!({ "artifacts": arts.len(), "world_frame": world_frame }))
    })?;
    read_outputs(dir)
}

fn change_job(
    dir: &Path,
    ctx: &RunCtx<'_>,
    a: (&str, &str),
    b: (&str, &str),
) -> Result<Vec<ArtifactOut>, StageError> {
    let stages = Stages::new(ctx.sink, &CHANGE_STAGES);
    let cloud_of = |job: &str| -> Result<std::path::PathBuf, StageError> {
        Ok(job_dir(ctx.work_root, job)?.join("out").join("cloud.laz"))
    };
    let (pa, pb) = (cloud_of(a.1)?, cloud_of(b.1)?);
    stages.run("prepare", |_| {
        for p in [&pa, &pb] {
            if !p.is_file() {
                return Err(StageError::new(
                    "E_NOT_RECONSTRUCTED",
                    format!("{} is missing", p.display()),
                ));
            }
        }
        Ok(serde_json::json!({ "a": a.0, "b": b.0, "a_job": a.1, "b_job": b.1 }))
    })?;
    stages.run("products", |p| {
        let ca = cloud::read_laz(&pa).map_err(StageError::io)?;
        let cb = cloud::read_laz(&pb).map_err(StageError::io)?;
        p(0.3);
        // Each cloud sits in its own session frame; b is aligned into a's
        // frame (anchor offset when both are georeferenced, then ICP) before
        // the comparison, or the job is refused when they cannot be related.
        let (anchor_a, anchor_b) = (cloud_anchor(&pa)?, cloud_anchor(&pb)?);
        let (result, _alignment) =
            change::align_and_compare(&ca, anchor_a.as_ref(), &cb, anchor_b.as_ref(), 0.05)
                .map_err(|e| match e {
                    crate::products::ProductError::FramesUnrelated(m) => {
                        StageError::new("E_FRAMES_UNRELATED", m)
                    }
                    crate::products::ProductError::AlignAmbiguous(m) => {
                        StageError::new("E_ALIGN_AMBIGUOUS", m)
                    }
                    other => StageError::io(other),
                })?;
        let out = dir.join("out");
        if out.exists() {
            fs::remove_dir_all(&out)?;
        }
        fs::create_dir_all(&out)?;
        cloud::write_laz(&out.join("change.laz"), &result.cloud).map_err(StageError::io)?;
        let summary = serde_json::to_value(&result.summary)?;
        fs::write(
            out.join("change_summary.json"),
            serde_json::to_vec_pretty(&summary)?,
        )?;
        let arts = vec![
            artifact(&out, "change", "change.laz", "laz"),
            artifact(&out, "change_summary", "change_summary.json", "json"),
        ];
        fs::write(out.join(OUTPUTS_FILE), serde_json::to_vec_pretty(&arts)?)?;
        Ok(summary)
    })?;
    read_outputs(dir)
}

/// The geo anchor recorded in a point cloud's placement sidecar.
fn cloud_anchor(cloud: &Path) -> Result<Option<ados_capture_contract::GeoAnchor>, StageError> {
    let sidecar = cloud.with_file_name(format!(
        "{}.place.json",
        cloud
            .file_name()
            .map(|n| n.to_string_lossy())
            .unwrap_or_default()
    ));
    match fs::read(&sidecar) {
        Ok(b) => {
            let d: ados_capture_contract::ArtifactDescriptor = serde_json::from_slice(&b)?;
            Ok(d.anchor)
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e.into()),
    }
}

fn calibrate(
    dir: &Path,
    session_dir: &Path,
    request: &crate::calib::CalibRequest,
    ctx: &RunCtx<'_>,
) -> Result<Vec<ArtifactOut>, StageError> {
    let stages = Stages::new(ctx.sink, &CALIBRATE_STAGES);
    stages.run("prepare", |_| {
        let reader = SessionReader::open(session_dir)?;
        reader.verify()?;
        prepare::check_ids(reader.rig())?;
        Ok(serde_json::json!({ "rig_id": reader.rig().rig_id }))
    })?;
    stages.run("calibrate", |p| {
        let work = dir.join("calib");
        fs::create_dir_all(&work)?;
        let outcome = crate::calib::run(request, session_dir, &work, ctx.runner, p)?;
        let out = dir.join("out");
        if out.exists() {
            fs::remove_dir_all(&out)?;
        }
        fs::create_dir_all(&out)?;
        fs::write(
            out.join("rig.json"),
            serde_json::to_vec_pretty(&outcome.rig)?,
        )?;
        fs::write(
            out.join("calib_report.json"),
            serde_json::to_vec_pretty(&outcome.report)?,
        )?;
        let arts = vec![
            artifact(&out, "rig", "rig.json", "json"),
            artifact(&out, "calib_report", "calib_report.json", "json"),
        ];
        fs::write(out.join(OUTPUTS_FILE), serde_json::to_vec_pretty(&arts)?)?;
        Ok(outcome.report)
    })?;
    read_outputs(dir)
}

fn artifact(out: &Path, kind: &str, file: &str, format: &str) -> ArtifactOut {
    let size = fs::metadata(out.join(file)).map(|m| m.len()).unwrap_or(0);
    ArtifactOut {
        kind: kind.into(),
        file: file.into(),
        format: format.into(),
        place_json: None,
        meta: serde_json::json!({ "size_bytes": size }),
    }
}

/// The deliverables a finished job listed.
pub fn read_outputs(job_dir: &Path) -> Result<Vec<ArtifactOut>, StageError> {
    Ok(serde_json::from_slice(&fs::read(
        job_dir.join("out").join(OUTPUTS_FILE),
    )?)?)
}

#[cfg(test)]
mod depth_prior_tests {
    use super::train_depth_prior;
    use serde_json::json;

    #[test]
    fn the_mono_depth_prior_trains_only_pose_free_camera_sessions() {
        let mapped = json!({"method": "mapper"});
        let posed = json!({"method": "known_poses"});
        assert!(train_depth_prior(true, false, &mapped));
        assert!(train_depth_prior(
            true,
            false,
            &json!({"method": "hierarchical_mapper"})
        ));
        assert!(!train_depth_prior(true, false, &posed));
        assert!(!train_depth_prior(
            true,
            false,
            &json!({"method": "pose_prior_mapper"})
        ));
        assert!(!train_depth_prior(true, true, &mapped));
        assert!(!train_depth_prior(false, false, &mapped));
    }
}
