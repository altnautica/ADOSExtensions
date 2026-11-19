//! The staged `change` job: two reconstructed sessions' clouds aligned and
//! compared.

use std::fs;
use std::path::Path;

use super::super::preset::CHANGE_STAGES;
use super::super::products_stage::{ArtifactOut, OUTPUTS_FILE};
use super::super::{job_dir, StageError};
use super::{artifact, read_outputs, RunCtx, Stages};
use crate::products::{change, cloud};

pub(super) fn change_job(
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
