use std::sync::Mutex;

use ados_mapping_protocol::edit::{DecimateTarget, ExportFormat, Keep, PatchSource, Transform2};

use super::*;
use crate::products::cloud::{read_laz_layers, write_laz, PointCloud};
use crate::recon::{ResolvedTool, StageSink, ToolInvocation, ToolRunner};
use crate::store_stages::{StageRow, StageState};

const SQUARE: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];

struct NoTools;

impl ToolRunner for NoTools {
    fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
        Err(StageError::new("E_TOOL_MISSING", tool))
    }
    fn run(&self, _: &ToolInvocation, _: &mut dyn FnMut(&str)) -> Result<(), StageError> {
        unreachable!("an edit runs no tool")
    }
}

/// Stage rows in memory; `cancel_at` cancels the job when that stage begins.
#[derive(Default)]
struct Sink {
    rows: Mutex<Vec<StageRow>>,
    cancel_at: Option<&'static str>,
    cancelled: Mutex<bool>,
}

impl StageSink for Sink {
    fn stages(&self) -> Vec<StageRow> {
        self.rows.lock().unwrap().clone()
    }
    fn init(&self, stages: &[&str]) {
        let mut rows = self.rows.lock().unwrap();
        if rows.is_empty() {
            *rows = stages
                .iter()
                .map(|s| StageRow {
                    id: s.to_string(),
                    state: StageState::Pending,
                    progress: 0.0,
                    started_ns: None,
                    ended_ns: None,
                    detail: serde_json::Value::Null,
                })
                .collect();
        }
    }
    fn begin(&self, stage: &str) {
        if self.cancel_at == Some(stage) {
            *self.cancelled.lock().unwrap() = true;
        }
    }
    fn progress(&self, _: &str, _: f32, _: f32) {}
    fn finish(&self, stage: &str, state: StageState, detail: &serde_json::Value) {
        let mut rows = self.rows.lock().unwrap();
        if let Some(r) = rows.iter_mut().find(|r| r.id == stage) {
            r.state = state;
            r.detail = detail.clone();
        }
    }
    fn cancelled(&self) -> bool {
        *self.cancelled.lock().unwrap()
    }
}

/// A product job `rec-1` holding a 20 × 20 grid at two heights (800 points).
fn product() -> tempfile::TempDir {
    let work = tempfile::tempdir().unwrap();
    let out = work.path().join("jobs/rec-1/out");
    fs::create_dir_all(&out).unwrap();
    let mut points = Vec::new();
    for i in 0..20 {
        for j in 0..20 {
            for z in [0.0, 2.0] {
                points.push([i as f64 + 0.5, j as f64 + 0.5, z]);
            }
        }
    }
    write_laz(
        &out.join("cloud.laz"),
        &PointCloud {
            points,
            colors: None,
        },
    )
    .unwrap();
    work
}

fn run(
    work: &Path,
    job: &str,
    sink: &Sink,
    source: &str,
    ops: Vec<EditOp>,
) -> Result<Vec<ArtifactOut>, StageError> {
    let ctx = RunCtx {
        work_root: work,
        runner: &NoTools,
        sink,
        tour_viewer: PathBuf::new(),
        vram_mb: None,
        person_model: None,
    };
    let dir = job_dir(work, job).unwrap();
    fs::create_dir_all(&dir).unwrap();
    let params = EditParams {
        job: None,
        source: source.into(),
        ops,
    };
    edit_job(&dir, &ctx, "rec-1", &params)
}

fn count(work: &Path, rel: &str) -> usize {
    read_laz_layers(&work.join("jobs/rec-1").join(rel))
        .unwrap()
        .0
        .points
        .len()
}

fn edits(work: &Path, rel: &str) -> EditsFile {
    serde_json::from_slice(&fs::read(work.join("jobs/rec-1").join(rel)).unwrap()).unwrap()
}

fn crop() -> EditOp {
    EditOp::Crop {
        keep: Keep::Inside,
        polygon: SQUARE.to_vec(),
        z: None,
    }
}

fn delete_corner() -> EditOp {
    EditOp::Delete {
        polygon: vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
        z: None,
    }
}

fn patch(layer: &str, from: [[f64; 2]; 2], east: f64) -> EditOp {
    EditOp::Patch {
        layer: layer.into(),
        source: PatchSource { bbox: from },
        target: Transform2 {
            translate: [east, 0.0],
            rotate_deg: 0.0,
        },
    }
}

#[test]
fn three_operations_write_three_versions_and_an_append_only_record() {
    let work = product();
    let arts = run(
        work.path(),
        "ed-1",
        &Sink::default(),
        "out/cloud.laz",
        vec![
            crop(),
            patch("patch-1", [[0.0, 0.0], [2.0, 2.0]], 30.0),
            delete_corner(),
        ],
    )
    .unwrap();
    assert_eq!(
        arts.iter().map(|a| a.file.as_str()).collect::<Vec<_>>(),
        ["cloud.v1.laz", "cloud.v2.laz", "cloud.v3.laz"]
    );
    assert!(arts.iter().all(|a| a.job.as_deref() == Some("rec-1")));
    assert_eq!(
        count(work.path(), "out/cloud.laz"),
        800,
        "the source is never rewritten"
    );
    assert_eq!(count(work.path(), "out/cloud.v1.laz"), 200);
    assert_eq!(count(work.path(), "out/cloud.v2.laz"), 208);
    assert_eq!(
        count(work.path(), "out/cloud.v3.laz"),
        200,
        "the corner of the base only: the patch sits 30 m east"
    );

    let latest = edits(work.path(), "out/edits.json");
    assert_eq!(latest.product, "out/cloud.v3.laz");
    assert_eq!(latest.source, "out/cloud.laz");
    assert!(latest.edited);
    let ops: Vec<&str> = latest.ops.iter().map(|r| r.op.name()).collect();
    assert_eq!(ops, ["crop", "patch", "delete"]);
    assert_eq!(latest.ops[0].removed, Some(600));
    assert_eq!(latest.ops[1].added, Some(8));
    assert_eq!(latest.ops[1].source_layers, ["base"]);
    assert_eq!(latest.ops[2].removed, Some(8));
    assert_eq!(
        latest
            .layers
            .iter()
            .map(|l| (l.name.as_str(), l.points))
            .collect::<Vec<_>>(),
        [("base", 192), ("patch-1", 8)]
    );
    // An earlier version's provenance still describes that version.
    let v1 = edits(work.path(), "out/cloud.v1.edits.json");
    assert_eq!(v1.product, "out/cloud.v1.laz");
    assert_eq!(v1.ops.len(), 1);
    assert_eq!(latest.ops[0], v1.ops[0]);
}

#[test]
fn undo_is_opening_the_previous_version_and_a_new_edit_branches_from_it() {
    let work = product();
    run(
        work.path(),
        "ed-1",
        &Sink::default(),
        "out/cloud.laz",
        vec![crop(), delete_corner()],
    )
    .unwrap();
    assert_eq!(
        count(work.path(), "out/cloud.v1.laz"),
        200,
        "the version before the delete keeps its exact count"
    );
    let arts = run(
        work.path(),
        "ed-2",
        &Sink::default(),
        "out/cloud.v1.laz",
        vec![delete_corner()],
    )
    .unwrap();
    assert_eq!(
        arts[0].file, "cloud.v3.laz",
        "a version number is never reused"
    );
    let branch = edits(work.path(), "out/cloud.v3.edits.json");
    assert_eq!(
        branch
            .ops
            .iter()
            .map(|r| r.result.as_str())
            .collect::<Vec<_>>(),
        ["out/cloud.v1.laz", "out/cloud.v3.laz"]
    );
    assert_eq!(
        edits(work.path(), "out/cloud.v2.edits.json").ops.len(),
        2,
        "the abandoned version keeps its record"
    );
}

#[test]
fn a_cancelled_operation_leaves_no_artifact_and_the_previous_version_current() {
    let work = product();
    let sink = Sink {
        cancel_at: Some("delete"),
        ..Sink::default()
    };
    let err = run(
        work.path(),
        "ed-1",
        &sink,
        "out/cloud.laz",
        vec![crop(), delete_corner()],
    )
    .unwrap_err();
    assert_eq!(err.code, "E_CANCELLED");
    let out = work.path().join("jobs/rec-1/out");
    let mut names: Vec<String> = fs::read_dir(&out)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    assert_eq!(
        names,
        [
            "cloud.laz",
            "cloud.v1.edits.json",
            "cloud.v1.laz",
            "edits.json"
        ]
    );
    assert_eq!(
        edits(work.path(), "out/edits.json").product,
        "out/cloud.v1.laz"
    );
}

#[test]
fn a_layer_another_patch_was_made_from_cannot_be_deleted() {
    let work = product();
    run(
        work.path(),
        "ed-1",
        &Sink::default(),
        "out/cloud.laz",
        vec![
            patch("patch-1", [[0.0, 0.0], [2.0, 2.0]], 30.0),
            patch("patch-2", [[30.0, 0.0], [32.0, 2.0]], 10.0),
        ],
    )
    .unwrap();
    let err = run(
        work.path(),
        "ed-2",
        &Sink::default(),
        "out/cloud.v2.laz",
        vec![EditOp::DeleteLayer {
            layer: "patch-1".into(),
        }],
    )
    .unwrap_err();
    assert_eq!(
        err.message,
        "patch-2 was made from this layer; delete patch-2 first."
    );
    run(
        work.path(),
        "ed-3",
        &Sink::default(),
        "out/cloud.v2.laz",
        vec![
            EditOp::DeleteLayer {
                layer: "patch-2".into(),
            },
            EditOp::DeleteLayer {
                layer: "patch-1".into(),
            },
        ],
    )
    .unwrap();
    let latest = edits(work.path(), "out/edits.json");
    assert_eq!(latest.layers.len(), 1);
    assert_eq!(count(work.path(), &latest.product), 800);
}

#[test]
fn an_unedited_export_ships_an_empty_record() {
    let work = product();
    let arts = run(
        work.path(),
        "ed-1",
        &Sink::default(),
        "out/cloud.laz",
        vec![EditOp::Export {
            format: ExportFormat::Ply,
            crs: None,
            decimate: None,
        }],
    )
    .unwrap();
    assert_eq!(arts[0].kind, "export");
    assert_eq!(arts[0].file, "exports/cloud.ply");
    let manifest: serde_json::Value = serde_json::from_slice(
        &fs::read(
            work.path()
                .join("jobs/rec-1/out/exports/cloud.manifest.json"),
        )
        .unwrap(),
    )
    .unwrap();
    assert_eq!(manifest["edits"]["ops"], serde_json::json!([]));
    assert_eq!(manifest["edited"], false);
    assert_eq!(
        manifest["crs"], "site-local frame",
        "no georeference: the frame is recorded, never invented"
    );
    let ply = fs::metadata(work.path().join("jobs/rec-1/out/exports/cloud.ply"))
        .unwrap()
        .len();
    let (points, predicted) = super::super::edit_export::estimate_size(
        &work.path().join("jobs/rec-1"),
        "out/cloud.laz",
        ExportFormat::Ply,
        None,
    )
    .unwrap();
    assert_eq!(
        (points, predicted),
        (800, ply),
        "a PLY's size is known exactly"
    );
}

#[test]
fn a_crs_the_node_cannot_write_is_refused() {
    let work = product();
    let err = run(
        work.path(),
        "ed-1",
        &Sink::default(),
        "out/cloud.laz",
        vec![EditOp::Export {
            format: ExportFormat::Laz,
            crs: Some("EPSG:32643".into()),
            decimate: Some(DecimateTarget {
                target_points: None,
                target_percent: Some(50.0),
            }),
        }],
    )
    .unwrap_err();
    assert!(
        err.message.contains("does not reproject to EPSG:32643"),
        "{}",
        err.message
    );
}

#[test]
fn a_missing_source_is_named() {
    let work = product();
    let err = run(
        work.path(),
        "ed-1",
        &Sink::default(),
        "out/cloud.v9.laz",
        vec![crop()],
    )
    .unwrap_err();
    assert_eq!(err.code, "E_SOURCE_MISSING");
    assert_eq!(err.message, "The source artifact is missing on this node.");
}

#[test]
fn version_names_strip_their_number() {
    assert_eq!(stem_of("out/cloud.laz"), "cloud");
    assert_eq!(stem_of("out/cloud.v12.laz"), "cloud");
    assert_eq!(stem_of("out/my.vista.laz"), "my.vista");
    assert_eq!(provenance_of("out/cloud.v2.laz"), "out/cloud.v2.edits.json");
}
