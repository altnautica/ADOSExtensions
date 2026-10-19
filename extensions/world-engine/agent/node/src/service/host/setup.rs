//! Host-service setup: the detector, node identity, state paths, the
//! live-reconstruction gate and the plugin-host connection.

use super::*;

pub(super) fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Pick the perception-offload detector. When the `onnx` feature is built AND
/// `ADOS_COMPUTE_DETECTOR_MODEL` (or the `serving.detector_model` plugin config)
/// names an `.onnx` file, load the real ONNX detector (CoreML-accelerated on
/// macOS via the `coreml` feature); otherwise fall back to the mock so the
/// offload path stays exercised with no model. A load failure logs and falls
/// back to the mock rather than refusing to start (the node still comes up).
pub(super) fn select_detector(config_model_path: Option<&str>) -> Arc<dyn Detector> {
    // Used only in the onnx build (the mock path ignores it).
    #[cfg(not(feature = "onnx"))]
    let _ = config_model_path;
    #[cfg(feature = "onnx")]
    {
        // The env wins (the bench override); else the operator-picked model from
        // `serving.detector_model` (resolved to a path). Empty ⇒ mock.
        let path = match env_or("ADOS_COMPUTE_DETECTOR_MODEL", "") {
            p if !p.is_empty() => p,
            _ => config_model_path.unwrap_or("").to_string(),
        };
        if !path.is_empty() {
            let (iw, ih) = parse_input_dims(&env_or("ADOS_COMPUTE_DETECTOR_INPUT", "640x640"));
            let classes: Vec<String> = std::env::var("ADOS_COMPUTE_DETECTOR_CLASSES")
                .ok()
                .map(|s| {
                    s.split(',')
                        .map(|c| c.trim().to_string())
                        .filter(|c| !c.is_empty())
                        .collect()
                })
                .unwrap_or_default();
            let head = match env_or("ADOS_COMPUTE_DETECTOR_HEAD", "yolo8").as_str() {
                "yolo5" => ados_protocol::framebus::DetectionHead::Yolo5,
                _ => ados_protocol::framebus::DetectionHead::Yolo8,
            };
            let meta = ados_protocol::framebus::ModelMetadata {
                id: "offload-detector".into(),
                kind: ados_protocol::framebus::ModelKind::Detection,
                execution: ados_protocol::framebus::ModelExecution::EngineRun,
                input_width: iw,
                input_height: ih,
                input_format: ados_protocol::framebus::FrameFormat::Rgb24,
                output_classes: classes,
                model_path: Some(path.clone()),
                head,
            };
            match world_engine_node::OnnxDetector::from_model(&meta) {
                Ok(det) => {
                    tracing::info!(model = %path, input = format!("{iw}x{ih}"), "perception offload: real ONNX detector loaded");
                    return Arc::new(det);
                }
                Err(e) => {
                    tracing::warn!(model = %path, error = %e, "perception offload: ONNX load failed, using mock");
                }
            }
        }
    }
    Arc::new(MockDetector)
}

/// Parse a `WxH` input-size string (e.g. `640x640`) into `(width, height)`,
/// defaulting to 640 square on a malformed value.
#[cfg(feature = "onnx")]
pub(super) fn parse_input_dims(s: &str) -> (u32, u32) {
    let mut it = s.split(['x', 'X']);
    let w = it.next().and_then(|v| v.trim().parse().ok()).unwrap_or(640);
    let h = it.next().and_then(|v| v.trim().parse().ok()).unwrap_or(w);
    (w, h)
}

/// Resolve a stable per-node id: the `ADOS_COMPUTE_NODE_ID` override if set,
/// else derived from the host `machine-id` so two compute nodes never collide on
/// the mDNS instance / deviceId / cluster identity, else a generic fallback.
/// Pure (the inputs are injected) so the derivation is unit-tested.
pub(super) fn derive_node_id(env: Option<String>, machine_id: Option<String>) -> String {
    if let Some(id) = env {
        let id = id.trim();
        if !id.is_empty() {
            return id.to_string();
        }
    }
    if let Some(mid) = machine_id {
        let mid = mid.trim();
        if !mid.is_empty() {
            return format!("compute-{}", &mid[..mid.len().min(12)]);
        }
    }
    "compute-node".to_string()
}

pub(super) fn resolve_node_id() -> String {
    derive_node_id(
        std::env::var("ADOS_COMPUTE_NODE_ID").ok(),
        std::fs::read_to_string("/etc/machine-id").ok(),
    )
}

pub(super) fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

/// True when a config flag string is an affirmative (`1`/`true`/`yes`/`on`).
pub(super) fn is_truthy(v: &str) -> bool {
    matches!(
        v.trim().to_ascii_lowercase().as_str(),
        "1" | "true" | "yes" | "on"
    )
}

/// Where the node keeps its state.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct StatePaths {
    pub(super) db: String,
    pub(super) work_root: PathBuf,
    pub(super) credentials: PathBuf,
}

impl StatePaths {
    /// Each path is its `ADOS_COMPUTE_*` override when set; else under
    /// `<data_dir>/node/` when the host gave a data dir; else the
    /// `/var/ados/compute` default. Pure (the env lookup is injected).
    pub(super) fn resolve(data_dir: Option<&str>, env: impl Fn(&str) -> Option<String>) -> Self {
        let data = data_dir
            .filter(|d| !d.trim().is_empty())
            .map(|d| Path::new(d).join("node"));
        let under = |leaf: &str, fallback: &str| match &data {
            Some(dir) => dir.join(leaf),
            None => PathBuf::from(fallback),
        };
        let pick = |key: &str, leaf: &str, fallback: &str| {
            env(key)
                .filter(|v| !v.trim().is_empty())
                .map(PathBuf::from)
                .unwrap_or_else(|| under(leaf, fallback))
        };
        StatePaths {
            db: pick("ADOS_COMPUTE_DB", "jobs.db", "/var/ados/compute/jobs.db")
                .to_string_lossy()
                .into_owned(),
            work_root: pick("ADOS_COMPUTE_WORK", "work", "/var/ados/compute/work"),
            credentials: pick(
                "ADOS_COMPUTE_NODE_CREDENTIALS",
                "node-credentials.json",
                DEFAULT_NODE_CREDENTIALS_PATH,
            ),
        }
    }
}

/// Whether live (in-flight) reconstruction is on: the `ADOS_ATLAS_LIVE` env
/// override wins, else the `atlas.live_reconstruct` plugin config. Opt-in
/// (default off). Pure so the precedence is unit-tested.
pub(super) fn live_enabled(env: Option<&str>, config: Option<bool>) -> bool {
    match env {
        Some(v) => is_truthy(v),
        None => config.unwrap_or(false),
    }
}

/// Build the live-reconstruction cadence config. The cadence defaults (every 30
/// keyframes / 20 s / 8-keyframe floor) come from
/// [`LiveReconstructConfig::default`]; `enabled` is set from the gate, and the
/// thresholds can be tuned via env.
pub(super) async fn live_reconstruct_config(ctx: Option<&PluginContext>) -> LiveReconstructConfig {
    let config = match ctx {
        Some(ctx) => match ctx
            .config
            .get(ATLAS_LIVE_KEY, rmpv::Value::Boolean(false))
            .await
        {
            Ok(v) => v.as_bool(),
            Err(e) => {
                tracing::warn!(key = ATLAS_LIVE_KEY, error = %e, "plugin config read failed; using the default");
                None
            }
        },
        None => None,
    };
    let mut cfg = LiveReconstructConfig {
        enabled: live_enabled(std::env::var("ADOS_ATLAS_LIVE").ok().as_deref(), config),
        ..LiveReconstructConfig::default()
    };
    if let Some(v) = env_u64("ADOS_ATLAS_LIVE_EVERY_KEYFRAMES") {
        cfg.every_keyframes = v.max(1);
    }
    if let Some(s) = env_u64("ADOS_ATLAS_LIVE_INTERVAL_S") {
        cfg.interval_ms = (s as i64).saturating_mul(1000);
    }
    if let Some(v) = env_u64("ADOS_ATLAS_LIVE_MIN_KEYFRAMES") {
        cfg.min_keyframes = v.max(1);
    }
    cfg
}

/// Parse a `u64` env var, or `None` when unset / unparseable.
pub(super) fn env_u64(key: &str) -> Option<u64> {
    std::env::var(key).ok().and_then(|v| v.trim().parse().ok())
}

/// The launch values the plugin host handed this process (the plugin id in
/// argv[1], the rest from its token credential), or `None` for a standalone dev
/// run with no plugin id.
pub(super) fn launch_args() -> Option<RunnerArgs> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    RunnerArgs::parse(&argv, |k| std::env::var(k).ok()).ok()
}

/// Connect to the plugin host when the node was started as the extension's
/// service (a socket and a token among its launch values). `Ok(None)` is a
/// standalone dev run: no host, defaults everywhere. A host that is named but
/// refuses the connection is an error, so the service fails loudly instead of
/// silently running unmanaged.
pub(super) async fn connect_host(
    args: Option<RunnerArgs>,
) -> Result<Option<(PluginContext, Arc<PluginIpcClient>)>, ados_sdk::RunnerError> {
    let Some(args) = args else {
        return Ok(None);
    };
    let (Some(socket), Some(token)) = (args.socket_path.clone(), args.token.clone()) else {
        return Ok(None);
    };
    let ipc = Arc::new(PluginIpcClient::new(args.plugin_id.clone(), token, socket));
    ipc.connect().await?;
    if let Some(dir) = args.data_dir.as_deref() {
        if let Err(e) = std::fs::create_dir_all(dir) {
            tracing::warn!(dir, error = %e, "could not create plugin data dir");
        }
    }
    let ctx = PluginContext::new(
        ipc.clone(),
        env!("CARGO_PKG_VERSION"),
        args.agent_id,
        args.data_dir,
        BTreeMap::new(),
    );
    Ok(Some((ctx, ipc)))
}

#[cfg(test)]
mod tests {
    use super::{derive_node_id, live_enabled, StatePaths};
    use std::path::PathBuf;

    #[test]
    fn node_id_prefers_the_env_override() {
        assert_eq!(
            derive_node_id(Some("rtx-box".to_string()), Some("mid".to_string())),
            "rtx-box"
        );
    }

    #[test]
    fn node_id_derives_from_machine_id_when_env_unset_and_is_unique_per_host() {
        let a = derive_node_id(None, Some("aaaaaaaaaaaaaaaa1111".to_string()));
        let b = derive_node_id(None, Some("bbbbbbbbbbbbbbbb2222".to_string()));
        assert_eq!(a, "compute-aaaaaaaaaaaa");
        assert_ne!(a, b, "distinct machine-ids must yield distinct node ids");
    }

    #[test]
    fn node_id_falls_back_when_nothing_is_available() {
        assert_eq!(derive_node_id(None, None), "compute-node");
        // A blank env / blank machine-id both fall through to the next source.
        assert_eq!(derive_node_id(Some("  ".to_string()), None), "compute-node");
    }

    #[test]
    fn live_reconstruct_is_opt_in_and_the_env_overrides_the_config() {
        assert!(!live_enabled(None, None), "absent everywhere reads off");
        assert!(live_enabled(None, Some(true)));
        assert!(!live_enabled(None, Some(false)));
        // The env wins over the config, both ways.
        assert!(!live_enabled(Some("0"), Some(true)));
        assert!(live_enabled(Some("true"), Some(false)));
    }

    #[test]
    fn state_paths_default_under_the_plugin_data_dir_and_keep_the_env_overrides() {
        let p = StatePaths::resolve(Some("/data/we"), |_| None);
        assert_eq!(p.db, "/data/we/node/jobs.db");
        assert_eq!(p.work_root, PathBuf::from("/data/we/node/work"));
        assert_eq!(
            p.credentials,
            PathBuf::from("/data/we/node/node-credentials.json")
        );

        // An explicit override wins over the data dir.
        let overridden = |k: &str| match k {
            "ADOS_COMPUTE_DB" => Some(":memory:".to_string()),
            "ADOS_COMPUTE_WORK" => Some("/scratch/work".to_string()),
            _ => None,
        };
        let p = StatePaths::resolve(Some("/data/we"), overridden);
        assert_eq!(p.db, ":memory:");
        assert_eq!(p.work_root, PathBuf::from("/scratch/work"));
        assert_eq!(
            p.credentials,
            PathBuf::from("/data/we/node/node-credentials.json")
        );

        // No host data dir: the standalone defaults.
        let p = StatePaths::resolve(None, |_| None);
        assert_eq!(p.db, "/var/ados/compute/jobs.db");
        assert_eq!(p.work_root, PathBuf::from("/var/ados/compute/work"));
    }
}
