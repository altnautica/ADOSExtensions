//! The node's offload-serving config, read from the plugin config
//! (`serving.enabled`, `serving.detector_model`).
//!
//! The node auto-serves perception offload by default (`serving.enabled:
//! auto`). The two operator controls are whether to serve at all and which
//! detector model to serve. The `ADOS_COMPUTE_DETECTOR_MODEL` env still wins
//! over the config (the bench override, applied where the detector is loaded).
//! A resolution to a path that does not exist simply falls back to the mock at
//! load time (the node still comes up).

use ados_sdk::PluginContext;

/// Where a bare model id resolves (`<dir>/<id>.onnx`).
const DEFAULT_MODELS_DIR: &str = "/opt/ados/models/vision";

/// Plugin config key: `auto` | `on` | `off`.
const SERVING_ENABLED_KEY: &str = "serving.enabled";
/// Plugin config key: a model path or a bare model id, or null.
const SERVING_DETECTOR_MODEL_KEY: &str = "serving.detector_model";

/// The resolved serving config the node acts on.
#[derive(Debug, Clone, PartialEq)]
pub struct ServingConfig {
    /// Whether to accept perception-offload sessions (`serving.enabled` !=
    /// "off"). Default true (auto-serve).
    pub serve_offload: bool,
    /// A model-path fallback for the served detector, resolved from
    /// `serving.detector_model` (a path, or a bare id under the models dir).
    /// `None` ⇒ the node uses the env / the mock.
    pub detector_model_path: Option<String>,
}

impl Default for ServingConfig {
    fn default() -> Self {
        ServingConfig {
            serve_offload: true,
            detector_model_path: None,
        }
    }
}

/// Resolve `detector_model` to a model path: an explicit path (contains `/` or
/// ends `.onnx`) is used verbatim; a bare id resolves to `<models_dir>/<id>.onnx`.
/// Empty / absent ⇒ `None`. Pure (testable).
fn resolve_detector_model(detector_model: Option<&str>, models_dir: &str) -> Option<String> {
    let m = detector_model.map(str::trim).filter(|s| !s.is_empty())?;
    if m.contains('/') || m.ends_with(".onnx") {
        Some(m.to_string())
    } else {
        Some(format!("{}/{m}.onnx", models_dir.trim_end_matches('/')))
    }
}

/// Interpret the two config values into the resolved serving config. Pure
/// (testable): the host read is in [`load_serving_config`].
pub fn resolve_serving_config(
    enabled: Option<&str>,
    detector_model: Option<&str>,
) -> ServingConfig {
    let serve_offload = enabled
        .map(|e| !e.trim().eq_ignore_ascii_case("off"))
        .unwrap_or(true);
    ServingConfig {
        serve_offload,
        detector_model_path: resolve_detector_model(detector_model, DEFAULT_MODELS_DIR),
    }
}

/// Read + resolve the serving config from the plugin config. Without a host
/// (a standalone dev run), or on a failed read, the defaults apply (serve, no
/// model override).
pub async fn load_serving_config(ctx: Option<&PluginContext>) -> ServingConfig {
    let Some(ctx) = ctx else {
        return ServingConfig::default();
    };
    let read = |key: &'static str, default: rmpv::Value| async move {
        match ctx.config.get(key, default).await {
            Ok(v) => v.as_str().map(str::to_string),
            Err(e) => {
                tracing::warn!(key, error = %e, "plugin config read failed; using the default");
                None
            }
        }
    };
    let enabled = read(SERVING_ENABLED_KEY, rmpv::Value::from("auto")).await;
    let model = read(SERVING_DETECTOR_MODEL_KEY, rmpv::Value::Nil).await;
    resolve_serving_config(enabled.as_deref(), model.as_deref())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_serve_with_no_model_override() {
        let cfg = resolve_serving_config(None, None);
        assert!(cfg.serve_offload);
        assert_eq!(cfg.detector_model_path, None);
    }

    #[test]
    fn enabled_off_disables_serving() {
        assert!(!resolve_serving_config(Some("off"), None).serve_offload);
        assert!(!resolve_serving_config(Some(" OFF "), None).serve_offload);
        // "on" / "auto" / anything-else all serve.
        for v in ["on", "auto", "ON", " "] {
            assert!(
                resolve_serving_config(Some(v), None).serve_offload,
                "value {v:?} should serve"
            );
        }
    }

    #[test]
    fn detector_model_resolves_a_bare_id_and_a_path() {
        // A bare id resolves under the models dir.
        assert_eq!(
            resolve_detector_model(Some("coco-yolov8n"), "/models/vision"),
            Some("/models/vision/coco-yolov8n.onnx".into())
        );
        // An explicit path is used verbatim.
        assert_eq!(
            resolve_detector_model(Some("/tmp/custom.onnx"), "/models/vision"),
            Some("/tmp/custom.onnx".into())
        );
        // A bare `.onnx` filename is treated as a path (verbatim).
        assert_eq!(
            resolve_detector_model(Some("my.onnx"), "/models"),
            Some("my.onnx".into())
        );
        // Empty / absent ⇒ None.
        assert_eq!(resolve_detector_model(Some("  "), "/models"), None);
        assert_eq!(resolve_detector_model(None, "/models"), None);
        // The config value flows through the same resolution.
        assert_eq!(
            resolve_serving_config(Some("auto"), Some("/tmp/custom.onnx")).detector_model_path,
            Some("/tmp/custom.onnx".into())
        );
    }
}
