//! Applying the exposure lock to the capture cameras.
//!
//! The capture loop asks every enabled capture camera to take the settled
//! exposure through the host's camera control (`camera.control.set`,
//! capability `camera.control.write`). A lock sends the exposure and gain
//! with auto exposure off; an unlock turns auto exposure back on. The
//! resulting [`ExposureView`] is `applied` only when every camera took the
//! values, and then carries the first camera's read-back.

use std::time::Duration;

use async_trait::async_trait;
use world_engine_protocol::capture_ctl::ExposureView;

/// Longest an apply may take, all cameras together; the control reply waits
/// on it, so it stays under the link's 3 s control-request limit.
pub const APPLY_TIMEOUT: Duration = Duration::from_millis(2500);

/// One camera's answer to a control request.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CameraOutcome {
    pub applied: bool,
    pub exposure_us: Option<u32>,
    pub gain: Option<f32>,
    pub reason: Option<String>,
}

/// The host's per-camera exposure control.
#[async_trait]
pub trait CameraControl: Send + Sync {
    /// Lock (`Some(us)`, gain) or release (`None`) one camera's exposure.
    async fn set_exposure(
        &self,
        camera_id: &str,
        exposure_us: Option<u32>,
        gain: Option<f32>,
    ) -> CameraOutcome;
}

/// Camera control through the plugin host.
pub struct SdkCameraControl(pub ados_sdk::PluginContext);

#[async_trait]
impl CameraControl for SdkCameraControl {
    async fn set_exposure(
        &self,
        camera_id: &str,
        exposure_us: Option<u32>,
        gain: Option<f32>,
    ) -> CameraOutcome {
        let lock = exposure_us.is_some();
        let req = ados_sdk::CameraControlRequest {
            camera_id: camera_id.to_string(),
            exposure_us,
            gain: if lock { gain } else { None },
            auto_exposure: Some(!lock),
        };
        match self.0.camera.set_controls(&req).await {
            Ok(r) => CameraOutcome {
                applied: r.applied,
                exposure_us: r.exposure_us,
                gain: r.gain,
                reason: r.reason,
            },
            Err(ados_sdk::ClientError::CapabilityDenied(_)) => CameraOutcome {
                reason: Some("capability_denied".into()),
                ..Default::default()
            },
            Err(e) => CameraOutcome {
                reason: Some(format!("control_failed: {e}")),
                ..Default::default()
            },
        }
    }
}

/// Apply `want` (already settled) to `cameras` within `limit`. Not applied:
/// the requested values are echoed with the first failing camera's reason.
pub async fn apply_exposure(
    ctl: &dyn CameraControl,
    cameras: &[String],
    want: ExposureView,
    limit: Duration,
) -> ExposureView {
    let not_applied = |reason: &str| ExposureView {
        applied: false,
        reason: Some(reason.to_string()),
        ..want.clone()
    };
    if cameras.is_empty() {
        return not_applied("no_camera");
    }
    let (us, gain) = if want.locked {
        (want.exposure_us, want.gain)
    } else {
        (None, None)
    };
    if want.locked && us.is_none() {
        return not_applied("no_exposure_value");
    }
    let all = async {
        let mut out = Vec::with_capacity(cameras.len());
        for id in cameras {
            out.push(ctl.set_exposure(id, us, gain).await);
        }
        out
    };
    let Ok(outcomes) = tokio::time::timeout(limit, all).await else {
        return not_applied("timeout");
    };
    if let Some(bad) = outcomes.iter().find(|o| !o.applied) {
        return not_applied(bad.reason.as_deref().unwrap_or("not_applied"));
    }
    let first = &outcomes[0];
    ExposureView {
        applied: true,
        exposure_us: first.exposure_us,
        gain: first.gain,
        reason: None,
        ..want
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use parking_lot::Mutex;

    /// One request the fake saw: camera, exposure, gain.
    type Seen = (String, Option<u32>, Option<f32>);

    /// Answers per camera id; `None` never answers.
    struct Fake {
        answers: Vec<(&'static str, Option<CameraOutcome>)>,
        seen: Mutex<Vec<Seen>>,
    }

    #[async_trait]
    impl CameraControl for Fake {
        async fn set_exposure(&self, id: &str, us: Option<u32>, g: Option<f32>) -> CameraOutcome {
            self.seen.lock().push((id.to_string(), us, g));
            match self
                .answers
                .iter()
                .find(|(k, _)| *k == id)
                .and_then(|(_, a)| a.clone())
            {
                Some(a) => a,
                None => std::future::pending().await,
            }
        }
    }

    fn fake(answers: Vec<(&'static str, Option<CameraOutcome>)>) -> Fake {
        Fake {
            answers,
            seen: Mutex::new(Vec::new()),
        }
    }

    fn took(us: u32, gain: f32) -> Option<CameraOutcome> {
        Some(CameraOutcome {
            applied: true,
            exposure_us: Some(us),
            gain: Some(gain),
            reason: None,
        })
    }

    fn lock() -> ExposureView {
        ExposureView {
            locked: true,
            exposure_us: Some(2000),
            gain: Some(2.0),
            ..Default::default()
        }
    }

    fn ids() -> Vec<String> {
        vec!["a".into(), "b".into()]
    }

    #[tokio::test]
    async fn every_camera_taking_it_applies_with_the_first_read_back() {
        let f = fake(vec![("a", took(1990, 2.0)), ("b", took(2010, 2.0))]);
        let v = apply_exposure(&f, &ids(), lock(), APPLY_TIMEOUT).await;
        assert!(v.applied && v.locked && v.reason.is_none(), "{v:?}");
        assert_eq!(v.exposure_us, Some(1990));
        assert_eq!(f.seen.lock()[1], ("b".to_string(), Some(2000), Some(2.0)));
    }

    #[tokio::test]
    async fn one_camera_refusing_leaves_it_unapplied_with_the_reason() {
        let rtsp = Some(CameraOutcome {
            reason: Some("not_v4l2".into()),
            ..Default::default()
        });
        let f = fake(vec![("a", took(2000, 2.0)), ("b", rtsp)]);
        let v = apply_exposure(&f, &ids(), lock(), APPLY_TIMEOUT).await;
        assert!(!v.applied && v.locked, "{v:?}");
        assert_eq!(v.reason.as_deref(), Some("not_v4l2"));
        assert_eq!(v.exposure_us, Some(2000));
    }

    #[tokio::test]
    async fn a_camera_that_never_answers_times_out() {
        let f = fake(vec![("a", took(2000, 2.0)), ("b", None)]);
        let v = apply_exposure(&f, &ids(), lock(), Duration::from_millis(50)).await;
        assert!(!v.applied, "{v:?}");
        assert_eq!(v.reason.as_deref(), Some("timeout"));
    }

    #[tokio::test]
    async fn an_unlock_asks_for_auto_exposure() {
        let f = fake(vec![("a", took(8000, 1.0)), ("b", took(8000, 1.0))]);
        let want = ExposureView {
            locked: false,
            ..lock()
        };
        let v = apply_exposure(&f, &ids(), want, APPLY_TIMEOUT).await;
        assert!(v.applied && !v.locked);
        assert_eq!(f.seen.lock()[0], ("a".to_string(), None, None));
    }
}
