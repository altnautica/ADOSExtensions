//! Applying the exposure lock to the capture cameras.
//!
//! The capture loop asks every enabled capture camera, in parallel, to take
//! the settled exposure through the host's camera control
//! (`camera.control.set`, capability `camera.control.write`). A lock sends
//! the exposure (µs) and the analogue gain (a multiplier, 1.0 = unity) with
//! auto exposure off; an unlock turns auto exposure back on. The resulting
//! [`ExposureView`] is `applied` only when every camera took the values, and
//! then carries the first camera's read-back.
//!
//! The control reply waits at most [`REPLY_BUDGET`]. An apply still running
//! then is answered `pending` and keeps going; its result lands in the state.

use std::sync::Arc;
use std::time::Duration;

use ados_protocol::camera_control::ENGINE_WAIT_MS;
use async_trait::async_trait;
use tokio::task::JoinSet;
use world_engine_protocol::capture_ctl::{ExposureView, CONTROL_REQUEST_TIMEOUT_MS};

/// Host RPC and loop overhead on top of the host's engine wait.
pub const REPLY_MARGIN_MS: u64 = 500;
/// How long the exposure reply waits for the cameras before answering
/// `pending`: the host's wait for the vision engine to set and read back the
/// controls, plus the margin.
pub const REPLY_BUDGET: Duration = Duration::from_millis(ENGINE_WAIT_MS + REPLY_MARGIN_MS);

// The reply must be sent before the link gives up on the control request.
const _: () = assert!(ENGINE_WAIT_MS + REPLY_MARGIN_MS < CONTROL_REQUEST_TIMEOUT_MS);

/// Reason while an apply is still running.
pub const PENDING: &str = "pending";

/// One camera's answer to a control request. `gain` is a multiplier.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CameraOutcome {
    pub applied: bool,
    pub exposure_us: Option<u32>,
    pub gain: Option<f32>,
    /// The host's reason code when not applied, passed through as is
    /// (`not_v4l2`, `control_unsupported`, `gain_unit_unknown`,
    /// `not_capture_camera`, ..).
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

fn not_applied(want: &ExposureView, reason: &str) -> ExposureView {
    ExposureView {
        applied: false,
        reason: Some(reason.to_string()),
        ..want.clone()
    }
}

/// Apply `want` (already settled) to every camera in parallel and wait for
/// all of them. Not applied: the requested values are echoed with the first
/// failing camera's reason.
pub async fn apply_exposure(
    ctl: Arc<dyn CameraControl>,
    cameras: Vec<String>,
    want: ExposureView,
) -> ExposureView {
    if cameras.is_empty() {
        return not_applied(&want, "no_camera");
    }
    let (us, gain) = if want.locked {
        (want.exposure_us, want.gain)
    } else {
        (None, None)
    };
    if want.locked && us.is_none() {
        return not_applied(&want, "no_exposure_value");
    }
    let mut set = JoinSet::new();
    for (i, id) in cameras.into_iter().enumerate() {
        let ctl = ctl.clone();
        set.spawn(async move { (i, ctl.set_exposure(&id, us, gain).await) });
    }
    let mut outcomes: Vec<(usize, CameraOutcome)> = Vec::with_capacity(set.len());
    while let Some(r) = set.join_next().await {
        match r {
            Ok(o) => outcomes.push(o),
            Err(_) => return not_applied(&want, "control_failed"),
        }
    }
    outcomes.sort_by_key(|(i, _)| *i);
    if let Some((_, bad)) = outcomes.iter().find(|(_, o)| !o.applied) {
        return not_applied(&want, bad.reason.as_deref().unwrap_or("not_applied"));
    }
    let first = &outcomes[0].1;
    ExposureView {
        applied: true,
        exposure_us: first.exposure_us,
        gain: first.gain,
        reason: None,
        ..want
    }
}

/// [`apply_exposure`], calling `on_pending` with a `pending` view if it is
/// still running after `budget`; the apply is never cut short. Returns its
/// final view.
pub async fn apply_within(
    ctl: Arc<dyn CameraControl>,
    cameras: Vec<String>,
    want: ExposureView,
    budget: Duration,
    on_pending: impl FnOnce(ExposureView),
) -> ExposureView {
    let mut work = tokio::spawn(apply_exposure(ctl, cameras, want.clone()));
    let done = match tokio::time::timeout(budget, &mut work).await {
        Ok(done) => done,
        Err(_) => {
            on_pending(not_applied(&want, PENDING));
            work.await
        }
    };
    done.unwrap_or_else(|_| not_applied(&want, "control_failed"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use parking_lot::Mutex;

    /// One request the fake saw: camera, exposure, gain.
    type Seen = (String, Option<u32>, Option<f32>);

    /// Answers per camera id after a delay.
    struct Fake {
        answers: Vec<(&'static str, u64, CameraOutcome)>,
        seen: Mutex<Vec<Seen>>,
    }

    #[async_trait]
    impl CameraControl for Fake {
        async fn set_exposure(&self, id: &str, us: Option<u32>, g: Option<f32>) -> CameraOutcome {
            self.seen.lock().push((id.to_string(), us, g));
            let (_, ms, a) = self.answers.iter().find(|(k, _, _)| *k == id).unwrap();
            tokio::time::sleep(Duration::from_millis(*ms)).await;
            a.clone()
        }
    }

    fn fake(answers: Vec<(&'static str, u64, CameraOutcome)>) -> Arc<Fake> {
        Arc::new(Fake {
            answers,
            seen: Mutex::new(Vec::new()),
        })
    }

    fn took(us: u32, gain: f32) -> CameraOutcome {
        CameraOutcome {
            applied: true,
            exposure_us: Some(us),
            gain: Some(gain),
            reason: None,
        }
    }

    fn refused(reason: &str) -> CameraOutcome {
        CameraOutcome {
            reason: Some(reason.into()),
            ..Default::default()
        }
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

    #[test]
    fn the_reply_budget_fits_the_link_wait() {
        assert!(REPLY_BUDGET < Duration::from_millis(CONTROL_REQUEST_TIMEOUT_MS));
    }

    #[tokio::test]
    async fn every_camera_taking_it_applies_with_the_first_read_back() {
        let f = fake(vec![("a", 0, took(1990, 2.0)), ("b", 0, took(2010, 2.0))]);
        let v = apply_exposure(f.clone(), ids(), lock()).await;
        assert!(v.applied && v.locked && v.reason.is_none(), "{v:?}");
        assert_eq!(v.exposure_us, Some(1990));
        assert!(f
            .seen
            .lock()
            .contains(&("b".to_string(), Some(2000), Some(2.0))));
    }

    #[tokio::test]
    async fn a_refusing_camera_leaves_it_unapplied_with_the_host_reason() {
        for reason in ["not_v4l2", "gain_unit_unknown", "not_capture_camera"] {
            let f = fake(vec![("a", 0, took(2000, 2.0)), ("b", 0, refused(reason))]);
            let v = apply_exposure(f, ids(), lock()).await;
            assert!(!v.applied && v.locked, "{v:?}");
            assert_eq!(v.reason.as_deref(), Some(reason));
            assert_eq!(v.exposure_us, Some(2000));
        }
    }

    #[tokio::test]
    async fn cameras_are_set_in_parallel() {
        let f = fake(vec![
            ("a", 300, took(2000, 2.0)),
            ("b", 300, took(2000, 2.0)),
        ]);
        let t = std::time::Instant::now();
        assert!(apply_exposure(f, ids(), lock()).await.applied);
        assert!(
            t.elapsed() < Duration::from_millis(550),
            "{:?}",
            t.elapsed()
        );
    }

    #[tokio::test]
    async fn a_slow_apply_answers_pending_then_finishes() {
        let f = fake(vec![("a", 200, took(1990, 2.0)), ("b", 0, took(2000, 2.0))]);
        let mut early = None;
        let v = apply_within(f, ids(), lock(), Duration::from_millis(30), |p| {
            early = Some(p)
        })
        .await;
        let early = early.expect("answered pending");
        assert!(!early.applied && early.locked);
        assert_eq!(early.reason.as_deref(), Some(PENDING));
        assert!(v.applied, "{v:?}");
        assert_eq!(v.exposure_us, Some(1990));
    }

    #[tokio::test]
    async fn a_fast_apply_never_answers_pending() {
        let f = fake(vec![("a", 0, took(2000, 2.0)), ("b", 0, took(2000, 2.0))]);
        let v = apply_within(f, ids(), lock(), REPLY_BUDGET, |_| panic!("pending")).await;
        assert!(v.applied);
    }

    #[tokio::test]
    async fn an_unlock_asks_for_auto_exposure() {
        let f = fake(vec![("a", 0, took(8000, 1.0)), ("b", 0, took(8000, 1.0))]);
        let want = ExposureView {
            locked: false,
            ..lock()
        };
        let v = apply_exposure(f.clone(), ids(), want).await;
        assert!(v.applied && !v.locked);
        assert!(f
            .seen
            .lock()
            .iter()
            .all(|(_, us, g)| us.is_none() && g.is_none()));
    }
}
