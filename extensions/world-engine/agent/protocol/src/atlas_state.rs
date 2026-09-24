//! The Atlas telemetry slice: the `atlas` half of the extension's `status`
//! telemetry channel, in the camelCase shape the GCS World Model pages read.
//!
//! The capture fields are the capture service's own (state / session /
//! keyframes / cameras / VIO health), taken from the latest capture-state event
//! on the private atlas bus. The three *transport* fields (the compute node, the
//! active bearer, and the last-forwarded-keyframe time) are known only by the
//! drone-side forwarder, which passes its [`AtlasForwardStatus`] in; with no
//! forward status the transport fields are omitted rather than defaulted.

use serde::Serialize;

use crate::atlas::{
    bearer_carries_keyframes, bearer_keyframe_degraded_reason, AtlasForwardStatus, CaptureState,
    CaptureStatus, PoseSource, VioHealth,
};

/// Schema version stamped on the slice, so a reader can detect a
/// producer/reader drift.
pub const ATLAS_STATE_VERSION: u16 = 1;

/// The Atlas telemetry slice.
#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct AtlasStateSlice<'a> {
    /// Schema version, [`ATLAS_STATE_VERSION`].
    version: u16,
    /// Local build time (epoch ms), for the consumer's own freshness reasoning.
    generated_at_ms: i64,
    /// `CaptureState` serializes snake_case (idle / capturing / paused / …).
    state: &'a CaptureState,
    session_id: &'a str,
    keyframes_ingested: u64,
    ingest_rate_hz: f32,
    camera_count: u32,
    vio_health: &'a VioHealth,
    /// The paired compute node (mDNS `deviceId`), from the forward status.
    #[serde(skip_serializing_if = "Option::is_none")]
    compute_node_id: Option<&'a str>,
    /// The active transport bearer (`direct-lan` / `wfb-relay` / `cloud`).
    #[serde(skip_serializing_if = "Option::is_none")]
    bearer: Option<&'a str>,
    /// Epoch ms a keyframe was last forwarded.
    #[serde(skip_serializing_if = "Option::is_none")]
    last_kf_at: Option<i64>,
    /// Whether the ACTIVE bearer can carry a full keyframe. Without it a
    /// `wfb-relay` bearer reads as a working lane while its payload ceiling
    /// means no keyframe ever crosses it. Omitted when no bearer is known.
    #[serde(skip_serializing_if = "Option::is_none")]
    keyframes_carried: Option<bool>,
    /// The operator-facing reason the active bearer cannot carry keyframes.
    #[serde(skip_serializing_if = "Option::is_none")]
    degraded_reason: Option<&'a str>,
    /// True once the session-wide keyframe cap stopped selection: the capture is
    /// still `capturing` and the count is frozen on purpose.
    capped: bool,
    /// True once the session's geo anchor latched. Keyframe selection is refused
    /// before it, so a capture with `anchored: false` is running and producing
    /// nothing, which is otherwise indistinguishable from a stalled camera.
    anchored: bool,
    /// Which producer filled the pose being tagged (`local_vio` /
    /// `offloaded_slam`), so a silent switch to offloaded SLAM is visible.
    pose_tier: &'a PoseSource,
    /// Keyframes that reached no subscriber, so the ingested count is not read
    /// as reconstruction input that exists.
    dropped_keyframes: u64,
}

/// Build the slice from a capture status and the forwarder's (optional)
/// transport facts, stamped `generated_at_ms`.
pub fn atlas_state_slice<'a>(
    status: &'a CaptureStatus,
    forward: Option<&'a AtlasForwardStatus>,
    generated_at_ms: i64,
) -> AtlasStateSlice<'a> {
    // The keyframe-carrying fact is a pure property of the bearer name, decided
    // once in the contract so the forwarder, this slice and the GCS cannot give
    // three answers. With no known bearer both fields stay absent.
    let bearer = forward.and_then(|f| f.bearer.as_deref());
    AtlasStateSlice {
        version: ATLAS_STATE_VERSION,
        generated_at_ms,
        state: &status.state,
        session_id: &status.session_id,
        keyframes_ingested: status.keyframes,
        ingest_rate_hz: status.ingest_rate_hz,
        camera_count: status.camera_count,
        vio_health: &status.vio_health,
        compute_node_id: forward.and_then(|f| f.compute_node_id.as_deref()),
        bearer,
        last_kf_at: forward.and_then(|f| f.last_kf_at_ms),
        keyframes_carried: bearer.map(bearer_carries_keyframes),
        degraded_reason: bearer.and_then(bearer_keyframe_degraded_reason),
        capped: status.capped,
        anchored: status.anchored,
        pose_tier: &status.pose_tier,
        dropped_keyframes: status.dropped_keyframes,
    }
}

/// [`atlas_state_slice`] as a JSON value, ready for `telemetry.extend`.
pub fn atlas_state_json(
    status: &CaptureStatus,
    forward: Option<&AtlasForwardStatus>,
    generated_at_ms: i64,
) -> serde_json::Value {
    serde_json::to_value(atlas_state_slice(status, forward, generated_at_ms))
        .unwrap_or(serde_json::Value::Null)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::atlas::ATLAS_FORWARD_STATUS_VERSION;

    fn status() -> CaptureStatus {
        CaptureStatus {
            session_id: "sess-1".into(),
            state: CaptureState::Capturing,
            keyframes: 42,
            vio_health: VioHealth::Good,
            camera_count: 3,
            ingest_rate_hz: 9.5,
            capped: false,
            anchored: true,
            pose_tier: PoseSource::LocalVio,
            dropped_keyframes: 0,
        }
    }

    #[test]
    fn the_slice_uses_the_camelcase_gcs_field_names() {
        let v = atlas_state_json(&status(), None, 7);
        assert_eq!(v["state"], "capturing");
        assert_eq!(v["sessionId"], "sess-1");
        assert_eq!(v["keyframesIngested"], 42);
        assert_eq!(v["ingestRateHz"], 9.5);
        assert_eq!(v["cameraCount"], 3);
        assert_eq!(v["vioHealth"], "good");
        assert_eq!(v["version"], ATLAS_STATE_VERSION);
        assert_eq!(v["generatedAtMs"], 7);
        // With no forward status, the transport fields are omitted (not null).
        assert!(v.get("computeNodeId").is_none());
        assert!(v.get("bearer").is_none());
        assert!(v.get("lastKfAt").is_none());
        assert!(v.get("keyframesCarried").is_none());
    }

    #[test]
    fn the_forward_status_folds_the_transport_fields_into_the_slice() {
        let forward = AtlasForwardStatus {
            version: ATLAS_FORWARD_STATUS_VERSION,
            compute_node_id: Some("rtx-box".into()),
            bearer: Some("wfb-relay".into()),
            last_kf_at_ms: Some(1_700),
            generated_at_ms: 1_699,
        };
        let v = atlas_state_json(&status(), Some(&forward), 1_800);
        assert_eq!(v["computeNodeId"], "rtx-box");
        assert_eq!(v["bearer"], "wfb-relay");
        assert_eq!(v["lastKfAt"], 1_700);
        // The relay lane is reported as unable to carry keyframes, with a reason.
        assert_eq!(v["keyframesCarried"], false);
        assert!(v["degradedReason"].as_str().is_some());
    }
}
