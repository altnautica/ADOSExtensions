//! The WFB-relay bearer: carry small Atlas events drone→ground over the WFB
//! radio link, for the field/outdoor topology where there is no shared LAN.
//!
//! The drone side rides the agent's **auxiliary application stream**, which the
//! plugin host exposes as `radio.aux_stream.*`: the bearer opens the lane once,
//! then sends one framed [`AtlasEvent`] (a self-delimiting msgpack frame) per
//! application-stream payload. The ground station's World Engine link receives
//! the same payloads through its own `radio.aux_stream` subscription and
//! re-emits them onto the LAN into the compute node's event router.
//!
//! **The WFB lane is decimated.** One aux payload is capped at
//! [`WFB_MAX_DATAGRAM`] bytes, so a full-res keyframe (the LAN bearer's
//! many-MB envelope) cannot ride it; only small descriptor/pose/status events
//! do. A framed event over the cap is rejected with
//! [`TransportError::PayloadTooLarge`] (a per-bearer capacity limit, so
//! retriable: the ladder declines WFB by its cap and offers the event to a
//! fatter downstream lane).
//!
//! The aux stream is safe-by-default off (a dead-switch in the radio service):
//! an open against a disabled deployment is refused, which this bearer surfaces
//! as [`TransportError::Unavailable`] so the ladder skips it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use world_engine_protocol::atlas::AtlasEvent;

use crate::{AtlasBearer, BearerKind, TransportError};

/// The per-payload ceiling on the auxiliary application stream
/// (`ados_protocol::aux_mux::AUX_MAX_PAYLOAD`): a framed event above it never
/// enters the lane.
pub const WFB_MAX_DATAGRAM: usize = ados_protocol::aux_mux::AUX_MAX_PAYLOAD;

/// The radio lane the relay bearer sends on. The World Engine link implements it
/// over the plugin host's `radio.aux_stream` methods; tests implement it in
/// memory.
#[async_trait::async_trait]
pub trait AuxLane: Send + Sync {
    /// Open the lane (idempotent). `Ok(true)` when the lane is active, `Ok(false)`
    /// when the deployment has it disabled or the radio service is down.
    async fn open(&self) -> Result<bool, TransportError>;
    /// Send one payload on the application-stream channel.
    async fn send(&self, payload: &[u8]) -> Result<(), TransportError>;
}

/// Carries small framed Atlas events over the WFB aux application stream.
pub struct WfbRelayBearer {
    lane: Arc<dyn AuxLane>,
    /// The framed-event size ceiling for this lane.
    max_datagram: usize,
    /// Whether the lane answered an open as active. Cleared on a send failure so
    /// the next call re-opens rather than blackholing into a dead lane.
    open: AtomicBool,
}

impl WfbRelayBearer {
    /// A bearer over `lane` with the standard payload ceiling.
    pub fn new(lane: Arc<dyn AuxLane>) -> Self {
        Self::with_limit(lane, WFB_MAX_DATAGRAM)
    }

    /// A bearer over `lane` with an explicit payload ceiling.
    pub fn with_limit(lane: Arc<dyn AuxLane>, max_datagram: usize) -> Self {
        Self {
            lane,
            max_datagram,
            open: AtomicBool::new(false),
        }
    }

    /// Ensure the lane is open. A disabled or unreachable lane is Unavailable
    /// (retriable), so the ladder skips it.
    async fn ensure_open(&self) -> Result<(), TransportError> {
        if self.open.load(Ordering::Acquire) {
            return Ok(());
        }
        if self.lane.open().await? {
            self.open.store(true, Ordering::Release);
            Ok(())
        } else {
            Err(TransportError::Unavailable)
        }
    }
}

#[async_trait::async_trait]
impl AtlasBearer for WfbRelayBearer {
    fn kind(&self) -> BearerKind {
        BearerKind::WfbRelay
    }

    async fn is_available(&self) -> bool {
        self.ensure_open().await.is_ok()
    }

    async fn send(&self, event: &AtlasEvent) -> Result<(), TransportError> {
        // Frame first so an oversized event is rejected before opening anything.
        let body = event.encode()?;
        if body.len() > self.max_datagram {
            // Retriable: a fatter downstream lane may carry it.
            return Err(TransportError::PayloadTooLarge(body.len()));
        }
        self.ensure_open().await?;
        // Fire-and-forget: Ok means the host accepted the payload for the radio,
        // NOT that the peer decoded it — the ground relay's received-side counter
        // is the delivery proof.
        match self.lane.send(&body).await {
            Ok(()) => Ok(()),
            Err(e) => {
                self.open.store(false, Ordering::Release);
                Err(e)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::Mutex;

    /// An in-memory lane: records sent payloads and answers opens with `active`.
    struct MemLane {
        active: bool,
        opens: Mutex<u32>,
        sent: Mutex<Vec<Vec<u8>>>,
    }

    impl MemLane {
        fn new(active: bool) -> Arc<Self> {
            Arc::new(Self {
                active,
                opens: Mutex::new(0),
                sent: Mutex::new(Vec::new()),
            })
        }
    }

    #[async_trait::async_trait]
    impl AuxLane for MemLane {
        async fn open(&self) -> Result<bool, TransportError> {
            *self.opens.lock().await += 1;
            Ok(self.active)
        }
        async fn send(&self, payload: &[u8]) -> Result<(), TransportError> {
            self.sent.lock().await.push(payload.to_vec());
            Ok(())
        }
    }

    fn event(topic: &str, payload: Vec<u8>) -> AtlasEvent {
        AtlasEvent::new(topic, None, payload)
    }

    #[tokio::test]
    async fn a_sent_event_rides_the_lane_as_one_framed_payload() {
        let lane = MemLane::new(true);
        let bearer = WfbRelayBearer::new(lane.clone());

        bearer
            .send(&event("atlas.pose", vec![1, 2, 3, 4]))
            .await
            .unwrap();
        bearer.send(&event("atlas.pose", vec![5])).await.unwrap();

        let sent = lane.sent.lock().await;
        assert_eq!(sent.len(), 2);
        let got = AtlasEvent::decode(&sent[0]).unwrap();
        assert_eq!(got.topic, "atlas.pose");
        assert_eq!(got.payload, vec![1, 2, 3, 4]);
        assert_eq!(*lane.opens.lock().await, 1, "the lane is opened once");
        assert!(bearer.is_available().await);
    }

    #[tokio::test]
    async fn an_oversized_event_is_rejected_retriably_without_sending() {
        let lane = MemLane::new(true);
        let bearer = WfbRelayBearer::with_limit(lane.clone(), 16);

        let err = bearer
            .send(&event("atlas.keyframe", vec![0u8; 4096]))
            .await
            .unwrap_err();
        assert!(matches!(err, TransportError::PayloadTooLarge(_)));
        assert!(
            err.is_retriable(),
            "WFB declines by its cap; the ladder falls over to a fatter lane"
        );
        assert!(lane.sent.lock().await.is_empty());
        assert_eq!(*lane.opens.lock().await, 0, "nothing is opened for it");
    }

    #[tokio::test]
    async fn a_disabled_aux_stream_is_unavailable_not_an_error() {
        let lane = MemLane::new(false);
        let bearer = WfbRelayBearer::new(lane.clone());
        assert!(!bearer.is_available().await);
        let err = bearer
            .send(&event("atlas.pose", vec![1]))
            .await
            .unwrap_err();
        assert!(matches!(err, TransportError::Unavailable));
        assert!(err.is_retriable(), "a down lane lets the ladder fall over");
        assert!(lane.sent.lock().await.is_empty());
    }
}
