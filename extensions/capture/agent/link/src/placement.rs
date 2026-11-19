//! Where Navigation builds the map products, as Capture's uploader reads it.
//!
//! `navigation.map_placement` is Navigation's setting; Capture reads its
//! effect from `plugin.navigation.map_status` (`placement`), never the other
//! extension's config. Without a fresh map status (no Navigation, or it
//! stopped) the placement is the default, which uploads after the flight.

use std::sync::Arc;
use std::time::{Duration, Instant};

use ados_mapping_protocol::atlas::ComputePlacement;
use ados_mapping_transport::host::{EventCallback, Host};
use parking_lot::Mutex;
use serde::Deserialize;

/// The shared topic the placement rides.
pub const MAP_STATUS_TOPIC: &str = "plugin.navigation.map_status";
/// A map status older than this no longer says where products are built.
pub const FRESH: Duration = Duration::from_secs(5);
/// How often a refused subscription is retried.
const RETRY: Duration = Duration::from_secs(30);

#[derive(Deserialize)]
struct PlacementOnly {
    placement: ComputePlacement,
}

/// The newest placement and when it arrived.
#[derive(Clone, Default)]
pub struct PlacementWatch(Arc<Mutex<Option<(Instant, ComputePlacement)>>>);

impl PlacementWatch {
    pub fn set(&self, at: Instant, placement: ComputePlacement) {
        *self.0.lock() = Some((at, placement));
    }

    /// The placement a map status no older than [`FRESH`] reported at `now`.
    pub fn current(&self, now: Instant) -> Option<ComputePlacement> {
        self.0
            .lock()
            .filter(|(at, _)| now.saturating_duration_since(*at) < FRESH)
            .map(|(_, p)| p)
    }

    /// Fold one map-status payload; one without a readable placement is
    /// ignored.
    pub fn on_payload(&self, payload: &rmpv::Value) {
        // Through JSON: rmpv's own deserializer takes no enum from a string.
        let placement = rmpv::ext::from_value::<serde_json::Value>(payload.clone())
            .ok()
            .and_then(|v| serde_json::from_value::<PlacementOnly>(v).ok());
        if let Some(p) = placement {
            self.set(Instant::now(), p.placement);
        }
    }
}

/// Subscribe `watch` to Navigation's map status, retrying every
/// [`RETRY`] until the host takes it (Navigation may be installed later) or
/// `shutdown`.
pub async fn subscribe(
    host: Arc<dyn Host>,
    watch: PlacementWatch,
    mut shutdown: tokio::sync::watch::Receiver<bool>,
) {
    loop {
        let w = watch.clone();
        let cb: EventCallback = Arc::new(move |payload| w.on_payload(&payload));
        match host.event_subscribe(MAP_STATUS_TOPIC, cb).await {
            Ok(()) => return,
            Err(e) => tracing::debug!(error = %e, "map status subscription refused; retrying"),
        }
        if crate::sleep_or_shutdown(&mut shutdown, RETRY).await {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_map_status_sets_the_placement_until_it_goes_stale() {
        let w = PlacementWatch::default();
        assert_eq!(w.current(Instant::now()), None);
        let status = serde_json::json!({"placement": "onboard", "voxel_m": 0.05});
        w.on_payload(&rmpv::ext::to_value(&status).unwrap());
        let now = Instant::now();
        assert_eq!(w.current(now), Some(ComputePlacement::Onboard));
        assert_eq!(w.current(now + FRESH), None);
        w.on_payload(&rmpv::ext::to_value(serde_json::json!({"placement": 7})).unwrap());
        assert_eq!(w.current(now), Some(ComputePlacement::Onboard));
    }
}
