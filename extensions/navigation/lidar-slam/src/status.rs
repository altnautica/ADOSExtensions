//! What the HTTP routes read from the running lane: the status block of
//! `GET /navigation/status`, the sweep history of `GET /navigation/sweep`
//! and the way to ask the lane for the whole cloud.

use ados_mapping_protocol::capture::{LidarStatus, MapStatus, NavPoseSource, VioHealth};
use parking_lot::Mutex;
use serde::Serialize;
use tokio::sync::mpsc;

use crate::lane::WholeSweep;
use crate::sweep::SweepHistory;

/// The Agent page's live block.
#[derive(Debug, Clone, Default, Serialize)]
pub struct StatusView {
    /// Whether a lane runs, and why not when it does not.
    pub running: bool,
    pub idle_reason: Option<String>,
    /// The pose provider running; null when none runs.
    pub provider: Option<NavPoseSource>,
    pub health: Option<VioHealth>,
    /// Age of the newest estimate, ms.
    pub pose_age_ms: Option<u64>,
    pub lidar: Option<LidarStatus>,
    pub map: Option<MapStatus>,
}

/// State shared between the lane and the routes.
#[derive(Default)]
pub struct Shared {
    pub view: Mutex<StatusView>,
    pub sweep: Mutex<SweepHistory>,
    /// The running lane's whole-cloud request queue; `None` between runs.
    pub whole: Mutex<Option<mpsc::Sender<WholeSweep>>>,
}

impl Shared {
    /// A run started: the history and status of the last one no longer hold.
    pub fn reset_run(&self) {
        *self.sweep.lock() = SweepHistory::default();
        *self.view.lock() = StatusView {
            running: true,
            ..StatusView::default()
        };
    }

    /// No run: say why.
    pub fn idle(&self, reason: &str) {
        let mut v = self.view.lock();
        *v = StatusView {
            running: false,
            idle_reason: Some(reason.to_string()),
            ..StatusView::default()
        };
    }
}
