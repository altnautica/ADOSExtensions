//! [`ReplayPose`]: a fixed estimate list, for tests and simulation runs that
//! carry a ground-truth trajectory.

use world_engine_protocol::atlas::PoseSource;

use super::{PoseEstimate, PoseProvider, PoseTrack};

pub struct ReplayPose {
    track: PoseTrack,
}

impl ReplayPose {
    /// Every estimate is available at once; `source` is forced to `Replay`.
    pub fn new(estimates: Vec<PoseEstimate>) -> Self {
        let track = PoseTrack::new();
        for mut e in estimates {
            e.source = PoseSource::Replay;
            track.push(e);
        }
        Self { track }
    }
}

impl PoseProvider for ReplayPose {
    fn source(&self) -> PoseSource {
        PoseSource::Replay
    }

    fn track(&self) -> &PoseTrack {
        &self.track
    }
}
