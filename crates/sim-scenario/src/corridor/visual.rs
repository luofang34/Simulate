use glam::{DMat3, DVec3};
use sim_core::{CameraFrameCapture, VisualEstimate, VisualFrame};
use sim_devices::WhiteNoise;

/// One-sigma position error of a warm track, per axis, metres.
const WARM_SIGMA_M: f64 = 2.0;
/// One-sigma position error of a corridor anchor, per axis, metres.
const ANCHOR_SIGMA_M: f64 = 8.0;

/// Stands in for `Navigate`'s visual stack: truth plus seeded Gaussian error,
/// with a covariance that states that error honestly. The harness around it
/// is the real one, so this exercises the latency and `MapCell` bookkeeping,
/// not visual positioning.
#[derive(Debug, Clone)]
pub(super) struct SyntheticVisualEstimator {
    noise: WhiteNoise,
}

impl SyntheticVisualEstimator {
    pub(super) const fn new(seed: u64) -> Self {
        Self {
            noise: WhiteNoise::from_seed(seed),
        }
    }

    fn estimate(&mut self, truth_m: DVec3, sigma_m: f64) -> VisualEstimate {
        VisualEstimate {
            position_enu_m: truth_m + self.noise.next_vec3(DVec3::splat(sigma_m)),
            covariance_enu_m2: DMat3::from_diagonal(DVec3::splat(sigma_m * sigma_m)),
        }
    }

    pub(super) fn frame(&mut self, capture: CameraFrameCapture) -> VisualFrame {
        let truth = DVec3::from_array(capture.eye_enu);
        VisualFrame {
            capture,
            warm_track: Some(self.estimate(truth, WARM_SIGMA_M)),
            corridor_anchor: Some(self.estimate(truth, ANCHOR_SIGMA_M)),
        }
    }
}
