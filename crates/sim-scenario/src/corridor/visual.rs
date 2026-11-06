use glam::{DMat3, DQuat, DVec3};
use sim_core::{CameraFrameCapture, SimInstant, VisualEstimate, VisualFrame};
use sim_devices::WhiteNoise;
use sim_geo::SensorIntrinsics;
use sim_scenery::LumaImage;

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

/// A frame the simulated camera rendered: pixels and when, nothing else.
#[derive(Debug, Clone, PartialEq)]
pub struct RenderedSensorFrame {
    /// The exposure instant.
    pub timestamp: SimInstant,
    /// The camera's optics.
    pub intrinsics: SensorIntrinsics,
    /// The rendered luma image.
    pub image: LumaImage,
}

/// Everything an image-driven estimator is given for one frame: the pixels
/// and the onboard attitude and altimeter estimates. It carries no position,
/// so an estimator behind [`CorridorVisualEstimator`] cannot see the truth.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorObservation {
    /// The rendered frame.
    pub frame: RenderedSensorFrame,
    /// The attitude reference's estimate of the camera's orientation:
    /// rotates camera components (`+X` right, `+Y` up, looking along `-Z`)
    /// into WGS84 body-fixed ones.
    pub camera_to_ecef_estimate: DQuat,
    /// The radar altimeter's height above the ground, metres.
    pub agl_estimate_m: f64,
}

/// What each tier of a visual estimator made of one frame; `None` where a
/// tier found nothing or did not run.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VisualTiers {
    /// Warm-track result.
    pub warm_track: Option<VisualEstimate>,
    /// Corridor map anchor result.
    pub corridor_anchor: Option<VisualEstimate>,
}

/// A visual position estimator that works from rendered pixels, as
/// [`crate::CorridorRun::image_driven`] drives it. Estimates are in the
/// corridor's east-north-up anchor frame, the frame the harness scores in.
pub trait CorridorVisualEstimator: std::fmt::Debug + Send {
    /// Processes one frame.
    fn estimate(&mut self, observation: &SensorObservation) -> VisualTiers;

    /// A copy of this estimator in its current state, so a run's snapshot
    /// can rewind it.
    fn boxed_clone(&self) -> Box<dyn CorridorVisualEstimator>;
}
