//! Bridge from Simulate's master clock and visual harness into Navigate's real
//! visual-fix admission and navigation filter.
//!
//! [`MasterClockBridge`] maps [`sim_core::SimInstant`] onto Navigate's monotonic
//! stamps. [`VisualFixAdapter`] turns harness fixes into
//! [`navigate_visual::Estimate`]s, labels each anchor in a real
//! [`navigate_visual::session::VisualSession`], takes the camera lever arm off,
//! and admits only first-in-cell anchors through
//! [`navigate_visual::fusion_bridge::VisualFixSource`]. [`NavigationLoop`] replays those
//! fixes and [`KinematicAiding`] velocities in acquisition order into
//! [`navigate_fusion::NavigationFilter`], and [`compare_with_truth`] scores each
//! solution against simulation truth.
//!
//! The corridor scenarios' default estimates are truth plus declared noise,
//! so those runs prove admission, timing, and covariance discipline, not image
//! matching. [`ImageCorridorEstimator`] is the image-driven alternative: it
//! positions a nadir camera from rendered pixels, the attitude reference and
//! the altimeter alone, by ZNCC registration against reference orthoimagery
//! and corner tracking between frames, for
//! `sim_scenario::CorridorRun::image_driven`. Navigate's filter publishes no
//! protection levels yet; nothing here invents them.
//!
//! [`SimulatedImu`], [`InitialAlignment`] and [`InertialNavigator`] carry
//! simulated strapdown increments into Navigate's 22-state
//! [`navigate_fusion::InertialNavigationFilter`] for unaided dead reckoning.
//!
//! [`CelestialClock`] turns a sky time mode and observer position into a
//! [`CelestialFrame`] of Sun, Moon, eclipse, Earth-shadow, and star geometry
//! from Navigate's analytic ephemeris. [`camera_stars`] gives a
//! [`sim_render::StarCamera`] the apparent catalogue stars of one frame on
//! the scenario clock.
//!
//! [`CameraFrameLink`] lends rendered exposures to Navigate as
//! `navigate_camera::ImageFrame`s, and [`navigate_mount`] states a simulator
//! camera mount in Navigate's axes. [`AblationEvaluation`] scores
//! [`Estimate`]s of each means, alone and fused, against [`Truth`], and
//! [`ComputeMeter`] reports their host cost beside the scores.

mod aiding;
mod celestial;
mod clock;
mod error;
mod estimate;
mod evaluation;
mod filter_loop;
mod frame_link;
mod image_nav;
mod inertial;
mod truth;
mod visual_fix;

pub use aiding::KinematicAiding;
pub use celestial::{
    CelestialClock, CelestialFrame, EarthShadowSample, FAINTEST_STAR_V_MAGNITUDE, MoonDiscSample,
    ReferenceBody, SkyTimeMode, StarSkySample, camera_stars, star_camera_epoch,
};
pub use clock::{MASTER_CLOCK_DOMAIN, MasterClockBridge};
pub use error::NavigateBridgeError;
pub use estimate::CorridorFrame;
pub use evaluation::{
    AblationEvaluation, AblationSet, AblationSummary, CameraTruth, ComputeMeter, ComputeSample,
    ComputeStats, Estimate, EvaluationSummary, ImageTruth, MeansEvaluation, NavMeans,
    NavigationOrigin, NavigationOutput, Percentiles, PositionScore, SensorTruth, SensorTruthRecord,
    SimTruth, StarFieldTruth, StarTruth, ThermalTruth, Truth, score_position,
};
pub use filter_loop::{NavigationLoop, airborne_fusion_config};
pub use frame_link::{
    CameraFrameLink, FramePixelFormat, OwnedCameraFrame, imager_model, navigate_mount,
};
pub use image_nav::{ImageCorridorEstimator, ImageEstimatorConfig, ImageEstimatorStats};
pub use inertial::{
    AlignmentFix, InertialNavigator, InitialAlignment, SimulatedImu, TruthGravity, ideal_increment,
    inertial_config,
};
pub use truth::{
    AttitudeComparison, TruthComparison, compare_attitude_with_truth, compare_with_truth,
    solution_position_ecef,
};
pub use visual_fix::{FixDisposition, VisualFixAdapter, VisualFixConfig};
