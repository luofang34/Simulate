//! Deterministic [`ScenarioProfile`]s: four corridor flights over synthetic
//! Delaware scenery, Apollo 11's lunar landing and Command Module entry, and
//! closed-loop C172 airport traffic patterns at `47N` (uncontrolled CTAF) and
//! `KEWR` (Class B towered) over OpenStreetMap aerodrome diagrams.
//!
//! A [`ScenarioRun`] advances on the master clock with no window or GPU, so a
//! test and `simulate-app` fly the same bits. [`ScenarioWorld`] builds the terrain
//! a scenario flies over, and, with the default `render` feature,
//! `render_capture` turns a camera capture into an offscreen frame at its
//! exact pose and intrinsics. Without that feature the crate is pure CPU.

#[cfg(feature = "render")]
mod bridge;
mod cameras;
mod corridor;
mod episode;
mod error;
mod lunar;
mod pattern;
mod profile;
mod reentry;
mod registry;
mod run;
mod supervisor;
mod world;

#[cfg(feature = "render")]
pub use bridge::{CaptureExposure, aperture_frame, render_capture};
pub use cameras::scenario_virtual_cameras;
pub use corridor::{
    CorridorReport, CorridorRun, CorridorVisualEstimator, FlightDynamics, ImageDrivenConfig,
    ImageNavigationReport, RenderedSensorFrame, SensorObservation, VisualTiers,
};
pub use episode::{
    BatchScenarioRunner, EpisodeConfig, EpisodeEnv, EpisodeObservation, EpisodeSnapshot,
    EpisodeStep,
};
pub use error::ScenarioError;
pub use lunar::{LunarLandingConfig, LunarLandingReport, LunarLandingRun};
pub use pattern::{
    AirportPatternRun, C172_PILOT_EYE_BODY_M, PatternLeg, PatternReport, TrafficDirection,
    pattern_virtual_cameras,
};
pub use profile::ScenarioProfile;
pub use reentry::{CapsuleReentryReport, CapsuleReentryRun};
pub use registry::{ScenarioCategory, ScenarioDescriptor, ScenarioRegistry};
pub use run::{ScenarioRun, ScenarioSnapshot};
pub use supervisor::{
    OperatorBeliefView, OperatorViewportMode, RadioBeliefPolicy, RunwaySpawnError, SimSupervisor,
    SimTimeScale,
};
pub use world::ScenarioWorld;
#[cfg(feature = "render")]
pub use world::WorldScene;
