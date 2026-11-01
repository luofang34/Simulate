//! Deterministic [`ScenarioProfile`]s: four corridor flights over synthetic
//! Delaware scenery, Apollo 11's lunar landing and Command Module entry, and
//! closed-loop C172 airport traffic patterns at `47N` (uncontrolled CTAF) and
//! `KEWR` (Class B towered) over OpenStreetMap aerodrome diagrams.
//!
//! A [`ScenarioRun`] advances on the master clock with no window or GPU, so a
//! test and `simulate-app` fly the same bits. [`ScenarioWorld`] builds the terrain
//! a scenario flies over, and [`render_capture`] turns a camera capture into
//! an offscreen frame at its exact pose and intrinsics.

mod bridge;
mod corridor;
mod error;
mod lunar;
mod pattern;
mod profile;
mod reentry;
mod run;
mod supervisor;
mod world;

pub use bridge::{CaptureExposure, aperture_frame, render_capture};
pub use corridor::{CorridorReport, CorridorRun, FlightDynamics};
pub use error::ScenarioError;
pub use lunar::{LunarLandingConfig, LunarLandingReport, LunarLandingRun};
pub use pattern::{
    AirportPatternRun, PatternLeg, PatternReport, TrafficDirection, pattern_virtual_cameras,
};
pub use profile::ScenarioProfile;
pub use reentry::{CapsuleReentryReport, CapsuleReentryRun};
pub use run::ScenarioRun;
pub use supervisor::{
    OperatorBeliefView, OperatorViewportMode, RadioBeliefPolicy, RunwaySpawnError, SimSupervisor,
    SimTimeScale,
};
pub use world::{ScenarioWorld, WorldScene};
