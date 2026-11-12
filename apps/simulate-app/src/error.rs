use indicate_instrument_scene::SceneError;
use sim_atmosphere::AtmosphereError;
use sim_aviate::AviateBridgeError;
use sim_core::SessionError;
use sim_indicate::{FeedError, PanelError};
use sim_maplibre::MapAdapterError;
use sim_orbital::{ConstantsError, LagrangeError};
use sim_render::RenderError;
use sim_scenario::ScenarioError;
use sim_spacetime::{SynodicError, TimeError};
use sim_universe::UniverseError;

use crate::flight::SolarSystemError;

/// Typed errors raised while initialising, stepping, or compositing the cockpit app.
#[derive(Debug, thiserror::Error)]
pub enum AppSimError {
    /// A flight model, vehicle package, or scripted scenario failed.
    #[error("flight or scenario simulation failed")]
    Scenario(#[from] ScenarioError),
    /// Tier-0 Aviate flight-controller kernel failed to build.
    #[error("Aviate direct flight-controller kernel could not be built: {detail}")]
    AviateKernel {
        /// Diagnostic description.
        detail: &'static str,
    },
    /// The navigation trial on the real control chain could not be built or stepped.
    #[error("the chain trial failed")]
    ChainTrial(#[from] sim_chain_trial::ChainTrialError),
    /// Tier-0 Aviate runner step or initialization failed.
    #[error("Aviate direct bridge failed")]
    Aviate(#[from] AviateBridgeError),
    /// The flight's start time is outside the time-scale record.
    #[error("the flight clock could not name a time")]
    Time(#[from] TimeError),
    /// Standard atmosphere sampling failed.
    #[error("atmosphere sampling failed")]
    Atmosphere(#[from] AtmosphereError),
    /// Indicate state feed rejected a flight sample.
    #[error("avionics state feed failed")]
    Feed(#[from] FeedError),
    /// Indicate software rasterizer or volumetric emitter failed.
    #[error("avionics panel rendering failed")]
    Panel(#[from] PanelError),
    /// Shared-GPU MapLibre truth or belief adapter failed.
    #[error("MapLibre scene or avionics underlay failed")]
    Map(#[from] MapAdapterError),
    /// Window or offscreen GPU renderer failed.
    #[error("GPU renderer failed")]
    Render(#[from] RenderError),
    /// The `Esc` menu scene did not fit its encoder.
    #[error("menu scene encoding failed: {cause:?}")]
    MenuScene {
        /// The encoder's refusal.
        cause: SceneError,
    },
    /// The free-flight simulation session refused an input, a sensor, or a step.
    #[error("free-flight simulation session failed")]
    Session(#[source] Box<SessionError>),
    /// The universe the flight's vessels share refused a step or a rewind.
    #[error("the flight's universe failed")]
    Universe(#[source] Box<UniverseError>),
    /// The solar system the universe propagates in could not be set up.
    #[error("the solar system could not be set up")]
    SolarSystem(#[from] SolarSystemError),
    /// Synodic frame or Lagrange-point targeting failed.
    #[error("Lagrange-point targeting failed")]
    Lagrange(#[from] LagrangeError),
    /// Rotating synodic frame construction failed.
    #[error("synodic frame construction failed")]
    Synodic(#[from] SynodicError),
    /// Gravitational constant lookup failed.
    #[error("gravitational constant lookup failed")]
    Constants(#[from] ConstantsError),
    /// Manual control cannot take over the active scenario.
    #[error("manual control is unavailable in {scenario}: {detail}")]
    ManualControlUnavailable {
        /// Scenario title.
        scenario: &'static str,
        /// Why the takeover was refused.
        detail: &'static str,
    },
    /// The simulation has no checkpoint to rewind to.
    #[error("no rewind checkpoint covers the requested {requested_s} s")]
    NoCheckpoint {
        /// Requested rewind span, seconds.
        requested_s: f64,
    },
    /// A requested runway designator does not exist on the target airport record.
    #[error("runway {designator} does not exist at {airport}")]
    SpawnRunwayMissing {
        /// Primary facility identifier.
        airport: String,
        /// Requested runway designator.
        designator: String,
    },
    /// Writing or reading a screenshot image on desktop failed.
    #[cfg(not(target_arch = "wasm32"))]
    #[error("screenshot export failed: {detail}")]
    Screenshot {
        /// Diagnostic description.
        detail: String,
    },
}

impl From<SessionError> for AppSimError {
    fn from(error: SessionError) -> Self {
        Self::Session(Box::new(error))
    }
}

impl From<SceneError> for AppSimError {
    fn from(cause: SceneError) -> Self {
        Self::MenuScene { cause }
    }
}
