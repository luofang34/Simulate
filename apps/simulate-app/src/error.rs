use sim_atmosphere::AtmosphereError;
use sim_aviate::AviateBridgeError;
use sim_indicate::{FeedError, PanelError};
use sim_maplibre::MapAdapterError;
use sim_render::RenderError;
use sim_scenario::ScenarioError;

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
    /// Tier-0 Aviate runner step or initialization failed.
    #[error("Aviate direct bridge failed")]
    Aviate(#[from] AviateBridgeError),
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
}
