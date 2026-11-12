use mbd_core::MbdError;
use sim_atmosphere::AtmosphereError;
use sim_core::{BlockError, VisualHarnessError};
use sim_devices::DeviceError;
use sim_pack::PackError;
#[cfg(feature = "render")]
use sim_render::RenderError;
use sim_scenery::SceneryError;
use sim_spacetime::TimeError;

/// Why a scenario could not be set up or advanced. The larger causes are boxed
/// so that stepping, which returns this every period, passes a small value.
#[derive(Debug, thiserror::Error)]
pub enum ScenarioError {
    /// A simulated block refused to start or step.
    #[error("a scenario block failed")]
    Block(#[source] Box<BlockError>),
    /// The vehicle's component tree could not be evaluated.
    #[error("the scenario vehicle model failed")]
    Mbd(#[from] MbdError),
    /// A vehicle package failed verification.
    #[error("the scenario vehicle package failed to load")]
    Package(#[source] Box<PackError>),
    /// The visual-navigation harness refused a frame.
    #[error("the visual harness refused a frame")]
    Visual(#[source] Box<VisualHarnessError>),
    /// The scenario's start time is outside the time-scale record.
    #[error("the scenario clock failed")]
    Time(#[from] TimeError),
    /// A simulated air-data or onboard sensor device rejected its configuration.
    #[error("the scenario sensor device failed")]
    Device(#[from] DeviceError),
    /// The atmosphere model could not be sampled at the vehicle position.
    #[error("the scenario atmosphere failed")]
    Atmosphere(#[from] AtmosphereError),
    /// The scenery has no data or is malformed.
    #[error("the scenario scenery failed")]
    Scenery(#[from] SceneryError),
    /// Terrain or a camera frame could not be rendered.
    #[cfg(feature = "render")]
    #[error("the scenario could not render")]
    Render(#[source] Box<RenderError>),
    /// A rewind snapshot was taken from a different scenario or flight
    /// dynamics source than the run it was offered to.
    #[error("the snapshot does not fit this scenario run: {detail}")]
    SnapshotMismatch {
        /// What did not match.
        detail: &'static str,
    },
    /// The request does not apply to this scenario or its state.
    #[error("unsupported: {detail}")]
    Unsupported {
        /// What was asked and why it cannot be done.
        detail: &'static str,
    },
    /// The scenery has no ground under a point the scenario depends on.
    #[error("no terrain at {lat_deg:.5}°, {lon_deg:.5}°")]
    NoTerrain {
        /// Latitude, degrees.
        lat_deg: f64,
        /// Longitude, degrees.
        lon_deg: f64,
    },
}

impl From<BlockError> for ScenarioError {
    fn from(error: BlockError) -> Self {
        Self::Block(Box::new(error))
    }
}

impl From<PackError> for ScenarioError {
    fn from(error: PackError) -> Self {
        Self::Package(Box::new(error))
    }
}

impl From<VisualHarnessError> for ScenarioError {
    fn from(error: VisualHarnessError) -> Self {
        Self::Visual(Box::new(error))
    }
}

#[cfg(feature = "render")]
impl From<RenderError> for ScenarioError {
    fn from(error: RenderError) -> Self {
        Self::Render(Box::new(error))
    }
}
