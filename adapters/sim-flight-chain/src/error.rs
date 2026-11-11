use sim_atmosphere::AtmosphereError;
use sim_aviate::AviateBridgeError;
use sim_core::BlockError;
use sim_devices::DeviceError;
use sim_pack::PackError;

/// Why the flight chain could not be built or stepped.
#[derive(Debug, thiserror::Error)]
pub enum ChainError {
    /// The airframe's flight model refused its configuration or a step.
    #[error("the airframe's flight model failed")]
    Plant(#[source] Box<BlockError>),
    /// The flight controller's kernel could not be assembled.
    #[error("the flight controller kernel could not be built")]
    KernelBuild,
    /// The flight controller refused its board configuration or a step.
    #[error("flight controller failed at {at_s:.3} s")]
    FlightController {
        /// Simulation time of the failure.
        at_s: f64,
        /// The bridge's reason.
        #[source]
        source: AviateBridgeError,
    },
    /// A device of the sensor suite refused its configuration or a sample.
    #[error("a sensor device failed")]
    Devices(#[source] DeviceError),
    /// The air field has no state where the vehicle is.
    #[error("no air state at the vehicle")]
    Air(#[source] AtmosphereError),
    /// The vehicle package could not supply its flight model.
    #[error("vehicle package failed")]
    Package(#[source] Box<PackError>),
    /// The airframe could not be trimmed for its starting flight condition.
    #[error("no trim at the start of flight")]
    Trim(#[source] Box<BlockError>),
    /// Pilotage's host core refused its configuration, the plan or a tick.
    #[error("the Pilotage host core failed")]
    Host(#[source] Box<pilotage_host_core::HostCoreError>),
    /// The envelope stated for the vehicle is not a valid one.
    #[error("the vehicle's envelope is invalid: {detail}")]
    Envelope {
        /// Pilotage's reason.
        detail: String,
    },
}
