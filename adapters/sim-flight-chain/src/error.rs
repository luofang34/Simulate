use mbd_core::MbdError;
use pilotage_mission::MissionBuildError;
use sim_aviate::AviateBridgeError;
use sim_core::BlockError;

/// Why the flight chain could not be built or stepped.
#[derive(Debug, thiserror::Error)]
pub enum ChainError {
    /// The rigid body refused its configuration or a step.
    #[error("X500 rigid body failed")]
    Plant(#[source] Box<BlockError>),
    /// The airframe's component tree could not be evaluated.
    #[error("X500 component tree failed")]
    Tree(#[source] MbdError),
    /// The pad is not a position the gravity model accepts.
    #[error("no normal gravity at latitude {lat_deg} deg, longitude {lon_deg} deg")]
    PadGravity {
        /// Pad latitude.
        lat_deg: f64,
        /// Pad longitude.
        lon_deg: f64,
    },
    /// The flight controller's kernel could not be assembled.
    #[error("the X500 flight controller kernel could not be built")]
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
    /// The flight controller's rotor command does not fit the airframe.
    #[error("rotor command does not fit the X500")]
    Rotors(#[source] AviateBridgeError),
    /// Pilotage's mission engine refused the route or its configuration.
    #[error("mission could not be built")]
    Mission(#[source] Box<MissionBuildError>),
    /// The vehicle stopped stepping earlier and its reason was already taken.
    #[error("the vehicle stopped stepping at an earlier fault")]
    Stopped,
    /// The rate does not divide the master clock into whole ticks.
    #[error("{rate_hz} Hz is not a whole number of master-clock ticks")]
    Rate {
        /// The rate asked for.
        rate_hz: u64,
    },
}
