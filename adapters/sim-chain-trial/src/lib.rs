//! A navigation trial on the vehicle's own control chain, stepped by its
//! host.
//!
//! Pilotage's host core flies a Cessna 172 through the Aviate flight
//! controller, steering on the navigation the spec names, while each of
//! Navigate's sensor sets navigates on the very devices the flight
//! controller reads. Simulate supplies the world, the aeroplane and the
//! devices. The host advances [`ChainTrial`] one flight-controller cycle
//! at a time or up to a route time, switches devices in and out of
//! service, and renders the terrain camera's frames when asked; the trial
//! reports the navigation trial's progress records, Pilotage's mission in
//! its own words, each set's terrain-camera fixes, and at a rate the host
//! asks for the flight controller's account beside the truth.

mod error;
mod first_link;
mod nav_log;
mod output;
mod spec;
mod trial;

pub use error::ChainTrialError;
pub use first_link::first_link;
pub use output::{FrameRequest, MissionUpdate, Outputs, Retained, StepOutcome};
pub use spec::{
    CalibrationName, ChainTrialSpec, DeviceOutage, GnssSpoofSpec, HoldSpec, HoldTurn, SpoofSpan,
    StartSpec, Steering,
};
pub use trial::{C172Chain, ChainTrial, FLIGHT_CONTROLLER_SET, Steered, TruthSnapshot};
