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
//! its own words, and each set's terrain-camera fixes.

mod error;
mod nav_log;
mod output;
mod spec;
mod trial;

pub use error::ChainTrialError;
pub use output::{FrameRequest, MissionUpdate, Outputs, StepOutcome};
pub use spec::{CalibrationName, ChainTrialSpec, DeviceOutage, StartSpec, Steering};
pub use trial::{ChainTrial, FLIGHT_CONTROLLER_SET};
