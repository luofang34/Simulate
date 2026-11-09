//! Inertial dead reckoning: the simulator's strapdown IMU and initial
//! alignment on one side, Navigate's 22-state inertial filter on the other.
//!
//! [`SimulatedImu`] turns successive truth states into the increments an IMU
//! on the body axes would report, through a seeded
//! [`sim_devices::ImuErrorModel`]. [`InitialAlignment`] is the one truth touch
//! the filter gets, as a sensor with declared errors. [`InertialNavigator`]
//! never sees truth; [`crate::compare_with_truth`] scores its solutions outside
//! it.
//!
//! Conventions the link relies on: sim body axes are forward, right, down,
//! and `VehicleState::attitude` rotates body into ECEF, a frame turning with
//! the Earth. The synthesis differences successive attitudes and composes the
//! Earth's own turn over the step, so its rotation is relative to inertial
//! space as Navigate expects, whatever dynamics source moved the vehicle.
//! Navigate's body axes are also forward, right, down, with attitude given
//! body-to-NED at alignment.

mod alignment;
mod navigator;
mod synthesis;

#[cfg(test)]
mod tests;

pub use alignment::{AlignmentFix, InitialAlignment};
pub use navigator::{InertialNavigator, inertial_config};
pub use synthesis::{SimulatedImu, TruthGravity, ideal_increment};
