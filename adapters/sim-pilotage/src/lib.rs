//! Bridge between Simulate and Pilotage's remote-operations platform: the
//! parts of a vehicle package Pilotage's own domains describe.
//!
//! [`package_camera_calibration`] turns a package camera into a
//! hash-verifiable [`pilotage_camera_calibration::CameraCalibration`].
//! [`aircraft_profile`], [`package_loading`], and [`apply_loading`] carry a
//! vehicle package to and from a [`pilotage_aircraft::AircraftProfile`].
//! [`video_bridge`] serves rendered camera frames to Pilotage's video
//! sidecar.
//!
//! Flying a vehicle for Pilotage is not here: a flight controller and its
//! estimator stand between Pilotage and an airframe, and `sim-flight-chain`
//! hosts the real ones.

mod calibration;
mod error;
mod profile;
pub mod video_bridge;

pub use calibration::{CalibratedCamera, CalibrationLifecycle, package_camera_calibration};
pub use error::PilotageBridgeError;
pub use profile::{
    AircraftSpec, EnergySpec, StationSite, TankSite, aircraft_profile, apply_loading,
    package_loading,
};
