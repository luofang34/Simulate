//! Bridge between Simulate and Pilotage's remote-operations platform.
//!
//! [`SimulateVehicleAdapter`] flies a [`sim_core::KinematicModel`] or
//! [`sim_core::JsbsimFdmBlock`] behind Pilotage's
//! [`pilotage_adapter_api::VehicleAdapter`]. [`package_camera_calibration`]
//! turns a package camera into a hash-verifiable
//! [`pilotage_camera_calibration::CameraCalibration`]. [`aircraft_profile`],
//! [`package_loading`], and [`apply_loading`] carry a vehicle package to and
//! from a [`pilotage_aircraft::AircraftProfile`].
//!
//! The adapter's avionics group is a noise model standing in for a flight
//! controller's estimator, so it is never simulator truth, which goes out
//! separately as [`pilotage_adapter_api::SimTruthSample`].

mod adapter;
mod calibration;
mod error;
mod profile;
pub mod video_bridge;

pub use adapter::{
    MOTION_SCOPE, PackageStart, SimulateAdapterConfig, SimulateVehicleAdapter, vehicle_class,
};
pub use calibration::{CalibratedCamera, CalibrationLifecycle, package_camera_calibration};
pub use error::PilotageBridgeError;
pub use profile::{
    AircraftSpec, EnergySpec, StationSite, TankSite, aircraft_profile, apply_loading,
    package_loading,
};
