//! Each free-flight vehicle's own camera views, read from its package's
//! `views/cameras.json` (or derived from its span) with cockpit looks taken
//! from the eye [`pilot_eye_mount_for`] seats.

use std::sync::OnceLock;

use sim_core::VirtualCameraSpec;
use sim_pack::{
    CameraViewSet, LoadedSyspack, PackError, builtin_c172_package, builtin_p38g_package,
};
use tracing::warn;

use super::pilot_eye_mount_for;
use crate::vehicles::FreeFlightVehicle;

/// A light single's chase distance behind and above its origin, metres: the
/// chase for a flight with no free-flight vehicle of its own.
pub const DEFAULT_CHASE_BACK_UP_M: (f64, f64) = (22.0, 6.5);
/// Span of the X500 quadcopter across its rotor tips, metres.
const X500_SPAN_M: f64 = 0.70;

/// One vehicle's chase distance and its cyclable views.
#[derive(Debug)]
pub struct VehicleCameraSuite {
    /// The chase camera's distance behind and above the origin, metres.
    pub chase_back_up_m: (f64, f64),
    /// The vehicle's views in cycling order.
    pub cameras: Vec<VirtualCameraSpec>,
}

static C172_SUITE: OnceLock<VehicleCameraSuite> = OnceLock::new();
static P38_SUITE: OnceLock<VehicleCameraSuite> = OnceLock::new();
static X500_SUITE: OnceLock<VehicleCameraSuite> = OnceLock::new();

/// `vehicle`'s camera suite, built once from its embedded package.
pub fn vehicle_camera_suite(vehicle: FreeFlightVehicle) -> &'static VehicleCameraSuite {
    match vehicle {
        FreeFlightVehicle::JsbsimC172
        | FreeFlightVehicle::AviateFixedWingDirect
        | FreeFlightVehicle::Kinematic => {
            C172_SUITE.get_or_init(|| suite(vehicle, package_views("c172", builtin_c172_package())))
        }
        FreeFlightVehicle::JsbsimP38g => {
            P38_SUITE.get_or_init(|| suite(vehicle, package_views("p38g", builtin_p38g_package())))
        }
        FreeFlightVehicle::AviateX500Direct | FreeFlightVehicle::X500Mission => {
            X500_SUITE.get_or_init(|| suite(vehicle, CameraViewSet::derived(X500_SPAN_M)))
        }
    }
}

fn package_views(name: &str, package: Result<LoadedSyspack, PackError>) -> CameraViewSet {
    match package {
        Ok(package) => package.camera_view_set(),
        Err(error) => {
            warn!(package = name, %error, "package unreadable; deriving default camera views");
            CameraViewSet::derived(f64::NAN)
        }
    }
}

fn suite(vehicle: FreeFlightVehicle, views: CameraViewSet) -> VehicleCameraSuite {
    VehicleCameraSuite {
        chase_back_up_m: views.chase_offsets_m().unwrap_or(DEFAULT_CHASE_BACK_UP_M),
        cameras: views.virtual_cameras(pilot_eye_mount_for(vehicle).offset_body_m),
    }
}
