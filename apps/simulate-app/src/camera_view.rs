use glam::{DMat3, DQuat, DVec3};
use sim_core::{CameraPoseEcef, VehicleState};
use sim_render::{Camera, CameraPose};

const COCKPIT_FOV_Y_DEG: f64 = 60.0;
const COCKPIT_NEAR_M: f64 = 0.05;

/// The eye at the vehicle reference point, looking along the body nose.
///
/// Body axes are forward/right/down; camera axes are right/up/back, so the
/// fixed rotation maps camera x→body y, camera y→body −z, camera z→body −x.
pub fn cockpit_camera(state: &VehicleState) -> Camera {
    let camera_to_body = DQuat::from_mat3(&DMat3::from_cols(DVec3::Y, -DVec3::Z, -DVec3::X));
    Camera {
        pose: CameraPose {
            position_ecef: state.position_ecef,
            orientation: (state.attitude * camera_to_body).normalize(),
        },
        fov_y_rad: COCKPIT_FOV_Y_DEG.to_radians(),
        near_m: COCKPIT_NEAR_M,
    }
}

/// The eye a scenario gives a person aboard.
pub fn scenario_camera(pose: CameraPoseEcef) -> Camera {
    Camera {
        pose: CameraPose {
            position_ecef: pose.position_ecef,
            orientation: pose.orientation,
        },
        fov_y_rad: COCKPIT_FOV_Y_DEG.to_radians(),
        near_m: COCKPIT_NEAR_M,
    }
}
