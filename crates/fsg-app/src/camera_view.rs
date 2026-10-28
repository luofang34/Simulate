use fsg_render::{Camera, CameraPose};
use fsg_sim::VehicleState;
use glam::{DMat3, DQuat, DVec3};

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
