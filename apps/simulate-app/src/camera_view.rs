use glam::{DMat3, DQuat, DVec3};
use sim_core::{CameraMount, CameraPoseEcef, VehicleState, VirtualCameraSpec};
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_render::{Camera, CameraPose};
use sim_scenario::OperatorViewportMode;

/// Vertical field of view of the primary OTW and chase cameras, degrees.
pub const COCKPIT_FOV_Y_DEG: f64 = 60.0;
/// Vertical field of view of the optical sensor gimbal aperture, degrees.
pub const SENSOR_FOV_Y_DEG: f64 = 45.0;
/// Vertical field of view of the PFD `ConformalSvs3D` underlay, degrees.
pub const SVS_FOV_Y_DEG: f64 = 50.0;

const COCKPIT_NEAR_M: f64 = 0.05;
const CHASE_BACK_M: f64 = 22.0;
const CHASE_UP_M: f64 = 6.5;
const VOLUMETRIC_BACK_M: f64 = 140.0;
const VOLUMETRIC_UP_M: f64 = 85.0;

/// Pilot eye mount slightly forward and above the aircraft CG in FRD body axes.
pub fn pilot_eye_mount() -> CameraMount {
    CameraMount {
        offset_body_m: DVec3::new(0.65, 0.0, -0.25),
        body_to_camera: DQuat::IDENTITY,
    }
}

/// Belly optical/EVS sensor mount pitched downward by `22°` in FRD body axes.
pub fn belly_sensor_mount() -> CameraMount {
    CameraMount {
        offset_body_m: DVec3::new(1.10, 0.0, 0.45),
        body_to_camera: DQuat::from_rotation_y(-22.0_f64.to_radians()),
    }
}

/// PFD `ConformalSvs3D` virtual camera mount aligned with the nose reference.
pub fn svs_nose_mount() -> CameraMount {
    CameraMount {
        offset_body_m: DVec3::ZERO,
        body_to_camera: DQuat::IDENTITY,
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

/// Converts a [`VirtualCameraSpec`] evaluated against `state` into a `sim_render` [`Camera`].
pub fn camera_from_virtual_spec(spec: &VirtualCameraSpec, state: &VehicleState) -> Option<Camera> {
    let (pose, intrinsics) = spec.pose_and_intrinsics(state, DVec3::ZERO).ok()?;
    Some(Camera {
        pose: CameraPose {
            position_ecef: pose.position_ecef,
            orientation: pose.orientation,
        },
        fov_y_rad: VirtualCameraSpec::vertical_fov_rad(&intrinsics),
        near_m: COCKPIT_NEAR_M,
    })
}

/// Camera placed according to `mode` around `state` (with `aboard` used for `Cockpit`).
pub fn viewport_camera(
    mode: OperatorViewportMode,
    state: &VehicleState,
    aboard: CameraPoseEcef,
) -> Camera {
    match mode {
        OperatorViewportMode::Cockpit => scenario_camera(aboard),
        OperatorViewportMode::SensorAperture => {
            let pose = belly_sensor_mount().pose(state, DVec3::ZERO);
            Camera {
                pose: CameraPose {
                    position_ecef: pose.position_ecef,
                    orientation: pose.orientation,
                },
                fov_y_rad: SENSOR_FOV_Y_DEG.to_radians(),
                near_m: COCKPIT_NEAR_M,
            }
        }
        OperatorViewportMode::Chase => external_offset_camera(state, CHASE_BACK_M, CHASE_UP_M),
        OperatorViewportMode::VolumetricScene => {
            external_offset_camera(state, VOLUMETRIC_BACK_M, VOLUMETRIC_UP_M)
        }
        OperatorViewportMode::ChartMap => nadir_chart_fallback_camera(state),
    }
}

fn external_offset_camera(state: &VehicleState, back_m: f64, up_m: f64) -> Camera {
    let geo = ecef_to_geodetic(state.position_ecef);
    let local = LocalFrame::at(&geo);
    let up = local.up();
    let body_fwd = (state.attitude * DVec3::X).normalize_or_zero();
    let horiz = body_fwd - up * body_fwd.dot(up);
    let forward = if horiz.length_squared() > 1.0e-6 {
        horiz.normalize()
    } else {
        local.north()
    };
    let eye = state.position_ecef - forward * back_m + up * up_m;
    let look_dir = (state.position_ecef - eye).normalize_or(forward);
    let right = look_dir.cross(up).normalize_or(local.east());
    let cam_up = right.cross(look_dir).normalize_or(up);
    let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -look_dir)).normalize();
    Camera {
        pose: CameraPose {
            position_ecef: eye,
            orientation,
        },
        fov_y_rad: COCKPIT_FOV_Y_DEG.to_radians(),
        near_m: 0.5,
    }
}

fn nadir_chart_fallback_camera(state: &VehicleState) -> Camera {
    let geo = ecef_to_geodetic(state.position_ecef);
    let local = LocalFrame::at(&geo);
    let eye = state.position_ecef + local.up() * 4_500.0;
    let right = local.east();
    let cam_up = local.north();
    let back = local.up();
    let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, back)).normalize();
    Camera {
        pose: CameraPose {
            position_ecef: eye,
            orientation,
        },
        fov_y_rad: COCKPIT_FOV_Y_DEG.to_radians(),
        near_m: 1.0,
    }
}
