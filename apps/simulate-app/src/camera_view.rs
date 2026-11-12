use glam::{DMat3, DQuat, DVec3};
use sim_core::{CameraMount, CameraPoseEcef, VehicleState, VirtualCameraSpec};
use sim_geo::{BodyEllipsoid, LocalFrame};
use sim_render::{Camera, CameraPose};
use sim_scenario::{C172_PILOT_EYE_BODY_M, OperatorViewportMode, ScenarioProfile};

use crate::vehicles::FreeFlightVehicle;

/// Vertical field of view of the primary cockpit OTW camera, degrees (`~72.7°` horizontal on `16:9`).
pub const COCKPIT_FOV_Y_DEG: f64 = 45.0;
/// Vertical field of view of external chase, volumetric, and chart cameras, degrees.
pub const EXTERNAL_FOV_Y_DEG: f64 = 55.0;
/// Vertical field of view of the optical sensor gimbal aperture, degrees.
pub const SENSOR_FOV_Y_DEG: f64 = 45.0;
/// Vertical field of view of the PFD `ConformalSvs3D` underlay, degrees.
pub const SVS_FOV_Y_DEG: f64 = 50.0;

/// Look-offset limits and full-deflection rate: wide enough to see over a
/// shoulder, short of flipping the view upside down.
const LOOK_YAW_LIMIT_DEG: f64 = 150.0;
const LOOK_PITCH_LIMIT_DEG: f64 = 75.0;
const LOOK_RATE_DEG_S: f64 = 120.0;
const COCKPIT_STICK_YAW_DEG: f64 = 100.0;
const COCKPIT_STICK_PITCH_DEG: f64 = 55.0;
const COCKPIT_STICK_SLEW_PER_S: f64 = 12.0;

#[cfg(test)]
mod tests;
mod vehicle_suite;

pub use vehicle_suite::{DEFAULT_CHASE_BACK_UP_M, vehicle_camera_suite};

const COCKPIT_NEAR_M: f64 = 0.05;
const VOLUMETRIC_BACK_M: f64 = 140.0;
const VOLUMETRIC_UP_M: f64 = 85.0;
/// How far the reference axis may lean toward the vertical before the external cameras
/// take their bearing from the vehicle's other forward axis (`|cos|` of the angle to up).
const NEAR_VERTICAL_COS: f64 = 0.7;

/// Left-seat pilot eye at [`C172_PILOT_EYE_BODY_M`], looking `-3.0°` over the cowling.
pub fn pilot_eye_mount() -> CameraMount {
    CameraMount::oblique(C172_PILOT_EYE_BODY_M, 0.0, -3.0_f64.to_radians(), 0.0)
}

/// Seated cockpit or nose FPV camera mount matched to `vehicle`'s airframe geometry.
#[must_use]
pub fn pilot_eye_mount_for(vehicle: FreeFlightVehicle) -> CameraMount {
    match vehicle {
        FreeFlightVehicle::JsbsimC172
        | FreeFlightVehicle::JsbsimC172p
        | FreeFlightVehicle::AviateFixedWingDirect
        | FreeFlightVehicle::C172Mission
        | FreeFlightVehicle::Kinematic => pilot_eye_mount(),
        FreeFlightVehicle::JsbsimP38g => {
            CameraMount::oblique(p38g_eye_frd_m(), 0.0, -2.5_f64.to_radians(), 0.0)
        }
        FreeFlightVehicle::AviateX500Direct => {
            CameraMount::forward_nose(DVec3::new(0.18, 0.0, -0.05))
        }
    }
}

/// The P-38G design eyepoint in FRD body axes; the airframe mesh's `(x right,
/// y forward, z up)` origin is the body origin.
fn p38g_eye_frd_m() -> DVec3 {
    let [x_right, y_fwd, z_up] = sim_p38g::p38g_cockpit_contract().design_eyepoint_m;
    DVec3::new(f64::from(y_fwd), f64::from(x_right), -f64::from(z_up))
}

/// Belly optical/EVS sensor mount pitched downward by `22°` in FRD body axes.
pub fn belly_sensor_mount() -> CameraMount {
    CameraMount::oblique(
        DVec3::new(1.10, 0.0, 0.45),
        0.0,
        -22.0_f64.to_radians(),
        0.0,
    )
}

/// Centerline-boresighted PFD `ConformalSvs3D` camera mount at the C172 pilot eye's station
/// and height (`0°` angular offset so the PFD zero-pitch horizon line matches aircraft
/// nose `+X`).
pub fn svs_nose_mount() -> CameraMount {
    CameraMount::forward_nose(DVec3::new(
        C172_PILOT_EYE_BODY_M.x,
        0.0,
        C172_PILOT_EYE_BODY_M.z,
    ))
}

/// Centerline-boresighted PFD `ConformalSvs3D` mount at `vehicle`'s cockpit eyepoint height.
#[must_use]
pub fn svs_mount_for(vehicle: Option<FreeFlightVehicle>) -> CameraMount {
    match vehicle {
        Some(
            FreeFlightVehicle::JsbsimC172
            | FreeFlightVehicle::JsbsimC172p
            | FreeFlightVehicle::AviateFixedWingDirect
            | FreeFlightVehicle::C172Mission
            | FreeFlightVehicle::Kinematic,
        )
        | None => svs_nose_mount(),
        Some(FreeFlightVehicle::JsbsimP38g) => CameraMount::forward_nose(p38g_eye_frd_m()),
        Some(FreeFlightVehicle::AviateX500Direct) => {
            CameraMount::forward_nose(DVec3::new(0.18, 0.0, -0.05))
        }
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

/// Converts a [`VirtualCameraSpec`] evaluated against `state` over `body` into a
/// `sim_render` [`Camera`], its body-mounted offsets measured from the drawn airframe's
/// origin `model_origin_from_cg_frd_m` ahead of the centre of gravity.
pub fn camera_from_virtual_spec_on(
    body: BodyEllipsoid,
    spec: &VirtualCameraSpec,
    state: &VehicleState,
    model_origin_from_cg_frd_m: DVec3,
) -> Option<Camera> {
    let (pose, intrinsics) = spec
        .pose_and_intrinsics_on(&body, state, -model_origin_from_cg_frd_m)
        .ok()?;
    Some(Camera {
        pose: CameraPose {
            position_ecef: pose.position_ecef,
            orientation: pose.orientation,
        },
        fov_y_rad: VirtualCameraSpec::vertical_fov_rad(&intrinsics),
        near_m: COCKPIT_NEAR_M,
    })
}

/// Camera placed according to `mode` around `state` (with `aboard` used for `Cockpit`) over
/// Earth.
#[cfg(test)]
pub fn viewport_camera(
    mode: OperatorViewportMode,
    state: &VehicleState,
    aboard: CameraPoseEcef,
) -> Camera {
    viewport_camera_for(mode, None, DEFAULT_CHASE_BACK_UP_M, state, aboard)
}

/// Camera placed according to `mode` around `state` flying `profile` (with `aboard` used
/// for `Cockpit`), the external views levelled on the body `profile` flies over and the
/// chase `chase_back_up_m` behind and above the vehicle.
pub fn viewport_camera_for(
    mode: OperatorViewportMode,
    profile: Option<ScenarioProfile>,
    (chase_back_m, chase_up_m): (f64, f64),
    state: &VehicleState,
    aboard: CameraPoseEcef,
) -> Camera {
    let frame = ExternalFrame::new(profile, state);
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
        OperatorViewportMode::Chase => frame.offset_camera(state, chase_back_m, chase_up_m),
        OperatorViewportMode::VolumetricScene => {
            frame.offset_camera(state, VOLUMETRIC_BACK_M, VOLUMETRIC_UP_M)
        }
        OperatorViewportMode::ChartMap => frame.nadir_chart_camera(state),
    }
}

/// The local level frame the external cameras stand in, and the horizontal direction
/// they treat as the vehicle's forward.
struct ExternalFrame {
    local: LocalFrame,
    forward: DVec3,
}

impl ExternalFrame {
    fn new(profile: Option<ScenarioProfile>, state: &VehicleState) -> Self {
        let body = profile.map_or(BodyEllipsoid::WGS84, |p| p.world().body());
        let local = LocalFrame::at(&body.fixed_to_geodetic(state.position_ecef));
        let up = local.up();
        let horizontal = |axis: DVec3| {
            let along = (state.attitude * axis).normalize_or_zero();
            along - up * along.dot(up)
        };
        let nose = (state.attitude * DVec3::X).normalize_or_zero();
        // The Lunar Module thrusts along body +X, vertical through the descent, while its
        // crew windows face body +Z: that is the way it faces.
        let reference = if profile == Some(ScenarioProfile::Apollo11LunarLanding)
            && nose.dot(up).abs() > NEAR_VERTICAL_COS
        {
            horizontal(DVec3::Z)
        } else {
            horizontal(DVec3::X)
        };
        let forward = if reference.length_squared() > 1.0e-6 {
            reference.normalize()
        } else {
            local.north()
        };
        Self { local, forward }
    }

    fn offset_camera(&self, state: &VehicleState, back_m: f64, up_m: f64) -> Camera {
        let up = self.local.up();
        let forward = self.forward;
        let eye = state.position_ecef - forward * back_m + up * up_m;
        let look_dir = (state.position_ecef - eye).normalize_or(forward);
        let right = look_dir.cross(up).normalize_or(self.local.east());
        let cam_up = right.cross(look_dir).normalize_or(up);
        let orientation = DQuat::from_mat3(&DMat3::from_cols(right, cam_up, -look_dir)).normalize();
        Camera {
            pose: CameraPose {
                position_ecef: eye,
                orientation,
            },
            fov_y_rad: EXTERNAL_FOV_Y_DEG.to_radians(),
            near_m: 0.5,
        }
    }

    fn nadir_chart_camera(&self, state: &VehicleState) -> Camera {
        let local = &self.local;
        let eye = state.position_ecef + local.up() * 4_500.0;
        let orientation =
            DQuat::from_mat3(&DMat3::from_cols(local.east(), local.north(), local.up()))
                .normalize();
        Camera {
            pose: CameraPose {
                position_ecef: eye,
                orientation,
            },
            fov_y_rad: EXTERNAL_FOV_Y_DEG.to_radians(),
            near_m: 1.0,
        }
    }
}

/// Pilot camera look away from the default view, steered by the right stick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LookOffset {
    /// Pan right of the default view, radians.
    pub yaw_rad: f64,
    /// Tilt up from the default view, radians.
    pub pitch_rad: f64,
}

impl LookOffset {
    /// Moves the look by `axes` (`[pan right, tilt up]`, `-1..=1`) held for `dt_s`.
    pub fn integrate(&mut self, axes: [f64; 2], dt_s: f64) {
        if !(dt_s.is_finite() && dt_s > 0.0) {
            return;
        }
        let [pan, tilt] = axes.map(|a| {
            if a.is_finite() {
                a.clamp(-1.0, 1.0)
            } else {
                0.0
            }
        });
        let step = LOOK_RATE_DEG_S.to_radians() * dt_s;
        let yaw_max = LOOK_YAW_LIMIT_DEG.to_radians();
        let pitch_max = LOOK_PITCH_LIMIT_DEG.to_radians();
        self.yaw_rad = (self.yaw_rad + pan * step).clamp(-yaw_max, yaw_max);
        self.pitch_rad = (self.pitch_rad + tilt * step).clamp(-pitch_max, pitch_max);
    }

    /// Spring-centred cockpit head look: slews smoothly toward `axes` (`[pan right, tilt up]`,
    /// `-1..=1`) and returns smoothly to dead-ahead `(0, 0)` when the stick is released.
    pub fn follow_stick(&mut self, axes: [f64; 2], dt_s: f64) {
        if !(dt_s.is_finite() && dt_s > 0.0) {
            return;
        }
        let [pan, tilt] = axes.map(|a| {
            if a.is_finite() {
                a.clamp(-1.0, 1.0)
            } else {
                0.0
            }
        });
        let target_yaw = pan * COCKPIT_STICK_YAW_DEG.to_radians();
        let target_pitch = tilt * COCKPIT_STICK_PITCH_DEG.to_radians();
        let blend = 1.0 - (-COCKPIT_STICK_SLEW_PER_S * dt_s).exp();
        self.yaw_rad += (target_yaw - self.yaw_rad) * blend;
        self.pitch_rad += (target_pitch - self.pitch_rad) * blend;
        if pan == 0.0 && self.yaw_rad.abs() < 1e-4 {
            self.yaw_rad = 0.0;
        }
        if tilt == 0.0 && self.pitch_rad.abs() < 1e-4 {
            self.pitch_rad = 0.0;
        }
    }

    /// Back to the default view.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// `camera` turned by the look over Earth: the cockpit eye turns in place, the chase
    /// camera orbits `state`; other views ignore it.
    #[cfg(test)]
    #[must_use]
    pub fn apply(self, mode: OperatorViewportMode, camera: Camera, state: &VehicleState) -> Camera {
        self.apply_on(BodyEllipsoid::WGS84, mode, camera, state)
    }

    /// `camera` turned by the look over `body` (whose local vertical the chase camera
    /// orbits about): the cockpit eye turns in place, the chase camera orbits `state`;
    /// other views ignore it.
    #[must_use]
    pub fn apply_on(
        self,
        body: BodyEllipsoid,
        mode: OperatorViewportMode,
        camera: Camera,
        state: &VehicleState,
    ) -> Camera {
        if self == Self::default() {
            return camera;
        }
        let pose = camera.pose;
        let turned = match mode {
            OperatorViewportMode::Cockpit => CameraPose {
                orientation: (pose.orientation
                    * DQuat::from_rotation_y(-self.yaw_rad)
                    * DQuat::from_rotation_x(self.pitch_rad))
                .normalize(),
                ..pose
            },
            OperatorViewportMode::Chase => {
                let up = LocalFrame::at(&body.fixed_to_geodetic(state.position_ecef)).up();
                let right = pose.orientation * DVec3::X;
                let orbit = DQuat::from_axis_angle(up, -self.yaw_rad)
                    * DQuat::from_axis_angle(right, self.pitch_rad);
                CameraPose {
                    position_ecef: state.position_ecef
                        + orbit * (pose.position_ecef - state.position_ecef),
                    orientation: (orbit * pose.orientation).normalize(),
                }
            }
            _ => return camera,
        };
        Camera {
            pose: turned,
            ..camera
        }
    }
}
