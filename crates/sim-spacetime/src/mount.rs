//! Unified vehicle-to-sensor mounting geometry (`SensorMount`) with lever-arm offset,
//! body-to-mount quaternion, `mbd_core::Placement` conversion, and $\boldsymbol{\omega} \times \mathbf{r}$
//! velocity transport.

use glam::{DMat3, DQuat, DVec3, EulerRot};
#[cfg(feature = "world-mbd-bridges")]
use mbd_core::Placement;
use serde::{Deserialize, Serialize};

use crate::scalar_math::{serde_dquat, serde_dvec3};

/// Rigid sensor, camera, or antenna mounting on a host vehicle (`+X` Forward, `+Y` Right, `+Z` Down body frame).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SensorMount {
    /// Sensor position in vehicle body coordinates (`m`).
    #[serde(with = "serde_dvec3")]
    pub offset_body_m: DVec3,
    /// Unit quaternion rotating body-axis components into sensor-mount components (`v_mount = body_to_mount * v_body`).
    #[serde(with = "serde_dquat")]
    pub body_to_mount: DQuat,
}

/// Instantaneous world-frame (`ICRF` or `ECEF`) kinematics of a mounted sensor including lever-arm
/// offset and $\boldsymbol{\omega} \times \mathbf{r}$ velocity transport.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MountedKinematics {
    /// Sensor origin position in world coordinates (`m`).
    pub position_world_m: DVec3,
    /// Sensor origin velocity in world coordinates (`m/s`), including $\boldsymbol{\omega}_{\text{body}} \times (\mathbf{r}_{\text{offset}} - \mathbf{r}_{\text{cg}})$.
    pub velocity_world_m_s: DVec3,
    /// Unit quaternion rotating sensor-mount coordinates into world coordinates (`v_world = q_mount_to_world * v_mount`).
    pub q_mount_to_world: DQuat,
    /// Orthogonal matrix $R_{\text{mount}\to\text{world}}$ mapping sensor-mount coordinates into world coordinates.
    pub r_mount_to_world: DMat3,
}

fn nose_camera_to_body() -> DQuat {
    DQuat::from_mat3(&DMat3::from_cols(DVec3::Y, DVec3::NEG_Z, DVec3::NEG_X))
}

impl SensorMount {
    /// Identity mount at the vehicle origin (`offset_body_m = 0`, `body_to_mount = IDENTITY`).
    pub const IDENTITY: Self = Self {
        offset_body_m: DVec3::ZERO,
        body_to_mount: DQuat::IDENTITY,
    };

    /// Unit quaternion rotating sensor-mount coordinates into vehicle body coordinates (`body_to_mount.inverse()`).
    #[must_use]
    pub fn mount_to_body(self) -> DQuat {
        self.body_to_mount.inverse().normalize()
    }

    /// Forward-looking nose camera mount (`+X` Right = Body `+Y`, `+Y` Up = Body `-Z`, `-Z` Forward = Body `+X`).
    #[must_use]
    pub fn forward_nose(offset_body_m: DVec3) -> Self {
        Self {
            offset_body_m,
            body_to_mount: nose_camera_to_body().inverse(),
        }
    }

    /// Nadir-pointing belly camera mount (`-Z` optical axis along Body `+Z` Down, `+Y` Up along Body `+X` Forward).
    #[must_use]
    pub fn nadir_belly(offset_body_m: DVec3) -> Self {
        let camera_to_body = DMat3::from_cols(DVec3::Y, DVec3::X, DVec3::NEG_Z);
        Self {
            offset_body_m,
            body_to_mount: DQuat::from_mat3(&camera_to_body).inverse(),
        }
    }

    /// Nose camera mount rotated by intrinsic aerospace yaw, pitch, and roll (`rad`) about the body axes.
    #[must_use]
    pub fn oblique(offset_body_m: DVec3, yaw_rad: f64, pitch_rad: f64, roll_rad: f64) -> Self {
        let turned = DQuat::from_euler(EulerRot::ZYX, yaw_rad, pitch_rad, roll_rad);
        Self {
            offset_body_m,
            body_to_mount: (turned * nose_camera_to_body()).normalize().inverse(),
        }
    }

    /// Looks along body-axis `forward_body` (`-Z` mount axis) with mount `+Y` Up aligned with `up_body`.
    #[must_use]
    pub fn looking_along(offset_body_m: DVec3, forward_body: DVec3, up_body: DVec3) -> Self {
        let forward = forward_body.try_normalize();
        let right = forward.and_then(|f| f.cross(up_body).try_normalize());
        let camera_to_body = match (forward, right) {
            (Some(forward), Some(right)) => {
                DQuat::from_mat3(&DMat3::from_cols(right, right.cross(forward), -forward))
            }
            _ => nose_camera_to_body(),
        };
        Self {
            offset_body_m,
            body_to_mount: camera_to_body.normalize().inverse(),
        }
    }

    /// Constructs a mount whose `+Z` mount axis points along `boresight_body` in vehicle body coordinates.
    #[must_use]
    pub fn with_boresight_z(offset_body_m: DVec3, boresight_body: DVec3) -> Self {
        let z = boresight_body.normalize_or_zero();
        if z.length_squared() <= 0.5 {
            return Self::IDENTITY;
        }
        let ref_up = if z.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
        let x = ref_up.cross(z).normalize();
        let y = z.cross(x);
        let mount_to_body = DMat3::from_cols(x, y, z);
        Self {
            offset_body_m,
            body_to_mount: DQuat::from_mat3(&mount_to_body).normalize().inverse(),
        }
    }

    /// Evaluates the sensor's world-frame position, velocity (with $\boldsymbol{\omega}_{\text{body}} \times (\mathbf{r}_{\text{offset}} - \mathbf{r}_{\text{cg}})$ transport),
    /// and orientation given the host vehicle's world-frame CG position, velocity, attitude `q_body_to_world`,
    /// body-frame angular rate `omega_body_rad_s`, and CG location `cg_body_m`.
    #[must_use]
    pub fn evaluate_world_kinematics(
        &self,
        vehicle_pos_world_m: DVec3,
        vehicle_vel_world_m_s: DVec3,
        q_body_to_world: DQuat,
        omega_body_rad_s: DVec3,
        cg_body_m: DVec3,
    ) -> MountedKinematics {
        let q_b2w = q_body_to_world.normalize();
        let lever_body_m = self.offset_body_m - cg_body_m;
        let lever_world_m = q_b2w * lever_body_m;
        let lever_vel_body_m_s = omega_body_rad_s.cross(lever_body_m);
        let lever_vel_world_m_s = q_b2w * lever_vel_body_m_s;
        let q_mount_to_world = (q_b2w * self.body_to_mount.inverse()).normalize();
        MountedKinematics {
            position_world_m: vehicle_pos_world_m + lever_world_m,
            velocity_world_m_s: vehicle_vel_world_m_s + lever_vel_world_m_s,
            q_mount_to_world,
            r_mount_to_world: DMat3::from_quat(q_mount_to_world),
        }
    }
}

/// Extension trait bridging [`SensorMount`] with `mbd_core::Placement`.
#[cfg(feature = "world-mbd-bridges")]
pub trait SensorMountPlacementExt {
    /// Creates a sensor mount from an `mbd_core::Placement` (`placement.rotation` is mount-to-parent/body).
    fn from_placement(placement: Placement) -> Self;

    /// Converts this sensor mount into a root-relative `mbd_core::Placement` (`rotation` = mount-to-body).
    fn to_placement(self) -> Placement;
}

#[cfg(feature = "world-mbd-bridges")]
impl SensorMountPlacementExt for SensorMount {
    fn from_placement(placement: Placement) -> Self {
        Self {
            offset_body_m: placement.translation_m,
            body_to_mount: placement.rotation.inverse().normalize(),
        }
    }

    fn to_placement(self) -> Placement {
        Placement {
            parent: None,
            translation_m: self.offset_body_m,
            rotation: self.mount_to_body(),
        }
    }
}
