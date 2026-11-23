//! Frames a vehicle is integrated and reported in.
//!
//! Dynamics run in inertial axes that coincide with the central body's fixed
//! axes at the start of a step; state is kept and reported in the fixed
//! axes. A vehicle's body axes are the root axes of its component tree, in
//! which the root link itself may be mounted with an offset.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, MbdError};
use mbd_multibody::{FloatingBaseState, PointKinematics};
use sim_geo::BodyEllipsoid;

use crate::VehicleState;

/// The central body's rotation vector in its own axes, rad/s.
pub(crate) fn spin(body: &BodyEllipsoid) -> DVec3 {
    DVec3::Z * body.omega_rad_s
}

/// Rotation taking inertial axes to the fixed axes `elapsed_s` after the
/// instant the two coincided.
pub(crate) fn fixed_from_inertial(body: &BodyEllipsoid, elapsed_s: f64) -> DQuat {
    DQuat::from_axis_angle(DVec3::Z, -body.omega_rad_s * elapsed_s)
}

/// A base given in fixed axes, expressed in the inertial axes that coincide
/// with them at this instant.
pub(crate) fn fixed_to_instantaneous_inertial(
    base_fixed: &FloatingBaseState,
    body: &BodyEllipsoid,
) -> FloatingBaseState {
    let omega_p = spin(body);
    let v_fixed = base_fixed.rotation_world_from_root * base_fixed.linear_vel_root_m_s;
    let v_inertial = v_fixed + omega_p.cross(base_fixed.position_world_m);
    let q_conj = base_fixed.rotation_world_from_root.conjugate();
    FloatingBaseState {
        position_world_m: base_fixed.position_world_m,
        rotation_world_from_root: base_fixed.rotation_world_from_root,
        linear_vel_root_m_s: q_conj * v_inertial,
        angular_vel_root_rad_s: base_fixed.angular_vel_root_rad_s + q_conj * omega_p,
    }
}

/// A base in the inertial axes of a step that began `dt_s` ago, expressed in
/// the fixed axes of now.
pub(crate) fn inertial_to_fixed_after_step(
    base_i_next: &FloatingBaseState,
    body: &BodyEllipsoid,
    dt_s: f64,
) -> FloatingBaseState {
    let q_f_from_i = fixed_from_inertial(body, dt_s);
    let pos_fixed = q_f_from_i * base_i_next.position_world_m;
    let rot_fixed = (q_f_from_i * base_i_next.rotation_world_from_root).normalize();
    let v_i_in_f = q_f_from_i * base_i_next.linear_vel_world_m_s();
    let omega_p = spin(body);
    let v_rel_fixed = v_i_in_f - omega_p.cross(pos_fixed);
    let q_conj = rot_fixed.conjugate();
    FloatingBaseState {
        position_world_m: pos_fixed,
        rotation_world_from_root: rot_fixed,
        linear_vel_root_m_s: q_conj * v_rel_fixed,
        angular_vel_root_rad_s: base_i_next.angular_vel_root_rad_s - q_conj * omega_p,
    }
}

/// A point of a link given in the step's inertial axes, as the ground sees
/// it `elapsed_s` into the step: position, velocity and rates relative to
/// the rotating body, in its fixed axes.
pub(crate) fn point_in_fixed_axes(
    point: &PointKinematics,
    body: &BodyEllipsoid,
    elapsed_s: f64,
) -> PointKinematics {
    let turn = DMat3::from_quat(fixed_from_inertial(body, elapsed_s));
    let omega_p = spin(body);
    PointKinematics {
        position_world_m: turn * point.position_world_m,
        velocity_world_m_s: turn
            * (point.velocity_world_m_s - omega_p.cross(point.position_world_m)),
        angular_vel_world_rad_s: turn * (point.angular_vel_world_rad_s - omega_p),
        rotation_world_from_link: turn * point.rotation_world_from_link,
    }
}

/// Where the root link sits in the body axes of its vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RootMount {
    /// The root component.
    pub(crate) component: ComponentId,
    /// Root link origin in body axes, m.
    pub(crate) origin_body_m: DVec3,
    /// Body-from-root-link rotation.
    pub(crate) body_from_link: DQuat,
}

impl RootMount {
    /// The mount of the root of `tree`.
    pub(crate) fn of(tree: &ComponentTree) -> Result<Self, MbdError> {
        let component = tree
            .ids()
            .find(|&id| {
                tree.spec(id)
                    .is_ok_and(|spec| spec.placement.parent.is_none())
            })
            .ok_or(MbdError::NoAttachedMass)?;
        let (origin_body_m, body_from_link) = tree.world_transform_in_root(component)?;
        Ok(Self {
            component,
            origin_body_m,
            body_from_link,
        })
    }

    /// A body-axes point in the root link's axes.
    pub(crate) fn point_in_link(&self, point_body_m: DVec3) -> DVec3 {
        self.body_from_link.conjugate() * (point_body_m - self.origin_body_m)
    }

    /// A body-axes vector in the root link's axes.
    pub(crate) fn vector_in_link(&self, vector_body: DVec3) -> DVec3 {
        self.body_from_link.conjugate() * vector_body
    }

    /// World-from-body rotation of a vehicle whose root link is at `base`.
    pub(crate) fn attitude(&self, base: &FloatingBaseState) -> DQuat {
        (base.rotation_world_from_root * self.body_from_link.conjugate()).normalize()
    }

    /// The vehicle's body axes as a moving frame, for a root link at `base`:
    /// their origin, attitude, and twist in their own axes.
    pub(crate) fn body_axes(&self, base: &FloatingBaseState) -> FloatingBaseState {
        let attitude = self.attitude(base);
        let rate_body = self.body_from_link * base.angular_vel_root_rad_s;
        let arm_world = attitude * self.origin_body_m;
        let velocity_world = base.linear_vel_world_m_s() - (attitude * rate_body).cross(arm_world);
        FloatingBaseState {
            position_world_m: base.position_world_m - arm_world,
            rotation_world_from_root: attitude,
            linear_vel_root_m_s: attitude.conjugate() * velocity_world,
            angular_vel_root_rad_s: rate_body,
        }
    }

    /// The root link's state for a vehicle whose centre of gravity, at
    /// `cg_body_m` in body axes, has the state `vehicle`.
    pub(crate) fn base_of(&self, vehicle: &VehicleState, cg_body_m: DVec3) -> FloatingBaseState {
        let arm_body = self.origin_body_m - cg_body_m;
        let arm_world = vehicle.attitude * arm_body;
        let rate_world = vehicle.attitude * vehicle.angular_rate_body;
        let rotation = (vehicle.attitude * self.body_from_link).normalize();
        let velocity_world = vehicle.velocity_ecef + rate_world.cross(arm_world);
        FloatingBaseState {
            position_world_m: vehicle.position_ecef + arm_world,
            rotation_world_from_root: rotation,
            linear_vel_root_m_s: rotation.conjugate() * velocity_world,
            angular_vel_root_rad_s: self.vector_in_link(vehicle.angular_rate_body),
        }
    }

    /// The state of the centre of gravity, at `cg_body_m` in body axes, of a
    /// vehicle whose root link is at `base`.
    pub(crate) fn vehicle_of(&self, base: &FloatingBaseState, cg_body_m: DVec3) -> VehicleState {
        let attitude = self.attitude(base);
        let rate_body = self.body_from_link * base.angular_vel_root_rad_s;
        let arm_world = attitude * (cg_body_m - self.origin_body_m);
        VehicleState {
            position_ecef: base.position_world_m + arm_world,
            velocity_ecef: base.linear_vel_world_m_s() + (attitude * rate_body).cross(arm_world),
            attitude,
            angular_rate_body: rate_body,
        }
    }
}
