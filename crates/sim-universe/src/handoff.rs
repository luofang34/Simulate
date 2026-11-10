//! The exact change of frame between flight dynamics and orbital propagation.
//!
//! `sim-core` flies a vehicle in its central body's rotating, body-fixed
//! frame: [`VehicleState`] velocity and angular rate are relative to that
//! frame. `sim-orbital` propagates in non-rotating ICRF axes centred on a
//! primary body. Both describe the same motion; the body's rotation state
//! (orientation, and its angular velocity) is the whole difference, so the
//! conversion is exact and invertible given one [`BodyRotationState`]. The
//! rotation must come from the same orientation catalog the orbital
//! propagator uses, or a vessel jumps at every handoff.

use glam::{DMat3, DQuat};
use sim_core::VehicleState;
use sim_geo::BodyEllipsoid;
use sim_orbital::{ContinuousPlantState, NaifId};
use sim_spacetime::BodyRotationState;

use crate::UniverseError;

/// The NAIF body whose fixed frame `body` describes, or `None` for a body
/// the orbital propagator has no ephemeris or orientation for.
#[must_use]
pub fn naif_body(body: BodyEllipsoid) -> Option<NaifId> {
    if body == BodyEllipsoid::WGS84 {
        Some(NaifId::EARTH)
    } else if body == BodyEllipsoid::MOON_IAU {
        Some(NaifId::MOON)
    } else if body == BodyEllipsoid::MARS_IAU {
        Some(NaifId::MARS)
    } else {
        None
    }
}

/// The reference ellipsoid whose fixed frame flight dynamics use about
/// `body`, or `None` for an origin nothing can fly a body-fixed state about,
/// such as the Sun or a barycentre.
#[must_use]
pub fn body_ellipsoid(body: NaifId) -> Option<BodyEllipsoid> {
    [
        BodyEllipsoid::WGS84,
        BodyEllipsoid::MOON_IAU,
        BodyEllipsoid::MARS_IAU,
    ]
    .into_iter()
    .find(|&ellipsoid| naif_body(ellipsoid) == Some(body))
}

/// What the orbital tier carries that a body-fixed state does not.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct InertialExtras {
    /// Usable propellant, kg.
    pub propellant_mass_kg: f64,
    /// Reaction-wheel angular momentum, body axes, N·m·s.
    pub h_rw_body_nms: glam::DVec3,
}

/// `state`, fixed to `primary`, as an orbital plant state in ICRF axes.
#[must_use]
pub fn to_inertial(
    state: &VehicleState,
    primary: NaifId,
    rotation: &BodyRotationState,
    extras: InertialExtras,
) -> ContinuousPlantState {
    let fixed_to_icrf = rotation.r_icrf_to_fixed.transpose();
    let spin_fixed = rotation.omega_body_rad_s;
    let r_fixed = state.position_ecef;
    let v_inertial_fixed_axes = state.velocity_ecef + spin_fixed.cross(r_fixed);
    let fixed_to_icrf_q = DQuat::from_mat3(&fixed_to_icrf);
    ContinuousPlantState {
        primary_body: primary,
        pos_rel_icrf_m: fixed_to_icrf * r_fixed,
        vel_rel_icrf_m_s: fixed_to_icrf * v_inertial_fixed_axes,
        q_body_to_icrf: (fixed_to_icrf_q * state.attitude).normalize(),
        omega_body_rad_s: state.angular_rate_body + state.attitude.inverse() * spin_fixed,
        propellant_mass_kg: extras.propellant_mass_kg,
        h_rw_body_nms: extras.h_rw_body_nms,
    }
}

/// `plant`, back in the fixed frame of its primary, with what that frame
/// cannot hold. Fails when `plant` is centred on another body than
/// `primary`.
pub fn to_body_fixed(
    plant: &ContinuousPlantState,
    primary: NaifId,
    rotation: &BodyRotationState,
) -> Result<(VehicleState, InertialExtras), UniverseError> {
    if plant.primary_body != primary {
        return Err(UniverseError::WrongPrimary {
            expected: primary,
            found: plant.primary_body,
        });
    }
    let icrf_to_fixed: DMat3 = rotation.r_icrf_to_fixed;
    let spin_fixed = rotation.omega_body_rad_s;
    let r_fixed = icrf_to_fixed * plant.pos_rel_icrf_m;
    let v_fixed = icrf_to_fixed * plant.vel_rel_icrf_m_s - spin_fixed.cross(r_fixed);
    let attitude = (DQuat::from_mat3(&icrf_to_fixed) * plant.q_body_to_icrf).normalize();
    let state = VehicleState {
        position_ecef: r_fixed,
        velocity_ecef: v_fixed,
        attitude,
        angular_rate_body: plant.omega_body_rad_s - attitude.inverse() * spin_fixed,
    };
    let extras = InertialExtras {
        propellant_mass_kg: plant.propellant_mass_kg,
        h_rw_body_nms: plant.h_rw_body_nms,
    };
    Ok((state, extras))
}

#[cfg(test)]
mod tests;
