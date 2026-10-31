use glam::{DMat3, DQuat, DVec3};
use mbd_core::{AeroFlowState, AeroModel, AeroWrench, AmbientFlow, MassProperties};
use sim_atmosphere::AtmosphereField;
use sim_geo::{BodyEllipsoid, LocalFrame, UtcTime};

use super::{RigidBodyInput, ThrusterWrench, block_error};
use crate::{BlockError, VehicleState};

/// The name [`BlockError`]s from this block carry.
pub(super) const BLOCK: &str = "rigid_body_6dof";

/// Smallest ratio of the inertia determinant to the cube of its mean moment.
/// Below this the tensor is a point or a line, whose Euler equation has no
/// solution.
const MIN_INERTIA_CONDITION: f64 = 1e-12;

/// The integrated state: translation in the rotating body-fixed frame, with
/// the angular rate kept inertial because Euler's equation is written in it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(in crate::rigid_body) struct RigidState {
    pub(super) position_fixed_m: DVec3,
    pub(super) velocity_fixed_m_s: DVec3,
    pub(super) attitude: DQuat,
    pub(super) omega_inertial_body: DVec3,
}

impl RigidState {
    pub(super) fn from_vehicle(vehicle: &VehicleState, body: &BodyEllipsoid) -> Self {
        Self {
            position_fixed_m: vehicle.position_ecef,
            velocity_fixed_m_s: vehicle.velocity_ecef,
            attitude: vehicle.attitude,
            omega_inertial_body: vehicle.angular_rate_body
                + vehicle.attitude.inverse() * spin_fixed(body),
        }
    }

    pub(super) fn vehicle(&self, body: &BodyEllipsoid) -> VehicleState {
        VehicleState {
            position_ecef: self.position_fixed_m,
            velocity_ecef: self.velocity_fixed_m_s,
            attitude: self.attitude,
            angular_rate_body: self.relative_rate_body(body),
        }
    }

    /// Angular rate relative to the central body, body axes, rad/s.
    fn relative_rate_body(&self, body: &BodyEllipsoid) -> DVec3 {
        self.omega_inertial_body - self.attitude.inverse() * spin_fixed(body)
    }

    /// This state referred to the material point `shift_body` metres from the
    /// current reference point, moving rigidly with the airframe.
    pub(super) fn transported(&self, body: &BodyEllipsoid, shift_body: DVec3) -> Self {
        let shift_fixed = self.attitude * shift_body;
        let rate_fixed = self.attitude * self.relative_rate_body(body);
        Self {
            position_fixed_m: self.position_fixed_m + shift_fixed,
            velocity_fixed_m_s: self.velocity_fixed_m_s + rate_fixed.cross(shift_fixed),
            ..*self
        }
    }

    pub(super) fn is_finite(&self) -> bool {
        self.position_fixed_m.is_finite()
            && self.velocity_fixed_m_s.is_finite()
            && self.attitude.is_finite()
            && self.omega_inertial_body.is_finite()
    }
}

/// The central body's rotation vector in its own frame, rad/s.
fn spin_fixed(body: &BodyEllipsoid) -> DVec3 {
    DVec3::Z * body.omega_rad_s
}

/// Rate of change of a [`RigidState`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Derivative {
    pub(super) velocity: DVec3,
    pub(super) acceleration: DVec3,
    pub(super) attitude_rate: DQuat,
    pub(super) alpha: DVec3,
}

/// A [`Derivative`] with the forces that produced it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Evaluation {
    pub(super) derivative: Derivative,
    pub(super) specific_force_cg_body: DVec3,
    pub(super) alpha_body: DVec3,
    pub(super) aero: AeroWrench,
}

/// Total thruster load, moments taken about the centre of gravity.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct ThrustLoad {
    force_body_n: DVec3,
    moment_cg_body_nm: DVec3,
}

impl ThrustLoad {
    fn about(thrusters: &[ThrusterWrench], cg_body_m: DVec3) -> Self {
        thrusters
            .iter()
            .fold(Self::default(), |total, thruster| Self {
                force_body_n: total.force_body_n + thruster.force_body_n,
                moment_cg_body_nm: total.moment_cg_body_nm
                    + thruster.moment_body_nm
                    + (thruster.application_point_body_m - cg_body_m).cross(thruster.force_body_n),
            })
    }
}

/// The inverse of the inertia about the centre of gravity, or an error for a
/// tensor too degenerate to invert.
fn inertia_inverse(mass: &MassProperties) -> Result<DMat3, BlockError> {
    let inertia = mass.inertia_cg_body;
    let mean_moment = (inertia.x_axis.x + inertia.y_axis.y + inertia.z_axis.z) / 3.0;
    if inertia.determinant() <= MIN_INERTIA_CONDITION * mean_moment.powi(3) || mean_moment <= 0.0 {
        return Err(BlockError::InputOutOfRange {
            block: BLOCK,
            input: "inertia_cg_body",
            detail: format!("{inertia} cannot be inverted"),
        });
    }
    Ok(inertia.inverse())
}

/// One vehicle configuration, frozen for the span of a step.
pub(super) struct Dynamics<'a> {
    body: BodyEllipsoid,
    atmosphere: &'a dyn AtmosphereField,
    aero: &'a dyn AeroModel,
    mass: &'a MassProperties,
    inertia_inverse: DMat3,
    thrust: ThrustLoad,
    control_surfaces_rad: [f64; 8],
}

impl<'a> Dynamics<'a> {
    pub(super) fn of(
        body: BodyEllipsoid,
        atmosphere: &'a dyn AtmosphereField,
        aero: &'a dyn AeroModel,
        mass: &'a MassProperties,
        input: &RigidBodyInput,
    ) -> Result<Self, BlockError> {
        Ok(Self {
            body,
            atmosphere,
            aero,
            mass,
            inertia_inverse: inertia_inverse(mass)?,
            thrust: ThrustLoad::about(&input.thrusters, mass.cg_body_m),
            control_surfaces_rad: input.control_surfaces_rad,
        })
    }

    /// The equations of motion: Newton's law with the rotating frame's
    /// apparent accelerations, Euler's equation about the centre of gravity,
    /// and quaternion kinematics on the rate relative to the rotating frame.
    pub(super) fn evaluate(
        &self,
        state: &RigidState,
        utc: UtcTime,
    ) -> Result<Evaluation, BlockError> {
        let aero = self.aero_wrench(state, utc)?;
        let cg = self.mass.cg_body_m;
        let specific_force_cg_body =
            (aero.force_body_n + self.thrust.force_body_n) / self.mass.total_mass_kg;

        let position = state.position_fixed_m;
        let velocity = state.velocity_fixed_m_s;
        let spin = spin_fixed(&self.body);
        let acceleration = self.body.gravity_fixed(position)
            + state.attitude * specific_force_cg_body
            - 2.0 * spin.cross(velocity)
            - spin.cross(spin.cross(position));

        let omega = state.omega_inertial_body;
        let moment = aero.moment_about_cg(cg) + self.thrust.moment_cg_body_nm;
        let alpha_body =
            self.inertia_inverse * (moment - omega.cross(self.mass.inertia_cg_body * omega));

        let rate = state.relative_rate_body(&self.body);
        let rate_quat = DQuat::from_xyzw(rate.x, rate.y, rate.z, 0.0);
        Ok(Evaluation {
            derivative: Derivative {
                velocity,
                acceleration,
                attitude_rate: (state.attitude * rate_quat) * 0.5,
                alpha: alpha_body,
            },
            specific_force_cg_body,
            alpha_body,
            aero,
        })
    }

    fn aero_wrench(&self, state: &RigidState, utc: UtcTime) -> Result<AeroWrench, BlockError> {
        let position = self.body.fixed_to_geodetic(state.position_fixed_m);
        let air =
            self.atmosphere
                .sample(&position, utc)
                .map_err(|source| BlockError::Environment {
                    block: BLOCK,
                    source,
                })?;
        let wind_fixed = LocalFrame::at(&position).ned_to_ecef() * air.wind_ned_m_s;
        let air_velocity_fixed = state.velocity_fixed_m_s - wind_fixed;
        let flow = AeroFlowState {
            velocity_air_body_m_s: state.attitude.inverse() * air_velocity_fixed,
            angular_rate_body_rad_s: state.relative_rate_body(&self.body),
            ambient: AmbientFlow {
                pressure_pa: air.pressure_pa,
                temperature_k: air.temperature_k,
                density_kg_m3: air.density_kg_m3,
                sound_speed_m_s: air.speed_of_sound_m_s,
            },
            control_surfaces_rad: self.control_surfaces_rad,
        };
        self.aero.evaluate(&flow).map_err(block_error)
    }
}
