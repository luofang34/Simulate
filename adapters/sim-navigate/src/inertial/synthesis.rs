use glam::{DQuat, DVec3};
use sim_core::{SimInstant, VehicleState};
use sim_devices::{ImuErrorModel, ImuErrorSpec, InertialIncrement};
use sim_geo::{BodyEllipsoid, WGS84_J2};

use crate::NavigateBridgeError;

/// Earth's rotation relative to inertial space, ECEF axes, rad/s.
const EARTH_RATE_ECEF: DVec3 = DVec3::new(0.0, 0.0, BodyEllipsoid::WGS84.omega_rad_s);

/// Three-point Gauss–Legendre nodes on `[0, 1]` with their weights. The
/// specific force varies smoothly inside one step, so its integral comes out
/// far below the filter's own discretisation error.
const QUADRATURE: [(f64, f64); 3] = [
    (0.112_701_665_379_258_31, 5.0 / 18.0),
    (0.5, 8.0 / 18.0),
    (0.887_298_334_620_741_7, 5.0 / 18.0),
];

/// The mass attraction the simulated world exerts, which the accelerometers
/// cannot sense and which the synthesis therefore removes from the
/// kinematic acceleration.
///
/// Navigate's surface filter models WGS84 point mass plus `J2`, so
/// [`Self::WgsJ2`] isolates mechanisation differences. Its `J2` constant
/// differs from sim-geo's by about 3e-6 of the `J2` term, some 5e-8 m/s².
/// [`Self::PointMass`] is what `sim_core::RigidBodyBlock` integrates; a filter
/// fed from it carries the whole `J2` term, up to about 0.03 m/s², as
/// unmodelled acceleration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TruthGravity {
    /// WGS84 point mass with the second zonal harmonic.
    WgsJ2,
    /// WGS84 point mass alone.
    PointMass,
}

impl TruthGravity {
    /// Mass attraction at `position_ecef`, ECEF m/s².
    pub fn attraction_ecef(self, position_ecef: DVec3) -> DVec3 {
        let earth = BodyEllipsoid::WGS84;
        match self {
            Self::WgsJ2 => earth.gravity_fixed_j2(position_ecef, WGS84_J2),
            Self::PointMass => earth.gravity_fixed(position_ecef),
        }
    }
}

/// The rotation vector of `rotation`, taking the short way round.
fn rotation_vector(rotation: DQuat) -> DVec3 {
    let short = if rotation.w < 0.0 {
        -rotation
    } else {
        rotation
    };
    short.to_scaled_axis()
}

/// What an ideal strapdown IMU on the vehicle's body axes senses between two
/// truth states `dt_s` apart.
///
/// Inside the step the body rate relative to the Earth is constant and the
/// ECEF acceleration is `Δv/dt`, which is how `sim_core::KinematicModel`
/// moves. The rotation increment relative to inertial space is exact:
/// `exp(Δθ) = q₀⁻¹·exp(Ω·dt)·q₁`. The velocity increment is the integral of
/// the body-axis specific force `f = a + 2Ω×v + Ω×(Ω×p) − g(p)`, by quadrature,
/// without any rotation correction, as an integrating IMU reports it.
pub fn ideal_increment(
    gravity: TruthGravity,
    from: &VehicleState,
    to: &VehicleState,
    dt_s: f64,
) -> InertialIncrement {
    let frame_turn = DQuat::from_scaled_axis(EARTH_RATE_ECEF * dt_s);
    let delta_theta_rad = rotation_vector(from.attitude.inverse() * frame_turn * to.attitude);
    let body_rate = rotation_vector(from.attitude.inverse() * to.attitude) / dt_s;
    let acceleration = (to.velocity_ecef - from.velocity_ecef) / dt_s;
    let delta_velocity_m_s = QUADRATURE
        .iter()
        .map(|&(node, weight)| {
            let t = node * dt_s;
            let velocity = from.velocity_ecef + acceleration * t;
            let position =
                from.position_ecef + from.velocity_ecef * t + acceleration * (0.5 * t * t);
            let specific_force = acceleration
                + 2.0 * EARTH_RATE_ECEF.cross(velocity)
                + EARTH_RATE_ECEF.cross(EARTH_RATE_ECEF.cross(position))
                - gravity.attraction_ecef(position);
            let attitude = from.attitude * DQuat::from_scaled_axis(body_rate * t);
            attitude.inverse() * specific_force * (weight * dt_s)
        })
        .sum();
    InertialIncrement {
        dt_s,
        delta_theta_rad,
        delta_velocity_m_s,
    }
}

/// A strapdown IMU on the vehicle: the ideal increments of
/// [`ideal_increment`] between successive truth states, through a seeded
/// [`ImuErrorModel`].
///
/// This is the simulator's side of the link; it reads truth because it is
/// the physics of the sensor. Only its output reaches a filter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulatedImu {
    gravity: TruthGravity,
    errors: ImuErrorModel,
    previous: Option<(SimInstant, VehicleState)>,
}

impl SimulatedImu {
    /// A unit with errors drawn from `spec` and `seed`, in a world whose
    /// mass attraction is `gravity`.
    pub fn new(
        spec: ImuErrorSpec,
        seed: u64,
        gravity: TruthGravity,
    ) -> Result<Self, NavigateBridgeError> {
        Ok(Self {
            gravity,
            errors: ImuErrorModel::new(spec, seed).map_err(NavigateBridgeError::ImuModel)?,
            previous: None,
        })
    }

    /// The unit's error state, for scoring its bias against an estimate.
    pub const fn errors(&self) -> &ImuErrorModel {
        &self.errors
    }

    /// The increment sensed since the previous call, ending at `at` with the
    /// vehicle in `truth`. The first call only starts the interval and gives
    /// `None`; `at` must advance on every call.
    pub fn sense(
        &mut self,
        at: SimInstant,
        truth: &VehicleState,
    ) -> Result<Option<InertialIncrement>, NavigateBridgeError> {
        let Some((start, from)) = self.previous.replace((at, *truth)) else {
            return Ok(None);
        };
        if at <= start {
            return Err(NavigateBridgeError::ImuIntervalNotAdvancing { start, end: at });
        }
        let dt_s = at.duration_since(start).as_seconds();
        let ideal = ideal_increment(self.gravity, &from, truth, dt_s);
        self.errors
            .sense(&ideal)
            .map(Some)
            .map_err(NavigateBridgeError::ImuModel)
    }
}
