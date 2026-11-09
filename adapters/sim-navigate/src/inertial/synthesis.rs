use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_core::{SimInstant, VehicleState};
use sim_devices::{ImuErrorModel, ImuErrorSpec, InertialIncrement};
use sim_geo::{BodyEllipsoid, GravityField};

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

/// The rotation vector of `rotation`, taking the short way round.
///
/// The angle comes from `atan2(|v|, w)` rather than from `acos(w)`, which
/// loses about half its digits near the identity: an Earth-rate step of
/// 6e-7 rad would carry a 1e-4 relative error, a spurious rate the size of a
/// navigation-grade gyro bias.
pub(crate) fn rotation_vector(rotation: DQuat) -> DVec3 {
    let short = if rotation.w < 0.0 {
        -rotation
    } else {
        rotation
    };
    let axis = short.xyz();
    let sine = axis.length();
    if sine == 0.0 {
        return DVec3::ZERO;
    }
    axis * (2.0 * sine.atan2(short.w) / sine)
}

/// What an ideal strapdown IMU on the vehicle's body axes reports between two
/// truth states `dt_s` apart, with exact coning and sculling compensation.
///
/// Inside the step the body rate relative to the Earth is constant and the
/// ECEF acceleration is `Δv/dt`, which is how `sim_core::KinematicModel`
/// moves. The rotation increment relative to inertial space is exact:
/// `exp(Δθ) = q₀⁻¹·exp(Ω·dt)·q₁`.
///
/// Navigate takes a velocity increment `Δv` whose rotation correction
/// `Δv + ½·Δθ×Δv` is the specific-force integral in the body axes fixed at
/// the start of the step. The synthesis forms that integral by quadrature of
/// `f = a + 2Ω×v + Ω×(Ω×p) − g(p)`, with `g` the world's truth
/// `gravity`, carried from each instant's ECEF axes
/// into the start's inertial body axes, and inverts the rotation correction
/// exactly. A raw `∫f_body dt` would differ from it by the sculling term,
/// which grows with turn rate times acceleration.
pub fn ideal_increment(
    gravity: &dyn GravityField,
    from: &VehicleState,
    to: &VehicleState,
    dt_s: f64,
) -> InertialIncrement {
    let frame_turn = DQuat::from_scaled_axis(EARTH_RATE_ECEF * dt_s);
    let delta_theta_rad = rotation_vector(from.attitude.inverse() * frame_turn * to.attitude);
    let acceleration = (to.velocity_ecef - from.velocity_ecef) / dt_s;
    let start_axes = from.attitude.inverse();
    let integral: DVec3 = QUADRATURE
        .iter()
        .map(|&(node, weight)| {
            let t = node * dt_s;
            let velocity = from.velocity_ecef + acceleration * t;
            let position =
                from.position_ecef + from.velocity_ecef * t + acceleration * (0.5 * t * t);
            let specific_force = acceleration
                + 2.0 * EARTH_RATE_ECEF.cross(velocity)
                + EARTH_RATE_ECEF.cross(EARTH_RATE_ECEF.cross(position))
                - gravity.attraction_fixed(position);
            let inertial_turn = DQuat::from_scaled_axis(EARTH_RATE_ECEF * t);
            start_axes * (inertial_turn * specific_force) * (weight * dt_s)
        })
        .sum();
    InertialIncrement {
        dt_s,
        delta_theta_rad,
        delta_velocity_m_s: without_rotation_correction(delta_theta_rad, integral),
    }
}

/// The `x` with `x + ½·θ×x = y`: `(I + [b×])⁻¹·y` with `b = ½·θ` is
/// `(y − b×y + (b·y)·b) / (1 + |b|²)`.
fn without_rotation_correction(theta: DVec3, y: DVec3) -> DVec3 {
    let b = theta * 0.5;
    (y - b.cross(y) + b * b.dot(y)) / (1.0 + b.length_squared())
}

/// A strapdown IMU on the vehicle: the ideal increments of
/// [`ideal_increment`] between successive truth states, through a seeded
/// [`ImuErrorModel`].
///
/// This is the simulator's side of the link; it reads truth because it is
/// the physics of the sensor. Only its output reaches a filter.
#[derive(Debug, Clone)]
pub struct SimulatedImu {
    gravity: Arc<dyn GravityField>,
    errors: ImuErrorModel,
    previous: Option<(SimInstant, VehicleState)>,
}

impl SimulatedImu {
    /// A unit with errors drawn from `spec` and `seed`, on Earth's reference
    /// field [`BodyEllipsoid::WGS84`].
    pub fn new(spec: ImuErrorSpec, seed: u64) -> Result<Self, NavigateBridgeError> {
        Ok(Self {
            gravity: Arc::new(BodyEllipsoid::WGS84),
            errors: ImuErrorModel::new(spec, seed).map_err(NavigateBridgeError::ImuModel)?,
            previous: None,
        })
    }

    /// The unit in a world whose truth gravity is `gravity`: the instance
    /// the vehicle's dynamics integrate, such as
    /// `sim_core::RigidBody6DofBlock::gravity`.
    #[must_use]
    pub fn with_gravity(mut self, gravity: Arc<dyn GravityField>) -> Self {
        self.gravity = gravity;
        self
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
        let ideal = ideal_increment(self.gravity.as_ref(), &from, truth, dt_s);
        self.errors
            .sense(&ideal)
            .map(Some)
            .map_err(NavigateBridgeError::ImuModel)
    }
}
