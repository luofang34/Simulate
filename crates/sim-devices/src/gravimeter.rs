//! What a gravimeter and a gravity gradiometer sense: truth, before any
//! instrument error.
//!
//! The sample is read from the world's one [`GravityField`], the instance
//! the vehicle's dynamics integrate and its inertial unit senses. A
//! navigator's reference model is another field; the difference between the
//! two at a place is the gravity anomaly a map-matching navigator steers by.

use glam::{DMat3, DVec3};
use sim_geo::{BodyEllipsoid, GravityField};

/// Gravity at an instrument at rest on a rotating body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravimeterTruth {
    /// Mass attraction of the body, body-fixed axes, m/s².
    pub attraction_fixed_m_s2: DVec3,
    /// Gravity as a plumb line feels it: the attraction and the centrifugal
    /// acceleration of the body's spin, body-fixed axes, m/s².
    pub gravity_fixed_m_s2: DVec3,
    /// The gradient of gravity, `∂g_i/∂x_j`, 1/s², with the spin's
    /// centrifugal part; `None` when the field cannot state its gradient.
    pub gradient_per_s2: Option<DMat3>,
}

impl GravimeterTruth {
    /// What a scalar gravimeter reads, m/s².
    #[must_use]
    pub fn magnitude_m_s2(&self) -> f64 {
        self.gravity_fixed_m_s2.length()
    }
}

/// Gravity at the body-fixed site `fixed` of `body`, in the world whose
/// truth field is `field`.
#[must_use]
pub fn gravimeter_truth(
    field: &dyn GravityField,
    body: &BodyEllipsoid,
    fixed: DVec3,
) -> GravimeterTruth {
    let spin = DVec3::new(0.0, 0.0, body.omega_rad_s);
    let attraction_fixed_m_s2 = field.attraction_fixed(fixed);
    let spin_sq = body.omega_rad_s * body.omega_rad_s;
    // The centrifugal acceleration ω²(x, y, 0) has the constant gradient
    // diag(ω², ω², 0).
    let centrifugal_gradient = DMat3::from_diagonal(DVec3::new(spin_sq, spin_sq, 0.0));
    GravimeterTruth {
        attraction_fixed_m_s2,
        gravity_fixed_m_s2: attraction_fixed_m_s2 - spin.cross(spin.cross(fixed)),
        gradient_per_s2: field
            .attraction_gradient_fixed(fixed)
            .map(|gradient| gradient + centrifugal_gradient),
    }
}

#[cfg(test)]
mod tests;
