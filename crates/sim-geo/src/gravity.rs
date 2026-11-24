//! The truth gravity of a simulated world.

use glam::{DMat3, DVec3};

use crate::BodyEllipsoid;

/// The mass attraction of a central body, m/s², at a body-fixed position in
/// metres. It excludes the centrifugal term of the rotating frame.
///
/// One field is the truth of one world. The vehicle dynamics and every
/// inertial or gravimetric sensor on the vehicle read the same instance, or
/// the sensors disagree with the path the vehicle flies.
///
/// [`BodyEllipsoid`] is the reference field: the point mass and `J2` that
/// Navigate's filter also models, so a world on it has no gravity model
/// error. A world with model error that the navigator does not know, such as
/// a high-degree spherical-harmonic expansion or a local anomaly grid for
/// gravimeter navigation, is another implementation.
pub trait GravityField: Send + Sync + core::fmt::Debug {
    /// The attraction at `fixed`, body-fixed m/s².
    fn attraction_fixed(&self, fixed: DVec3) -> DVec3;

    /// The gravity-gradient tensor `∂g_i/∂x_j` at `fixed`, 1/s², in
    /// body-fixed axes: what a gradiometer senses and what torques an
    /// extended body. `None` for a field that cannot state one, so a
    /// consumer never mistakes a missing tensor for a flat field.
    fn attraction_gradient_fixed(&self, _fixed: DVec3) -> Option<DMat3> {
        None
    }
}

impl GravityField for BodyEllipsoid {
    fn attraction_fixed(&self, fixed: DVec3) -> DVec3 {
        self.gravity_fixed(fixed)
    }
}
