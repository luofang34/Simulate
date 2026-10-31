//! Celestial reference for the simulator: WGS84 and lunar ellipsoids with
//! their gravity, the J2000 inertial frame and Apollo navigation stars, local
//! tangent frames, a floating origin for camera-relative `f32` rendering,
//! pinhole camera intrinsics, and UTC time.
//!
//! All world positions are `f64` body-fixed metres. Precision is only reduced
//! to `f32` relative to a nearby origin, never in absolute coordinates.

mod body;
mod ellipsoid;
mod frame;
mod inertial;
mod origin;
mod pinhole;
mod stars;
mod utc;

pub use body::{BodyEllipsoid, WGS84_J2};
pub use ellipsoid::{
    Geodetic, WGS84_A, WGS84_B, WGS84_E2, WGS84_F, ecef_to_geodetic, geodetic_to_ecef,
};
pub use frame::LocalFrame;
pub use inertial::InertialTransform;
pub use origin::FloatingOrigin;
pub use pinhole::SensorIntrinsics;
pub use stars::{APOLLO_NAV_STARS, NavStar, nav_star_by_octal_id, stars_in_cone};
pub use utc::UtcTime;
