//! Celestial reference for the simulator: WGS84, lunar, and Martian ellipsoids
//! with their gravity, the J2000 inertial frame and Apollo navigation stars,
//! the truth gravity field of a world, local tangent frames, vertical datums with the EGM96 geoid, a floating
//! origin for camera-relative `f32` rendering,
//! pinhole camera intrinsics, and UTC time.
//!
//! All world positions are `f64` body-fixed metres. Precision is only reduced
//! to `f32` relative to a nearby origin, never in absolute coordinates.

mod body;
mod datum;
mod ellipsoid;
mod frame;
mod geoid;
mod gravity;
mod inertial;
mod origin;
mod pinhole;
mod stars;
mod utc;

pub use body::BodyEllipsoid;
pub use datum::{
    AglHeightM, EllipsoidalHeightM, OrthometricHeightM, VerticalDatumKind,
    ellipsoidal_to_msl_egm96, msl_egm96_to_ellipsoidal,
};
pub use ellipsoid::{
    Geodetic, WGS84_A, WGS84_B, WGS84_E2, WGS84_F, ecef_to_geodetic, geodetic_to_ecef,
};
pub use frame::LocalFrame;
pub use geoid::egm96_undulation_m;
pub use gravity::GravityField;
pub use inertial::InertialTransform;
pub use origin::FloatingOrigin;
pub use pinhole::SensorIntrinsics;
pub use stars::{APOLLO_NAV_STARS, NavStar, nav_star_by_octal_id, stars_in_cone};
pub use utc::UtcTime;
