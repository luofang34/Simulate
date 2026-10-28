//! Earth reference for the simulator: the WGS84 ellipsoid, local tangent
//! frames, a floating origin for camera-relative `f32` rendering, and UTC time.
//!
//! All world positions are `f64` ECEF metres. Precision is only reduced to
//! `f32` relative to a nearby origin, never in absolute coordinates.

mod ellipsoid;
mod frame;
mod origin;
mod utc;

pub use ellipsoid::{
    Geodetic, WGS84_A, WGS84_B, WGS84_E2, WGS84_F, ecef_to_geodetic, geodetic_to_ecef,
};
pub use frame::LocalFrame;
pub use origin::FloatingOrigin;
pub use utc::UtcTime;
