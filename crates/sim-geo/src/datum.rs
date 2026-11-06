//! Vertical datums: the surfaces a height can be measured from, and typed
//! heights so an MSL altitude is never mistaken for an ellipsoidal one.

use crate::{Geodetic, egm96_undulation_m};

/// The surface a height is measured from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VerticalDatumKind {
    /// The WGS84 reference ellipsoid (GNSS heights, [`Geodetic::height_m`]).
    EllipsoidWgs84,
    /// Mean sea level as modelled by the EGM96 geoid (charted elevations,
    /// barometric altitude).
    OrthometricMslEgm96,
    /// The IAU lunar reference sphere, which has no geoid of its own.
    LunarSphereIau,
}

/// Height `h` above the body's reference ellipsoid, metres.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct EllipsoidalHeightM(pub f64);

/// Height `H` above mean sea level (the EGM96 geoid), metres; `h = H + N`
/// with `N` from [`egm96_undulation_m`].
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct OrthometricHeightM(pub f64);

/// Vertical clearance above the local ground surface, metres.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct AglHeightM(pub f64);

/// The mean-sea-level height of `position`, whose height is ellipsoidal:
/// `H = h - N(φ, λ)`.
#[must_use]
pub fn ellipsoidal_to_msl_egm96(position: &Geodetic) -> OrthometricHeightM {
    OrthometricHeightM(position.height_m - egm96_undulation_m(position.lat_rad, position.lon_rad))
}

/// The ellipsoidal height of a point `msl` above mean sea level at a
/// latitude and longitude: `h = H + N(φ, λ)`.
#[must_use]
pub fn msl_egm96_to_ellipsoidal(
    lat_rad: f64,
    lon_rad: f64,
    msl: OrthometricHeightM,
) -> EllipsoidalHeightM {
    EllipsoidalHeightM(msl.0 + egm96_undulation_m(lat_rad, lon_rad))
}
