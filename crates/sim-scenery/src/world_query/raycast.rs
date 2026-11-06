//! Segment-against-ground intersection: a chord march brackets the first
//! crossing, bisection narrows it, and a final secant places the hit.

use glam::DVec3;
use sim_geo::{EllipsoidalHeightM, Geodetic, LocalFrame, OrthometricHeightM};

use super::{AuthoritativeWorldSurface, RayHit, SurfaceSample, SurfaceSampleSource};
use crate::{SceneryError, WorldSurfaceQuery};

const MIN_STEPS: u32 = 4;
const MAX_STEPS: u32 = 256;
const REFINEMENTS: u32 = 6;

/// Height of a point above the ground below it, or above the ellipsoid where
/// no ground is known.
struct Clearance {
    metres: f64,
    geodetic: Geodetic,
}

pub(super) fn first_hit(
    surface: &AuthoritativeWorldSurface,
    from_ecef: DVec3,
    to_ecef: DVec3,
    steps: u32,
) -> Result<Option<RayHit>, SceneryError> {
    if !(from_ecef.is_finite() && to_ecef.is_finite()) {
        return Ok(None);
    }
    let chords = steps.clamp(MIN_STEPS, MAX_STEPS);
    let point = |t: f64| from_ecef + (to_ecef - from_ecef) * t;
    let mut lower = (0.0, clearance(surface, point(0.0))?);
    if lower.1.metres <= 0.0 {
        return hit(surface, from_ecef, point(0.0), lower.1.geodetic).map(Some);
    }
    for chord in 1..=chords {
        let t = f64::from(chord) / f64::from(chords);
        let upper = (t, clearance(surface, point(t))?);
        if upper.1.metres <= 0.0 {
            let t_hit = refine(surface, &point, lower, upper)?;
            let position = point(t_hit);
            let geodetic = surface.ellipsoid().fixed_to_geodetic(position);
            return hit(surface, from_ecef, position, geodetic).map(Some);
        }
        lower = upper;
    }
    Ok(None)
}

fn clearance(
    surface: &AuthoritativeWorldSurface,
    position_ecef: DVec3,
) -> Result<Clearance, SceneryError> {
    let geodetic = surface.ellipsoid().fixed_to_geodetic(position_ecef);
    let ground = surface
        .heights_at(geodetic.lat_rad, geodetic.lon_rad)?
        .map_or(0.0, |(height, _, _)| height.0);
    Ok(Clearance {
        metres: geodetic.height_m - ground,
        geodetic,
    })
}

/// Bisects the bracket `above..below` and returns the secant root of the
/// last one, which is where the hit lies to well under a metre.
fn refine(
    surface: &AuthoritativeWorldSurface,
    point: &dyn Fn(f64) -> DVec3,
    mut above: (f64, Clearance),
    mut below: (f64, Clearance),
) -> Result<f64, SceneryError> {
    for _ in 0..REFINEMENTS {
        let mid = 0.5 * (above.0 + below.0);
        let sample = (mid, clearance(surface, point(mid))?);
        if sample.1.metres <= 0.0 {
            below = sample;
        } else {
            above = sample;
        }
    }
    let span = above.1.metres - below.1.metres;
    let fraction = if span > 0.0 {
        above.1.metres / span
    } else {
        1.0
    };
    Ok(above.0 + (below.0 - above.0) * fraction)
}

fn hit(
    surface: &AuthoritativeWorldSurface,
    from_ecef: DVec3,
    position_ecef: DVec3,
    geodetic: Geodetic,
) -> Result<RayHit, SceneryError> {
    let ground = match surface.sample_surface(&geodetic)? {
        Some(sample) => sample,
        None => bare_ellipsoid(surface, &geodetic),
    };
    Ok(RayHit {
        distance_m: (position_ecef - from_ecef).length(),
        position_ecef,
        geodetic,
        surface: ground,
    })
}

fn bare_ellipsoid(surface: &AuthoritativeWorldSurface, geodetic: &Geodetic) -> SurfaceSample {
    let undulation = match surface.vertical_datum() {
        Some(_) => sim_geo::egm96_undulation_m(geodetic.lat_rad, geodetic.lon_rad),
        None => 0.0,
    };
    SurfaceSample {
        ellipsoidal_height_m: EllipsoidalHeightM(0.0),
        orthometric_msl_m: OrthometricHeightM(-undulation),
        normal_enu: DVec3::Z,
        normal_ecef: LocalFrame::at(geodetic).up(),
        source: SurfaceSampleSource::ReferenceEllipsoid,
        world_version: surface.world_version(),
    }
}
