//! Runway strips: the graded surface around a runway, at its surveyed
//! threshold heights, feathered into the terrain around it.

use flightbag::RunwayEndSpec;
use glam::DVec3;
use sim_geo::{Geodetic, LocalFrame, geodetic_to_ecef};

/// Graded shoulder beyond each paved edge.
const SHOULDER_M: f64 = 15.0;
/// Graded overrun before the threshold and past the far end.
const OVERRUN_M: f64 = 60.0;
/// Width of the skirt over which the strip blends into the terrain.
const SKIRT_M: f64 = 40.0;
/// How close the reciprocal threshold must lie to this end's far point.
const RECIPROCAL_TOLERANCE_M: f64 = 60.0;
/// Extra 3D chord margin beyond the horizontal skirt for ellipsoid sagitta.
const CHORD_SAGITTA_MARGIN_M: f64 = 1_000.0;

/// One runway end's graded strip — pavement, shoulders, overruns and a
/// skirt feathering into the terrain — that an
/// [`crate::AuthoritativeWorldSurface`] holds at its surveyed heights.
///
/// Laid out in the 3D WGS84 tangent plane at the threshold so polar runways
/// near `±90°` stay straight to sub-centimetre with zero meridian skew.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunwayStrip {
    origin_ecef: DVec3,
    along_ecef: DVec3,
    across_ecef: DVec3,
    max_chord_sq: f64,
    length_m: f64,
    half_width_m: f64,
    threshold_m: f64,
    far_end_m: f64,
}

impl RunwayStrip {
    /// A strip per end. Both ends of a runway describe the same strip, so the
    /// far height comes from the reciprocal threshold when one is listed.
    ///
    /// Each threshold's `ellipsoidal_height_m` is read as a height in the
    /// datum of the surface the strips join, whatever the field's name says:
    /// MSL for an orthometric surface. Ends with a non-finite field or a
    /// non-positive length or width are skipped.
    #[must_use]
    pub fn from_ends(ends: &[RunwayEndSpec]) -> Vec<Self> {
        let mut strips: Vec<Self> = ends.iter().filter_map(Self::from_end).collect();
        let thresholds: Vec<(DVec3, f64)> = strips
            .iter()
            .map(|strip| (strip.origin_ecef, strip.threshold_m))
            .collect();
        for strip in &mut strips {
            let reciprocal = thresholds.iter().find(|(ecef, _)| {
                strip.local_ecef(*ecef).is_some_and(|(along, across)| {
                    (along - strip.length_m).hypot(across) <= RECIPROCAL_TOLERANCE_M
                })
            });
            if let Some(&(_, height)) = reciprocal {
                strip.far_end_m = height;
            }
        }
        strips
    }

    fn from_end(end: &RunwayEndSpec) -> Option<Self> {
        let threshold = &end.threshold;
        let usable = [
            threshold.latitude_deg,
            threshold.longitude_deg,
            threshold.ellipsoidal_height_m,
            end.true_bearing_rad,
            end.length_m,
            end.width_m,
        ]
        .iter()
        .all(|value| value.is_finite())
            && threshold.latitude_deg.abs() <= 90.0
            && end.length_m > 0.0
            && end.width_m > 0.0;
        if !usable {
            return None;
        }
        let origin = Geodetic::from_degrees(threshold.latitude_deg, threshold.longitude_deg, 0.0);
        let origin_ecef = geodetic_to_ecef(&origin);
        let local = LocalFrame::at(&origin);
        let (sin_course, cos_course) = end.true_bearing_rad.sin_cos();
        let along_ecef = local.north() * cos_course + local.east() * sin_course;
        let across_ecef = local.east() * cos_course - local.north() * sin_course;
        let half_width_m = end.width_m * 0.5;
        let max_reach = (end.length_m + OVERRUN_M + SKIRT_M)
            .hypot(half_width_m + SHOULDER_M + SKIRT_M)
            + CHORD_SAGITTA_MARGIN_M;
        Some(Self {
            origin_ecef,
            along_ecef,
            across_ecef,
            max_chord_sq: max_reach * max_reach,
            length_m: end.length_m,
            half_width_m,
            threshold_m: threshold.ellipsoidal_height_m,
            far_end_m: threshold.ellipsoidal_height_m,
        })
    }

    /// Metres along the centreline from the threshold and across it to the
    /// right, or `None` outside the strip's 3D bounding sphere.
    fn local_ecef(&self, surface_ecef: DVec3) -> Option<(f64, f64)> {
        let delta = surface_ecef - self.origin_ecef;
        if delta.length_squared() > self.max_chord_sq {
            return None;
        }
        Some((delta.dot(self.along_ecef), delta.dot(self.across_ecef)))
    }

    /// The strip height at `surface_ecef` and how strongly it applies: 1 on
    /// the strip, easing to 0 across the skirt; `None` beyond the skirt.
    fn height_and_weight_ecef(&self, surface_ecef: DVec3) -> Option<(f64, f64)> {
        let (along, across) = self.local_ecef(surface_ecef)?;
        let over_across = (across.abs() - (self.half_width_m + SHOULDER_M)).max(0.0);
        let over_along = (-OVERRUN_M - along)
            .max(along - (self.length_m + OVERRUN_M))
            .max(0.0);
        let outside = over_across.hypot(over_along);
        if outside >= SKIRT_M {
            return None;
        }
        let t = 1.0 - outside / SKIRT_M;
        let weight = t * t * (3.0 - 2.0 * t);
        let fraction = (along / self.length_m).clamp(0.0, 1.0);
        let height = self.threshold_m + (self.far_end_m - self.threshold_m) * fraction;
        Some((height, weight))
    }
}

/// The height of the strip that most strongly covers `position`, with its
/// weight in `(0, 1]`.
pub(super) fn strongest<'a>(
    strips: impl IntoIterator<Item = &'a RunwayStrip>,
    position: &Geodetic,
) -> Option<(f64, f64)> {
    let surface_ecef = geodetic_to_ecef(&Geodetic {
        height_m: 0.0,
        ..*position
    });
    strips
        .into_iter()
        .filter_map(|strip| strip.height_and_weight_ecef(surface_ecef))
        .filter(|&(_, weight)| weight > 0.0)
        .max_by(|a, b| a.1.total_cmp(&b.1))
}
