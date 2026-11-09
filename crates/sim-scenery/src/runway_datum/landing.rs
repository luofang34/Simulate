//! The landing runway a conformal display draws, laid on the runway datum. A head-up
//! display draws a runway as one plane, so the plane is the datum's chord over the first
//! kilometre past the threshold, through the touchdown zone, where conformity matters
//! most.

use sim_geo::{
    Geodetic, LocalFrame, OrthometricHeightM, ecef_to_geodetic, geodetic_to_ecef,
    msl_egm96_to_ellipsoidal,
};

use super::RunwayDatum;
use crate::{CatalogVerticalDatum, LandingRunwayGeometry};

/// How far past the threshold the conformal plane follows the datum: the 3,000 ft
/// touchdown zone and a margin.
const CONFORMAL_CHORD_M: f64 = 1_000.0;

impl RunwayDatum {
    /// `runway` with its threshold and pavement start at the datum's heights and its far
    /// end where the datum's chord over the first kilometre reaches, so a planar
    /// conformal runway meets the drawn one from the threshold through the touchdown
    /// zone. Heights the datum does not cover stay as published.
    #[must_use]
    pub fn touchdown_plane(
        &self,
        runway: &LandingRunwayGeometry,
        datum: CatalogVerticalDatum,
    ) -> LandingRunwayGeometry {
        let on_runway = |point: &Geodetic| {
            self.height_and_weight(point)
                .filter(|&(_, weight)| weight >= 1.0)
                .map(|(height, _)| height)
        };
        let threshold_msl =
            on_runway(&runway.threshold).unwrap_or(runway.threshold_elevation_msl_m);
        let start_msl = on_runway(&runway.pavement_start).unwrap_or(runway.start_elevation_msl_m);
        let chord_m = CONFORMAL_CHORD_M.min(runway.landing_length_m);
        let local = LocalFrame::at(&runway.threshold);
        let (sin_b, cos_b) = runway.true_bearing_rad.sin_cos();
        let ahead = local.north() * cos_b + local.east() * sin_b;
        let zone_end = ecef_to_geodetic(geodetic_to_ecef(&runway.threshold) + ahead * chord_m);
        let far_msl = match on_runway(&zone_end) {
            Some(zone_msl) if chord_m > 1.0 => {
                threshold_msl + (zone_msl - threshold_msl) * runway.landing_length_m / chord_m
            }
            _ => runway.far_elevation_msl_m,
        };
        let at = |point: &Geodetic, msl: f64| Geodetic {
            height_m: ellipsoidal_m(datum, point, msl),
            ..*point
        };
        LandingRunwayGeometry {
            pavement_start: at(&runway.pavement_start, start_msl),
            threshold: at(&runway.threshold, threshold_msl),
            far_end: at(&runway.far_end, far_msl),
            start_elevation_msl_m: start_msl,
            threshold_elevation_msl_m: threshold_msl,
            far_elevation_msl_m: far_msl,
            ..*runway
        }
    }
}

fn ellipsoidal_m(datum: CatalogVerticalDatum, point: &Geodetic, msl: f64) -> f64 {
    match datum {
        CatalogVerticalDatum::OrthometricMslEgm96 => {
            msl_egm96_to_ellipsoidal(point.lat_rad, point.lon_rad, OrthometricHeightM(msl)).0
        }
        CatalogVerticalDatum::EllipsoidWgs84 => msl,
    }
}
