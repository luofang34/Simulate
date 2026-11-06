//! Where an airport's runways, tower and reference point are, in a world
//! read in either vertical datum, and the surface graph built from them.

use std::f64::consts::FRAC_PI_2;

use flightbag::{AirportSurfaceGraph, AuthoritySource, RunwayDesignator, RunwayEndSpec};
use sim_geo::{
    Geodetic, OrthometricHeightM, ecef_to_geodetic, geodetic_to_ecef, msl_egm96_to_ellipsoidal,
};
use world_core::GeodeticPosition;

use super::geometry::{bearing_and_distance, offset_deg};
use super::model::{
    AirportRecord, CompactCode, FrequencyRole, RunwayEndRecord, RunwayRecord, RunwaySurfaceKind,
};
use crate::{AerodromeId, CatalogVerticalDatum, OsmAerodromeDiagram, RunwayStrip};

/// Default visual/instrument glidepath when none is published (`3.0°`).
const DEFAULT_GLIDEPATH_DEG: f64 = 3.0;

/// Surveyed 3D landing geometry for one runway end, preserving displaced
/// landing threshold, opposite-end slope, published glidepath angle, and
/// navdata authority provenance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingRunwayGeometry {
    /// Runway-end designator (`01`, `22L`, `07`, `02T`).
    pub designator: CompactCode,
    /// Physical pavement start `(lat, lon, ellipsoidal_height_m)` in the world's datum.
    pub pavement_start: Geodetic,
    /// Landing threshold `(lat, lon, ellipsoidal_height_m)`, displaced inward by
    /// [`Self::displaced_threshold_m`] along the runway chord.
    pub threshold: Geodetic,
    /// Opposite physical pavement end `(lat, lon, ellipsoidal_height_m)`.
    pub far_end: Geodetic,
    /// Full physical pavement length between the two surveyed ends, metres.
    pub full_length_m: f64,
    /// Published displaced landing threshold distance from [`Self::pavement_start`], metres.
    pub displaced_threshold_m: f64,
    /// Published displaced landing threshold distance at the opposite end (`far_end`), metres.
    pub reciprocal_displaced_m: f64,
    /// Landing distance available from [`Self::threshold`] to [`Self::far_end`], metres.
    pub landing_length_m: f64,
    /// Published runway width, metres.
    pub width_m: f64,
    /// True bearing (`rad`, `[0, 2π)`) at [`Self::threshold`] toward [`Self::far_end`].
    pub true_bearing_rad: f64,
    /// Published visual/approach glidepath angle (`rad`).
    pub glidepath_rad: f64,
    /// Surveyed pavement-start elevation above MSL, metres.
    pub start_elevation_msl_m: f64,
    /// Landing-threshold elevation above MSL, metres.
    pub threshold_elevation_msl_m: f64,
    /// Surveyed opposite-end elevation above MSL, metres.
    pub far_elevation_msl_m: f64,
    /// Published aerodrome reference elevation (`ARP`, highest usable landing-area point), metres MSL.
    pub field_elevation_msl_m: f64,
    /// Runway surface classification (`Asphalt`, `PavedTurf`, `Turf`, `Gravel`, ...).
    pub surface: RunwaySurfaceKind,
    /// Navdata authority this runway geometry came from.
    pub authority: AuthoritySource,
    /// Effective date `[year, month, day]` of [`Self::authority`]'s edition.
    pub effective_ymd: [u16; 3],
}

/// Vertical tolerance band (`±3 m`) around surveyed runway-end and field (`ARP`) elevations
/// for DO-315B onboard terrain database fusion.
const ONBOARD_DEM_MARGIN_M: f64 = 3.0;
/// Maximum deviation from surveyed bounds before a DEM sample is rejected as unrelated/corrupt.
const ONBOARD_DEM_REJECT_M: f64 = 12.0;

impl LandingRunwayGeometry {
    /// Builds unsloped, non-displaced landing geometry from a `Flightbag` [`RunwayEndSpec`]
    /// when no full [`AirportRecord`] is present.
    #[must_use]
    pub fn from_runway_end_spec(spec: &RunwayEndSpec) -> Self {
        let thr_lat = spec.threshold.latitude_deg;
        let thr_lon = spec.threshold.longitude_deg;
        let thr_h = spec.threshold.ellipsoidal_height_m;
        let (far_lat, far_lon) =
            offset_deg((thr_lat, thr_lon), spec.true_bearing_rad, spec.length_m);
        let threshold = Geodetic::from_degrees(thr_lat, thr_lon, thr_h);
        let far_end = Geodetic::from_degrees(far_lat, far_lon, thr_h);
        Self {
            designator: CompactCode::parse(spec.designator.as_str()).unwrap_or_default(),
            pavement_start: threshold,
            threshold,
            far_end,
            full_length_m: spec.length_m,
            displaced_threshold_m: 0.0,
            reciprocal_displaced_m: 0.0,
            landing_length_m: spec.length_m,
            width_m: spec.width_m,
            true_bearing_rad: spec.true_bearing_rad,
            glidepath_rad: DEFAULT_GLIDEPATH_DEG.to_radians(),
            start_elevation_msl_m: thr_h,
            threshold_elevation_msl_m: thr_h,
            far_elevation_msl_m: thr_h,
            field_elevation_msl_m: thr_h,
            surface: RunwaySurfaceKind::Paved,
            authority: AuthoritySource::OpenStreetMap,
            effective_ymd: [1970, 1, 1],
        }
    }

    /// Physical MSL envelope `[min_msl_m, max_msl_m]` bounded by the surveyed runway ends
    /// and the published aerodrome elevation (`ARP`, ICAO/FAA highest landing-area point).
    #[must_use]
    pub fn valid_surface_msl_range(&self) -> (f64, f64) {
        let lo = self.start_elevation_msl_m.min(self.far_elevation_msl_m) - ONBOARD_DEM_MARGIN_M;
        let hi = self
            .start_elevation_msl_m
            .max(self.far_elevation_msl_m)
            .max(self.field_elevation_msl_m)
            + ONBOARD_DEM_MARGIN_M;
        (lo, hi)
    }

    /// Refines longitudinal endpoint and displaced-threshold elevations from an onboard
    /// terrain database (`sample_msl(lat_deg, lon_deg)`), gated by [`Self::valid_surface_msl_range`]
    /// so crowned runways (whose midpoint or displaced threshold rises toward `ARP` above the
    /// straight endpoint chord) register cleanly while corrupt/unrelated DEM samples are ignored.
    #[must_use]
    pub fn with_onboard_dem(
        &self,
        datum: CatalogVerticalDatum,
        mut sample_msl: impl FnMut(f64, f64) -> Option<f64>,
    ) -> Self {
        let (lo, hi) = self.valid_surface_msl_range();
        let mut refine = |point: Geodetic, surveyed_msl: f64| -> (Geodetic, f64) {
            let (lat, lon) = (point.lat_deg(), point.lon_deg());
            let Some(dem_msl) = sample_msl(lat, lon).filter(|h| h.is_finite()) else {
                return (point, surveyed_msl);
            };
            if dem_msl < lo - ONBOARD_DEM_REJECT_M || dem_msl > hi + ONBOARD_DEM_REJECT_M {
                return (point, surveyed_msl);
            }
            let msl = dem_msl.clamp(lo, hi);
            (
                Geodetic::from_degrees(lat, lon, ellipsoidal_m(datum, lat, lon, msl)),
                msl,
            )
        };
        let (pavement_start, start_elevation_msl_m) =
            refine(self.pavement_start, self.start_elevation_msl_m);
        let (threshold, threshold_elevation_msl_m) =
            refine(self.threshold, self.threshold_elevation_msl_m);
        let (far_end, far_elevation_msl_m) = refine(self.far_end, self.far_elevation_msl_m);
        Self {
            pavement_start,
            threshold,
            far_end,
            start_elevation_msl_m,
            threshold_elevation_msl_m,
            far_elevation_msl_m,
            ..*self
        }
    }
}

/// How far abeam the primary runway's midpoint a synthesized tower stands.
const TOWER_ABEAM_M: f64 = 180.0;
/// Synthesized tower cab height above the field: towered, then CTAF.
const TOWER_CAB_M: (f32, f32) = (38.0, 14.0);

/// The ellipsoidal height a charted MSL height names in a world read in
/// `datum`: MSL plus the EGM96 undulation when the world is orthometric,
/// the charted number itself in a synthetic world that takes charted heights
/// as ellipsoidal.
fn ellipsoidal_m(datum: CatalogVerticalDatum, lat_deg: f64, lon_deg: f64, msl_m: f64) -> f64 {
    match datum {
        CatalogVerticalDatum::OrthometricMslEgm96 => {
            msl_egm96_to_ellipsoidal(
                lat_deg.to_radians(),
                lon_deg.to_radians(),
                OrthometricHeightM(msl_m),
            )
            .0
        }
        CatalogVerticalDatum::EllipsoidWgs84 => msl_m,
    }
}

impl RunwayEndRecord {
    /// The runway end on WGS84 at its surveyed elevation, as an ellipsoidal
    /// height in a world read in `datum` (see
    /// [`AirportRecord::arp_geodetic`]).
    #[must_use]
    pub fn threshold_geodetic(&self, datum: CatalogVerticalDatum) -> Geodetic {
        let height = ellipsoidal_m(
            datum,
            self.lat_deg,
            self.lon_deg,
            f64::from(self.elevation_msl_m),
        );
        Geodetic::from_degrees(self.lat_deg, self.lon_deg, height)
    }
}

impl RunwayRecord {
    /// The two ends as `Flightbag` runway-end specs, each threshold at its
    /// runway end with the height `height_m` gives it, bearing and length
    /// measured between the surveyed ends. `None` for an end whose designator
    /// `Flightbag` rejects.
    fn end_specs(&self, height_m: impl Fn(&RunwayEndRecord) -> f64) -> [Option<RunwayEndSpec>; 2] {
        let [a, b] = &self.ends;
        let (bearing_ab, length) =
            bearing_and_distance((a.lat_deg, a.lon_deg), (b.lat_deg, b.lon_deg));
        let (bearing_ba, _) = bearing_and_distance((b.lat_deg, b.lon_deg), (a.lat_deg, a.lon_deg));
        let spec = |end: &RunwayEndRecord, bearing: f64| {
            Some(RunwayEndSpec {
                designator: RunwayDesignator::parse(end.designator.as_str()).ok()?,
                threshold: GeodeticPosition::wgs84(end.lat_deg, end.lon_deg, height_m(end)),
                true_bearing_rad: bearing,
                length_m: length,
                width_m: f64::from(self.width_m),
            })
        };
        [spec(a, bearing_ab), spec(b, bearing_ba)]
    }

    /// The 3D midpoint between the two ends on WGS84, degrees.
    fn midpoint_deg(&self) -> (f64, f64) {
        let [a, b] = &self.ends;
        let mid_ecef = 0.5
            * (geodetic_to_ecef(&Geodetic::from_degrees(a.lat_deg, a.lon_deg, 0.0))
                + geodetic_to_ecef(&Geodetic::from_degrees(b.lat_deg, b.lon_deg, 0.0)));
        let mid = ecef_to_geodetic(mid_ecef);
        (mid.lat_deg(), mid.lon_deg())
    }
}

impl AirportRecord {
    /// The built-in OpenStreetMap aerodrome this airport is, if any.
    #[must_use]
    pub fn osm_aerodrome(&self) -> Option<AerodromeId> {
        AerodromeId::ALL
            .into_iter()
            .find(|id| self.ids.codes().any(|code| code.as_str() == id.code()))
    }

    /// The aerodrome reference point at the field elevation, as an
    /// ellipsoidal height in a world read in `datum`: MSL plus the EGM96
    /// undulation in an orthometric world, the charted MSL number in a
    /// synthetic ellipsoidal one.
    #[must_use]
    pub fn arp_geodetic(&self, datum: CatalogVerticalDatum) -> Geodetic {
        let height = ellipsoidal_m(
            datum,
            self.lat_deg,
            self.lon_deg,
            f64::from(self.elevation_msl_m),
        );
        Geodetic::from_degrees(self.lat_deg, self.lon_deg, height)
    }

    /// The longest runway.
    #[must_use]
    pub fn primary_runway(&self) -> Option<&RunwayRecord> {
        self.runways
            .iter()
            .max_by(|a, b| a.length_m.total_cmp(&b.length_m))
    }

    /// A tower-cab or field camera viewpoint, as an ellipsoidal height in a
    /// world read in `datum`: the mapped tower of a built-in OpenStreetMap
    /// aerodrome, else a point abeam the primary runway's midpoint at the
    /// field elevation plus a cab height (38 m towered, 14 m CTAF).
    #[must_use]
    pub fn tower_viewpoint(&self, datum: CatalogVerticalDatum) -> Geodetic {
        if let Some(diagram) = self.osm_diagram() {
            return diagram.tower_viewpoint_for_datum(datum);
        }
        let (lat, lon) = self
            .primary_runway()
            .map_or((self.lat_deg, self.lon_deg), |runway| {
                let [_, b] = &runway.ends;
                let mid = runway.midpoint_deg();
                let (bearing, _) = bearing_and_distance(mid, (b.lat_deg, b.lon_deg));
                offset_deg(mid, bearing + FRAC_PI_2, TOWER_ABEAM_M)
            });
        let cab = if self.towered {
            TOWER_CAB_M.0
        } else {
            TOWER_CAB_M.1
        };
        let msl = f64::from(self.elevation_msl_m + cab);
        Geodetic::from_degrees(lat, lon, ellipsoidal_m(datum, lat, lon, msl))
    }

    /// The frequency to call first, megahertz: the tower when there is one,
    /// else the CTAF, else UNICOM.
    #[must_use]
    pub fn primary_frequency_mhz(&self) -> Option<(FrequencyRole, f64)> {
        [
            FrequencyRole::Tower,
            FrequencyRole::Ctaf,
            FrequencyRole::Unicom,
        ]
        .into_iter()
        .find_map(|role| {
            self.frequencies
                .iter()
                .find(|frequency| frequency.role == role)
                .map(|frequency| (role, frequency.mhz()))
        })
    }

    /// The runway strips an [`crate::AuthoritativeWorldSurface`] read in
    /// `datum` should hold, for
    /// [`crate::AuthoritativeWorldSurface::set_extra_runways`]. A surface
    /// reads its runway heights as charted — MSL in an orthometric world, the
    /// same number taken as ellipsoidal in a synthetic one — so every strip
    /// carries its surveyed MSL threshold elevations in either datum.
    #[must_use]
    pub fn runway_strips(&self, datum: CatalogVerticalDatum) -> Vec<RunwayStrip> {
        let charted = |end: &RunwayEndRecord| match datum {
            CatalogVerticalDatum::OrthometricMslEgm96 | CatalogVerticalDatum::EllipsoidWgs84 => {
                f64::from(end.elevation_msl_m)
            }
        };
        let ends: Vec<RunwayEndSpec> = self
            .runways
            .iter()
            .flat_map(|runway| runway.end_specs(charted))
            .flatten()
            .collect();
        RunwayStrip::from_ends(&ends)
    }

    /// The `Flightbag` surface graph with every height a true WGS84
    /// ellipsoidal height for a world read in `datum` (see
    /// [`Self::arp_geodetic`]). Runway ends are synthesized from the surveyed
    /// runway records, thresholds at the runway ends; a built-in
    /// OpenStreetMap aerodrome also brings its mapped taxiways, hold-shorts
    /// and stands.
    #[must_use]
    pub fn to_flightbag_graph(&self, datum: CatalogVerticalDatum) -> AirportSurfaceGraph {
        let runway_ends: Vec<RunwayEndSpec> = self
            .runways
            .iter()
            .flat_map(|runway| {
                runway.end_specs(|end| {
                    ellipsoidal_m(
                        datum,
                        end.lat_deg,
                        end.lon_deg,
                        f64::from(end.elevation_msl_m),
                    )
                })
            })
            .flatten()
            .collect();
        let arp = self.arp_geodetic(datum);
        let reference_point = GeodeticPosition::wgs84(self.lat_deg, self.lon_deg, arp.height_m);
        match self.osm_diagram() {
            Some(diagram) => AirportSurfaceGraph {
                reference_point,
                runway_ends,
                ..diagram.to_flightbag_graph_for_datum(datum)
            },
            None => AirportSurfaceGraph {
                facility: self.facility,
                reference_point,
                runway_ends,
                taxi_edges: Vec::new(),
                hold_short_nodes: Vec::new(),
                parking_spots: Vec::new(),
            },
        }
    }

    /// Surveyed 3D landing geometry for runway end `designator` (`"01"`, `"22L"`,
    /// `"07"`, `"02T"`), accounting for published [`RunwayEndRecord::displaced_threshold_m`],
    /// opposite-end elevation grade, and [`RunwayEndRecord::glidepath_deg`] in `datum`.
    #[must_use]
    pub fn landing_runway(
        &self,
        designator: &str,
        datum: CatalogVerticalDatum,
    ) -> Option<LandingRunwayGeometry> {
        let (runway, end, far) = self.find_runway_end(designator)?;
        let a_ecef = geodetic_to_ecef(&Geodetic::from_degrees(end.lat_deg, end.lon_deg, 0.0));
        let b_ecef = geodetic_to_ecef(&Geodetic::from_degrees(far.lat_deg, far.lon_deg, 0.0));
        let full_length_m = (b_ecef - a_ecef).length();
        let displaced_threshold_m = if end.displaced_threshold_m.is_finite() {
            f64::from(end.displaced_threshold_m).clamp(0.0, (full_length_m - 60.0).max(0.0))
        } else {
            0.0
        };
        let frac = if full_length_m > 1.0 {
            displaced_threshold_m / full_length_m
        } else {
            0.0
        };
        let thr_zero = ecef_to_geodetic(a_ecef + (b_ecef - a_ecef) * frac);
        let (thr_lat, thr_lon) = (thr_zero.lat_deg(), thr_zero.lon_deg());
        let start_elevation_msl_m = f64::from(end.elevation_msl_m);
        let far_elevation_msl_m = f64::from(far.elevation_msl_m);
        let threshold_elevation_msl_m =
            start_elevation_msl_m + (far_elevation_msl_m - start_elevation_msl_m) * frac;
        let (true_bearing_rad, landing_length_m) =
            bearing_and_distance((thr_lat, thr_lon), (far.lat_deg, far.lon_deg));
        let reciprocal_displaced_m = if far.displaced_threshold_m.is_finite() {
            f64::from(far.displaced_threshold_m).clamp(0.0, (landing_length_m - 60.0).max(0.0))
        } else {
            0.0
        };
        let gp_deg = f64::from(end.glidepath_deg);
        let glidepath_rad = if (0.5..=12.0).contains(&gp_deg) {
            gp_deg.to_radians()
        } else {
            DEFAULT_GLIDEPATH_DEG.to_radians()
        };
        let field_elevation_msl_m = if self.elevation_msl_m.is_finite() {
            f64::from(self.elevation_msl_m)
        } else {
            start_elevation_msl_m.max(far_elevation_msl_m)
        };
        Some(LandingRunwayGeometry {
            designator: end.designator,
            pavement_start: Geodetic::from_degrees(
                end.lat_deg,
                end.lon_deg,
                ellipsoidal_m(datum, end.lat_deg, end.lon_deg, start_elevation_msl_m),
            ),
            threshold: Geodetic::from_degrees(
                thr_lat,
                thr_lon,
                ellipsoidal_m(datum, thr_lat, thr_lon, threshold_elevation_msl_m),
            ),
            far_end: Geodetic::from_degrees(
                far.lat_deg,
                far.lon_deg,
                ellipsoidal_m(datum, far.lat_deg, far.lon_deg, far_elevation_msl_m),
            ),
            full_length_m,
            displaced_threshold_m,
            reciprocal_displaced_m,
            landing_length_m,
            width_m: f64::from(runway.width_m),
            true_bearing_rad,
            glidepath_rad,
            start_elevation_msl_m,
            threshold_elevation_msl_m,
            far_elevation_msl_m,
            field_elevation_msl_m,
            surface: runway.surface,
            authority: self.authority,
            effective_ymd: self.effective_ymd,
        })
    }

    fn find_runway_end<'a>(
        &'a self,
        designator: &str,
    ) -> Option<(&'a RunwayRecord, &'a RunwayEndRecord, &'a RunwayEndRecord)> {
        let target = designator.trim();
        let parsed_target = RunwayDesignator::parse(target).ok();
        let matches_end = |end: &RunwayEndRecord| {
            end.designator.as_str().eq_ignore_ascii_case(target)
                || RunwayDesignator::parse(end.designator.as_str())
                    .ok()
                    .zip(parsed_target)
                    .is_some_and(|(a, b)| a == b)
        };
        self.runways.iter().find_map(|rwy| {
            let [a, b] = &rwy.ends;
            if matches_end(a) {
                Some((rwy, a, b))
            } else if matches_end(b) {
                Some((rwy, b, a))
            } else {
                None
            }
        })
    }

    /// The built-in OpenStreetMap diagram of this airport, if it is one.
    pub(super) fn osm_diagram(&self) -> Option<OsmAerodromeDiagram> {
        OsmAerodromeDiagram::builtin(self.osm_aerodrome()?).ok()
    }
}
