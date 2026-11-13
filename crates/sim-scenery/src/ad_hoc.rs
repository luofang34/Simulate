//! Runtime ad-hoc aerodrome and runway delineation from two 3D/map threshold points,
//! plus historic presets such as Mathias Rust's 1987 departure (`EFHF` Helsinki-Malmi)
//! and Red Square bridge landing (`ZMRS` Bolshoy Moskvoretsky Bridge).

mod presets;
#[cfg(test)]
mod tests;

pub use presets::MATHIAS_RUST_1987_WAYPOINTS;

use std::fmt;

use flightbag::{AuthoritySource, FacilityCode, ParkingKind, ParkingSpot};
use serde_json::Value;
use sim_geo::{Geodetic, ecef_to_geodetic, geodetic_to_ecef};
use world_core::GeodeticPosition;

use crate::airport_db::geometry::{bearing_and_distance, offset_deg};
use crate::airport_db::portrayal::append_record_runway_features;
use crate::{
    AerodromeRecord, AerodromeTier, AirportFrequency, AirportIds, AirportRecord,
    CatalogVerticalDatum, CompactCode, FrequencyRole, RunwayEndRecord, RunwayRecord, RunwayStrip,
    RunwaySurfaceKind, magnetic_heading_deg,
};

/// Minimum usable ad-hoc runway chord length, metres.
pub const MIN_AD_HOC_LENGTH_M: f64 = 120.0;
/// Maximum usable ad-hoc runway chord length, metres.
pub const MAX_AD_HOC_LENGTH_M: f64 = 12_000.0;
/// Minimum usable ad-hoc runway width, metres.
pub const MIN_AD_HOC_WIDTH_M: f64 = 6.0;
/// Maximum usable ad-hoc runway width, metres.
pub const MAX_AD_HOC_WIDTH_M: f64 = 200.0;
/// Maximum longitudinal slope (`3%`) allowed when grading an ad-hoc runway from terrain.
const MAX_GRADE: f64 = 0.03;
/// Midpoint river/gorge dip threshold below both abutments that triggers flat bridge grading.
const BRIDGE_DIP_THRESHOLD_M: f64 = 4.0;

/// Why an [`AdHocRunwaySpec`] could not be turned into a flyable aerodrome.
#[derive(Debug, Clone, PartialEq)]
pub enum AdHocRunwayError {
    /// Airport identifier is empty, longer than 8 ASCII characters, or lacks letters/digits.
    InvalidCode(String),
    /// One of the threshold coordinates or heights is non-finite or outside WGS84 bounds.
    NonFiniteCoordinate,
    /// Distance between Threshold A and Threshold B lies outside
    /// `[MIN_AD_HOC_LENGTH_M, MAX_AD_HOC_LENGTH_M]`.
    LengthOutOfRange {
        /// Measured chord length between the two thresholds, metres.
        length_m: f64,
    },
    /// Runway width lies outside `[MIN_AD_HOC_WIDTH_M, MAX_AD_HOC_WIDTH_M]`.
    WidthOutOfRange {
        /// Requested runway width, metres.
        width_m: f64,
    },
}

impl fmt::Display for AdHocRunwayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCode(code) => write!(f, "invalid ad-hoc aerodrome code {code:?}"),
            Self::NonFiniteCoordinate => {
                f.write_str("ad-hoc threshold coordinates must be finite WGS84 degrees")
            }
            Self::LengthOutOfRange { length_m } => write!(
                f,
                "ad-hoc runway length {length_m:.1} m outside [{MIN_AD_HOC_LENGTH_M:.0}, {MAX_AD_HOC_LENGTH_M:.0}] m"
            ),
            Self::WidthOutOfRange { width_m } => write!(
                f,
                "ad-hoc runway width {width_m:.1} m outside [{MIN_AD_HOC_WIDTH_M:.0}, {MAX_AD_HOC_WIDTH_M:.0}] m"
            ),
        }
    }
}

impl std::error::Error for AdHocRunwayError {}

/// Specification for delineating an ad-hoc runway between two WGS84 threshold points.
#[derive(Debug, Clone, PartialEq)]
pub struct AdHocRunwaySpec {
    /// Four-to-eight character identifier (for example `"ZMRS"` or `"USR1"`).
    pub code: String,
    /// Human-readable aerodrome / landing strip name.
    pub name: String,
    /// Nearest city, town, or landmark.
    pub municipality: String,
    /// ISO 3166-1 alpha-2 country bytes (`*b"RU"`, `*b"US"`, `*b"XX"`).
    pub country_iso2: [u8; 2],
    /// Region tag (`"RU-MOW"`, `"US-NJ"`, `"XX-ADHOC"`).
    pub region_code: String,
    /// First landing threshold `[lat_deg, lon_deg]`.
    pub threshold_a_deg: [f64; 2],
    /// Reciprocal landing threshold `[lat_deg, lon_deg]`.
    pub threshold_b_deg: [f64; 2],
    /// Runway width, metres.
    pub width_m: f32,
    /// Landing surface material (`Asphalt`, `Turf`, `Gravel`, ...).
    pub surface: RunwaySurfaceKind,
    /// Magnetic variation at the site, degrees, east positive.
    pub mag_var_deg: f32,
    /// Optional fixed MSL elevation override (for bridges or elevated pads), metres.
    pub override_elevation_msl_m: Option<f32>,
    /// Optional primary CTAF/Tower frequency, kilohertz (`122_800` for `122.800 MHz`).
    pub frequency_khz: Option<u32>,
    /// Whether the aerodrome is controlled by a tower (`false` -> CTAF).
    pub towered: bool,
    /// Whether edge lights are installed.
    pub lighted: bool,
}

impl AdHocRunwaySpec {
    /// Creates a default asphalt ad-hoc strip between `threshold_a_deg` and `threshold_b_deg`
    /// with an estimated dipole magnetic variation for the midpoint.
    #[must_use]
    pub fn between(
        code: impl Into<String>,
        name: impl Into<String>,
        threshold_a_deg: [f64; 2],
        threshold_b_deg: [f64; 2],
    ) -> Self {
        let [lat_a, lon_a] = threshold_a_deg;
        let [lat_b, lon_b] = threshold_b_deg;
        let mid_lat = 0.5 * (lat_a + lat_b);
        let mid_lon = 0.5 * (lon_a + lon_b);
        Self {
            code: code.into(),
            name: name.into(),
            municipality: "Ad-Hoc Strip".to_owned(),
            country_iso2: *b"XX",
            region_code: "XX-ADHOC".to_owned(),
            threshold_a_deg,
            threshold_b_deg,
            width_m: 24.0,
            surface: RunwaySurfaceKind::Asphalt,
            mag_var_deg: approx_magnetic_variation_deg(mid_lat, mid_lon),
            override_elevation_msl_m: None,
            frequency_khz: Some(122_800),
            towered: false,
            lighted: false,
        }
    }
}

/// Complete flyable ad-hoc aerodrome: database record, spawnable surface graph, graded
/// collision/DEM runway strips, and GeoJSON surface diagram.
#[derive(Debug, Clone, PartialEq)]
pub struct AdHocAerodrome {
    /// Airport database record supporting `landing_runway`, `to_flightbag_graph`, and search.
    pub airport: AirportRecord,
    /// Registry record with `RunwayThreshold`, `ShortFinal`, and `Ramp` spawn anchors.
    pub aerodrome: AerodromeRecord,
    /// Graded 3D [`RunwayStrip`]s for [`crate::RunwayDatum`] and
    /// [`crate::AuthoritativeWorldSurface`].
    pub strips: Vec<RunwayStrip>,
    /// GeoJSON `FeatureCollection` (`runway_pavement`/`runway_turf`, `runway_centerline`,
    /// `threshold_bar`) for EFB and 3D map portrayal.
    pub diagram_geojson: Value,
}

impl AdHocAerodrome {
    /// Builds an [`AdHocAerodrome`] from two threshold points, sampling terrain MSL heights
    /// with `sample_msl_m(lat_deg, lon_deg)` when [`AdHocRunwaySpec::override_elevation_msl_m`]
    /// is `None`.
    pub fn from_spec(
        spec: &AdHocRunwaySpec,
        mut sample_msl_m: impl FnMut(f64, f64) -> Option<f64>,
    ) -> Result<Self, AdHocRunwayError> {
        let [lat_a, lon_a] = spec.threshold_a_deg;
        let [lat_b, lon_b] = spec.threshold_b_deg;
        if !valid_lat_lon(lat_a, lon_a)
            || !valid_lat_lon(lat_b, lon_b)
            || !spec.mag_var_deg.is_finite()
        {
            return Err(AdHocRunwayError::NonFiniteCoordinate);
        }
        let width_m = f64::from(spec.width_m);
        if !width_m.is_finite() || !(MIN_AD_HOC_WIDTH_M..=MAX_AD_HOC_WIDTH_M).contains(&width_m) {
            return Err(AdHocRunwayError::WidthOutOfRange { width_m });
        }
        let (bearing_ab_rad, length_m) = bearing_and_distance((lat_a, lon_a), (lat_b, lon_b));
        if !length_m.is_finite() || !(MIN_AD_HOC_LENGTH_M..=MAX_AD_HOC_LENGTH_M).contains(&length_m)
        {
            return Err(AdHocRunwayError::LengthOutOfRange { length_m });
        }
        let (bearing_ba_rad, _) = bearing_and_distance((lat_b, lon_b), (lat_a, lon_a));
        let mid = midpoint_geodetic((lat_a, lon_a), (lat_b, lon_b));
        let (elev_a, elev_b, field_elev) = grade_elevations_msl(
            spec.override_elevation_msl_m,
            (lat_a, lon_a),
            mid,
            (lat_b, lon_b),
            length_m,
            &mut sample_msl_m,
        )?;
        let record = build_single_runway_record(
            spec,
            mid,
            (bearing_ab_rad, bearing_ba_rad, length_m),
            (elev_a, elev_b, field_elev),
        )?;
        Self::from_airport_record(record)
    }

    /// Wraps a complete [`AirportRecord`] (single- or multi-runway) as an [`AdHocAerodrome`].
    pub fn from_airport_record(mut airport: AirportRecord) -> Result<Self, AdHocRunwayError> {
        airport.classify();
        let mut graph = airport.to_flightbag_graph(CatalogVerticalDatum::EllipsoidWgs84);
        if graph.parking_spots.is_empty()
            && let Some(first_end) = graph.runway_ends.first()
        {
            let right_bearing = first_end.true_bearing_rad + std::f64::consts::FRAC_PI_2;
            let along = (
                first_end.threshold.latitude_deg,
                first_end.threshold.longitude_deg,
            );
            let moved_along = offset_deg(along, first_end.true_bearing_rad, 45.0);
            let stand_pos = offset_deg(moved_along, right_bearing, first_end.width_m + 20.0);
            graph.parking_spots.push(ParkingSpot {
                label: "RAMP 1".to_owned(),
                kind: ParkingKind::GeneralAviationRamp,
                position: GeodeticPosition::wgs84(
                    stand_pos.0,
                    stand_pos.1,
                    f64::from(airport.elevation_msl_m),
                ),
                heading_rad: first_end.true_bearing_rad,
                connected_node: None,
            });
        }
        let aerodrome = AerodromeRecord::from_surface_graph(
            graph,
            airport.name.to_string(),
            airport.region_code.as_str(),
            f64::from(airport.elevation_msl_m),
        );
        let strips = airport.runway_strips(CatalogVerticalDatum::OrthometricMslEgm96);
        let mut features = Vec::new();
        append_record_runway_features(&airport, &mut features);
        let diagram_geojson = serde_json::json!({
            "type": "FeatureCollection",
            "features": features,
        });
        Ok(Self {
            airport,
            aerodrome,
            strips,
            diagram_geojson,
        })
    }

    /// Primary code (`"ZMRS"`, `"EFHF"`, `"USR1"`).
    #[must_use]
    pub fn code(&self) -> &str {
        self.airport.ids.primary.as_str()
    }
}

/// Smooth dipole approximation of Earth's magnetic variation (degrees, east positive) for
/// sites that have no surveyed airport nearby.
#[must_use]
pub fn approx_magnetic_variation_deg(lat_deg: f64, lon_deg: f64) -> f32 {
    if !lat_deg.is_finite() || !lon_deg.is_finite() {
        return 0.0;
    }
    let pole_lat = 80.7_f64.to_radians();
    let pole_lon = (-72.7_f64).to_radians();
    let lat = lat_deg.clamp(-89.5, 89.5).to_radians();
    let d_lon = pole_lon - lon_deg.to_radians();
    let y = pole_lat.cos() * d_lon.sin();
    let x = lat.cos() * pole_lat.sin() - lat.sin() * pole_lat.cos() * d_lon.cos();
    (y.atan2(x).to_degrees().clamp(-45.0, 45.0)) as f32
}

fn valid_lat_lon(lat_deg: f64, lon_deg: f64) -> bool {
    lat_deg.is_finite()
        && lon_deg.is_finite()
        && (-90.0..=90.0).contains(&lat_deg)
        && (-180.0..=180.0).contains(&lon_deg)
}

fn midpoint_geodetic(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let mid_ecef = 0.5
        * (geodetic_to_ecef(&Geodetic::from_degrees(a.0, a.1, 0.0))
            + geodetic_to_ecef(&Geodetic::from_degrees(b.0, b.1, 0.0)));
    let mid = ecef_to_geodetic(mid_ecef);
    (mid.lat_deg(), mid.lon_deg())
}

fn grade_elevations_msl(
    override_msl_m: Option<f32>,
    a: (f64, f64),
    mid: (f64, f64),
    b: (f64, f64),
    length_m: f64,
    sample_msl_m: &mut dyn FnMut(f64, f64) -> Option<f64>,
) -> Result<(f32, f32, f32), AdHocRunwayError> {
    if let Some(fixed) = override_msl_m {
        if !fixed.is_finite() {
            return Err(AdHocRunwayError::NonFiniteCoordinate);
        }
        return Ok((fixed, fixed, fixed));
    }
    let sample = |pt: (f64, f64), sample_fn: &mut dyn FnMut(f64, f64) -> Option<f64>| {
        sample_fn(pt.0, pt.1).filter(|h| h.is_finite())
    };
    let h_a = sample(a, sample_msl_m);
    let h_mid = sample(mid, sample_msl_m);
    let h_b = sample(b, sample_msl_m);
    let fallback = h_a.or(h_b).or(h_mid).unwrap_or(0.0);
    let raw_a = h_a.unwrap_or(fallback);
    let raw_b = h_b.unwrap_or(fallback);
    // When the midpoint dips sharply below both thresholds (e.g. a river channel under a
    // road bridge), hold the strip flat across the abutment average so the deck never sags.
    if let Some(mid_h) = h_mid
        && mid_h < raw_a.min(raw_b) - BRIDGE_DIP_THRESHOLD_M
    {
        let deck = (0.5 * (raw_a + raw_b)) as f32;
        return Ok((deck, deck, deck));
    }
    let mean = 0.5 * (raw_a + raw_b);
    let max_half_delta = 0.5 * MAX_GRADE * length_m;
    let half_delta = (0.5 * (raw_b - raw_a)).clamp(-max_half_delta, max_half_delta);
    let elev_a = (mean - half_delta) as f32;
    let elev_b = (mean + half_delta) as f32;
    let field_elev = elev_a.max(elev_b);
    Ok((elev_a, elev_b, field_elev))
}

fn designator_pair(
    true_ab_deg: f32,
    true_ba_deg: f32,
    mag_var_deg: f32,
    polar_true: bool,
) -> Option<(CompactCode, CompactCode, CompactCode)> {
    let bucket = |deg: f32| {
        let rounded = ((deg.rem_euclid(360.0) / 10.0).round() as u32) % 36;
        if rounded == 0 { 36 } else { rounded }
    };
    let (hdg_a, hdg_b) = if polar_true {
        (true_ab_deg, true_ba_deg)
    } else {
        (
            magnetic_heading_deg(true_ab_deg, mag_var_deg),
            magnetic_heading_deg(true_ba_deg, mag_var_deg),
        )
    };
    let num_a = bucket(hdg_a);
    let mut num_b = bucket(hdg_b);
    if num_b == num_a {
        num_b = (num_a % 36).wrapping_add(1);
    }
    let suffix = if polar_true { "T" } else { "" };
    let desig_a = CompactCode::parse(&format!("{num_a:02}{suffix}"))?;
    let desig_b = CompactCode::parse(&format!("{num_b:02}{suffix}"))?;
    let rwy_id = CompactCode::joined(desig_a, desig_b)?;
    Some((rwy_id, desig_a, desig_b))
}

fn parse_ad_hoc_codes(
    code_str: &str,
    region_str: &str,
) -> Result<(AirportIds, FacilityCode, CompactCode), AdHocRunwayError> {
    let primary = CompactCode::parse(code_str)
        .ok_or_else(|| AdHocRunwayError::InvalidCode(code_str.to_owned()))?;
    let facility = crate::airport_db::facility_code_of(primary)
        .ok_or_else(|| AdHocRunwayError::InvalidCode(code_str.to_owned()))?;
    let ids = if primary.is_icao_like() {
        AirportIds::new(Some(primary), None, None)
    } else {
        AirportIds::new(None, Some(primary), None)
    }
    .ok_or_else(|| AdHocRunwayError::InvalidCode(code_str.to_owned()))?;
    let region_code = CompactCode::parse(region_str)
        .or_else(|| CompactCode::parse("XX-ADHOC"))
        .unwrap_or_default();
    Ok((ids, facility, region_code))
}

fn build_single_runway_record(
    spec: &AdHocRunwaySpec,
    mid: (f64, f64),
    (bearing_ab_rad, bearing_ba_rad, length_m): (f64, f64, f64),
    (elev_a, elev_b, field_elev): (f32, f32, f32),
) -> Result<AirportRecord, AdHocRunwayError> {
    let (ids, facility, region_code) = parse_ad_hoc_codes(&spec.code, &spec.region_code)?;
    let polar_true = mid.0.abs() >= 80.0;
    let true_ab_deg = bearing_ab_rad.to_degrees() as f32;
    let true_ba_deg = bearing_ba_rad.to_degrees() as f32;
    let (rwy_id, desig_a, desig_b) =
        designator_pair(true_ab_deg, true_ba_deg, spec.mag_var_deg, polar_true)
            .ok_or_else(|| AdHocRunwayError::InvalidCode(spec.code.clone()))?;
    let [lat_a, lon_a] = spec.threshold_a_deg;
    let [lat_b, lon_b] = spec.threshold_b_deg;
    let end = |designator, lat_deg, lon_deg, elevation_msl_m, true_heading_deg| RunwayEndRecord {
        designator,
        lat_deg,
        lon_deg,
        elevation_msl_m,
        displaced_threshold_m: 0.0,
        true_heading_deg,
        magnetic_heading_deg: magnetic_heading_deg(true_heading_deg, spec.mag_var_deg),
        glidepath_deg: 3.0,
        has_ils: false,
        has_vgsi: false,
        has_approach_lights: false,
    };
    let runway = RunwayRecord {
        id: rwy_id,
        length_m: length_m as f32,
        width_m: spec.width_m,
        surface: spec.surface,
        lighted: spec.lighted,
        ends: [
            end(desig_a, lat_a, lon_a, elev_a, true_ab_deg),
            end(desig_b, lat_b, lon_b, elev_b, true_ba_deg),
        ],
    };
    let frequencies = spec
        .frequency_khz
        .map(|khz| {
            Box::from([AirportFrequency {
                role: if spec.towered {
                    FrequencyRole::Tower
                } else {
                    FrequencyRole::Ctaf
                },
                khz,
            }])
        })
        .unwrap_or_default();
    Ok(AirportRecord {
        ids,
        facility,
        name: spec.name.clone().into_boxed_str(),
        municipality: spec.municipality.clone().into_boxed_str(),
        country_iso2: spec.country_iso2,
        region_code,
        lat_deg: mid.0,
        lon_deg: mid.1,
        elevation_msl_m: field_elev,
        mag_var_deg: spec.mag_var_deg,
        towered: spec.towered,
        tier: AerodromeTier::Tier3Local,
        longest_runway_m: runway.length_m,
        hard_surface: runway.surface.is_hard(),
        authority: AuthoritySource::OpenStreetMap,
        effective_ymd: [2026, 11, 12],
        runways: Box::from([runway]),
        frequencies,
    })
}
