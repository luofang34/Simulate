//! Host-reported selections and navigation guidance carried into Indicate's state.
//!
//! Selections are expressed against the same references the feed declares for
//! heading and altitude, because Indicate draws a bug only when its reference
//! matches the displayed one. Approach guidance models a localizer and glide
//! slope receiver over the selected runway, so the HSI deviation bar and
//! glideslope diamond move with the vehicle's true geometry.

use glam::DVec3;
use indicate_instrument_state::{
    AltitudeDeclaration, HeadingReference, HeadingSample, IdentStr, MagneticVariation, NavData,
    NavFacility, NavFromTo, NavScale, NavSequencing, NavSource, PositionIntegrity, PositionSource,
    Selections, Stamped, VariationSourceId,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use world_core::{DecimalYear, GeodeticPosition};
use world_geomag::wmm2025;

mod host;
#[cfg(test)]
mod tests;

pub(super) use host::host_nav;
pub use host::{AutopilotModeSummary, HostFlightGuidance, HostNavGuidance};

/// Localizer antenna distance past the far runway end, metres.
const LOCALIZER_BEYOND_STOP_END_M: f64 = 300.0;
/// Glide-path intercept distance past the landing threshold, metres.
const GLIDE_PATH_INTERCEPT_M: f64 = 300.0;
/// Localizer angle per dot (full scale two dots, `±2.5°`).
const LOCALIZER_DEG_PER_DOT: f64 = 1.25;
/// Glide slope angle per dot (`0.35°`, full scale `±0.7°` at two dots).
const GLIDE_SLOPE_DEG_PER_DOT: f64 = 0.35;
const METRES_PER_NM: f64 = 1_852.0;
/// Fixed WMM2025 evaluation epoch (`2025.0..=2030.0`), keeping replays deterministic.
const WMM2025_EPOCH: DecimalYear = DecimalYear(2026.0);
/// Horizontal geomagnetic intensity floor (`nT`) below which avionics revert to `TRU`.
const POLAR_HORIZONTAL_FLOOR_NT: f64 = 6_000.0;
/// Standard ARINC 424 / Airbus / Boeing northern magnetic-reference latitude limit (`82° N`).
const POLAR_NORTH_MAG_LIMIT_DEG: f64 = 82.0;
/// Standard ARINC 424 / Airbus / Boeing southern magnetic-reference latitude limit (`60° S`).
const POLAR_SOUTH_MAG_LIMIT_DEG: f64 = -60.0;
/// Source identifier stamped on WMM2025 magnetic-variation samples.
const WMM2025_VARIATION_SOURCE: VariationSourceId = VariationSourceId(1);

/// North reference convention [`super::StateFeed`] stamps on primary headings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HeadingMode {
    /// Simulator local true north (`SIM`), used off-Earth or in synthetic tests.
    #[default]
    SimLocalTrue,
    /// ICAO/ARINC magnetic north (`MAG`) from WMM2025 within `60° S..=82° N` where horizontal
    /// geomagnetic intensity is at least `6_000 nT`, reverting automatically to `TRU` in
    /// polar airspace (`> 82° N`, `< 60° S`, or `< 6_000 nT`).
    EarthMagnetic,
}

/// Resolves the primary heading sample and magnetic variation for `geodetic`.
pub(super) fn heading_and_variation(
    mode: HeadingMode,
    body: &BodyEllipsoid,
    geodetic: Geodetic,
    true_heading: Option<f64>,
) -> (Option<HeadingSample>, Option<MagneticVariation>) {
    if mode == HeadingMode::EarthMagnetic && *body == BodyEllipsoid::WGS84 {
        let lat_deg = geodetic.lat_rad.to_degrees();
        let position =
            GeodeticPosition::wgs84(lat_deg, geodetic.lon_rad.to_degrees(), geodetic.height_m);
        let in_mag_lat = (POLAR_SOUTH_MAG_LIMIT_DEG..=POLAR_NORTH_MAG_LIMIT_DEG).contains(&lat_deg);
        if in_mag_lat
            && let Ok(field) = wmm2025(position, WMM2025_EPOCH)
            && field.horizontal_nt >= POLAR_HORIZONTAL_FLOOR_NT
        {
            let decl_rad = field.declination_rad();
            let variation = MagneticVariation {
                east_positive_rad: decl_rad as f32,
                source: WMM2025_VARIATION_SOURCE,
            };
            let heading = true_heading.map(|tru| HeadingSample {
                heading_rad: (tru - decl_rad).rem_euclid(std::f64::consts::TAU) as f32,
                reference: HeadingReference::Magnetic,
            });
            return (heading, Some(variation));
        }
        let heading = true_heading.map(|tru| HeadingSample {
            heading_rad: tru as f32,
            reference: HeadingReference::True,
        });
        return (heading, None);
    }
    let heading = true_heading.map(|tru| HeadingSample {
        heading_rad: tru as f32,
        reference: HeadingReference::SimLocalTrue,
    });
    (heading, None)
}

/// What the pilot has set on the flight-guidance panel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotSelections {
    /// Heading bug, radians clockwise from the simulation's local true north.
    pub heading_rad: f64,
    /// Preselected altitude, metres, in the feed's displayed altitude reference.
    pub altitude_m: Option<f64>,
    /// Altimeter setting the pilot dialled in, hectopascals.
    pub baro_setting_hpa: Option<f64>,
}

/// A runway approach a navigation receiver is tuned to: a localizer course on the
/// runway centreline and a glide slope descending to the threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunwayNavGuidance {
    /// Landing threshold centre; height is above the feed body's ellipsoid.
    pub threshold: Geodetic,
    /// True inbound course along the centreline, radians.
    pub course_rad: f64,
    /// Runway length from the threshold to the far end, metres.
    pub length_m: f64,
    /// Glide path angle above the horizontal, radians.
    pub glide_path_rad: f64,
    /// Ident shown as the active waypoint.
    pub ident: IdentStr,
}

/// Indicate's selections for `selections`, matched to the declared altitude so the
/// altitude bug shows against the tape it was set on.
pub(super) fn indicate_selections(
    selections: &PilotSelections,
    altitude: &AltitudeDeclaration,
    air_setting_hpa: Option<f32>,
) -> Selections {
    Selections {
        heading_bug_rad: selections.heading_rad as f32,
        heading_bug_reference: HeadingReference::SimLocalTrue,
        altitude_sel_m: selections.altitude_m.map(|metres| metres as f32),
        altitude_sel_class: altitude.reference_class,
        altitude_sel_origin: altitude.origin,
        altitude_sel_model: altitude.geoid_model,
        baro_sel_hpa: selections
            .baro_setting_hpa
            .map(|hpa| hpa as f32)
            .or(air_setting_hpa),
    }
}

/// Localizer and glide slope deviations of `position_fixed_m` from `guidance`,
/// with the selected inbound course projected into ownship's local NED frame so
/// HSI course pointers stay aligned with ownship heading across `±90°`.
pub(super) fn runway_nav(
    guidance: &RunwayNavGuidance,
    body: &BodyEllipsoid,
    position_fixed_m: DVec3,
) -> Stamped<NavData> {
    let threshold_fixed = body.geodetic_to_fixed(&guidance.threshold);
    let thr_local = LocalFrame::at(&guidance.threshold);
    let offset = thr_local.ecef_to_ned_vector(position_fixed_m - threshold_fixed);
    let (sin_c, cos_c) = guidance.course_rad.sin_cos();
    let along_m = offset.x * cos_c + offset.y * sin_c;
    let right_m = -offset.x * sin_c + offset.y * cos_c;
    let height_m = -offset.z;

    let rwy_dir_ecef = thr_local.north() * cos_c + thr_local.east() * sin_c;
    let own_geodetic = body.fixed_to_geodetic(position_fixed_m);
    let own_ned = LocalFrame::at(&own_geodetic).ecef_to_ned_vector(rwy_dir_ecef);
    let course_rad = if own_ned.x.hypot(own_ned.y) > 1e-9 {
        own_ned.y.atan2(own_ned.x).rem_euclid(std::f64::consts::TAU) as f32
    } else {
        guidance.course_rad.rem_euclid(std::f64::consts::TAU) as f32
    };

    let to_localizer_m = guidance.length_m + LOCALIZER_BEYOND_STOP_END_M - along_m;
    let lateral_deg = right_m.atan2(to_localizer_m).to_degrees();
    // The bar shows where the course lies: right of the aircraft when it is left of it.
    let cdi_dots = (-lateral_deg / LOCALIZER_DEG_PER_DOT) as f32;

    let to_intercept_m = GLIDE_PATH_INTERCEPT_M - along_m;
    let vdev_dots = (to_intercept_m > 0.0).then(|| {
        let above_deg = height_m.atan2(to_intercept_m).to_degrees();
        ((above_deg - guidance.glide_path_rad.to_degrees()) / GLIDE_SLOPE_DEG_PER_DOT) as f32
    });
    let fromto = if along_m < guidance.length_m {
        NavFromTo::To
    } else {
        NavFromTo::From
    };
    Stamped {
        data: Some(NavData {
            source: NavSource::Nav1,
            course_rad,
            course_reference: HeadingReference::SimLocalTrue,
            cdi_dots,
            fromto,
            vdev_dots,
            dist_nm: Some((offset.length() / METRES_PER_NM) as f32),
            to_ident: guidance.ident,
            from_ident: IdentStr::EMPTY,
            scale: NavScale::Approach,
            facility: NavFacility::Localizer,
            sequencing: NavSequencing::Sequencing,
            // A receiver is its own sensor: no position source stands behind it.
            position_source: PositionSource::None,
            position_integrity: PositionIntegrity::Normal,
        }),
        age_ms: Some(0.0),
    }
}
