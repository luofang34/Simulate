//! Global `AirportDatabase`-backed destination & approach runway resolver, conformal [`HudRunway`]
//! projector on `maplibre`'s render sphere, and surveyed airport runway GeoJSON generator.

use crate::planet::PlanetaryReference;
use crate::telemetry::OwnshipState;
use crate::terrain::{RunwayOverlay3D, build_hits_gates};
use indicate_instrument_hud::Runway as HudRunway;
use indicate_instrument_state::SignalStatus;
use sim_geo::Geodetic;
use sim_scenery::{AirportDatabase, AirportRecord, CatalogVerticalDatum, LandingRunwayGeometry};

/// Active landing/approach runway resolved from `sim_scenery::AirportDatabase::builtin()`.
#[derive(Clone, Debug, PartialEq)]
pub struct ApproachRunwaySelection {
    /// Primary airport code (`"N51"`, `"KEWR"`, `"47N"`).
    pub airport_code: String,
    /// Runway-end designator (`"22"`, `"04"`, `"22L"`).
    pub designator: String,
    /// Surveyed 3D landing runway geometry.
    pub landing: LandingRunwayGeometry,
    /// Conformal HUD runway projected from the active camera eye on `planet.render_radius_m`.
    pub hud_runway: HudRunway,
    /// Horizontal distance from ownship to the landing threshold (`m`).
    pub distance_to_threshold_m: f64,
    /// Signed lateral cross-track offset (`m`, right of course positive).
    pub cross_track_m: f64,
    /// Approach lateral CDI needle deflection in dots (`[-2.4, 2.4]`, positive = course is right).
    pub lateral_cdi_dots: f32,
    /// Approach vertical glidepath needle deflection in dots (`[-2.4, 2.4]`, positive = path is below).
    pub vertical_dev_dots: f32,
}

/// Camera eye coordinates `(lat_deg, lon_deg, alt_datum_m)` for conformal [`HudRunway`] projection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunwayEyeAnchor {
    /// Planetographic latitude (`deg`).
    pub latitude_deg: f64,
    /// Planetographic longitude (`deg`).
    pub longitude_deg: f64,
    /// Altitude above `planet`'s declared vertical datum (`m`).
    pub altitude_datum_m: f64,
    /// Optional onboard DEM ground sample (`m` above datum) near the runway for DO-315B gating.
    pub onboard_dem_datum_m: Option<f64>,
}

/// Extracts candidate airport identifiers from `route_label` (`"N51 → N51"`, `"N51 - KEWR"`)
/// and `ownship.route.waypoints`, destination-last so the arrival airport is prioritized first.
#[must_use]
pub fn extract_route_airport_codes(
    ownship: &OwnshipState,
    route_label: Option<&str>,
) -> Vec<String> {
    let mut codes = Vec::new();
    let mut push_unique = |raw: &str| {
        let token: String = raw
            .trim()
            .chars()
            .filter(|c| c.is_ascii_alphanumeric())
            .map(|c| c.to_ascii_uppercase())
            .collect();
        if (2..=5).contains(&token.len()) && !codes.iter().any(|c| c == &token) {
            codes.push(token);
        }
    };
    if let Some(label) = route_label {
        let parts: Vec<&str> = label
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|s| !s.is_empty())
            .collect();
        for part in parts.into_iter().rev() {
            push_unique(part);
        }
    }
    for wp in ownship.route.waypoints.iter().rev() {
        push_unique(&wp.ident);
    }
    codes
}

/// Resolves nearby 3D [`RunwayOverlay3D`] strips and the best active [`ApproachRunwaySelection`]
/// from `AirportDatabase::builtin()` on `WGS-84` Earth (`empty` / `None` on non-Earth bodies).
#[must_use]
pub fn resolve_approach_and_overlays(
    ownship: &OwnshipState,
    planet: &PlanetaryReference,
    own_flat: &Geodetic,
    own_alt_datum_m: f64,
    eye: RunwayEyeAnchor,
    route_label: Option<&str>,
) -> (Vec<RunwayOverlay3D>, Option<ApproachRunwaySelection>) {
    let Some(datum) = planet.catalog_vertical_datum() else {
        return (Vec::new(), None);
    };
    let db = AirportDatabase::builtin();
    let own_lat = own_flat.lat_rad.to_degrees();
    let own_lon = own_flat.lon_rad.to_degrees();
    let route_codes = extract_route_airport_codes(ownship, route_label);
    let candidates = gather_candidate_airports(db, own_lat, own_lon, &route_codes);
    let own_track_rad = ownship
        .ground_track_true_deg
        .or(ownship.true_heading_deg)
        .map(f64::to_radians);

    let mut overlays = Vec::new();
    let mut best: Option<(f64, ApproachRunwaySelection)> = None;

    for (is_dest, airport) in candidates {
        let airport_has_paved = airport.runways.iter().any(|r| !r.surface.is_unpaved());
        for rwy_rec in &airport.runways {
            if rwy_rec.surface.is_unpaved() && (airport_has_paved || !is_dest) {
                continue;
            }
            for end_rec in &rwy_rec.ends {
                let Some(raw_geom) = airport.landing_runway(end_rec.designator.as_str(), datum)
                else {
                    continue;
                };
                let geom = refine_with_dem(&raw_geom, datum, eye.onboard_dem_datum_m);
                let thr_flat = planet.geodetic_from_datum(
                    geom.threshold.lat_deg(),
                    geom.threshold.lon_deg(),
                    0.0,
                );
                let thr_enu = planet.enu_between(own_flat, &thr_flat);
                let horiz_d = libm::hypot(thr_enu.x, thr_enu.y);
                if horiz_d > 34_000.0 {
                    continue;
                }
                if overlays.len() < 12 && (!geom.surface.is_unpaved() || is_dest) {
                    overlays.push(build_overlay_3d(
                        airport,
                        &geom,
                        thr_enu.x,
                        thr_enu.y,
                        own_alt_datum_m,
                    ));
                }
                if let Some((score, sel)) = score_and_build_selection(
                    airport,
                    &geom,
                    is_dest,
                    thr_enu.x,
                    thr_enu.y,
                    own_alt_datum_m,
                    own_track_rad,
                    planet,
                    eye,
                ) && best.as_ref().is_none_or(|(b, _)| score < *b)
                {
                    best = Some((score, sel));
                }
            }
        }
    }
    (overlays, best.map(|(_, sel)| sel))
}

fn gather_candidate_airports<'a>(
    db: &'a AirportDatabase,
    own_lat: f64,
    own_lon: f64,
    route_codes: &[String],
) -> Vec<(bool, &'a AirportRecord)> {
    let mut out: Vec<(bool, &'a AirportRecord)> = Vec::new();
    let mut push_record = |is_dest: bool, rec: &'a AirportRecord| {
        if !out
            .iter()
            .any(|(_, existing)| existing.ids.primary == rec.ids.primary)
        {
            out.push((is_dest, rec));
        }
    };
    for code in route_codes {
        if let Some((_, rec)) = db.by_code(code) {
            push_record(true, rec);
        }
        if code.len() == 3 {
            let k_code = format!("K{code}");
            if let Some((_, rec)) = db.by_code(&k_code) {
                push_record(true, rec);
            }
        }
    }
    for idx in db.query_globe_visible(own_lat, own_lon, 11.2, 20) {
        if let Some(rec) = db.get(idx) {
            push_record(false, rec);
        }
    }
    out
}

fn refine_with_dem(
    geom: &LandingRunwayGeometry,
    datum: CatalogVerticalDatum,
    dem_datum_m: Option<f64>,
) -> LandingRunwayGeometry {
    let Some(dem_m) = dem_datum_m.filter(|v| v.is_finite()) else {
        return *geom;
    };
    geom.with_onboard_dem(datum, |_, _| Some(dem_m))
}

fn build_overlay_3d(
    airport: &AirportRecord,
    geom: &LandingRunwayGeometry,
    thr_east_m: f64,
    thr_north_m: f64,
    own_alt_datum_m: f64,
) -> RunwayOverlay3D {
    let up_m = (geom.threshold_elevation_msl_m - own_alt_datum_m) as f32 + 0.5;
    let e0 = thr_east_m as f32;
    let n0 = thr_north_m as f32;
    let dir_e = libm::sin(geom.true_bearing_rad) as f32;
    let dir_n = libm::cos(geom.true_bearing_rad) as f32;
    let right_e = dir_n;
    let right_n = -dir_e;
    let half_w = (geom.width_m as f32 * 0.5).max(10.0);
    let len = geom.landing_length_m as f32;
    let e1 = e0 + dir_e * len;
    let n1 = n0 + dir_n * len;
    RunwayOverlay3D {
        label: format!("{} RWY {}", airport.ids.primary, geom.designator),
        corners_enu_m: [
            [e0 - right_e * half_w, n0 - right_n * half_w, up_m],
            [e0 + right_e * half_w, n0 + right_n * half_w, up_m],
            [e1 + right_e * half_w, n1 + right_n * half_w, up_m],
            [e1 - right_e * half_w, n1 - right_n * half_w, up_m],
        ],
        centerline_enu_m: [
            [e0, n0, up_m],
            [e0 - dir_e * 6_500.0, n0 - dir_n * 6_500.0, up_m + 340.0],
        ],
        glideslope_gates_enu_m: build_hits_gates(
            [e0, n0, up_m],
            [dir_e, dir_n],
            [right_e, right_n],
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn score_and_build_selection(
    airport: &AirportRecord,
    geom: &LandingRunwayGeometry,
    is_dest: bool,
    thr_east_m: f64,
    thr_north_m: f64,
    own_alt_datum_m: f64,
    own_track_rad: Option<f64>,
    planet: &PlanetaryReference,
    eye: RunwayEyeAnchor,
) -> Option<(f64, ApproachRunwaySelection)> {
    let hud_runway = project_hud_runway_from_eye(planet, eye, geom)?;
    let (sin_b, cos_b) = geom.true_bearing_rad.sin_cos();
    // Vector from threshold to ownship in ENU: `(-thr_east_m, -thr_north_m)`.
    let own_from_thr_e = -thr_east_m;
    let own_from_thr_n = -thr_north_m;
    let along_m = own_from_thr_e * sin_b + own_from_thr_n * cos_b;
    let xtk_m = own_from_thr_e * cos_b - own_from_thr_n * sin_b;
    let dist_m = libm::hypot(thr_east_m, thr_north_m);

    let heading_err_deg = own_track_rad.map_or(0.0, |trk| {
        let diff = (trk - geom.true_bearing_rad + core::f64::consts::PI)
            .rem_euclid(core::f64::consts::TAU)
            - core::f64::consts::PI;
        diff.abs().to_degrees()
    });
    let on_approach_side = along_m <= geom.landing_length_m + 400.0 && along_m >= -22_000.0;
    let mut cost = dist_m + heading_err_deg * 130.0 + xtk_m.abs() * 1.8;
    if is_dest {
        cost -= 18_000.0;
    }
    if on_approach_side && heading_err_deg <= 48.0 {
        cost -= 12_000.0;
    } else if !on_approach_side {
        cost += 14_000.0;
    }
    if geom.surface.is_unpaved() {
        cost += 4_500.0;
    }

    // LOC/LNAV-style lateral sensitivity (±2.0° full-scale = ±2.0 dots, min ±45 m at threshold).
    let half_scale_m = (dist_m * libm::tan(2.0_f64.to_radians())).max(45.0);
    let lateral_cdi_dots = ((-xtk_m / half_scale_m) * 2.0).clamp(-2.4, 2.4) as f32;

    // GS/VNAV-style vertical sensitivity relative to `threshold + 15 m` (50 ft TCH) along approach.
    let approach_range_m = (-along_m).max(80.0);
    let target_alt_m =
        geom.threshold_elevation_msl_m + 15.0 + approach_range_m * libm::tan(geom.glidepath_rad);
    let alt_err_deg = libm::atan2(target_alt_m - own_alt_datum_m, approach_range_m).to_degrees();
    let vertical_dev_dots = ((alt_err_deg / 0.7) * 2.0).clamp(-2.4, 2.4) as f32;

    Some((
        cost,
        ApproachRunwaySelection {
            airport_code: airport.ids.primary.to_string(),
            designator: geom.designator.to_string(),
            landing: *geom,
            hud_runway,
            distance_to_threshold_m: dist_m,
            cross_track_m: xtk_m,
            lateral_cdi_dots,
            vertical_dev_dots,
        },
    ))
}

/// Projects `geom` into [`HudRunway`] from `eye` on `planet.render_radius_m` so the conformal HUD
/// outline aligns pixel-for-pixel with `maplibre`'s spherical globe terrain.
#[must_use]
pub fn project_hud_runway_from_eye(
    planet: &PlanetaryReference,
    eye: RunwayEyeAnchor,
    geom: &LandingRunwayGeometry,
) -> Option<HudRunway> {
    let start_ned = planet.render_globe_ned_between(
        eye.latitude_deg,
        eye.longitude_deg,
        eye.altitude_datum_m,
        geom.pavement_start.lat_deg(),
        geom.pavement_start.lon_deg(),
        geom.start_elevation_msl_m,
    )?;
    let thr_ned = planet.render_globe_ned_between(
        eye.latitude_deg,
        eye.longitude_deg,
        eye.altitude_datum_m,
        geom.threshold.lat_deg(),
        geom.threshold.lon_deg(),
        geom.threshold_elevation_msl_m,
    )?;
    let far_ned = planet.render_globe_ned_between(
        eye.latitude_deg,
        eye.longitude_deg,
        eye.altitude_datum_m,
        geom.far_end.lat_deg(),
        geom.far_end.lon_deg(),
        geom.far_elevation_msl_m,
    )?;
    let dn = far_ned.x - thr_ned.x;
    let de = far_ned.y - thr_ned.y;
    let horiz = libm::hypot(dn, de);
    let scale = if geom.landing_length_m > 1.0 && horiz > 1.0 {
        horiz / geom.landing_length_m
    } else {
        1.0
    };
    let pre_horiz = libm::hypot(thr_ned.x - start_ned.x, thr_ned.y - start_ned.y);
    let pre_threshold_m = if geom.displaced_threshold_m <= 0.0 {
        0.0
    } else if pre_horiz > 0.1 {
        pre_horiz
    } else {
        geom.displaced_threshold_m * scale
    };
    let (course_rad, length_m) = if horiz > 1.0 {
        (
            libm::atan2(de, dn).rem_euclid(core::f64::consts::TAU) as f32,
            horiz as f32,
        )
    } else {
        (geom.true_bearing_rad as f32, geom.landing_length_m as f32)
    };
    Some(HudRunway {
        threshold_m: [thr_ned.x as f32, thr_ned.y as f32, thr_ned.z as f32],
        far_down_offset_m: (far_ned.z - thr_ned.z) as f32,
        pre_threshold_m: pre_threshold_m.max(0.0) as f32,
        reciprocal_displaced_m: (geom.reciprocal_displaced_m.max(0.0) * scale) as f32,
        unpaved: geom.surface.is_unpaved(),
        course_rad,
        length_m: length_m.clamp(80.0, 6_000.0),
        width_m: (geom.width_m as f32).clamp(8.0, 120.0),
        glide_path_rad: geom.glidepath_rad as f32,
        status: SignalStatus::Valid,
    })
}

pub use super::runways_geojson::{build_airport_runways_geojson, lookup_airport_json};
