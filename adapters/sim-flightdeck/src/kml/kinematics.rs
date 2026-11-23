//! Smoothed local ENU track velocity, vertical rate, flight-path pitch, and coordinated turn bank
//! on an explicitly declared [`PlanetaryReference`] (`sim-geo` ellipsoid + `J2` gravity).

use super::KmlTrackPoint;
use super::parser::RawSample;
use crate::planet::PlanetaryReference;
use crate::telemetry::{KNOTS_TO_MPS, RouteWaypoint};
use sim_scenery::{AerodromeTier, AirportDatabase, AirportRecord, CatalogVerticalDatum};

const MPS_TO_KNOTS: f64 = 1.0 / KNOTS_TO_MPS;
const MPS_TO_FPM: f64 = 60.0 / 0.3048;
const NOMINAL_CRUISE_SPEED_MPS: f64 = 54.0;

pub(super) fn normalize_sample_offsets(
    samples: &[RawSample],
    planet: &PlanetaryReference,
) -> Vec<(f64, RawSample)> {
    let mut out: Vec<(f64, RawSample)> = Vec::with_capacity(samples.len());
    let base_t = samples.iter().find_map(|s| s.timestamp_s);
    for &s in samples {
        let offset = match (base_t, s.timestamp_s, out.last().copied()) {
            (Some(t0), Some(ts), Some((prev_off, _))) => {
                let candidate = (ts - t0).max(0.0);
                if candidate <= prev_off {
                    continue;
                }
                candidate
            }
            (Some(t0), Some(ts), None) => (ts - t0).max(0.0),
            (_, _, Some((prev_off, prev_s))) => {
                let dist = enu_delta(&prev_s, &s, planet);
                let step_m = libm::hypot(dist.0, dist.1);
                prev_off + (step_m / NOMINAL_CRUISE_SPEED_MPS).max(1.0)
            }
            (_, _, None) => 0.0,
        };
        out.push((offset, s));
    }
    out
}

pub(super) fn build_smoothed_points(
    timed: &[(f64, RawSample)],
    planet: &PlanetaryReference,
) -> Vec<KmlTrackPoint> {
    let n = timed.len();
    let mut raw_vel = Vec::with_capacity(n);
    for i in 0..n {
        raw_vel.push(vertex_velocity_enu(timed, i, planet));
    }
    let pass1 = smooth_vel_pass(&raw_vel);
    let vel = smooth_vel_pass(&pass1);

    let mut last_track_deg = 0.0_f64;
    let mut tracks_deg = Vec::with_capacity(n);
    for &(ve, vn, _) in &vel {
        if libm::hypot(ve, vn) > 1.0 {
            last_track_deg = libm::atan2(ve, vn).to_degrees().rem_euclid(360.0);
        }
        tracks_deg.push(last_track_deg);
    }

    let mut out = Vec::with_capacity(n);
    for (i, &(offset_s, sample)) in timed.iter().enumerate() {
        let &(ve, vn, vu) = vel.get(i).unwrap_or(&(0.0, 0.0, 0.0));
        let horiz_mps = libm::hypot(ve, vn);
        let track_deg = *tracks_deg.get(i).unwrap_or(&0.0);
        let omega = turn_rate_rad_s(timed, &tracks_deg, i);
        let speed_gate = ((horiz_mps - 6.0) / 10.0).clamp(0.0, 1.0);
        let sample_geo = planet.geodetic_from_datum(
            sample.latitude_deg,
            sample.longitude_deg,
            sample.altitude_msl_m,
        );
        let g_local = planet.gravity_mps2(&sample_geo);
        let roll_deg =
            (libm::atan2(horiz_mps * omega, g_local).to_degrees() * speed_gate).clamp(-35.0, 35.0);
        let gamma_deg = if horiz_mps > 3.0 {
            libm::atan2(vu, horiz_mps).to_degrees()
        } else {
            0.0
        };
        let pitch_deg = (gamma_deg + 2.0 * speed_gate).clamp(-12.0, 18.0);
        out.push(KmlTrackPoint {
            offset_s,
            latitude_deg: sample.latitude_deg,
            longitude_deg: sample.longitude_deg,
            altitude_msl_m: sample.altitude_msl_m,
            ground_speed_kt: horiz_mps * MPS_TO_KNOTS,
            ground_track_true_deg: track_deg,
            vertical_speed_fpm: vu * MPS_TO_FPM,
            pitch_deg,
            roll_deg,
            east_mps: ve,
            north_mps: vn,
            up_mps: vu,
        });
    }
    out
}

fn vertex_velocity_enu(
    timed: &[(f64, RawSample)],
    idx: usize,
    planet: &PlanetaryReference,
) -> (f64, f64, f64) {
    let Some(&(t_cur, s_cur)) = timed.get(idx) else {
        return (0.0, 0.0, 0.0);
    };
    let prev = idx.checked_sub(1).and_then(|p| timed.get(p)).copied();
    let next = timed.get(idx.wrapping_add(1)).copied();
    match (prev, next) {
        (Some((t_p, s_p)), Some((t_n, s_n))) => {
            let dt_p = (t_cur - t_p).max(0.25);
            let dt_n = (t_n - t_cur).max(0.25);
            let dp = enu_delta(&s_p, &s_cur, planet);
            let dn = enu_delta(&s_cur, &s_n, planet);
            let w_sum = dt_p + dt_n;
            (
                ((dp.0 / dt_p) * dt_n + (dn.0 / dt_n) * dt_p) / w_sum,
                ((dp.1 / dt_p) * dt_n + (dn.1 / dt_n) * dt_p) / w_sum,
                ((dp.2 / dt_p) * dt_n + (dn.2 / dt_n) * dt_p) / w_sum,
            )
        }
        (None, Some((t_n, s_n))) => {
            let dt = (t_n - t_cur).max(0.25);
            let d = enu_delta(&s_cur, &s_n, planet);
            (d.0 / dt, d.1 / dt, d.2 / dt)
        }
        (Some((t_p, s_p)), None) => {
            let dt = (t_cur - t_p).max(0.25);
            let d = enu_delta(&s_p, &s_cur, planet);
            (d.0 / dt, d.1 / dt, d.2 / dt)
        }
        (None, None) => (0.0, 0.0, 0.0),
    }
}

fn smooth_vel_pass(vel: &[(f64, f64, f64)]) -> Vec<(f64, f64, f64)> {
    let mut out = Vec::with_capacity(vel.len());
    for (i, &cur) in vel.iter().enumerate() {
        let prev = i.checked_sub(1).and_then(|p| vel.get(p)).copied();
        let next = vel.get(i.wrapping_add(1)).copied();
        let smoothed = match (prev, next) {
            (Some(a), Some(b)) => (
                0.25 * a.0 + 0.5 * cur.0 + 0.25 * b.0,
                0.25 * a.1 + 0.5 * cur.1 + 0.25 * b.1,
                0.25 * a.2 + 0.5 * cur.2 + 0.25 * b.2,
            ),
            _ => cur,
        };
        out.push(smoothed);
    }
    out
}

fn turn_rate_rad_s(timed: &[(f64, RawSample)], tracks_deg: &[f64], idx: usize) -> f64 {
    let i_prev = idx.saturating_sub(1);
    let i_next = idx.wrapping_add(1).min(timed.len().saturating_sub(1));
    let (Some(&(t0, _)), Some(&(t1, _)), Some(&trk0), Some(&trk1)) = (
        timed.get(i_prev),
        timed.get(i_next),
        tracks_deg.get(i_prev),
        tracks_deg.get(i_next),
    ) else {
        return 0.0;
    };
    let dt = t1 - t0;
    if dt < 0.5 {
        return 0.0;
    }
    let d_deg = (trk1 - trk0 + 540.0).rem_euclid(360.0) - 180.0;
    d_deg.to_radians() / dt
}

fn enu_delta(from: &RawSample, to: &RawSample, planet: &PlanetaryReference) -> (f64, f64, f64) {
    let a = planet.geodetic_from_datum(from.latitude_deg, from.longitude_deg, from.altitude_msl_m);
    let b = planet.geodetic_from_datum(to.latitude_deg, to.longitude_deg, to.altitude_msl_m);
    let enu = planet.enu_between(&a, &b);
    (enu.x, enu.y, to.altitude_msl_m - from.altitude_msl_m)
}

/// Calibrates a single constant barometric QNH offset when a Mode-C pressure-altitude KML track
/// on `WGS-84` Earth starts/ends near a surveyed `AirportDatabase` aerodrome below physical
/// runway/approach elevation (leaving GNSS geometric-altitude tracks and derivatives untouched).
pub(super) fn calibrate_baro_qnh_offset(
    points: &mut [KmlTrackPoint],
    waypoints: &[RouteWaypoint],
    route_label: &str,
    planet: &PlanetaryReference,
) {
    let Some(datum) = planet.catalog_vertical_datum() else {
        return;
    };
    let (Some(&first), Some(&last)) = (points.first(), points.last()) else {
        return;
    };
    let db = AirportDatabase::builtin();
    let dest_rec = resolve_endpoint_airport(
        db,
        last.latitude_deg,
        last.longitude_deg,
        waypoints,
        route_label,
        true,
    );
    let orig_rec = resolve_endpoint_airport(
        db,
        first.latitude_deg,
        first.longitude_deg,
        waypoints,
        route_label,
        false,
    );

    let mut offset_m = 0.0_f64;
    if let Some(dest) = dest_rec {
        offset_m = offset_m.max(arrival_qnh_deficit_m(dest, &last, datum, planet));
    }
    if let Some(orig) = orig_rec {
        let orig_arp = planet.geodetic_from_datum(orig.lat_deg, orig.lon_deg, 0.0);
        let first_geo = planet.geodetic_from_datum(first.latitude_deg, first.longitude_deg, 0.0);
        if planet.enu_between(&orig_arp, &first_geo).length() <= 3_800.0 {
            let min_dep_m = f64::from(orig.elevation_msl_m) + 4.0;
            offset_m = offset_m.max(min_dep_m - first.altitude_msl_m);
        }
    }
    let clamped = offset_m.clamp(0.0, 420.0);
    if clamped >= 8.0 {
        for pt in points.iter_mut() {
            pt.altitude_msl_m += clamped;
        }
    }
}

fn resolve_endpoint_airport<'a>(
    db: &'a AirportDatabase,
    lat_deg: f64,
    lon_deg: f64,
    waypoints: &[RouteWaypoint],
    route_label: &str,
    is_arrival: bool,
) -> Option<&'a AirportRecord> {
    let wp = if is_arrival {
        waypoints.last()
    } else {
        waypoints.first()
    };
    if let Some(w) = wp
        && let Some((_, rec)) = db.by_code(&w.ident)
    {
        return Some(rec);
    }
    let tokens: Vec<&str> = route_label
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|s| (2..=5).contains(&s.len()))
        .collect();
    let code = if is_arrival {
        tokens.last().copied()
    } else {
        tokens.first().copied()
    };
    if let Some(c) = code
        && let Some((_, rec)) = db.by_code(c)
    {
        return Some(rec);
    }
    db.nearest(lat_deg, lon_deg, 4.5, AerodromeTier::Tier4Minor)
        .and_then(|idx| db.get(idx))
}

fn arrival_qnh_deficit_m(
    airport: &AirportRecord,
    last: &KmlTrackPoint,
    datum: CatalogVerticalDatum,
    planet: &PlanetaryReference,
) -> f64 {
    let arp_flat = planet.geodetic_from_datum(airport.lat_deg, airport.lon_deg, 0.0);
    let last_flat = planet.geodetic_from_datum(last.latitude_deg, last.longitude_deg, 0.0);
    if planet.enu_between(&arp_flat, &last_flat).length() > 4_800.0 {
        return 0.0;
    }
    let horiz_mps = libm::hypot(last.east_mps, last.north_mps);
    if last.ground_speed_kt >= 30.0 && last.up_mps < -0.25 && horiz_mps > 10.0 {
        let trk_rad = last.ground_track_true_deg.to_radians();
        for rwy in &airport.runways {
            for end in &rwy.ends {
                let Some(geom) = airport.landing_runway(end.designator.as_str(), datum) else {
                    continue;
                };
                let diff = (trk_rad - geom.true_bearing_rad + core::f64::consts::PI)
                    .rem_euclid(core::f64::consts::TAU)
                    - core::f64::consts::PI;
                if diff.abs().to_degrees() > 35.0 {
                    continue;
                }
                let thr_flat = planet.geodetic_from_datum(
                    geom.threshold.lat_deg(),
                    geom.threshold.lon_deg(),
                    0.0,
                );
                let to_last = planet.enu_between(&thr_flat, &last_flat);
                let (sb, cb) = geom.true_bearing_rad.sin_cos();
                let along_m = to_last.x * sb + to_last.y * cb;
                if (-4_500.0..=150.0).contains(&along_m) {
                    let slope = (-last.up_mps / horiz_mps).clamp(0.03, 0.075);
                    let expected_m = geom.threshold_elevation_msl_m + ((-along_m) + 180.0) * slope;
                    return (expected_m - last.altitude_msl_m).max(0.0);
                }
            }
        }
    }
    (f64::from(airport.elevation_msl_m) - last.altitude_msl_m).max(0.0)
}
