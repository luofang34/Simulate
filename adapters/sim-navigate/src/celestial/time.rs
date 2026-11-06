use glam::DVec3;

use super::{ReferenceBody, SkyTimeMode};

const SECONDS_PER_DAY: f64 = 86_400.0;
/// Mean solar time advances one hour per 15° of east longitude.
const SECONDS_PER_LONGITUDE_DEG: f64 = 240.0;

/// The UTC instant, Unix seconds, that `mode` selects.
pub(super) fn utc_for_mode(
    mode: SkyTimeMode,
    body: ReferenceBody,
    observer_fixed_m: DVec3,
    sim_elapsed_s: f64,
    scenario_base_utc_s: Option<f64>,
    live_wall_utc_s: f64,
) -> f64 {
    let local_seconds_of_day = match mode {
        SkyTimeMode::Auto => {
            return scenario_base_utc_s.map_or(live_wall_utc_s, |base| base + sim_elapsed_s);
        }
        SkyTimeMode::LiveUtc => return live_wall_utc_s,
        SkyTimeMode::Dawn => hours_minutes(6, 5),
        SkyTimeMode::Noon => hours_minutes(12, 0),
        SkyTimeMode::Dusk => hours_minutes(18, 10),
        SkyTimeMode::Night => hours_minutes(23, 30),
    };
    let longitude_deg = body
        .ellipsoid()
        .fixed_to_geodetic(observer_fixed_m)
        .lon_rad
        .to_degrees();
    let utc_midnight = (live_wall_utc_s / SECONDS_PER_DAY).floor() * SECONDS_PER_DAY;
    let utc_seconds_of_day = (local_seconds_of_day - longitude_deg * SECONDS_PER_LONGITUDE_DEG)
        .rem_euclid(SECONDS_PER_DAY);
    utc_midnight + utc_seconds_of_day + sim_elapsed_s
}

/// `Some(value)` for a finite value.
pub(super) fn finite(value: f64) -> Option<f64> {
    value.is_finite().then_some(value)
}

fn hours_minutes(hours: u8, minutes: u8) -> f64 {
    f64::from(hours) * 3600.0 + f64::from(minutes) * 60.0
}
