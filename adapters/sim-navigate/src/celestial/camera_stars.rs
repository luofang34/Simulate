use glam::DVec3;
use navigate_celestial::{Duration, Epoch, civil_time, stars_ecef_tt};
use sim_core::SimInstant;
use sim_geo::UtcTime;
use sim_render::CatalogStar;

use crate::NavigateBridgeError;

#[cfg(test)]
mod tests;

/// The civil instant of a frame: the scenario's start plus simulated time.
/// Wall-clock time never enters, so reruns see the same sky.
pub fn star_camera_epoch(scenario_start: UtcTime, now: SimInstant) -> Epoch {
    let elapsed_s = now.duration_since(SimInstant::START).as_seconds();
    Epoch::from_unix_seconds(scenario_start.unix_seconds()) + Duration::from_seconds(elapsed_s)
}

/// The catalogue stars at least as bright as `faintest_v_magnitude`, at their
/// apparent ECEF places for the frame at `now`, brightest first, for
/// [`sim_render::StarCamera`].
///
/// The ephemeris takes an instant, which [`Epoch`] already is for a UTC
/// reading from 1972; [`civil_time`] also reads an earlier scenario clock as
/// UT1, as civil time then was, so any epoch the ephemeris covers works.
/// Directions
/// carry proper motion and annual aberration; Navigate bounds them at
/// 3 arcseconds against Skyfield from 1975 to 2026.
pub fn camera_stars(
    scenario_start: UtcTime,
    now: SimInstant,
    faintest_v_magnitude: f64,
) -> Result<Vec<CatalogStar>, NavigateBridgeError> {
    let epoch = star_camera_epoch(scenario_start, now);
    let sights =
        stars_ecef_tt(civil_time(epoch).terrestrial, faintest_v_magnitude).map_err(|source| {
            NavigateBridgeError::StarEphemeris {
                start_unix_s: scenario_start.unix_seconds(),
                elapsed_s: now.duration_since(SimInstant::START).as_seconds(),
                source,
            }
        })?;
    Ok(sights
        .iter()
        .map(|sight| CatalogStar {
            id: sight.hr,
            direction_ecef: DVec3::from_array(sight.direction),
            v_magnitude: sight.v_magnitude,
        })
        .collect())
}
