//! The plan a link asks for: its legs, its time, and the Sun along it.

use navigate_celestial::{AnalyticEphemeris, Epoch, sun_ecef_with};
use sim_geo::{Geodetic, geodetic_to_ecef};
use sim_link::SimulationLink;
use sim_navigate::trial::{WaypointSpec, vertical_plan};

use crate::{HostError, HostedRequest};

#[cfg(test)]
mod tests;

/// The name of the leg's origin when the link gives a start position.
const START_IDENT: &str = "START";
/// Sun elevation below which no scattered sunlight is left in the sky,
/// degrees: the end of astronomical twilight.
const NIGHT_BELOW_DEG: f64 = -18.0;

/// One leg of the plan.
#[derive(Debug, Clone, PartialEq)]
pub struct LegSummary {
    /// The point it leaves.
    pub from: String,
    /// The waypoint it flies to.
    pub to: String,
    /// Geodesic length, metres.
    pub length_m: f64,
    /// Planned speed, m/s.
    pub speed_m_s: f64,
    /// Length over speed, seconds: turns and climbs add to it.
    pub duration_s: f64,
    /// Altitude to gain, negative to lose, metres.
    pub altitude_change_m: f64,
}

/// What the Sun leaves of the sky.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Light {
    /// The Sun is below −18°: a dark sky and unlit ground.
    Night,
    /// The Sun is between −18° and the horizon.
    Twilight,
    /// The Sun is above the horizon.
    Day,
}

/// The Sun at one point of the plan when the vehicle is planned to be there.
#[derive(Debug, Clone, PartialEq)]
pub struct SunSample {
    /// The start or the waypoint.
    pub at: String,
    /// Planned time of passing, seconds from the start of the flight.
    pub time_s: f64,
    /// The Sun's geometric elevation above the horizon, degrees; refraction
    /// is not applied.
    pub elevation_deg: f64,
}

impl SunSample {
    /// The light this elevation leaves.
    pub fn light(&self) -> Light {
        if self.elevation_deg > 0.0 {
            Light::Day
        } else if self.elevation_deg >= NIGHT_BELOW_DEG {
            Light::Twilight
        } else {
            Light::Night
        }
    }
}

/// The plan of a link.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanSummary {
    /// The legs in fly order; the first leaves the start position when the
    /// link gives one.
    pub legs: Vec<LegSummary>,
    /// Sum of the leg lengths, metres.
    pub length_m: f64,
    /// Sum of the leg durations, seconds.
    pub duration_s: f64,
    /// The Sun where the flight begins and at each waypoint after it.
    pub sun: Vec<SunSample>,
}

impl PlanSummary {
    /// The darkest and the brightest light along the plan.
    pub fn light_range(&self) -> Option<(Light, Light)> {
        let lights = || self.sun.iter().map(SunSample::light);
        Some((lights().min()?, lights().max()?))
    }
}

/// Summarises `request`, the mapping of `link`: legs from Navigate's route
/// geometry as the trial plans them, and the Sun from Navigate's analytic
/// ephemeris at the planned time of passing each point.
///
/// # Errors
///
/// Returns the trial library's refusal of the route and the ephemeris's
/// refusal of the time.
pub fn plan_summary(
    link: &SimulationLink,
    request: &HostedRequest,
) -> Result<PlanSummary, HostError> {
    let mut points: Vec<WaypointSpec> = Vec::new();
    if let (Some(start), Some(first)) = (request.start, request.trial.waypoints.first()) {
        points.push(WaypointSpec {
            ident: START_IDENT.to_owned(),
            lat_deg: start.lat_deg,
            lon_deg: start.lon_deg,
            altitude_m: start.altitude_m,
            ..first.clone()
        });
    }
    points.extend(request.trial.waypoints.iter().cloned());
    let (planned, _) = vertical_plan(&points, &request.trial.vehicle)?;
    let legs: Vec<LegSummary> = planned
        .into_iter()
        .map(|leg| LegSummary {
            duration_s: leg.length_m / leg.speed_m_s,
            from: leg.from,
            to: leg.to,
            length_m: leg.length_m,
            speed_m_s: leg.speed_m_s,
            altitude_change_m: leg.altitude_change_m,
        })
        .collect();
    let start_unix_s = link.time.unix_seconds() as f64;
    let mut time_s = 0.0;
    let mut sun = Vec::with_capacity(points.len());
    for (index, point) in points.iter().enumerate() {
        if let Some(leg) = index.checked_sub(1).and_then(|before| legs.get(before)) {
            time_s += leg.duration_s;
        }
        sun.push(SunSample {
            at: point.ident.clone(),
            time_s,
            elevation_deg: sun_elevation_deg(link, start_unix_s + time_s, point)?,
        });
    }
    Ok(PlanSummary {
        length_m: legs.iter().map(|leg| leg.length_m).sum(),
        duration_s: legs.iter().map(|leg| leg.duration_s).sum(),
        legs,
        sun,
    })
}

/// The Sun's elevation above the ellipsoid's horizon at `point`.
fn sun_elevation_deg(
    link: &SimulationLink,
    unix_s: f64,
    point: &WaypointSpec,
) -> Result<f64, HostError> {
    let position = Geodetic::from_degrees(point.lat_deg, point.lon_deg, point.altitude_m);
    let observer = geodetic_to_ecef(&position).to_array();
    let sight = sun_ecef_with(
        &AnalyticEphemeris,
        Epoch::from_unix_seconds(unix_s),
        Some(observer),
    )
    .map_err(|error| HostError::Sun {
        time_utc: link.time.iso8601(),
        waypoint: point.ident.clone(),
        detail: format!("{error:?}"),
    })?;
    let (sin_lat, cos_lat) = position.lat_rad.sin_cos();
    let (sin_lon, cos_lon) = position.lon_rad.sin_cos();
    let up = [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat];
    let [x, y, z] = sight.direction;
    let [up_x, up_y, up_z] = up;
    let along_up = x * up_x + y * up_y + z * up_z;
    Ok(along_up.clamp(-1.0, 1.0).asin().to_degrees())
}
