//! Refusing a request the trial cannot run as stated, naming what is wrong.

use navigate_startrack::catalogue_ecef;
use sim_core::SimInstant;
use sim_geo::{Geodetic, geodetic_to_ecef};

use super::{SIMULATED_MEANS, TrialRequest, WaypointSpec};
use crate::trial::availability::{check_schedules, check_set};
use crate::trial::epoch::parse_utc;
use crate::trial::sky::sun_direction_ecef;
use crate::trial::vertical::{check_reachable, vertical_plan};
use crate::trial::{TrialError, VehicleSpec};
use crate::{NavMeans, star_camera_epoch};

/// The kinematic vehicle's slowest and fastest speeds, m/s.
const SPEED_RANGE_M_S: std::ops::RangeInclusive<f64> = 30.0..=120.0;
/// A leg must be this many capture radii long, or the executor captures
/// its waypoint as soon as the leg begins.
const MIN_LEG_CAPTURE_RADII: f64 = 2.0;

impl TrialRequest {
    /// Refuses a request the trial cannot run as stated.
    pub fn validate(&self) -> Result<(), TrialError> {
        let invalid =
            |field: &'static str, detail: String| TrialError::InvalidRequest { field, detail };
        if self.waypoints.len() < 2 {
            return Err(invalid("waypoints", "at least two".into()));
        }
        check_vehicle(&self.vehicle)?;
        let termination = self.termination.unwrap_or_default();
        let heights = termination.min_height_m..=termination.max_height_m;
        for (row, waypoint) in self.waypoints.iter().enumerate() {
            check_waypoint(row, waypoint, &heights)?;
        }
        check_legs(&self.waypoints, &self.vehicle)?;
        check_reachable(&vertical_plan(&self.waypoints, &self.vehicle)?.0)?;
        if self.sensor_sets.is_empty() {
            return Err(invalid("sensor_sets", "at least one".into()));
        }
        check_schedules(&self.availability)?;
        for (row, set) in self.sensor_sets.iter().enumerate() {
            let named_before = self
                .sensor_sets
                .iter()
                .take(row)
                .any(|s| s.name == set.name);
            let set_error = |detail: String| TrialError::InvalidSet {
                set: set.name.clone(),
                detail,
            };
            if named_before {
                return Err(set_error("the name is used twice".into()));
            }
            if !set.means.contains(&NavMeans::DeadReckoning) {
                return Err(set_error("no INS (`dead_reckoning`)".into()));
            }
            if let Some(means) = set.means.iter().find(|m| !SIMULATED_MEANS.contains(m)) {
                return Err(set_error(format!("{means:?} is not simulated")));
            }
            check_set(self, set)?;
        }
        let start = parse_utc(&self.epoch_utc)?;
        if self.uses_stars() || self.uses_visual() {
            check_sky_coverage(self, start)?;
        }
        check_terrain_camera(self)
    }

    /// Whether any set takes terrain-camera fixes.
    pub fn uses_visual(&self) -> bool {
        self.sensor_sets
            .iter()
            .any(|set| set.means.contains(&NavMeans::Visual))
    }

    /// Whether any set takes star sightings, so frames need rendering.
    pub fn uses_stars(&self) -> bool {
        self.sensor_sets
            .iter()
            .any(|set| set.means.contains(&NavMeans::Celestial))
    }
}

fn check_vehicle(vehicle: &VehicleSpec) -> Result<(), TrialError> {
    let invalid = |field: &'static str, value: f64| TrialError::InvalidRequest {
        field,
        detail: format!("{value} is outside its range"),
    };
    if !SPEED_RANGE_M_S.contains(&vehicle.cruise_speed_m_s) {
        return Err(invalid(
            "vehicle.cruise_speed_m_s",
            vehicle.cruise_speed_m_s,
        ));
    }
    if !(vehicle.bank_limit_deg > 0.0 && vehicle.bank_limit_deg < 90.0) {
        return Err(invalid("vehicle.bank_limit_deg", vehicle.bank_limit_deg));
    }
    if !(vehicle.max_climb_m_s > 0.0 && vehicle.max_climb_m_s < vehicle.cruise_speed_m_s) {
        return Err(invalid("vehicle.max_climb_m_s", vehicle.max_climb_m_s));
    }
    if !(vehicle.capture_radius_m > 0.0 && vehicle.capture_radius_m.is_finite()) {
        return Err(invalid(
            "vehicle.capture_radius_m",
            vehicle.capture_radius_m,
        ));
    }
    Ok(())
}

fn check_waypoint(
    row: usize,
    waypoint: &WaypointSpec,
    heights: &std::ops::RangeInclusive<f64>,
) -> Result<(), TrialError> {
    let invalid =
        |field: &'static str, detail: String| TrialError::InvalidWaypoint { row, field, detail };
    if waypoint.ident.is_empty() || waypoint.ident.len() > 32 {
        return Err(invalid("ident", "1 to 32 bytes".into()));
    }
    if !(waypoint.lat_deg > -90.0 && waypoint.lat_deg < 90.0) {
        return Err(invalid(
            "lat_deg",
            format!("{} is not inside (-90, 90)", waypoint.lat_deg),
        ));
    }
    if !(-180.0..=180.0).contains(&waypoint.lon_deg) {
        return Err(invalid(
            "lon_deg",
            format!("{} is not inside [-180, 180]", waypoint.lon_deg),
        ));
    }
    if !heights.contains(&waypoint.altitude_m) {
        return Err(invalid(
            "altitude_m",
            format!(
                "{} m is outside the flight envelope {} to {} m (the termination heights)",
                waypoint.altitude_m,
                heights.start(),
                heights.end()
            ),
        ));
    }
    if let Some(speed) = waypoint.speed_m_s
        && !SPEED_RANGE_M_S.contains(&speed)
    {
        return Err(invalid(
            "speed_m_s",
            format!("{speed} m/s is outside the vehicle's 30 to 120 m/s"),
        ));
    }
    Ok(())
}

fn check_legs(waypoints: &[WaypointSpec], vehicle: &VehicleSpec) -> Result<(), TrialError> {
    let shortest_m = MIN_LEG_CAPTURE_RADII * vehicle.capture_radius_m;
    let (legs, _) = vertical_plan(waypoints, vehicle)?;
    match legs.iter().find(|leg| leg.length_m < shortest_m) {
        Some(leg) => Err(TrialError::InvalidWaypoint {
            row: leg.leg,
            field: "lat_deg",
            detail: format!(
                "the leg from {} is {:.0} m, shorter than {shortest_m} m (two capture radii)",
                leg.from, leg.length_m
            ),
        }),
        None => Ok(()),
    }
}

/// The star model needs the Sun and the star catalogue at the epoch; the
/// analytic ephemeris refuses epochs outside its range.
fn check_sky_coverage(request: &TrialRequest, start: sim_geo::UtcTime) -> Result<(), TrialError> {
    let outside = |what: String| TrialError::InvalidRequest {
        field: "epoch_utc",
        detail: format!("outside the star model's ephemeris coverage: {what}"),
    };
    let first = request.waypoints.first().map_or(
        Geodetic {
            lat_rad: 0.0,
            lon_rad: 0.0,
            height_m: 0.0,
        },
        |w| Geodetic {
            lat_rad: w.lat_deg.to_radians(),
            lon_rad: w.lon_deg.to_radians(),
            height_m: w.altitude_m,
        },
    );
    sun_direction_ecef(start, SimInstant::START, geodetic_to_ecef(&first))
        .map_err(|error| outside(error.to_string()))?;
    let faintest = navigate_onboard::StarCatalogueConfig::with_utc_at_zero(start.unix_seconds())
        .catalogue_faintest_v;
    catalogue_ecef(star_camera_epoch(start, SimInstant::START), faintest)
        .map_err(|error| outside(error.to_string()))?;
    Ok(())
}

/// The terrain camera, its seed and the map packages are present together
/// whenever a set takes visual fixes, and each is usable as stated. The
/// packages themselves are opened at the start of the run.
fn check_terrain_camera(request: &TrialRequest) -> Result<(), TrialError> {
    let invalid = |field: &'static str, detail: &str| TrialError::InvalidRequest {
        field,
        detail: detail.into(),
    };
    if let Some(camera) = &request.terrain_camera {
        camera.validate()?;
    }
    if let Some(accuracy) = request
        .maps
        .as_ref()
        .and_then(|maps| maps.onboard.assumed_accuracy)
    {
        let usable = [accuracy.horizontal_m, accuracy.vertical_m]
            .iter()
            .all(|sigma| sigma.is_finite() && *sigma > 0.0);
        if !usable {
            return Err(invalid(
                "maps.onboard.assumed_accuracy",
                "sigmas must be finite and positive",
            ));
        }
    }
    if !request.uses_visual() {
        return Ok(());
    }
    if request.terrain_camera.is_none() {
        return Err(invalid("terrain_camera", "a set takes visual fixes"));
    }
    if request.maps.is_none() {
        return Err(invalid("maps", "a set takes visual fixes"));
    }
    if request.seeds.terrain_camera.is_none() {
        return Err(invalid("seeds.terrain_camera", "a set takes visual fixes"));
    }
    if request.vehicle.guidance == super::GuidanceSource::Estimate {
        return Err(invalid(
            "vehicle.guidance",
            "the terrain camera renders frames along a recorded flight only; closed loop with visual fixes is not wired",
        ));
    }
    Ok(())
}
