//! From a link to the trial request.

use sim_link::{Camera, HeightDatum, Means, Micro, Milli, Outage, Sensor, SimulationLink, Vehicle};
use sim_navigate::NavMeans;
use sim_navigate::trial::{
    AvailabilitySchedule, GuidanceSource, RoutePoint, SeedSpec, SensorKind, SensorSetSpec,
    TrialRequest, UnavailableInterval, request_from_route,
};

use crate::HostError;

#[cfg(test)]
mod tests;

/// The end, in seconds from the start, a schedule gives an outage that runs
/// to the end of the flight: the trial's intervals are closed, and no flight
/// of a link lasts thirty years.
pub const UNTIL_END_S: f64 = 1.0e9;

/// The terrain camera a link without `cam` gets when a set is visual: the
/// camera of the reference runs, 800 by 600 pixels, 67.4 degrees across,
/// looking straight down, one frame a second. It stands in for a camera the
/// vehicle definition will state.
pub const VEHICLE_CAMERA: CameraGeometry = CameraGeometry {
    width_px: 800,
    height_px: 600,
    horizontal_fov_deg: 67.4,
    depression_deg: 90.0,
    rate_hz: 1.0,
};

/// The link as its hosts fly it: the trial request, and beside it what the
/// link says that the trial request has no field for.
#[derive(Debug, Clone, PartialEq)]
pub struct HostedRequest {
    /// The request shared with the headless trial: epoch, route on the
    /// ellipsoid, cruise speed, closed-loop guidance, sets, schedules and
    /// seeds. Its `terrain_camera` and `maps` are absent: the camera's
    /// radiometry and the packages are the host's to add.
    pub trial: TrialRequest,
    /// The vehicle the link names.
    pub vehicle: Vehicle,
    /// Where the flight begins when the link says so.
    pub start: Option<StartState>,
    /// The name, in `trial.sensor_sets`, of the set that steers the flight.
    pub steering_set: String,
    /// The terrain camera when a set is visual: the link's, or
    /// [`VEHICLE_CAMERA`].
    pub camera: Option<CameraGeometry>,
}

/// The airborne start a link gives.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StartState {
    /// Latitude, degrees north.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid, metres.
    pub altitude_m: f64,
    /// True heading, degrees; absent means toward the first waypoint.
    pub heading_deg: Option<f64>,
}

/// The geometry and rate of the terrain camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraGeometry {
    /// Image width, pixels.
    pub width_px: u32,
    /// Image height, pixels.
    pub height_px: u32,
    /// Horizontal field of view, degrees.
    pub horizontal_fov_deg: f64,
    /// Angle of the optical axis below the horizon, degrees.
    pub depression_deg: f64,
    /// Frames a second.
    pub rate_hz: f64,
}

impl From<Camera> for CameraGeometry {
    fn from(camera: Camera) -> Self {
        Self {
            width_px: camera.width_px,
            height_px: camera.height_px,
            horizontal_fov_deg: camera.fov_deg.as_f64(),
            depression_deg: camera.depression_deg.as_f64(),
            rate_hz: camera.rate_hz.as_f64(),
        }
    }
}

/// The cruise speed a vehicle flies when the link gives none, m/s.
pub const fn own_cruise_speed_m_s(vehicle: Vehicle) -> f64 {
    match vehicle {
        Vehicle::C172 => 55.0,
        Vehicle::X500 => 10.0,
    }
}

/// Maps `link` onto the trial request. The request is not validated here:
/// the flight's own checks run when it starts.
///
/// # Errors
///
/// Returns the trial library's refusal of the route, and refuses a link
/// value in which a waypoint or the start has no height and the link no
/// default.
pub fn hosted_request(link: &SimulationLink) -> Result<HostedRequest, HostError> {
    let height = |what: String, lat: Micro, lon: Micro, own: Option<Milli>| {
        let stated = link
            .height_or_default(own)
            .ok_or(HostError::MissingHeight { what })?;
        let (lat_deg, lon_deg) = (lat.as_f64(), lon.as_f64());
        Ok::<f64, HostError>(ellipsoid_height_m(
            link.datum,
            lat_deg,
            lon_deg,
            stated.as_f64(),
        ))
    };
    let mut route: Vec<RoutePoint> = Vec::with_capacity(link.route.len());
    for (index, waypoint) in link.route.iter().enumerate() {
        let ident = waypoint
            .name
            .clone()
            .unwrap_or_else(|| format!("WP{}", index.saturating_add(1)));
        route.push(RoutePoint {
            altitude_m: height(ident.clone(), waypoint.lat, waypoint.lon, waypoint.height)?,
            ident: Some(ident),
            lat_deg: waypoint.lat_deg(),
            lon_deg: waypoint.lon_deg(),
            speed_m_s: None,
        });
    }
    let start = match link.start {
        Some(start) => Some(StartState {
            lat_deg: start.lat_deg(),
            lon_deg: start.lon_deg(),
            altitude_m: height("the start".to_owned(), start.lat, start.lon, start.height)?,
            heading_deg: start.heading.map(Milli::as_f64),
        }),
        None => None,
    };
    let mut trial = request_from_route(&route, None)?;
    for (spec, waypoint) in trial.waypoints.iter_mut().zip(&link.route) {
        spec.fly_over = waypoint.turn == sim_link::Turn::FlyOver;
    }
    trial.epoch_utc = link.time.iso8601();
    trial.vehicle.cruise_speed_m_s = link
        .cruise_speed
        .map_or_else(|| own_cruise_speed_m_s(link.vehicle), Milli::as_f64);
    trial.vehicle.guidance = GuidanceSource::Estimate;
    trial.availability = link.outages.iter().map(schedule).collect();
    trial.sensor_sets = link
        .sets
        .iter()
        .map(|set| sensor_set(set, &link.outages))
        .collect();
    let visual = link.sets.iter().any(|set| set.contains(Means::Visual));
    trial.seeds = sensor_seeds(link.seed, visual);
    Ok(HostedRequest {
        trial,
        vehicle: link.vehicle,
        start,
        steering_set: link
            .steering()
            .map(sim_link::NavSet::text)
            .unwrap_or_default(),
        camera: visual.then(|| link.camera.map_or(VEHICLE_CAMERA, CameraGeometry::from)),
    })
}

/// A height in `datum` as a height above the WGS84 ellipsoid, metres. Mean
/// sea level is the world's geoid, EGM96 as `sim_geo` grids it.
fn ellipsoid_height_m(datum: HeightDatum, lat_deg: f64, lon_deg: f64, height_m: f64) -> f64 {
    match datum {
        HeightDatum::Ellipsoid => height_m,
        HeightDatum::MeanSeaLevel => {
            height_m + sim_geo::egm96_undulation_m(lat_deg.to_radians(), lon_deg.to_radians())
        }
    }
}

/// The seed of each simulated sensor from the link's one master seed: the
/// SplitMix64 stream of the master, drawn in the order IMU, alignment,
/// GNSS, barometer, star camera, terrain camera. Every draw is made whether
/// or not its sensor flies, so adding a means to a link leaves the other
/// sensors' realisations as they were.
pub fn sensor_seeds(master: u64, terrain_camera: bool) -> SeedSpec {
    let mut state = master;
    let mut draw = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut mixed = state;
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        mixed ^ (mixed >> 31)
    };
    let (imu, alignment, gnss, baro, star_camera, camera) =
        (draw(), draw(), draw(), draw(), draw(), draw());
    SeedSpec {
        imu,
        alignment,
        gnss,
        baro,
        star_camera,
        terrain_camera: terrain_camera.then_some(camera),
    }
}

const fn sensor_kind(sensor: Sensor) -> SensorKind {
    match sensor {
        Sensor::Gnss => SensorKind::Gnss,
        Sensor::Baro => SensorKind::Baro,
        Sensor::Stars => SensorKind::Stars,
        Sensor::Cam => SensorKind::TerrainCamera,
    }
}

const fn nav_means(means: Means) -> NavMeans {
    match means {
        Means::Ins => NavMeans::DeadReckoning,
        Means::Baro => NavMeans::Barometric,
        Means::Gnss => NavMeans::Gnss,
        Means::Stars => NavMeans::Celestial,
        Means::Visual => NavMeans::Visual,
    }
}

/// One outage as a schedule named after its sensor. Spans that touch join
/// into one interval: the trial's intervals include both ends.
fn schedule(outage: &Outage) -> AvailabilitySchedule {
    let mut unavailable: Vec<UnavailableInterval> = Vec::new();
    for span in &outage.spans {
        let start_s = span.start.as_f64();
        let end_s = span.end.map_or(UNTIL_END_S, Milli::as_f64);
        match unavailable.last_mut() {
            Some(before) if before.end_s == start_s => before.end_s = end_s,
            _ => unavailable.push(UnavailableInterval {
                start_s,
                end_s,
                reason: Some("link outage".to_owned()),
            }),
        }
    }
    let sensor = sensor_kind(outage.sensor);
    AvailabilitySchedule {
        name: sensor.name().to_owned(),
        sensor,
        unavailable,
    }
}

/// One set under its link text, honouring the outage of every sensor it
/// carries. An outage of a sensor the set lacks does not concern it.
fn sensor_set(set: &sim_link::NavSet, outages: &[Outage]) -> SensorSetSpec {
    let means: Vec<NavMeans> = set.0.iter().copied().map(nav_means).collect();
    let availability = outages
        .iter()
        .map(|outage| sensor_kind(outage.sensor))
        .filter(|sensor| means.contains(&sensor.means()))
        .map(|sensor| (sensor, sensor.name().to_owned()))
        .collect();
    SensorSetSpec {
        name: set.text(),
        means,
        availability,
    }
}
