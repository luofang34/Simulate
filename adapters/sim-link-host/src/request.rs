//! From a link to the trial request.

use sim_link::{Camera, HeightDatum, Means, Micro, Milli, Outage, Sensor, SimulationLink, Vehicle};
use sim_navigate::NavMeans;
use sim_navigate::trial::{
    AvailabilitySchedule, GuidanceSource, RoutePoint, SeedSpec, SensorKind, SensorSetSpec,
    TerrainCameraSpec, TrialError, TrialRequest, UnavailableInterval, request_from_route,
};

use crate::{FlyableRoute, HostError, TerminalHold, flyable_route};

#[cfg(test)]
mod tests;

/// The terrain camera a link without `cam` gets when a set is visual: 400
/// by 300 pixels, 67.4 degrees across, looking straight down, one frame a
/// second. Half the reference runs' resolution keeps a navigator frame
/// affordable in a browser tab; a link asks for more with `cam`. It stands
/// in for a camera the vehicle definition will state.
pub const VEHICLE_CAMERA: CameraGeometry = CameraGeometry {
    width_px: 400,
    height_px: 300,
    horizontal_fov_deg: 67.4,
    depression_deg: 90.0,
    rate_hz: 1.0,
};

/// The link as its hosts fly it: the trial request, and beside it what the
/// link says that the trial request has no field for. The link's spoofer is
/// a fault of the vehicle's own receiver, which only the chain flies:
/// [`crate::chain_spec`] takes it from the link.
#[derive(Debug, Clone, PartialEq)]
pub struct HostedRequest {
    /// The request shared with the headless trial: epoch, route on the
    /// ellipsoid, cruise speed, closed-loop guidance, sets, schedules and
    /// seeds, and the terrain camera by its geometry, the trial's
    /// documented defaults filling the rest. Its `maps` are absent: the
    /// packages are the host's to add.
    pub trial: TrialRequest,
    /// The vehicle the link names.
    pub vehicle: Vehicle,
    /// The route the vehicle's software flies, as the link states it.
    pub route: FlyableRoute,
    /// The hold at the last fix, its altitude above the WGS84 ellipsoid;
    /// absent for the standard hold the mission flies when its plan ends.
    pub hold: Option<HostedHold>,
    /// Where the flight begins when the link says so.
    pub start: Option<StartState>,
    /// The name, in `trial.sensor_sets`, of the set that steers the flight.
    pub steering_set: String,
    /// The terrain camera when a set is visual: the link's, or
    /// [`VEHICLE_CAMERA`].
    pub camera: Option<CameraGeometry>,
}

/// The hold at the last fix, as the vehicle's software flies it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HostedHold {
    /// The hold the link states.
    pub stated: TerminalHold,
    /// The holding altitude above the WGS84 ellipsoid, metres; absent for
    /// the last fix's.
    pub altitude_m: Option<f64>,
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
        Vehicle::C172 | Vehicle::C172s => 55.0,
        Vehicle::C172p => 52.0,
        Vehicle::X500 => 10.0,
    }
}

/// The height `own`, or the link's default, in the link's datum, metres.
pub(crate) fn stated_height_m(link: &SimulationLink, own: Option<f64>) -> Option<f64> {
    own.or_else(|| link.default_height.map(Milli::as_f64))
}

/// The ident a fix flies under: its name, or `WP` and its place among the
/// fixes, counted from one.
pub(crate) fn fix_ident(name: Option<&str>, index: usize) -> String {
    name.map_or_else(|| format!("WP{}", index.saturating_add(1)), str::to_owned)
}

/// Maps `link` onto the trial request. The request is not validated here:
/// the flight's own checks run when it starts.
///
/// # Errors
///
/// Refuses a leg the vehicle's software does not fly yet
/// ([`flyable_route`]), returns the trial library's refusal of the route,
/// and refuses a fix or start that has no height when the link has no
/// default.
pub fn hosted_request(link: &SimulationLink) -> Result<HostedRequest, HostError> {
    let flyable = flyable_route(link)?;
    let height = |what: String, lat: Micro, lon: Micro, own: Option<f64>| {
        let stated = stated_height_m(link, own).ok_or(HostError::MissingHeight { what })?;
        Ok::<f64, HostError>(ellipsoid_height_m(
            link.datum,
            lat.as_f64(),
            lon.as_f64(),
            stated,
        ))
    };
    let (route, route_fly_over) = corridor_points(link, &flyable)?;
    let start = match link.start {
        Some(start) => Some(StartState {
            lat_deg: start.lat_deg(),
            lon_deg: start.lon_deg(),
            altitude_m: height(
                "the start".to_owned(),
                start.lat,
                start.lon,
                start.height.map(Milli::as_f64),
            )?,
            heading_deg: start.heading.map(Milli::as_f64),
        }),
        None => None,
    };
    let hold = match (flyable.hold, flyable.fixes.last()) {
        (Some(stated), Some(last)) => Some(HostedHold {
            stated,
            altitude_m: stated.height_m.map(|own| {
                ellipsoid_height_m(link.datum, last.lat.as_f64(), last.lon.as_f64(), own)
            }),
        }),
        _ => None,
    };
    let mut trial = request_from_route(&route, None)?;
    for (spec, fly_over) in trial.waypoints.iter_mut().zip(route_fly_over) {
        spec.fly_over = fly_over;
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
    let camera = visual.then(|| link.camera.map_or(VEHICLE_CAMERA, CameraGeometry::from));
    trial.seeds = SeedSpec::from_master(link.seed);
    if camera.is_none() {
        trial.seeds.terrain_camera = None;
    }
    trial.terrain_camera = camera.map(terrain_camera_spec).transpose()?;
    Ok(HostedRequest {
        trial,
        vehicle: link.vehicle,
        route: flyable,
        hold,
        start,
        steering_set: link
            .steering()
            .map(sim_link::NavSet::text)
            .unwrap_or_default(),
        camera,
    })
}

fn corridor_points(
    link: &SimulationLink,
    flyable: &FlyableRoute,
) -> Result<(Vec<RoutePoint>, Vec<bool>), HostError> {
    let mut route: Vec<RoutePoint> = Vec::with_capacity(flyable.fixes.len());
    let mut route_fly_over: Vec<bool> = Vec::with_capacity(flyable.fixes.len());
    let mut last_coords: Option<(Micro, Micro)> = None;
    for (index, fix) in flyable.fixes.iter().enumerate() {
        let ident = fix_ident(fix.name.as_deref(), index);
        let stated =
            stated_height_m(link, fix.height_m).ok_or_else(|| HostError::MissingHeight {
                what: ident.clone(),
            })?;
        let alt_m = ellipsoid_height_m(link.datum, fix.lat.as_f64(), fix.lon.as_f64(), stated);
        let coords = (fix.lat, fix.lon);
        if last_coords == Some(coords) {
            if let Some(prev) = route.last_mut() {
                prev.altitude_m = alt_m;
            }
            continue;
        }
        last_coords = Some(coords);
        route.push(RoutePoint {
            altitude_m: alt_m,
            ident: Some(ident),
            lat_deg: fix.lat.as_f64(),
            lon_deg: fix.lon.as_f64(),
            speed_m_s: None,
        });
        route_fly_over.push(fix.fly_over);
    }
    Ok((route, route_fly_over))
}

/// The trial's terrain camera of `camera`: its size, field of view, view
/// below the horizon from the nose and frame period; band, exposure,
/// radiometry and the lowest Sun take the defaults the trial documents.
fn terrain_camera_spec(camera: CameraGeometry) -> Result<TerrainCameraSpec, HostError> {
    let stated = serde_json::json!({
        "width": camera.width_px,
        "height": camera.height_px,
        "horizontal_fov_deg": camera.horizontal_fov_deg,
        "mount": { "pitch_deg": -camera.depression_deg },
        "period_s": 1.0 / camera.rate_hz,
    });
    serde_json::from_value(stated).map_err(|source| {
        HostError::from(TrialError::Json {
            what: "terrain camera",
            source,
        })
    })
}

/// A height in `datum` as a height above the WGS84 ellipsoid, metres. Mean
/// sea level is the world's geoid, EGM96 as `sim_geo` grids it.
pub(crate) fn ellipsoid_height_m(
    datum: HeightDatum,
    lat_deg: f64,
    lon_deg: f64,
    height_m: f64,
) -> f64 {
    match datum {
        HeightDatum::Ellipsoid => height_m,
        HeightDatum::MeanSeaLevel => {
            height_m + sim_geo::egm96_undulation_m(lat_deg.to_radians(), lon_deg.to_radians())
        }
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
        let end_s = span.end.map(Milli::as_f64);
        match unavailable.last_mut() {
            Some(before) if before.end_s == Some(start_s) => before.end_s = end_s,
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
