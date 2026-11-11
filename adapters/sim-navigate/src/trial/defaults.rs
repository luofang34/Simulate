//! A request completed from a route alone: the waypoints come from the
//! caller, everything else from documented defaults, optionally overridden
//! by a partial request.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use super::{
    AlignmentSpec, BaroSpec, GnssSpec, ImuGradeName, OutputSpec, SeedSpec, SensorSetSpec, StarSpec,
    TrialError, TrialRequest, VehicleSpec, WaypointSpec,
};
use crate::NavMeans;

/// One waypoint of a route file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutePoint {
    /// Identifier; absent takes `WP` and the row number from 0.
    #[serde(default)]
    pub ident: Option<String>,
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid, metres.
    pub altitude_m: f64,
    /// Speed limit on the leg toward this waypoint, m/s.
    #[serde(default)]
    pub speed_m_s: Option<f64>,
}

impl RoutePoint {
    /// Parses a route file: a JSON array of points in fly order.
    pub fn parse_route(text: &str) -> Result<Vec<Self>, TrialError> {
        serde_json::from_str(text).map_err(|source| TrialError::Json {
            what: "route",
            source,
        })
    }
}

/// The epoch a route runs at unless told otherwise: astronomical night over
/// the eastern United States, so the star camera has a dark sky there.
/// Elsewhere pass an epoch that is night at the route.
pub const DEFAULT_EPOCH_UTC: &str = "2026-03-20T02:00:00Z";

/// Each aiding means alone on the inertial core, and their fusions.
pub fn default_sensor_sets() -> Vec<SensorSetSpec> {
    use NavMeans::{Barometric, Celestial, DeadReckoning, Gnss};
    let set = |name: &str, means: &[NavMeans]| SensorSetSpec {
        name: name.into(),
        means: means.to_vec(),
        availability: Default::default(),
    };
    vec![
        set("ins", &[DeadReckoning]),
        set("ins+baro", &[DeadReckoning, Barometric]),
        set("ins+gnss", &[DeadReckoning, Gnss]),
        set("ins+stars", &[DeadReckoning, Celestial]),
        set("ins+baro+gnss", &[DeadReckoning, Barometric, Gnss]),
        set("ins+baro+stars", &[DeadReckoning, Barometric, Celestial]),
        set(
            "ins+baro+gnss+stars",
            &[DeadReckoning, Barometric, Gnss, Celestial],
        ),
    ]
}

/// The defaults every field but the waypoints takes: [`DEFAULT_EPOCH_UTC`];
/// a 60 m/s vehicle banking 18° and climbing 5 m/s with a 100 m capture
/// radius, guided on truth; a tactical IMU; each sensor model's own
/// default; no availability schedules; [`default_sensor_sets`]; fixed
/// seeds; and the default output.
fn default_request(waypoints: Vec<WaypointSpec>) -> TrialRequest {
    TrialRequest {
        epoch_utc: DEFAULT_EPOCH_UTC.into(),
        waypoints,
        vehicle: VehicleSpec {
            cruise_speed_m_s: 60.0,
            bank_limit_deg: 18.0,
            max_climb_m_s: 5.0,
            capture_radius_m: 100.0,
            guidance: Default::default(),
            guidance_min_quality: None,
        },
        imu_grade: ImuGradeName::Tactical,
        alignment: AlignmentSpec::default(),
        gnss: GnssSpec::default(),
        baro: BaroSpec::default(),
        stars: StarSpec::default(),
        availability: Vec::new(),
        sensor_sets: default_sensor_sets(),
        seeds: SeedSpec {
            imu: 22_434,
            alignment: 31,
            gnss: 28_245,
            baro: 47_648,
            star_camera: 7,
        },
        output: OutputSpec::default(),
        termination: None,
    }
}

/// The request flying `route` with the defaults, each field `defaults`
/// (a partial request as JSON) gives taking its place: objects merge
/// field by field, anything else is replaced whole. The route alone gives
/// the waypoints. The request is not validated here.
pub fn request_from_route(
    route: &[RoutePoint],
    defaults: Option<&str>,
) -> Result<TrialRequest, TrialError> {
    let waypoints = route
        .iter()
        .enumerate()
        .map(|(row, point)| WaypointSpec {
            ident: point.ident.clone().unwrap_or_else(|| format!("WP{row}")),
            lat_deg: point.lat_deg,
            lon_deg: point.lon_deg,
            altitude_m: point.altitude_m,
            speed_m_s: point.speed_m_s,
            fly_over: false,
        })
        .collect();
    let json = |what: &'static str| move |source| TrialError::Json { what, source };
    let mut request = serde_json::to_value(default_request(waypoints)).map_err(json("request"))?;
    if let Some(text) = defaults {
        let overrides: Value = serde_json::from_str(text).map_err(json("defaults"))?;
        if overrides.get("waypoints").is_some() {
            return Err(TrialError::InvalidRequest {
                field: "waypoints",
                detail: "the route gives the waypoints, not the defaults".into(),
            });
        }
        merge(&mut request, overrides);
    }
    serde_json::from_value(request).map_err(json("defaults"))
}

fn merge(base: &mut Value, overrides: Value) {
    match (base, overrides) {
        (Value::Object(base), Value::Object(overrides)) => {
            for (key, value) in overrides {
                match base.get_mut(&key) {
                    Some(slot) => merge(slot, value),
                    None => {
                        base.insert(key, value);
                    }
                }
            }
        }
        (slot, value) => *slot = value,
    }
}
