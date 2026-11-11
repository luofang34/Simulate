//! The link flown on the vehicle's own control chain: the chain trial's
//! spec from the link, and the chain trial stepped as a page's flight, its
//! terrain-camera frames rendered on the trial's own renderer when asked.

use navigate_data::{MemoryStore, Sha256Digest};
use sim_chain_trial::{
    CalibrationName, ChainTrialSpec, DeviceOutage, FrameRequest, Outputs, StartSpec, Steering,
};
use sim_geo::SensorIntrinsics;
use sim_link::{Sensor, SimulationLink};
use sim_navigate::trial::{
    AssumedAccuracy, EventKind, FinishedRecord, GeoidChoice, MapStore, MapsSpec, NamedStores,
    OnboardMapSpec, PackageRef, ProgressRecord, SensorKind,
};

use crate::{HostError, HostedRequest, MonitorFeed};

mod flight;

#[cfg(test)]
mod tests;

pub use flight::{ChainFlight, MAX_STEP_S, OVERRUN_S};

/// The store name the page registers its sealed package under.
pub const PAGE_STORE: &str = "page";

/// The page's stores: its one sealed package, under [`PAGE_STORE`].
pub fn page_stores(package: MemoryStore) -> NamedStores {
    let mut stores = NamedStores::new();
    stores.insert(PAGE_STORE, MapStore::new(package));
    stores
}

/// The accuracy the navigator is told to assume for the package, whose
/// imagery source states none: 3 m horizontal and 10 m vertical, 1-sigma,
/// the values the desktop visual trials assumed for the same source.
pub const ASSUMED_ACCURACY: AssumedAccuracy = AssumedAccuracy {
    horizontal_m: 3.0,
    vertical_m: 10.0,
};

const fn trial_sensor(sensor: Sensor) -> SensorKind {
    match sensor {
        Sensor::Gnss => SensorKind::Gnss,
        Sensor::Baro => SensorKind::Baro,
        Sensor::Stars => SensorKind::Stars,
        Sensor::Cam => SensorKind::TerrainCamera,
    }
}

/// The initial great-circle bearing from `from` to `to`, degrees.
fn bearing_deg(from: [f64; 2], to: [f64; 2]) -> f64 {
    let (lat1, lat2) = (from[0].to_radians(), to[0].to_radians());
    let d_lon = (to[1] - from[1]).to_radians();
    let y = d_lon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * d_lon.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// The chain trial of `link`, mapped onto `hosted`, flying on the sealed
/// package `package` when a set is visual.
///
/// The link's outages are outages of the devices, for every consumer, so
/// they ride as the spec's outages and not as the sets' schedules. A link
/// without a start begins over its first waypoint heading for the second,
/// as the headless trial does; without a heading it heads for the first
/// waypoint.
///
/// # Errors
///
/// Refuses a visual link without a package, and a route of fewer than two
/// waypoints.
pub fn chain_spec(
    link: &SimulationLink,
    hosted: &HostedRequest,
    package: Option<Sha256Digest>,
) -> Result<ChainTrialSpec, HostError> {
    let mut request = hosted.trial.clone();
    request.availability.clear();
    for set in &mut request.sensor_sets {
        set.availability.clear();
    }
    request.maps = match (hosted.camera, package) {
        (None, _) => None,
        (Some(_), Some(identity)) => Some(MapsSpec {
            onboard: OnboardMapSpec {
                store: PAGE_STORE.to_owned(),
                package: identity.to_string(),
                assumed_accuracy: Some(ASSUMED_ACCURACY),
                max_match_pixels: None,
            },
            world: PackageRef {
                store: PAGE_STORE.to_owned(),
                package: identity.to_string(),
            },
            geoid: Some(GeoidChoice::World),
        }),
        (Some(_), None) => {
            return Err(HostError::Render {
                detail: "a visual set needs the sealed map package".to_owned(),
            });
        }
    };
    let waypoints = &request.waypoints;
    let (Some(first), Some(second)) = (waypoints.first(), waypoints.get(1)) else {
        return Err(HostError::Render {
            detail: "a route of fewer than two waypoints".to_owned(),
        });
    };
    let first_at = [first.lat_deg, first.lon_deg];
    let start = match hosted.start {
        Some(start) => StartSpec {
            lat_deg: start.lat_deg,
            lon_deg: start.lon_deg,
            height_m: start.altitude_m,
            heading_deg: start
                .heading_deg
                .unwrap_or_else(|| bearing_deg([start.lat_deg, start.lon_deg], first_at)),
            indicated_m_s: request.vehicle.cruise_speed_m_s,
        },
        None => StartSpec {
            lat_deg: first.lat_deg,
            lon_deg: first.lon_deg,
            height_m: first.altitude_m,
            heading_deg: bearing_deg(first_at, [second.lat_deg, second.lon_deg]),
            indicated_m_s: request.vehicle.cruise_speed_m_s,
        },
    };
    let outages = link
        .outages
        .iter()
        .flat_map(|outage| {
            outage.spans.iter().map(|span| DeviceOutage {
                sensor: trial_sensor(outage.sensor),
                start_s: span.start.as_f64(),
                end_s: span.end.map(sim_link::Milli::as_f64),
            })
        })
        .collect();
    Ok(ChainTrialSpec {
        request,
        start,
        steering: Steering::Set(hosted.steering_set.clone()),
        calibration: CalibrationName::Calibrated,
        outages,
    })
}

/// The renderer's pinhole of a frame request.
fn intrinsics(request: &FrameRequest) -> SensorIntrinsics {
    let [fx, fy, cx, cy] = request.pinhole;
    SensorIntrinsics {
        width: request.size.0,
        height: request.size.1,
        fx,
        fy,
        cx,
        cy,
    }
}

/// What a chain trial's outputs tell the monitor, in order.
fn feeds(outputs: Outputs, out: &mut Vec<MonitorFeed>) {
    out.extend(outputs.records.into_iter().map(MonitorFeed::Progress));
    for update in outputs.mission {
        if let Some(phase) = update.phase {
            out.push(MonitorFeed::Phase(phase));
        }
        out.extend(update.events.into_iter().map(|event| MonitorFeed::Mission {
            t_s: update.t_s,
            event,
        }));
    }
    out.extend(
        outputs
            .terrain_fixes
            .into_iter()
            .map(|(set, fixes)| MonitorFeed::TerrainFixes { set, fixes }),
    );
}

/// Whether a record ends the flight: the plan completed, or a
/// termination rule tripped.
fn ends_flight(record: &ProgressRecord) -> bool {
    match record {
        ProgressRecord::Event(event) => matches!(
            event.kind,
            EventKind::WaypointSequenced {
                plan_complete: true,
                ..
            } | EventKind::Terminated { .. }
        ),
        ProgressRecord::Finished(_) => true,
        _ => false,
    }
}

/// The record a host adds when it ends a run: the stream's last.
fn finished(wall_time_s: f64) -> ProgressRecord {
    ProgressRecord::Finished(FinishedRecord {
        result_path: None,
        wall_time_s,
    })
}

/// A device outage as the link's spans see it.
pub fn link_outages(outages: &[DeviceOutage]) -> Vec<(SensorKind, f64, Option<f64>)> {
    outages
        .iter()
        .map(|outage| (outage.sensor, outage.start_s, outage.end_s))
        .collect()
}
