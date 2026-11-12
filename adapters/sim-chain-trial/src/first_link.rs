//! The first shareable link: the C172 around a closed route over land near
//! Zaragoza at noon, Pilotage flying on Navigate's GNSS-aided set.

use sim_geo::egm96_undulation_m;
use sim_navigate::NavMeans;
use sim_navigate::trial::{RoutePoint, SensorSetSpec, request_from_route};

use crate::{CalibrationName, ChainTrialError, ChainTrialSpec, StartSpec, Steering};

/// The route's fixes, degrees north and east.
const FIXES: [(f64, f64); 5] = [
    (41.70, -1.05),
    (41.70, -0.80),
    (41.80, -0.80),
    (41.80, -1.00),
    (41.74, -1.05),
];
/// The start, degrees north and east; the aeroplane heads east.
const START: (f64, f64) = (41.70, -1.07);
const START_HEADING_DEG: f64 = 90.0;
/// The route's and the start's height above mean sea level, metres.
const MSL_M: f64 = 2000.0;
const INDICATED_M_S: f64 = 55.0;
const EPOCH_UTC: &str = "2026-06-15T12:04:00Z";
/// The set Pilotage flies on.
const STEERING_SET: &str = "ins.baro.gnss";

/// Height above the ellipsoid at `lat_deg`, `lon_deg` of the link's mean
/// sea level height.
fn ellipsoidal_m(lat_deg: f64, lon_deg: f64) -> f64 {
    MSL_M + egm96_undulation_m(lat_deg.to_radians(), lon_deg.to_radians())
}

/// The first link as a chain trial: its route, start and epoch, the
/// GNSS-aided set Pilotage steers on and a barometric set beside it.
pub fn first_link() -> Result<ChainTrialSpec, ChainTrialError> {
    let route: Vec<RoutePoint> = FIXES
        .iter()
        .map(|&(lat_deg, lon_deg)| RoutePoint {
            ident: None,
            lat_deg,
            lon_deg,
            altitude_m: ellipsoidal_m(lat_deg, lon_deg),
            speed_m_s: None,
        })
        .collect();
    let mut request = request_from_route(&route, None)?;
    request.epoch_utc = EPOCH_UTC.to_owned();
    request.output.progress_period_s = 1.0;
    let set = |name: &str, means: &[NavMeans]| SensorSetSpec {
        name: name.to_owned(),
        means: means.to_vec(),
        availability: Default::default(),
    };
    use NavMeans::{Barometric, DeadReckoning, Gnss};
    request.sensor_sets = vec![
        set(STEERING_SET, &[DeadReckoning, Barometric, Gnss]),
        set("ins.baro", &[DeadReckoning, Barometric]),
    ];
    Ok(ChainTrialSpec {
        variant: crate::ChainC172Variant::C172s,
        request,
        start: StartSpec {
            lat_deg: START.0,
            lon_deg: START.1,
            height_m: ellipsoidal_m(START.0, START.1),
            heading_deg: START_HEADING_DEG,
            indicated_m_s: INDICATED_M_S,
        },
        steering: Steering::Set(STEERING_SET.to_owned()),
        calibration: CalibrationName::Calibrated,
        outages: Vec::new(),
        gnss_spoof: None,
        hold: None,
    })
}
