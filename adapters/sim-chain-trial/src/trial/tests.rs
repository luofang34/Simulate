use sim_geo::egm96_undulation_m;
use sim_navigate::NavMeans;
use sim_navigate::trial::{
    CameraBand, CameraMountSpec, DirectoryStores, EventKind, ProgressRecord, RadiometrySpec,
    RoutePoint, SampleRecord, SensorSetSpec, SensorState, TerrainCameraSpec, request_from_route,
};

use super::*;
use crate::{CalibrationName, StartSpec};

mod gnss_check;
mod steered;

/// The first shareable link's route, degrees north and east.
const FIXES: [(f64, f64); 5] = [
    (41.70, -1.05),
    (41.70, -0.80),
    (41.80, -0.80),
    (41.80, -1.00),
    (41.74, -1.05),
];
const MSL_M: f64 = 2000.0;

fn ellipsoidal_m(lat_deg: f64, lon_deg: f64) -> f64 {
    MSL_M + egm96_undulation_m(lat_deg.to_radians(), lon_deg.to_radians())
}

fn set(name: &str, means: &[NavMeans]) -> SensorSetSpec {
    SensorSetSpec {
        name: name.into(),
        means: means.to_vec(),
        availability: Default::default(),
    }
}

pub(super) fn spec(camera: Option<TerrainCameraSpec>) -> ChainTrialSpec {
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
    let mut request = request_from_route(&route, None).unwrap();
    request.epoch_utc = "2026-06-15T12:04:00Z".into();
    request.output.progress_period_s = 1.0;
    use NavMeans::{Barometric, DeadReckoning, Gnss};
    request.sensor_sets = vec![
        set("ins.baro.gnss", &[DeadReckoning, Barometric, Gnss]),
        set("ins.baro", &[DeadReckoning, Barometric]),
    ];
    request.terrain_camera = camera;
    ChainTrialSpec {
        request,
        start: StartSpec {
            lat_deg: 41.70,
            lon_deg: -1.07,
            height_m: ellipsoidal_m(41.70, -1.07),
            heading_deg: 90.0,
            indicated_m_s: 55.0,
        },
        steering: Steering::FlightController,
        calibration: CalibrationName::Calibrated,
        outages: Vec::new(),
        hold: None,
        gnss_spoof: None,
    }
}

fn camera() -> TerrainCameraSpec {
    TerrainCameraSpec {
        band: CameraBand::Visible,
        width: 16,
        height: 12,
        fx_px: Some(16.0),
        horizontal_fov_deg: None,
        fy_px: None,
        cx_px: Some(8.0),
        cy_px: Some(6.0),
        mount: CameraMountSpec {
            offset_body_m: [0.0, 0.0, 0.0],
            roll_deg: 0.0,
            pitch_deg: -90.0,
            yaw_deg: 0.0,
        },
        period_s: 1.0,
        exposure_s: 0.05,
        radiometry: RadiometrySpec {
            gain: 1.0,
            offset_dn: 4.0,
            gamma: 1.0,
            noise_dn: 1.0,
        },
        min_sun_elevation_deg: 5.0,
    }
}

pub(super) fn advanced(outcome: StepOutcome) -> Outputs {
    match outcome {
        StepOutcome::Advanced(outputs) => outputs,
        StepOutcome::FrameWanted(request, _) => panic!("unexpected frame request {request:?}"),
    }
}

pub(super) fn samples_of<'a>(
    outputs: &'a Outputs,
    set: &'a str,
) -> impl Iterator<Item = &'a SampleRecord> {
    outputs
        .records
        .iter()
        .filter_map(move |record| match record {
            ProgressRecord::Sample(sample) if sample.set == set => Some(sample),
            _ => None,
        })
}

#[test]
fn the_sets_navigate_on_the_flight_controllers_devices() {
    let mut trial = ChainTrial::new(&spec(None), &DirectoryStores).unwrap();
    let outputs = advanced(trial.step_until(60.0).unwrap());
    assert!(matches!(
        outputs.records.first(),
        Some(ProgressRecord::Started(_))
    ));
    let aided: Vec<_> = samples_of(&outputs, "ins.baro.gnss").collect();
    assert_eq!(aided.len(), 60, "a sample a second");
    let worst = aided
        .iter()
        .map(|s| s.horizontal_error_m)
        .fold(0.0, f64::max);
    tracing::info!(worst, "GNSS-aided horizontal error");
    assert!(worst < 15.0, "GNSS-aided set {worst} m off");
    let baro_aiding = samples_of(&outputs, "ins.baro")
        .filter(|s| s.sensors.baro == SensorState::Aiding)
        .count();
    assert!(
        baro_aiding >= 55,
        "the altimeter aided {baro_aiding} of 60 samples"
    );
    let controller: Vec<_> = samples_of(&outputs, FLIGHT_CONTROLLER_SET).collect();
    assert!(!controller.is_empty());
    assert!(controller.iter().all(|s| s.horizontal_error_m < 15.0));
    assert!(
        outputs
            .mission
            .first()
            .and_then(|m| m.phase.as_deref())
            .is_some(),
        "Pilotage's phase is reported"
    );
}

#[test]
fn a_switch_flies_the_same_as_the_outage_the_spec_states() {
    let mut switched = ChainTrial::new(&spec(None), &DirectoryStores).unwrap();
    let mut first = advanced(switched.step_until(20.0).unwrap());
    assert_eq!(switched.set_sensor(SensorKind::Gnss, false), 20.0);
    first.extend(advanced(switched.step_until(30.0).unwrap()));
    assert_eq!(switched.set_sensor(SensorKind::Gnss, true), 30.0);
    first.extend(advanced(switched.step_until(40.0).unwrap()));

    let mut stated = spec(None);
    stated.outages = switched.outages().to_vec();
    assert_eq!(
        stated.outages,
        vec![DeviceOutage {
            sensor: SensorKind::Gnss,
            start_s: 20.0,
            end_s: Some(30.0),
        }]
    );
    let mut replay = ChainTrial::new(&stated, &DirectoryStores).unwrap();
    let second = advanced(replay.step_until(40.0).unwrap());
    // The digest of the request differs; everything flown does not.
    assert_eq!(first.records.get(1..), second.records.get(1..));
    assert_eq!(first.mission, second.mission);
    let starts = first
        .records
        .iter()
        .filter(|r| {
            matches!(r, ProgressRecord::Event(e)
                if matches!(e.kind, EventKind::AvailabilityStart { .. }))
        })
        .count();
    assert_eq!(starts, 1, "one set has the receiver");
    let withheld = samples_of(&first, "ins.baro.gnss")
        .filter(|s| s.sensors.gnss == SensorState::ScheduledUnavailable)
        .count();
    assert_eq!(withheld, 10);
}

#[test]
fn a_frame_is_asked_for_at_mid_exposure_and_awaited_at_readout() {
    let mut trial = ChainTrial::new(&spec(Some(camera())), &DirectoryStores).unwrap();
    let StepOutcome::FrameWanted(asked, _) = trial.step_until(5.0).unwrap() else {
        panic!("no frame was asked for");
    };
    assert_eq!(asked.index, 1);
    assert!((asked.mid_s - 1.0).abs() < 1e-9);
    let StepOutcome::FrameWanted(waiting, _) = trial.step_until(5.0).unwrap() else {
        panic!("the readout passed without its frame");
    };
    assert_eq!(waiting, asked);
    assert!(trial.now_s() <= asked.readout_s);
    assert!(
        trial.deliver_frame(1, &[0; 10]).is_err(),
        "a short image is refused"
    );
    assert!(
        trial.deliver_frame(2, &[0; 192]).is_err(),
        "frames come back in order"
    );
    trial.deliver_frame(1, &[128; 192]).unwrap();
    let StepOutcome::FrameWanted(next, _) = trial.step_until(5.0).unwrap() else {
        panic!("no second frame was asked for");
    };
    assert_eq!(next.index, 2);
    assert_eq!(trial.nav.log.terrain.len(), 1);
}

#[test]
fn a_set_on_the_stars_gets_the_star_cameras_frames() {
    let mut stars = spec(None);
    stars.request.sensor_sets.push(set(
        "ins.stars",
        &[NavMeans::DeadReckoning, NavMeans::Celestial],
    ));
    let mut trial = ChainTrial::new(&stars, &DirectoryStores).unwrap();
    let outputs = advanced(trial.step_until(10.0).unwrap());
    assert!(!trial.nav.log.stars.is_empty(), "no star frame was taken");
    assert!(
        samples_of(&outputs, "ins.stars").all(|s| s.sensors.stars != SensorState::NotInSet),
        "the set has the camera"
    );
}

#[test]
fn the_navigators_declare_the_inertial_unit_the_devices_simulate() {
    for calibration in [
        CalibrationName::Calibrated,
        CalibrationName::Uncalibrated20Mg,
    ] {
        let declared = setup::resolve(&ChainTrialSpec {
            calibration,
            ..spec(None)
        })
        .unwrap()
        .request
        .imu_grade
        .grade()
        .spec();
        let simulated =
            sim_flight_chain::c172_sensor_suite_with(2026.5, 1, calibration.calibration()).imu;
        assert_eq!(declared, simulated, "{calibration:?}");
    }
}

#[test]
fn pilotages_sequencing_reaches_the_progress_stream() {
    let mut trial = ChainTrial::new(&spec(None), &DirectoryStores).unwrap();
    let outputs = advanced(trial.step_until(60.0).unwrap());
    let sequenced: Vec<_> = outputs
        .records
        .iter()
        .filter_map(|r| match r {
            ProgressRecord::Event(e) => match &e.kind {
                EventKind::WaypointSequenced {
                    waypoint_index,
                    ident,
                    plan_complete,
                } => Some((e.t_s, *waypoint_index, ident.clone(), *plan_complete)),
                _ => None,
            },
            _ => None,
        })
        .collect();
    assert_eq!(sequenced.len(), 1, "{sequenced:?}");
    let (t_s, index, ident, complete) = sequenced[0].clone();
    assert_eq!((index, ident.as_str(), complete), (0, "WP0", false));
    assert!((20.0..35.0).contains(&t_s), "sequenced at {t_s} s");
}

/// About sixteen simulated minutes.
#[test]
#[ignore = "the whole route; run it by name"]
fn the_last_fix_sequenced_completes_the_plan() {
    let mut route = spec(None);
    route.request.sensor_sets.truncate(1);
    let mut trial = ChainTrial::new(&route, &DirectoryStores).unwrap();
    let outputs = advanced(trial.step_until(1_000.0).unwrap());
    let complete: Vec<_> = outputs
        .records
        .iter()
        .filter_map(|r| match r {
            ProgressRecord::Event(e) => match &e.kind {
                EventKind::WaypointSequenced {
                    waypoint_index,
                    plan_complete: true,
                    ..
                } => Some((e.t_s, *waypoint_index)),
                _ => None,
            },
            _ => None,
        })
        .collect();
    assert_eq!(complete.len(), 1, "{complete:?}");
    assert_eq!(complete[0].1, FIXES.len() - 1);
}

/// The feed carries the flight controller's account and the truth on the
/// same instants at the rate asked for, and nothing before it is asked.
#[test]
fn the_feed_keeps_its_rate_and_its_two_sides_apart() {
    let mut trial = ChainTrial::new(&spec(None), &DirectoryStores).unwrap();
    let quiet = advanced(trial.step_until(0.5).unwrap());
    assert!(quiet.avionics.is_empty() && quiet.truth.is_empty());
    trial.set_feed_rate_hz(Some(30.0)).unwrap();
    assert!(trial.set_feed_rate_hz(Some(0.0)).is_err());
    let fed = advanced(trial.step_until(2.5).unwrap());
    assert!(
        (59..=61).contains(&fed.avionics.len()),
        "{}",
        fed.avionics.len()
    );
    assert_eq!(fed.avionics.len(), fed.truth.len());
    for (avionics, truth) in fed.avionics.iter().zip(&fed.truth) {
        assert_eq!(avionics.at, truth.at);
    }
    let last = fed.avionics.last().unwrap();
    assert!(last.estimate.is_some() && last.air.is_some());
    trial.set_feed_rate_hz(None).unwrap();
    let after = advanced(trial.step_until(3.0).unwrap());
    assert!(after.avionics.is_empty());
    assert_eq!(trial.truth().at, trial.avionics().at);
}

/// Flies `trial` to `until_s`, handing back every frame it asks for.
fn fly_with_frames(trial: &mut ChainTrial, until_s: f64) -> crate::Retained {
    while trial.now_s() < until_s {
        if let StepOutcome::FrameWanted(asked, _) = trial.step_until(until_s).unwrap() {
            trial.deliver_frame(asked.index, &[128; 192]).unwrap();
        }
    }
    trial.retained()
}

/// The most a log keeps whatever the flight's length: the steps of one star
/// exposure and a few more, the readings and frames of those steps, and the
/// pixels of the one terrain frame read out but not yet taken.
const KEPT_STEPS: usize = 64;
const KEPT_READINGS: usize = 16;
const KEPT_FRAMES: usize = 4;
const KEPT_BYTES: usize = 64 * 1024;

fn assert_bounded(retained: &crate::Retained) {
    assert!(retained.steps_kept <= KEPT_STEPS, "{retained:?}");
    assert!(retained.readings_kept <= KEPT_READINGS, "{retained:?}");
    assert!(retained.frames_kept <= KEPT_FRAMES, "{retained:?}");
    assert!(retained.terrain_pixel_bytes <= 192, "{retained:?}");
    assert!(retained.approximate_bytes <= KEPT_BYTES, "{retained:?}");
}

#[test]
fn a_long_flights_log_keeps_only_what_a_reader_has_yet_to_take() {
    let mut flight = spec(Some(camera()));
    flight.request.sensor_sets.push(set(
        "ins.stars",
        &[NavMeans::DeadReckoning, NavMeans::Celestial],
    ));
    let mut trial = ChainTrial::new(&flight, &DirectoryStores).unwrap();
    let early = fly_with_frames(&mut trial, 5.0);
    let late = fly_with_frames(&mut trial, 40.0);
    assert!(late.steps >= early.steps * 7, "{early:?} {late:?}");
    assert!(
        !trial.nav.log.stars.is_empty() && trial.nav.log.terrain.len() > 30,
        "both cameras took frames"
    );
    // Eight times the flight, the same bound: what is kept follows the
    // readers' lag, so an hours-long hold keeps no more than this.
    assert_bounded(&early);
    assert_bounded(&late);
}

/// The link's waypoint names are the plan's: Pilotage's navigation display,
/// its leg and its holding pattern after the plan name the fixes as the
/// link does, and so does the hold in the progress stream.
#[test]
fn the_links_waypoint_names_reach_the_display_and_the_hold() {
    let mut short = spec(None);
    let fixes = [(41.70, -1.055, "ZA1"), (41.70, -1.04, "ZA2")];
    short.request.waypoints.truncate(2);
    for (waypoint, (lat_deg, lon_deg, ident)) in short.request.waypoints.iter_mut().zip(fixes) {
        waypoint.lat_deg = lat_deg;
        waypoint.lon_deg = lon_deg;
        waypoint.ident = ident.to_owned();
    }
    short.request.sensor_sets.truncate(1);
    let mut trial = ChainTrial::new(&short, &DirectoryStores).unwrap();
    advanced(trial.step_until(10.0).unwrap());
    let names: Vec<String> = trial
        .navigation()
        .unwrap()
        .waypoints
        .iter()
        .map(|w| w.fix.ident.clone())
        .collect();
    assert_eq!(names, ["ZA1", "ZA2"]);
    assert_eq!(
        trial.navigation().unwrap().leg.as_ref().unwrap().to.ident,
        "ZA1"
    );
    let mut held = None;
    while trial.now_s() < 150.0 && held.is_none() {
        let outputs = advanced(trial.step_until(trial.now_s() + 5.0).unwrap());
        held = outputs.records.into_iter().find_map(|record| match record {
            ProgressRecord::Event(event) => match event.kind {
                EventKind::HoldingStarted { pattern } => Some(pattern.fix_ident),
                _ => None,
            },
            _ => None,
        });
    }
    assert_eq!(held.as_deref(), Some("ZA2"));
    let hold = trial.navigation().unwrap().holding_pattern.clone().unwrap();
    assert_eq!(hold.pattern.fix_ident, "ZA2");
}

/// A short route, so the plan ends within two simulated minutes and the
/// hold the spec states starts: the turns, leg time and altitude reach
/// Pilotage's mission as stated.
#[test]
fn the_stated_hold_is_the_one_pilotage_flies() {
    let fixes = [(41.70, -1.05), (41.70, -1.02)];
    let route: Vec<RoutePoint> = fixes
        .iter()
        .map(|&(lat_deg, lon_deg)| RoutePoint {
            ident: None,
            lat_deg,
            lon_deg,
            altitude_m: ellipsoidal_m(lat_deg, lon_deg),
            speed_m_s: None,
        })
        .collect();
    let mut stated = spec(None);
    stated.request.waypoints = request_from_route(&route, None).unwrap().waypoints;
    stated.request.sensor_sets.truncate(1);
    let altitude_m = ellipsoidal_m(41.70, -1.02) + 100.0;
    stated.hold = Some(crate::HoldSpec {
        turn: crate::HoldTurn::Left,
        leg_time_s: 90.0,
        altitude_m: Some(altitude_m),
    });
    let mut trial = ChainTrial::new(&stated, &DirectoryStores).unwrap();
    let outputs = advanced(trial.step_until(150.0).unwrap());
    let started: Vec<_> = outputs
        .records
        .iter()
        .filter_map(|r| match r {
            ProgressRecord::Event(e) => match &e.kind {
                EventKind::HoldingStarted { pattern } => Some(pattern.clone()),
                _ => None,
            },
            _ => None,
        })
        .collect();
    let [pattern] = started.as_slice() else {
        panic!("one hold started: {started:?}");
    };
    assert_eq!(pattern.turn, sim_navigate::trial::HoldingTurnName::Left);
    assert_eq!(pattern.leg_time_s, 90.0);
    assert_eq!(pattern.fix.altitude_m, altitude_m);
}
