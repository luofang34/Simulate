use sim_geo::egm96_undulation_m;
use sim_navigate::NavMeans;
use sim_navigate::trial::{
    CameraBand, CameraMountSpec, DirectoryStores, EventKind, ProgressRecord, RadiometrySpec,
    RoutePoint, SampleRecord, SensorSetSpec, SensorState, TerrainCameraSpec, request_from_route,
};

use super::*;
use crate::{CalibrationName, StartSpec};

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

fn spec(camera: Option<TerrainCameraSpec>) -> ChainTrialSpec {
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

fn advanced(outcome: StepOutcome) -> Outputs {
    match outcome {
        StepOutcome::Advanced(outputs) => outputs,
        StepOutcome::FrameWanted(request, _) => panic!("unexpected frame request {request:?}"),
    }
}

fn samples_of<'a>(outputs: &'a Outputs, set: &'a str) -> impl Iterator<Item = &'a SampleRecord> {
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
