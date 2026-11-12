use sim_navigate::trial::{ProgressRecord, SensorKind};

use super::{SensorFault, SpoofDrag, sensor_controls};
use crate::{Monitor, MonitorFeed};

const STARTED: &str = r#"{"type":"started","schema":1,"request_sha256":"00","epoch_utc":"2026-06-15T12:00:00Z","route":{"waypoints":[{"ident":"Z0","lat_deg":41.7,"lon_deg":-1.05,"altitude_m":2000.0},{"ident":"Z1","lat_deg":41.7,"lon_deg":-0.8,"altitude_m":2000.0}],"planned_duration_s":500.0},"sets":[{"name":"ins.baro.gnss","means":["dead_reckoning","barometric","gnss"]},{"name":"ins.baro","means":["dead_reckoning","barometric"]},{"name":"flight-controller","means":[]}],"guidance":"estimate"}"#;

const BY_HAND: SpoofDrag = SpoofDrag {
    rate_m_s: 1.0,
    bearing_deg: 90.0,
};

fn feed(monitor: &mut Monitor, line: &str) {
    monitor.apply(MonitorFeed::Progress(
        ProgressRecord::from_json_line(line).unwrap(),
    ));
}

fn sample(t_s: f64, set: &str, error_m: f64, sigma: &str, gnss: &str) -> String {
    format!(
        r#"{{"type":"sample","t_s":{t_s},"set":"{set}","truth":{{"lat_deg":41.7,"lon_deg":-1.0,"altitude_m":2000.0}},"estimate":{{"lat_deg":41.7,"lon_deg":-1.0,"altitude_m":2000.0}},"horizontal_error_m":{error_m},"vertical_error_m":0.5,"sigma_horizontal_m":{sigma},"cross_track_true_m":null,"sensors":{{"gnss":"{gnss}","baro":"aiding","stars":"not_in_set","terrain_camera":"not_in_set"}}}}"#
    )
}

fn samples(monitor: &mut Monitor, t_s: f64, gnss: &str, error_m: f64) {
    feed(monitor, &sample(t_s, "ins.baro.gnss", error_m, "1.4", gnss));
    feed(
        monitor,
        &sample(t_s, "ins.baro", 300.0, "450.0", "not_in_set"),
    );
    feed(
        monitor,
        &sample(t_s, "flight-controller", error_m, "null", gnss),
    );
}

#[test]
fn a_run_has_a_stop_for_each_sensor_it_uses_and_a_spoofer_with_gnss() {
    let mut monitor = Monitor::default();
    assert!(sensor_controls(&monitor, BY_HAND).is_empty());
    feed(&mut monitor, STARTED);
    let faults: Vec<SensorFault> = sensor_controls(&monitor, BY_HAND)
        .iter()
        .map(|control| control.fault)
        .collect();
    assert_eq!(
        faults,
        [
            SensorFault::Stop(SensorKind::Gnss),
            SensorFault::Stop(SensorKind::Baro),
            SensorFault::SpoofGnss
        ]
    );
    let without_gnss = STARTED.replace(r#""barometric","gnss"]"#, r#""barometric","visual"]"#);
    feed(&mut monitor, &without_gnss);
    assert!(
        sensor_controls(&monitor, BY_HAND)
            .iter()
            .all(|control| control.fault != SensorFault::SpoofGnss)
    );
}

#[test]
fn a_stop_names_every_consumer_of_the_device_and_no_other() {
    let mut monitor = Monitor::default();
    feed(&mut monitor, STARTED);
    samples(&mut monitor, 10.0, "aiding", 1.0);
    let controls = sensor_controls(&monitor, BY_HAND);
    let gnss = &controls[0];
    assert_eq!(gnss.label, "Stop GNSS");
    assert!(!gnss.active);
    assert_eq!(
        gnss.state,
        "delivering · ins.baro.gnss aiding · flight-controller aiding"
    );
    assert_eq!(
        controls[1].state,
        "delivering · ins.baro.gnss aiding · ins.baro aiding · flight-controller aiding"
    );
    samples(&mut monitor, 11.0, "scheduled_unavailable", 1.0);
    let gnss = &sensor_controls(&monitor, BY_HAND)[0];
    assert!(gnss.active);
    assert_eq!(
        gnss.state,
        "STOPPED for every consumer · ins.baro.gnss withheld · flight-controller withheld"
    );
}

#[test]
fn the_spoofer_states_its_drag_and_what_each_consumer_reports_of_it() {
    let mut monitor = Monitor::default();
    feed(&mut monitor, STARTED);
    samples(&mut monitor, 10.0, "aiding", 1.0);
    let off = &sensor_controls(&monitor, BY_HAND)[2];
    assert!(!off.active);
    assert_eq!(
        off.state,
        "true sky · on, every consumer's GNSS position is dragged 1 m/s toward 090°, its \
         velocity with it"
    );
    feed(
        &mut monitor,
        r#"{"type":"event","t_s":65.0,"set":null,"kind":"spoofing_start","sensor":"gnss","rate_m_s":2.0,"bearing_deg":45.0}"#,
    );
    samples(&mut monitor, 125.0, "aiding", 118.0);
    feed(
        &mut monitor,
        &sample(125.0, "ins.baro.gnss", 12.0, "9.5", "rejected"),
    );
    let on = &sensor_controls(&monitor, BY_HAND)[2];
    assert!(on.active);
    assert_eq!(
        on.state,
        "SPOOFING since 1:05, reported position 120 m from true toward 045° · ins.baro.gnss \
         refused by the filter, 12 m off, σ 9.5 · flight-controller aiding, 118 m off"
    );
    feed(
        &mut monitor,
        r#"{"type":"event","t_s":130.0,"set":null,"kind":"spoofing_end","sensor":"gnss"}"#,
    );
    assert!(!sensor_controls(&monitor, BY_HAND)[2].active);
}

#[test]
fn each_gnss_sets_check_reads_as_its_navigator_states_it() {
    use sim_navigate::trial::{GnssCheck, GnssCheckSeparation, GnssCheckState, GnssReference};

    use super::gnss_check_lines;

    let mut monitor = Monitor::default();
    feed(&mut monitor, STARTED);
    assert!(gnss_check_lines(&monitor).is_empty());
    let last = Some(GnssCheckSeparation {
        separation_m: 31.4,
        caution_bound_m: 12.2,
        exclusion_bound_m: 18.0,
    });
    let check = |state, reference, checkable| GnssCheck {
        state,
        since_s: Some(92.0),
        last,
        reference,
        checkable,
    };
    let inertial = GnssReference::InertialOnly {
        detectable_rate_m_s: Some(0.94),
    };
    let mut say = |set: &str, check: GnssCheck| {
        monitor.apply(MonitorFeed::GnssCheck {
            set: set.to_owned(),
            check,
        });
        let lines = gnss_check_lines(&monitor);
        let line = lines.iter().find(|line| line.set == set).unwrap();
        (line.words.clone(), line.alert)
    };
    assert_eq!(
        say(
            "ins.baro.gnss",
            check(GnssCheckState::Excluded, GnssReference::PositionFix, true)
        ),
        (
            "GNSS unreliable — excluded since 1:32, separation 31 m (caution 12 m, exclusion \
             18 m), checked against its camera-fixed solution"
                .to_owned(),
            true
        )
    );
    assert_eq!(
        say(
            "ins.baro.gnss",
            check(GnssCheckState::Disagreeing, inertial, true)
        ),
        (
            "GNSS doubted since 1:32, separation 31 m (caution 12 m, exclusion 18 m), still \
             used, checked against inertial navigation only, which cannot check a drift below \
             0.9 m/s"
                .to_owned(),
            true
        )
    );
    // The switches' lines name the exclusion too, in place of a device state.
    samples(&mut monitor, 100.0, "physically_unavailable", 6.0);
    monitor.apply(MonitorFeed::GnssCheck {
        set: "ins.baro.gnss".to_owned(),
        check: check(GnssCheckState::Excluded, GnssReference::PositionFix, true),
    });
    assert_eq!(
        sensor_controls(&monitor, BY_HAND)[0].state,
        "delivering · ins.baro.gnss excluded by its navigator · flight-controller no reading"
    );
    let mut say = |set: &str, check: GnssCheck| {
        monitor.apply(MonitorFeed::GnssCheck {
            set: set.to_owned(),
            check,
        });
        let lines = gnss_check_lines(&monitor);
        let line = lines.iter().find(|line| line.set == set).unwrap();
        (line.words.clone(), line.alert)
    };
    let (words, alert) = say(
        "ins.baro.gnss",
        check(GnssCheckState::Consistent, inertial, true),
    );
    assert!(words.starts_with("GNSS consistent, separation 31 m") && !alert);
    let (words, alert) = say(
        "ins.baro.gnss",
        check(GnssCheckState::Excluded, inertial, false),
    );
    assert!(words.starts_with("GNSS unchecked") && !alert, "{words}");
}
