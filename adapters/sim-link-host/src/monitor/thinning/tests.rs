use super::{SAMPLES_KEPT, Thinned};

#[test]
fn a_series_stays_within_its_limit_and_keeps_its_span_and_peaks() {
    let larger = |a: (u64, u64), b: (u64, u64)| if b.1 > a.1 { b } else { a };
    let mut series = Thinned::new(larger);
    let total = SAMPLES_KEPT as u64 * 10;
    for t in 0..total {
        let value = if t == 12_345 { 1_000_000 } else { t % 7 };
        series.push((t, value));
        assert!(series.items().len() < SAMPLES_KEPT);
    }
    let items = series.items();
    // Every kept sample stands for the same span: the first stands for the start.
    assert!(items.first().map(|p| p.0) < Some(16), "{:?}", items.first());
    assert!(items.last().map(|p| p.0) > Some(total - 32));
    assert!(items.len() > SAMPLES_KEPT / 3);
    assert!(items.iter().any(|p| p.1 == 1_000_000), "the peak survives");
    assert!(items.windows(2).all(|pair| pair[0].0 < pair[1].0));
    assert_eq!(series.latest().map(|p| p.0), Some(total - 1));
}

/// A monitor fed a run many times longer than its series keep stays within them:
/// what a flight that holds indefinitely leaves on the page is bounded.
#[test]
fn a_long_runs_monitor_keeps_bounded_series() {
    use sim_navigate::trial::{
        ProgressPosition, ProgressRecord, SampleRecord, SensorState, SensorStates,
    };

    use crate::{Monitor, MonitorFeed};

    let started = r#"{"type":"started","schema":2,"request_sha256":"00","epoch_utc":"2026-06-15T12:00:00Z","route":{"waypoints":[],"planned_duration_s":60.0},"sets":[{"name":"a","means":["dead_reckoning"]}],"guidance":"truth"}"#;
    let mut monitor = Monitor::default();
    monitor.apply(MonitorFeed::Progress(
        ProgressRecord::from_json_line(started).unwrap(),
    ));
    let at = ProgressPosition {
        lat_deg: 41.7,
        lon_deg: -1.0,
        altitude_m: 2000.0,
    };
    let states = SensorStates {
        gnss: SensorState::NotInSet,
        baro: SensorState::NotInSet,
        stars: SensorState::NotInSet,
        terrain_camera: SensorState::NotInSet,
    };
    let samples = SAMPLES_KEPT * 6;
    for step in 0..samples {
        monitor.apply(MonitorFeed::Progress(ProgressRecord::Sample(
            SampleRecord {
                t_s: step as f64,
                set: "a".to_owned(),
                truth: at,
                estimate: at,
                horizontal_error_m: step as f64,
                vertical_error_m: 0.0,
                sigma_horizontal_m: None,
                cross_track_true_m: None,
                sensors: states,
            },
        )));
    }
    let trace = &monitor.traces()[0];
    assert!(trace.points().len() < SAMPLES_KEPT);
    assert!(monitor.truth().len() < SAMPLES_KEPT);
    assert_eq!(trace.latest().map(|p| p.t_s), Some((samples - 1) as f64));
    assert_eq!(trace.max_error_m(), Some((samples - 1) as f64));
}
