use std::cell::Cell;
use std::collections::BTreeMap;
use std::convert::Infallible;

use sim_navigate::trial::{ProgressRecord, SensorKind, SensorState};

use super::{
    Clock, EVENTS_KEPT, FlightStepper, MAX_FRAME_S, Monitor, MonitorFeed, PaceLimit, Pacer, Replay,
    Slice, StepState, run_slice,
};

const STARTED: &str = r#"{"type":"started","schema":1,"request_sha256":"00","epoch_utc":"2026-06-15T12:00:00Z","route":{"waypoints":[{"ident":"Z0","lat_deg":41.7,"lon_deg":-1.05,"altitude_m":2000.0},{"ident":"Z1","lat_deg":41.7,"lon_deg":-0.8,"altitude_m":2000.0}],"planned_duration_s":500.0},"sets":[{"name":"ins+baro","means":["dead_reckoning","barometric"]},{"name":"ins+baro+visual","means":["dead_reckoning","visual","barometric"]}],"guidance":"truth"}"#;

fn sample(t_s: f64, set: &str, error_m: f64) -> String {
    format!(
        r#"{{"type":"sample","t_s":{t_s},"set":"{set}","truth":{{"lat_deg":41.7,"lon_deg":-1.0,"altitude_m":2000.0}},"estimate":{{"lat_deg":41.7,"lon_deg":-1.0,"altitude_m":2000.0}},"horizontal_error_m":{error_m},"vertical_error_m":0.5,"sigma_horizontal_m":{sigma},"cross_track_true_m":null,"sensors":{{"gnss":"not_in_set","baro":"aiding","stars":"not_in_set","terrain_camera":"rejected"}}}}"#,
        sigma = error_m * 2.0
    )
}

fn waypoint(t_s: f64, index: usize, ident: &str) -> String {
    format!(
        r#"{{"type":"event","t_s":{t_s},"set":null,"kind":"waypoint_sequenced","waypoint_index":{index},"ident":"{ident}","plan_complete":false}}"#
    )
}

const FINISHED: &str = r#"{"type":"finished","result_path":null,"wall_time_s":2.0}"#;

fn record(line: &str) -> MonitorFeed {
    MonitorFeed::Progress(ProgressRecord::from_json_line(line).unwrap())
}

fn stream() -> String {
    [
        STARTED.to_owned(),
        waypoint(0.0, 0, "Z0"),
        sample(1.0, "ins+baro", 2.0),
        sample(1.0, "ins+baro+visual", 3.0),
        sample(2.0, "ins+baro", 5.0),
        sample(2.0, "ins+baro+visual", 4.0),
        waypoint(2.5, 1, "Z1"),
        sample(3.0, "ins+baro", 9.0),
        sample(3.0, "ins+baro+visual", 1.0),
        FINISHED.to_owned(),
    ]
    .join("\n")
}

#[test]
fn the_monitor_follows_the_stream() {
    let mut monitor = Monitor::default();
    for line in stream().lines() {
        monitor.apply(record(line));
    }
    let names: Vec<&str> = monitor.traces().iter().map(|t| t.name.as_str()).collect();
    assert_eq!(names, ["ins+baro", "ins+baro+visual"]);
    assert_eq!(monitor.traces()[0].points().len(), 3);
    assert_eq!(monitor.traces()[0].max_error_m(), Some(9.0));
    assert_eq!(
        monitor.traces()[1].latest().unwrap().horizontal_error_m,
        1.0
    );
    assert_eq!(monitor.last_waypoint(), Some((1, "Z1")));
    assert_eq!(monitor.sim_time_s(), 3.0);
    assert_eq!(monitor.planned_duration_s(), Some(500.0));
    assert!(monitor.finished());
    assert_eq!(
        monitor.sensors("ins+baro+visual").unwrap().terrain_camera,
        SensorState::Rejected
    );
    assert_eq!(monitor.phase(), None);
    monitor.apply(MonitorFeed::Phase("Enroute".to_owned()));
    assert_eq!(monitor.phase(), Some("Enroute"));
}

#[test]
fn a_new_start_clears_the_last_run() {
    let mut monitor = Monitor::default();
    for line in stream().lines() {
        monitor.apply(record(line));
    }
    monitor.apply(record(STARTED));
    assert!(monitor.traces().iter().all(|t| t.points().is_empty()));
    assert!(!monitor.finished() && monitor.events().is_empty());
}

#[test]
fn only_the_means_of_the_run_have_switches() {
    let mut monitor = Monitor::default();
    assert!(monitor.switches().is_empty());
    monitor.apply(record(STARTED));
    let sensors: Vec<SensorKind> = monitor.switches().iter().map(|s| s.sensor).collect();
    assert_eq!(sensors, [SensorKind::Baro, SensorKind::TerrainCamera]);
    let with_gnss = STARTED.replace(r#""barometric"]}]"#, r#""barometric","gnss"]}]"#);
    monitor.apply(record(&with_gnss));
    let sensors: Vec<SensorKind> = monitor.switches().iter().map(|s| s.sensor).collect();
    assert_eq!(
        sensors,
        [
            SensorKind::Gnss,
            SensorKind::Baro,
            SensorKind::TerrainCamera
        ]
    );
}

#[test]
fn unused_frames_count_by_reason_matched_or_not() {
    let mut monitor = Monitor::default();
    assert_eq!(monitor.unused_reasons("ins+baro+visual"), None);
    let fixes: BTreeMap<String, u64> = [
        ("accepted", 155),
        ("context:none", 797),
        ("unused:shared_map_error:matched", 453),
        ("unused:shared_map_error:unmatched", 797),
        ("unused:featureless_frame:unmatched", 20),
        ("unused:outside_coverage:unmatched", 20),
        ("rejected:innovation", 3),
    ]
    .into_iter()
    .map(|(key, count)| (key.to_owned(), count))
    .collect();
    monitor.apply(MonitorFeed::TerrainFixes {
        set: "ins+baro+visual".to_owned(),
        fixes,
    });
    let (accepted, reasons) = monitor.unused_reasons("ins+baro+visual").unwrap();
    assert_eq!(accepted, 155);
    assert_eq!(
        reasons,
        [
            ("shared_map_error".to_owned(), 1250),
            ("featureless_frame".to_owned(), 20),
            ("outside_coverage".to_owned(), 20),
        ]
    );
}

#[test]
fn the_event_log_keeps_the_latest() {
    let mut monitor = Monitor::default();
    monitor.apply(record(STARTED));
    for index in 0..EVENTS_KEPT + 5 {
        monitor.apply(record(&waypoint(index as f64, index, "W")));
    }
    assert_eq!(monitor.events().len(), EVENTS_KEPT);
    assert_eq!(monitor.events()[0].t_s, 5.0);
}

#[test]
fn a_trace_reduces_to_columns() {
    let mut monitor = Monitor::default();
    for line in stream().lines() {
        monitor.apply(record(line));
    }
    let trace = &monitor.traces()[0];
    let columns = trace.columns(0.0, 4.0, 4);
    assert_eq!(columns[0], None);
    assert_eq!(columns[1].unwrap().max_m, 2.0);
    assert_eq!(columns[2].unwrap().min_m, 5.0);
    assert_eq!(columns[3].unwrap().sigma_m, Some(18.0));
    let squeezed = trace.columns(1.0, 3.0, 1);
    let only = squeezed[0].unwrap();
    assert_eq!((only.min_m, only.max_m), (2.0, 9.0));
    assert!(trace.columns(3.0, 3.0, 4).iter().all(Option::is_none));
    assert!(trace.columns(0.0, 4.0, 0).is_empty());
}

#[test]
fn a_replay_hands_over_one_recorded_instant_a_step() {
    let mut replay = Replay::from_lines(&format!("{}\n\n", stream())).unwrap();
    assert_eq!(replay.len(), 10);
    let mut out = Vec::new();
    assert_eq!(replay.step(f64::INFINITY, &mut out), Ok(StepState::Running));
    assert_eq!((out.len(), replay.sim_time_s()), (2, 0.0));
    let mut times = vec![replay.sim_time_s()];
    while replay.step(f64::INFINITY, &mut out) == Ok(StepState::Running) {
        times.push(replay.sim_time_s());
    }
    times.push(replay.sim_time_s());
    assert_eq!(times, [0.0, 1.0, 2.0, 2.5, 3.0]);
    let all: Vec<MonitorFeed> = stream().lines().map(record).collect();
    assert_eq!(out, all);
    assert_eq!(
        replay.step(f64::INFINITY, &mut out),
        Ok(StepState::Finished)
    );
    assert_eq!(out.len(), all.len());
}

#[test]
fn a_replay_names_the_line_it_cannot_read() {
    let text = format!("{STARTED}\n{{\"type\":\"sample\"}}\n");
    let error = Replay::from_lines(&text).unwrap_err().to_string();
    assert!(
        error.starts_with("line 2 of the recorded progress stream"),
        "{error}"
    );
}

/// A flight of `ticks` fixed ticks that says which tick it is.
struct Ticks {
    tick: u32,
    ticks: u32,
}

impl FlightStepper for Ticks {
    type Error = Infallible;

    fn sim_time_s(&self) -> f64 {
        f64::from(self.tick) * 0.1
    }

    fn step(
        &mut self,
        _target_s: f64,
        out: &mut Vec<MonitorFeed>,
    ) -> Result<StepState, Infallible> {
        self.tick = self.tick.wrapping_add(1);
        out.push(MonitorFeed::Phase(self.tick.to_string()));
        Ok(if self.tick >= self.ticks {
            StepState::Finished
        } else {
            StepState::Running
        })
    }
}

/// A wall clock that moves `cost_s` each time it is read.
struct Costly {
    now: Cell<f64>,
    cost_s: f64,
}

impl Clock for Costly {
    fn now_s(&self) -> f64 {
        let now = self.now.get() + self.cost_s;
        self.now.set(now);
        now
    }
}

#[test]
fn slicing_never_changes_what_the_flight_says() {
    let fly = |budget_s: f64| {
        let mut flight = Ticks { tick: 0, ticks: 50 };
        let clock = Costly {
            now: Cell::new(0.0),
            cost_s: 0.003,
        };
        let (mut out, mut slices) = (Vec::new(), 0);
        while !run_slice(&mut flight, &clock, budget_s, f64::INFINITY, &mut out)
            .unwrap()
            .finished
        {
            slices += 1;
        }
        (out, slices)
    };
    let (whole, one) = fly(1.0);
    assert_eq!((whole.len(), one), (50, 0));
    for budget_s in [0.0, 0.001, 0.008, 0.02] {
        let (sliced, slices) = fly(budget_s);
        assert_eq!(sliced, whole, "budget {budget_s}");
        assert!(slices > 0, "budget {budget_s}");
    }
}

#[test]
fn a_slice_stops_at_its_target_and_its_budget() {
    let clock = Costly {
        now: Cell::new(0.0),
        cost_s: 0.004,
    };
    let mut flight = Ticks {
        tick: 0,
        ticks: 100,
    };
    let mut out = Vec::new();
    let slice = run_slice(&mut flight, &clock, 1.0, 0.35, &mut out).unwrap();
    assert_eq!((slice.steps, flight.tick), (4, 4));
    let slice = run_slice(&mut flight, &clock, 0.0, 10.0, &mut out).unwrap();
    assert_eq!(
        slice.steps, 1,
        "a flight behind its target always steps once"
    );
    let slice = run_slice(&mut flight, &clock, 1.0, 0.0, &mut out).unwrap();
    assert_eq!(slice.steps, 0);
}

#[test]
fn the_pace_follows_the_wall_clock_without_a_debt() {
    let mut pacer = Pacer::new(Some(10.0));
    assert_eq!(pacer.frame(100.0, 0.0), 0.0);
    assert!((pacer.frame(100.05, 0.0) - 0.5).abs() < 1e-9);
    // The tab was hidden for a minute: the pace resumes, it does not race.
    let resumed = pacer.frame(160.05, 0.5);
    assert!((resumed - (0.5 + MAX_FRAME_S * 10.0)).abs() < 1e-9);
    // A flight that cannot keep up falls behind; its target stays close.
    let mut target = 0.0;
    for frame in 1..=100 {
        target = pacer.frame(160.05 + f64::from(frame) * 0.05, 1.0);
    }
    assert!(target <= 1.0 + MAX_FRAME_S * 10.0 + 1e-9, "{target}");
    pacer.set_speed(None);
    assert_eq!(pacer.speed(), None);
    assert_eq!(pacer.frame(200.0, 1.0), f64::INFINITY);
}

/// A flight whose tick is one route second, as a recorded run at 1 Hz.
struct Seconds(u32);

impl FlightStepper for Seconds {
    type Error = Infallible;

    fn sim_time_s(&self) -> f64 {
        f64::from(self.0)
    }

    fn step(
        &mut self,
        _target_s: f64,
        _out: &mut Vec<MonitorFeed>,
    ) -> Result<StepState, Infallible> {
        self.0 = self.0.wrapping_add(1);
        Ok(StepState::Running)
    }
}

#[test]
fn a_tick_longer_than_a_frame_still_keeps_the_pace() {
    let clock = Costly {
        now: Cell::new(0.0),
        cost_s: 0.0,
    };
    let mut flight = Seconds(0);
    let mut pacer = Pacer::new(Some(16.0));
    let mut out = Vec::new();
    // Ten wall seconds at 120 frames a second.
    for frame in 0..=1200 {
        let target_s = pacer.frame(f64::from(frame) / 120.0, flight.sim_time_s());
        run_slice(&mut flight, &clock, 0.008, target_s, &mut out).unwrap();
    }
    assert!(
        (159..=161).contains(&flight.0),
        "{} route seconds",
        flight.0
    );
    // A change of speed starts the pace again from where the flight is.
    pacer.set_speed(Some(1.0));
    for frame in 1201..=1320 {
        let target_s = pacer.frame(f64::from(frame) / 120.0, flight.sim_time_s());
        run_slice(&mut flight, &clock, 0.008, target_s, &mut out).unwrap();
    }
    assert!(
        (160..=162).contains(&flight.0),
        "{} route seconds",
        flight.0
    );
}

#[test]
fn the_pace_is_held_to_what_the_page_can_afford() {
    let slice = |route_s: f64, wall_s: f64| Slice {
        steps: 1,
        finished: false,
        route_s,
        wall_s,
    };
    let mut pacer = Pacer::new(Some(16.0));
    // One route second costs 0.25 s of wall time, 0.2 s of it rendering.
    pacer.observe(&slice(1.0, 0.25), 0.2);
    assert_eq!(pacer.limit(), None, "too little measured yet");
    for _ in 0..40 {
        pacer.observe(&slice(1.0, 0.25), 0.2);
    }
    let limit = pacer.limit().unwrap();
    assert!((limit.speed - 2.0).abs() < 1e-9, "{limit:?}");
    assert!(limit.by_camera);
    // Held: ten wall seconds at 60 frames move the target 20 route seconds.
    let mut target = pacer.frame(0.0, 0.0);
    for frame in 1..=600 {
        target = pacer.frame(f64::from(frame) / 60.0, target);
    }
    assert!((target - 20.0).abs() < 1e-6, "{target}");
    // A pace the page affords is not held, nor is as fast as possible.
    pacer.set_speed(Some(1.0));
    assert_eq!(pacer.limit(), None);
    pacer.set_speed(None);
    assert_eq!(pacer.limit(), None);
    let cheap: Option<PaceLimit> = {
        let mut cheap = Pacer::new(Some(16.0));
        for _ in 0..40 {
            cheap.observe(&slice(1.0, 0.25), 0.01);
        }
        cheap.limit()
    };
    assert_eq!(cheap.map(|limit| limit.by_camera), Some(false));
}

#[test]
fn a_slice_reports_what_it_stepped_and_what_it_took() {
    let clock = Costly {
        now: Cell::new(0.0),
        cost_s: 0.001,
    };
    let mut flight = Ticks {
        tick: 0,
        ticks: 100,
    };
    let slice = run_slice(&mut flight, &clock, 1.0, 0.5, &mut Vec::new()).unwrap();
    assert!((slice.route_s - 0.5).abs() < 1e-9, "{slice:?}");
    assert!(slice.wall_s > 0.0);
}

#[test]
fn the_route_end_stays_as_the_plan_completed_while_the_hold_flies_on() {
    let mut monitor = Monitor::default();
    let complete = r#"{"type":"event","t_s":2.5,"set":null,"kind":"waypoint_sequenced","waypoint_index":1,"ident":"Z1","plan_complete":true}"#;
    for line in [
        STARTED.to_owned(),
        sample(2.0, "ins+baro", 5.0),
        sample(2.0, "ins+baro+visual", 4.0),
        complete.to_owned(),
        sample(3.0, "ins+baro", 9.0),
        sample(3.0, "ins+baro+visual", 1.0),
        complete.replace("2.5", "3.5"),
    ] {
        monitor.apply(record(&line));
    }
    let end = monitor.route_end().unwrap();
    assert_eq!((end.t_s, end.ident.as_str()), (2.5, "Z1"));
    let errors: Vec<Option<f64>> = end
        .samples
        .iter()
        .map(|sample| sample.map(|s| s.horizontal_error_m))
        .collect();
    assert_eq!(errors, [Some(5.0), Some(4.0)]);
    assert_eq!(
        monitor.traces()[0].latest().unwrap().horizontal_error_m,
        9.0
    );
    assert!(!monitor.finished());
    let lines = super::run_lines("", &monitor, &Pacer::new(Some(1.0)), None);
    assert_eq!(lines[2], "Plan done at Z1 · 0:02");
}

mod route;
