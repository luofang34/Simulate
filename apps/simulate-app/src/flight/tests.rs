use glam::DVec3;
use sim_core::{ControlInputs, SimInstant};
use sim_scenario::{ScenarioProfile, SimTimeScale};

use super::rewind::{CHECKPOINT_INTERVAL_S, MAX_CHECKPOINTS, seconds};
use super::*;
use crate::host_guidance::StandInPilotageHost;
use crate::vehicles::FreeFlightVehicle;

const CHUNK_S: f64 = 0.25;

fn cruise() -> ControlInputs {
    ControlInputs {
        throttle: 0.72,
        ..ControlInputs::default()
    }
}

/// Flies `total_s` in quarter-second frames, recording `(instant, position)` after each.
fn fly(flight: &mut Flight, total_s: f64) -> Vec<(SimInstant, DVec3)> {
    let frames = (total_s / CHUNK_S).round() as usize;
    (0..frames)
        .map(|_| {
            flight
                .advance_by(CHUNK_S, cruise(), SimTimeScale::RealTime)
                .expect("flight step");
            (flight.now(), flight.vehicle_state().position_ecef)
        })
        .collect()
}

fn recorded_at(track: &[(SimInstant, DVec3)], at: SimInstant) -> DVec3 {
    track
        .iter()
        .find(|(t, _)| *t == at)
        .map(|(_, p)| *p)
        .expect("the checkpoint instant is a recorded frame")
}

/// Rewinds 10 s then 30 s, checking time, position, replay, and the host's epoch.
fn assert_rewinds_deterministically(flight: &mut Flight) {
    let mut host = StandInPilotageHost::new();
    let start_epoch = host.publish(flight, None, 0.0).source_epoch;
    let track = fly(flight, 40.0);
    assert!(
        flight.history_span_s() >= 39.0,
        "{}",
        flight.history_span_s()
    );

    let before = seconds(flight.now());
    let rewound = flight.rewind_by_seconds(10.0).expect("rewind 10 s");
    assert!((rewound - 10.0).abs() <= CHECKPOINT_INTERVAL_S, "{rewound}");
    assert!((before - seconds(flight.now()) - rewound).abs() < 1e-9);
    let at = flight.now();
    assert_eq!(
        flight.vehicle_state().position_ecef,
        recorded_at(&track, at)
    );
    assert_eq!(flight.display_vehicle_state(), flight.vehicle_state());
    let guidance = host.publish(flight, None, 0.0);
    assert_eq!(guidance.source_epoch, start_epoch.wrapping_add(1));
    assert_eq!(host.sequence(), 1, "a new epoch restarts the sequence");

    // Flying on from the checkpoint replays the same bits.
    let replay = fly(flight, 1.0);
    for (t, p) in &replay {
        assert_eq!(*p, recorded_at(&track, *t));
    }

    let before = seconds(flight.now());
    let rewound = flight.rewind_by_seconds(30.0).expect("rewind 30 s");
    assert!((rewound - 30.0).abs() <= CHECKPOINT_INTERVAL_S, "{rewound}");
    assert!(seconds(flight.now()) < before - 29.0);
    assert_eq!(
        flight.vehicle_state().position_ecef,
        recorded_at(&track, flight.now())
    );
    assert_eq!(
        host.publish(flight, None, 0.0).source_epoch,
        start_epoch.wrapping_add(2)
    );
}

#[test]
fn scenario_rewinds_ten_and_thirty_seconds_deterministically() {
    let mut flight = Flight::new().expect("flight");
    flight
        .select(Some(ScenarioProfile::CorridorNominalCruise))
        .expect("scenario");
    assert_rewinds_deterministically(&mut flight);
}

#[test]
fn jsbsim_free_flight_rewinds_ten_and_thirty_seconds_deterministically() {
    let mut flight = Flight::new().expect("flight");
    assert_eq!(flight.free_vehicle(), Some(FreeFlightVehicle::JsbsimC172));
    assert_rewinds_deterministically(&mut flight);
}

#[test]
fn repeated_rewinds_walk_back_and_history_is_bounded() {
    let mut flight = Flight::new().expect("flight");
    flight
        .select_vehicle(FreeFlightVehicle::Kinematic)
        .expect("kinematic");
    let _track = fly(&mut flight, 80.0);
    assert_eq!(flight.history.len(), MAX_CHECKPOINTS);
    assert!(flight.history_span_s() <= 66.0 + 1e-9);
    let mut times = vec![seconds(flight.now())];
    for _ in 0..3 {
        let rewound = flight.rewind_by_seconds(10.0).expect("rewind");
        assert!(rewound > 9.0);
        times.push(seconds(flight.now()));
    }
    assert!(times.windows(2).all(|w| w[1] < w[0] - 9.0), "{times:?}");
    let rewound = flight.rewind_by_seconds(600.0).expect("rewind to oldest");
    assert!(rewound > 0.0);
    assert_eq!(
        flight.history.len(),
        MAX_CHECKPOINTS,
        "scrubbing keeps the history"
    );
    assert!(flight.rewind_by_seconds(10.0).is_err(), "nothing older");
    let _ = fly(&mut flight, CHUNK_S);
    assert!(flight.history.len() <= 2, "{}", flight.history.len());
    flight.restart().expect("restart");
    assert_eq!(flight.now(), SimInstant::START);
}

#[test]
fn aviate_plants_rewind_and_re_arm() {
    for vehicle in [
        FreeFlightVehicle::AviateX500Direct,
        FreeFlightVehicle::AviateFixedWingDirect,
    ] {
        let mut flight = Flight::new().expect("flight");
        flight.select_vehicle(vehicle).expect("aviate");
        let track = fly(&mut flight, 3.0);
        let rewound = flight.rewind_by_seconds(1.0).expect("rewind");
        assert!((rewound - 1.0).abs() <= CHECKPOINT_INTERVAL_S);
        assert_eq!(
            flight.vehicle_state().position_ecef,
            recorded_at(&track, flight.now())
        );
        assert_eq!(flight.aviate_armed(), Some(true), "{vehicle:?}");
        let _ = fly(&mut flight, 1.0);
        assert_eq!(
            flight.aviate_armed(),
            Some(true),
            "{vehicle:?} keeps flying"
        );
    }
}

#[test]
fn display_state_interpolates_between_physics_steps() {
    let mut flight = Flight::new().expect("flight");
    let period_s = flight.mode.period().as_seconds();
    flight
        .advance_by(period_s, cruise(), SimTimeScale::RealTime)
        .expect("step");
    let prev = flight.vehicle_state();
    flight
        .advance_by(period_s * 1.5, cruise(), SimTimeScale::RealTime)
        .expect("step");
    let phase = flight.display_phase();
    assert!((phase - 0.5).abs() < 1e-6, "{phase}");
    let exact = flight.vehicle_state();
    let shown = flight.display_vehicle_state();
    let expected = prev.position_ecef.lerp(exact.position_ecef, phase);
    assert!((shown.position_ecef - expected).length() < 1e-6);
    assert!(shown.position_ecef != exact.position_ecef);
    let art = flight.articulation();
    let shown_art = flight.display_articulation();
    let advance = (shown_art.propeller_phase_rad - art.propeller_phase_rad).rem_euclid(TAU_F64);
    let expected = 0.5 * period_s * art.propeller_rpm * TAU_F64 / 60.0;
    assert!((advance - expected.rem_euclid(TAU_F64)).abs() < 1e-9);
    flight.note_wall_break();
    assert!(flight.last_frame.is_none());
}

const TAU_F64: f64 = std::f64::consts::TAU;

#[test]
fn pacing_records_real_time_frames_only_while_unpaused() {
    let mut flight = Flight::new().expect("flight");
    let _first = flight
        .advance_to_now(cruise(), SimTimeScale::RealTime)
        .expect("frame");
    assert_eq!(
        flight.pacing_session().frames,
        0,
        "the first frame has no gap"
    );
    let mut frames = 0;
    while frames < 3 {
        let elapsed = flight
            .advance_to_now(cruise(), SimTimeScale::RealTime)
            .expect("frame");
        if elapsed > 0.0 {
            frames += 1;
        }
    }
    assert!(flight.pacing_session().frames >= 3);
    let recorded = flight.pacing_session().frames;
    let _paused = flight
        .advance_to_now(cruise(), SimTimeScale::Paused)
        .expect("paused frame");
    assert_eq!(flight.pacing_session().frames, recorded);
}

/// Exact `(instant, truth state)` of the flight.
fn snapshot(flight: &Flight) -> (SimInstant, sim_core::VehicleState) {
    (flight.now(), flight.vehicle_state())
}

#[test]
fn scrubbing_back_and_forth_returns_to_the_exact_live_state() {
    for profile in [None, Some(ScenarioProfile::CorridorNominalCruise)] {
        let mut flight = Flight::new().expect("flight");
        flight.select(profile).expect("flight mode");
        let _ = fly(&mut flight, 60.25);
        let live = snapshot(&flight);
        let span = flight.history_span_s();
        assert_eq!(flight.rewind_offset_s(), 0.0);

        assert!(flight.scrub_by_seconds(-15.0).expect("-15 s"));
        assert!((flight.rewind_offset_s() - 15.0).abs() <= CHECKPOINT_INTERVAL_S);
        assert!(flight.scrub_by_seconds(-30.0).expect("-30 s"));
        let back = flight.rewind_offset_s();
        assert!((back - 45.0).abs() <= CHECKPOINT_INTERVAL_S, "{back}");
        assert!(flight.scrub_by_seconds(15.0).expect("+15 s"));
        assert!((flight.rewind_offset_s() - 30.0).abs() <= CHECKPOINT_INTERVAL_S);
        assert!(
            (flight.history_span_s() - span).abs() < 1e-9,
            "scrubbing keeps the buffer"
        );
        // Paused frames under the hub do not fly on, so nothing is committed.
        flight
            .advance_by(0.5, cruise(), SimTimeScale::Paused)
            .expect("paused frame");
        assert!(flight.rewind_offset_s() > 29.0);

        assert!(flight.scrub_to_live().expect("live"));
        assert_eq!(snapshot(&flight), live, "{profile:?}");
        assert_eq!(flight.rewind_offset_s(), 0.0);
        assert!(!flight.scrub_to_live().expect("already live"));
        assert!(!flight.scrub_by_seconds(5.0).expect("nothing ahead"));
        assert_eq!(snapshot(&flight), live);

        assert!(flight.scrub_to_fraction(0.0).expect("oldest"));
        assert!((flight.rewind_offset_s() - span).abs() <= CHECKPOINT_INTERVAL_S);
        assert!(flight.scrub_to_fraction(1.0).expect("frontier"));
        assert_eq!(snapshot(&flight), live, "the frontier is live");
    }
}

#[test]
fn flying_on_after_a_scrub_commits_the_branch() {
    let mut flight = Flight::new().expect("flight");
    flight
        .select_vehicle(FreeFlightVehicle::Kinematic)
        .expect("kinematic");
    let track = fly(&mut flight, 30.0);
    let live_s = seconds(flight.now());
    assert!(flight.scrub_by_seconds(-15.0).expect("-15 s"));
    let branch_s = seconds(flight.now());
    assert_eq!(
        flight.vehicle_state().position_ecef,
        recorded_at(&track, flight.now())
    );
    let held = flight.history.len();

    let replay = fly(&mut flight, CHUNK_S);
    assert_eq!(flight.rewind_offset_s(), 0.0, "the branch is live now");
    assert!(
        flight.history.len() < held,
        "newer checkpoints are forgotten"
    );
    for (t, p) in &replay {
        assert_eq!(
            *p,
            recorded_at(&track, *t),
            "the branch replays the same bits"
        );
    }
    let span = flight.history_span_s();
    assert!(
        (span - (branch_s + CHUNK_S)).abs() <= CHECKPOINT_INTERVAL_S,
        "{span} after branching at {branch_s} of {live_s}"
    );
    assert!(!flight.scrub_to_live().expect("live"), "no future left");
}
