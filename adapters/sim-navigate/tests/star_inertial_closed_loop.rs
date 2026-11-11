//! Celestial navigation as its minimal unit, star camera plus INS, under
//! acceleration. The simulator renders a roof star camera at 1 Hz along a
//! corridor flight, moving from its true pose at the shutter's opening to its
//! pose at the closing, so stars streak. Navigate detects; identifies the
//! first frame lost in space and later ones with an attitude prior taken from
//! the navigator's own solution and covariance at mid-exposure; solves; and
//! at readout feeds each sighting, stamped at mid-exposure, as a line of
//! sight to the inertial filter. The same IMU
//! stream also runs unaided, and both are scored against truth outside the
//! filters.
//!
//! Star sightings measure direction only. They bound attitude, heading
//! included, and through tilt the part of the position error that a gyro
//! bias drives; they say nothing about where the vehicle is, so the error an
//! accelerometer bias or the alignment's own position and velocity error
//! drives still grows.

use glam::DVec3;
use navigate_contract::SourceId;
use navigate_startrack::PairCatalogue;
use sim_core::{SimInstant, VehicleState};
use sim_devices::ImuGrade;
use sim_geo::UtcTime;
use sim_navigate::trial::{
    StarExposure, StarFrame, StarScene, StarSighting, StarSky, pair_table, render_star_frames,
};
use sim_navigate::{
    InertialNavigator, InitialAlignment, MasterClockBridge, SimulatedImu, Truth,
    compare_attitude_with_truth, compare_with_truth, inertial_config,
};
use sim_scenario::{CorridorRun, FlightDynamics, ScenarioProfile};

#[cfg(test)]
#[path = "trial_support/consistency.rs"]
mod consistency;

/// 2026-03-20T02:00:00Z, a night over the Delaware corridor.
const START_UNIX_S: f64 = 1_773_972_000.0;
const FLIGHT_S: f64 = 60.0;
/// Simulation steps per second.
const STEP_HZ: f64 = 120.0;
/// Simulation steps between star frames: 1 Hz at 120 Hz.
const STAR_PERIOD_STEPS: usize = 120;
/// Simulation steps between scores: 10 Hz.
const SCORE_PERIOD_STEPS: usize = 12;
/// Camera-to-body alignment knowledge, radians RMS, added to each sighting.
const MOUNT_ALIGNMENT_RAD: f64 = 2.0e-5;
/// Streak below which every frame must solve. The roof camera's 12 mm,
/// 0.1 s exposure keeps enough stars above threshold up to about 20 px of
/// smear, a camera turn of about 0.07 rad/s; beyond about 60 px it detects
/// almost none.
const ASSURED_STREAK_PX: f64 = 15.0;
const IMU_SOURCE: SourceId = SourceId::new(80);
const STAR_SOURCE: SourceId = SourceId::new(81);
const IMU_SEED: u64 = 0x5_7A2;
/// An in-flight alignment: position, velocity and attitude 1-sigma.
const ALIGNMENT: InitialAlignment = InitialAlignment {
    position_sigma_m: DVec3::splat(2.0),
    velocity_sigma_m_s: DVec3::splat(0.1),
    attitude_sigma_rad: DVec3::splat(1e-3),
    seed: 31,
};

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

#[cfg(test)]
fn flight(profile: ScenarioProfile) -> Vec<Truth<(SimInstant, VehicleState)>> {
    let mut run = CorridorRun::new(profile, FlightDynamics::Kinematic).unwrap();
    let mut truth = vec![Truth::new((run.now(), run.vehicle()))];
    while run.now().duration_since(SimInstant::START).as_seconds() < FLIGHT_S && !run.finished() {
        run.step().unwrap();
        truth.push(Truth::new((run.now(), run.vehicle())));
    }
    truth
}

#[cfg(test)]
#[derive(Debug, Default, Clone, Copy)]
struct Flown {
    final_horizontal_m: f64,
    final_vertical_m: f64,
    final_attitude_rad: f64,
    final_heading_rad: f64,
    worst_attitude_rad: f64,
    position_nees_sum: f64,
    attitude_nees_sum: f64,
    scored: u64,
    frames_solved: u64,
    frames_unsolved: u64,
    lost_in_space_tried: u64,
    lost_in_space_solved: u64,
    first_frame_solved: bool,
    /// The shortest streak of a frame that did not solve, pixels.
    shortest_unsolved_streak_px: Option<f64>,
    /// The longest streak of a frame that solved, pixels.
    longest_solved_streak_px: f64,
    admitted: u64,
    refused: u64,
}

#[cfg(test)]
impl Flown {
    fn mean_position_nees(&self) -> f64 {
        self.position_nees_sum / self.scored as f64
    }

    fn mean_attitude_nees(&self) -> f64 {
        self.attitude_nees_sum / self.scored as f64
    }
}

/// Navigate's tracker on one frame, through the trial's star sighting,
/// with the frame's scene truth kept for the bookkeeping.
#[cfg(test)]
fn aid(
    sighting: &mut StarSighting<'_>,
    frame: &StarFrame,
    navigator: &mut InertialNavigator,
    now: SimInstant,
    flown: &mut Flown,
) {
    let first = flown.frames_solved + flown.frames_unsolved == 0;
    let outcome = sighting.aid(&frame.image, navigator, now).unwrap();
    flown.lost_in_space_tried += u64::from(outcome.lost_in_space);
    let streak_px = frame.truth.get().streak_px;
    if !outcome.solved {
        flown.frames_unsolved += 1;
        let shortest = flown.shortest_unsolved_streak_px.get_or_insert(streak_px);
        *shortest = shortest.min(streak_px);
        return;
    }
    flown.longest_solved_streak_px = flown.longest_solved_streak_px.max(streak_px);
    flown.frames_solved += 1;
    flown.lost_in_space_solved += u64::from(outcome.lost_in_space);
    flown.first_frame_solved |= first;
    flown.admitted += outcome.accepted;
    flown.refused += outcome.rejected.len() as u64;
}

/// Flies `truth` on `grade`, star-aided from `frames` when given.
#[cfg(test)]
fn fly(
    truth: &[Truth<(SimInstant, VehicleState)>],
    grade: ImuGrade,
    frames: Option<(&[StarFrame], &PairCatalogue)>,
) -> Flown {
    let spec = grade.spec();
    let clock = MasterClockBridge::default();
    let mut imu = SimulatedImu::new(spec, IMU_SEED).unwrap();
    let mut navigator = InertialNavigator::new(inertial_config(&spec), clock, IMU_SOURCE);
    let (start, initial) = *truth[0].get();
    navigator
        .align(&ALIGNMENT.measure(&initial), start)
        .unwrap();
    assert!(imu.sense(start, &initial).unwrap().is_none());
    let (frames, pairs) = frames.map_or((&[][..], None), |(f, p)| (f, Some(p)));
    let start = UtcTime::from_unix_seconds(START_UNIX_S);
    let mut tracker = pairs
        .map(|pairs| StarSighting::new(pairs, start, STAR_SOURCE, MOUNT_ALIGNMENT_RAD).unwrap());
    let mut opening = frames.iter().peekable();
    let mut reading = frames.iter().peekable();
    let mut flown = Flown::default();
    for (step, sample) in truth.iter().enumerate().skip(1) {
        let (at, vehicle) = *sample.get();
        let increment = imu.sense(at, &vehicle).unwrap().unwrap();
        navigator.ingest(&increment, at).unwrap();
        if let Some(tracker) = tracker.as_mut() {
            if opening
                .next_if(|frame| frame.image.mid_step == step)
                .is_some()
            {
                tracker.hold_prior(&mut navigator, at).unwrap();
            }
            if let Some(frame) = reading.next_if(|frame| frame.image.end_step == step) {
                aid(tracker, frame, &mut navigator, at, &mut flown);
            }
        }
        if step % SCORE_PERIOD_STEPS == 0 || step == truth.len() - 1 {
            score(&mut flown, &mut navigator, at, &vehicle);
        }
    }
    flown
}

#[cfg(test)]
fn score(
    flown: &mut Flown,
    navigator: &mut InertialNavigator,
    at: SimInstant,
    truth: &VehicleState,
) {
    let solution = navigator.solution(at).unwrap().unwrap();
    let position = compare_with_truth(&solution, truth.position_ecef).unwrap();
    let attitude = compare_attitude_with_truth(&solution, truth.attitude).unwrap();
    flown.final_horizontal_m = position.horizontal_error_m;
    flown.final_vertical_m = position.vertical_error_m;
    flown.final_attitude_rad = attitude.error_rad;
    flown.final_heading_rad = attitude.heading_rad;
    flown.worst_attitude_rad = flown.worst_attitude_rad.max(attitude.error_rad);
    flown.position_nees_sum += position.position_nees;
    flown.attitude_nees_sum += attitude.attitude_nees;
    flown.scored += 1;
}

#[cfg(test)]
fn log(profile: ScenarioProfile, grade: ImuGrade, means: &str, flown: &Flown) {
    tracing::info!(
        ?profile,
        ?grade,
        means,
        final_horizontal_m = flown.final_horizontal_m,
        final_vertical_m = flown.final_vertical_m,
        final_attitude_mrad = flown.final_attitude_rad * 1e3,
        final_heading_mrad = flown.final_heading_rad * 1e3,
        worst_attitude_mrad = flown.worst_attitude_rad * 1e3,
        position_nees = flown.mean_position_nees(),
        attitude_nees = flown.mean_attitude_nees(),
        frames_solved = flown.frames_solved,
        frames_unsolved = flown.frames_unsolved,
        lost_in_space_tried = flown.lost_in_space_tried,
        lost_in_space_solved = flown.lost_in_space_solved,
        shortest_unsolved_streak_px = flown.shortest_unsolved_streak_px,
        longest_solved_streak_px = flown.longest_solved_streak_px,
        admitted = flown.admitted,
        refused = flown.refused,
        "star + INS vs dead reckoning"
    );
}

/// Flies `profile` with and without stars on each grade and checks what the
/// stars can bound; returns the star-aided runs.
#[cfg(test)]
fn compare(profile: ScenarioProfile, exposure: StarExposure, pairs: &PairCatalogue) -> Vec<Flown> {
    let truth = flight(profile);
    let scene = StarScene {
        start: UtcTime::from_unix_seconds(START_UNIX_S),
        step_hz: STEP_HZ,
        period_steps: STAR_PERIOD_STEPS,
        exposure,
        sky: StarSky::Fixed,
        seed: 7,
    };
    let frames = render_star_frames(&truth, &scene).unwrap();
    let mut aided_runs = Vec::new();
    for grade in [ImuGrade::Tactical, ImuGrade::ConsumerMems] {
        let unaided = fly(&truth, grade, None);
        let aided = fly(&truth, grade, Some((&frames, pairs)));
        log(profile, grade, "dead reckoning", &unaided);
        log(profile, grade, &format!("star + INS, {exposure:?}"), &aided);
        // The first frame locks on with no prior at all.
        assert!(
            aided.first_frame_solved && aided.lost_in_space_solved >= 1,
            "{aided:?}"
        );
        assert!(aided.admitted > 0, "{aided:?}");
        // Stars bound attitude, heading included, whatever the gyro: within a
        // milliradian once the gyro bias is learnt, against hundreds of
        // milliradians unaided on a consumer part.
        // Smear is the only thing that loses a frame: a streak spreads a
        // star's light until it falls below the detection threshold.
        assert!(
            aided
                .shortest_unsolved_streak_px
                .is_none_or(|streak| streak > ASSURED_STREAK_PX),
            "{aided:?}"
        );
        assert!(
            aided.final_attitude_rad < consistency::STAR_ATTITUDE_RAD,
            "{aided:?}"
        );
        assert!(aided.final_attitude_rad < unaided.final_attitude_rad);
        for flown in [&unaided, &aided] {
            assert!(
                consistency::MANOEUVRE_NEES.contains(&flown.mean_position_nees()),
                "{flown:?}"
            );
            assert!(
                (0.1..=10.0).contains(&flown.mean_attitude_nees()),
                "{flown:?}"
            );
        }
        aided_runs.push(aided);
    }
    aided_runs
}

#[test]
fn stars_bound_attitude_through_the_aggressive_bank() {
    init_tracing();
    let aided = compare(
        ScenarioProfile::CorridorAggressiveBank,
        StarExposure::Moving,
        &pair_table(UtcTime::from_unix_seconds(START_UNIX_S)).unwrap(),
    );
    // The bank turns the camera fast enough to smear most frames away.
    assert!(aided.iter().all(|flown| flown.frames_unsolved > 0));
    // After each run of smeared frames the tracker drops the inertial prior
    // and finds the sky again by lost-in-space, not only at the start.
    assert!(
        aided.iter().all(|flown| flown.lost_in_space_solved > 1),
        "{aided:?}"
    );
}

/// The same bank with the camera held at its mid-exposure pose, to show
/// what the smear costs.
#[test]
fn a_frozen_exposure_solves_every_frame_of_the_bank() {
    init_tracing();
    let aided = compare(
        ScenarioProfile::CorridorAggressiveBank,
        StarExposure::Frozen,
        &pair_table(UtcTime::from_unix_seconds(START_UNIX_S)).unwrap(),
    );
    assert!(aided.iter().all(|flown| flown.frames_unsolved == 0));
}

#[test]
fn stars_bound_attitude_in_cruise() {
    init_tracing();
    compare(
        ScenarioProfile::CorridorNominalCruise,
        StarExposure::Moving,
        &pair_table(UtcTime::from_unix_seconds(START_UNIX_S)).unwrap(),
    );
}
