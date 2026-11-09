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

use glam::{DQuat, DVec3};
use nalgebra::{Quaternion, UnitQuaternion};
use navigate_camera::{CameraMount as NavigateMount, ImagerModel};
use navigate_contract::{NavigationSolution, SourceEpoch, SourceId, WrappingSequence};
use navigate_fusion::IngestOutcome;
use navigate_startrack::{
    AttitudePrior, IdentifyConfig, IdentifyMode, PairCatalogue, SolveConfig, identify_stars,
    line_of_sight_observations, solve_attitude,
};
use sim_core::{SimInstant, VehicleState};
use sim_devices::ImuGrade;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_navigate::{
    InertialNavigator, InitialAlignment, MasterClockBridge, SimulatedImu, Truth, TruthGravity,
    compare_attitude_with_truth, compare_with_truth, imager_model, inertial_config, navigate_mount,
    solution_position_ecef,
};
use sim_render::StarCameraConfig;
use sim_scenario::{CorridorRun, FlightDynamics, ScenarioProfile};

#[cfg(test)]
#[path = "star_inertial_closed_loop/scene.rs"]
mod scene;
#[cfg(test)]
use scene::{Exposure, StarFrame, pair_table, sim_mount, star_frames};

/// 2026-03-20T02:00:00Z, a night over the Delaware corridor.
const START_UNIX_S: f64 = 1_773_972_000.0;
const FLIGHT_S: f64 = 60.0;
/// Simulation steps per second.
const STEP_HZ: f64 = 120.0;
/// Simulation steps between star frames: 1 Hz at 120 Hz.
const STAR_PERIOD_STEPS: usize = 120;
/// Simulation steps between scores: 10 Hz.
const SCORE_PERIOD_STEPS: usize = 12;
const SKY_FAINTEST_V: f64 = 6.5;
const CATALOGUE_FAINTEST_V: f64 = 6.0;
/// Faintest star of the lost-in-space pair table: brighter than the
/// catalogue, so the stars a pyramid is built from are ones the camera
/// detects reliably.
const LIS_FAINTEST_V: f64 = 5.5;
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

/// The navigator's own attitude belief, as an identification prior: its
/// published body-to-NED attitude carried into ECEF at its own position, and
/// the RMS of its published attitude covariance.
#[cfg(test)]
fn own_prior(solution: &NavigationSolution) -> AttitudePrior {
    let attitude = solution.attitude.unwrap();
    let body_to_ned = DQuat::from_xyzw(attitude.x, attitude.y, attitude.z, attitude.w);
    let frame = LocalFrame::at(&ecef_to_geodetic(solution_position_ecef(solution)));
    let ecef = DQuat::from_mat3(&frame.ned_to_ecef()) * body_to_ned;
    let (xx, yy, zz) = solution.attitude_cov.unwrap().diagonal();
    AttitudePrior {
        ecef_from_body: UnitQuaternion::from_quaternion(Quaternion::new(
            ecef.w, ecef.x, ecef.y, ecef.z,
        )),
        sigma_rad: (xx + yy + zz).sqrt(),
    }
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

/// The navigator side of the star camera: the declared camera models, the
/// lost-in-space table, and the prior held from mid-exposure until the image
/// is read out.
#[cfg(test)]
struct StarTracker<'a> {
    imager: ImagerModel,
    mount: NavigateMount,
    pairs: &'a PairCatalogue,
    next: WrappingSequence,
    held_prior: Option<AttitudePrior>,
    locked: bool,
}

#[cfg(test)]
impl<'a> StarTracker<'a> {
    fn new(pairs: &'a PairCatalogue) -> Self {
        let (mount, cg_body_m) = sim_mount();
        Self {
            imager: imager_model(&StarCameraConfig::star_tracker_1024().intrinsics).unwrap(),
            mount: navigate_mount(&mount, cg_body_m),
            pairs,
            next: WrappingSequence::new(0),
            held_prior: None,
            locked: false,
        }
    }

    /// At mid-exposure: the navigator's own attitude belief, once a frame
    /// has locked the tracker onto the sky; before that there is none.
    fn hold_prior(&mut self, navigator: &mut InertialNavigator, at: SimInstant) {
        self.held_prior = self
            .locked
            .then(|| own_prior(&navigator.solution(at).unwrap().unwrap()));
    }

    /// At readout: sights `frame`, lost in space while unlocked, and offers
    /// every line of sight stamped at mid-exposure.
    fn aid(
        &mut self,
        frame: &StarFrame,
        navigator: &mut InertialNavigator,
        now: SimInstant,
        flown: &mut Flown,
    ) {
        let first = flown.frames_solved + flown.frames_unsolved == 0;
        let mode = match self.held_prior.take() {
            Some(prior) => IdentifyMode::Prior(prior),
            None => {
                flown.lost_in_space_tried += 1;
                IdentifyMode::LostInSpace(self.pairs)
            }
        };
        let lost = matches!(mode, IdentifyMode::LostInSpace(_));
        let solved = identify_stars(
            &frame.centroids,
            &self.imager,
            &self.mount,
            mode,
            &frame.catalogue,
            &IdentifyConfig::default(),
        )
        .and_then(|matches| {
            solve_attitude(
                &matches,
                &frame.centroids,
                &self.imager,
                &self.mount,
                &frame.catalogue,
                &SolveConfig::default(),
            )
        });
        let streak_px = *frame.streak_px.get();
        let Ok(sighting) = solved else {
            flown.frames_unsolved += 1;
            let shortest = flown.shortest_unsolved_streak_px.get_or_insert(streak_px);
            *shortest = shortest.min(streak_px);
            return;
        };
        flown.longest_solved_streak_px = flown.longest_solved_streak_px.max(streak_px);
        flown.frames_solved += 1;
        flown.lost_in_space_solved += u64::from(lost);
        flown.first_frame_solved |= first;
        self.locked = true;
        let clock = MasterClockBridge::default();
        let stamp = clock
            .stamp(STAR_SOURCE, SourceEpoch::new(0), self.next, frame.mid)
            .unwrap();
        let variance = sighting.direction_sigma_rad.powi(2) + MOUNT_ALIGNMENT_RAD.powi(2);
        let frame_id = navigator.filter().frame();
        let observations =
            line_of_sight_observations(&sighting, stamp, frame_id, variance).unwrap();
        for observation in &observations {
            self.next = self.next.next();
            match navigator.observe(observation, now).unwrap() {
                IngestOutcome::Accepted => flown.admitted += 1,
                IngestOutcome::Rejected(_) => flown.refused += 1,
            }
        }
    }
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
    let mut imu = SimulatedImu::new(spec, IMU_SEED, TruthGravity::WgsJ2).unwrap();
    let mut navigator = InertialNavigator::new(inertial_config(&spec), clock, IMU_SOURCE);
    let (start, initial) = *truth[0].get();
    navigator
        .align(&ALIGNMENT.measure(&initial), start)
        .unwrap();
    assert!(imu.sense(start, &initial).unwrap().is_none());
    let (frames, pairs) = frames.map_or((&[][..], None), |(f, p)| (f, Some(p)));
    let mut tracker = pairs.map(StarTracker::new);
    let mut opening = frames.iter().peekable();
    let mut reading = frames.iter().peekable();
    let mut flown = Flown::default();
    for (step, sample) in truth.iter().enumerate().skip(1) {
        let (at, vehicle) = *sample.get();
        let increment = imu.sense(at, &vehicle).unwrap().unwrap();
        navigator.ingest(&increment, at).unwrap();
        if let Some(tracker) = tracker.as_mut() {
            if opening.next_if(|frame| frame.mid_step == step).is_some() {
                tracker.hold_prior(&mut navigator, at);
            }
            if let Some(frame) = reading.next_if(|frame| frame.end_step == step) {
                tracker.aid(frame, &mut navigator, at, &mut flown);
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
fn compare(profile: ScenarioProfile, exposure: Exposure, pairs: &PairCatalogue) -> Vec<Flown> {
    let truth = flight(profile);
    let frames = star_frames(&truth, exposure);
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
        assert!(aided.final_attitude_rad < 1e-3, "{aided:?}");
        assert!(aided.final_attitude_rad < unaided.final_attitude_rad);
        for flown in [&unaided, &aided] {
            assert!(
                (0.3..=10.0).contains(&flown.mean_position_nees()),
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
        Exposure::Moving,
        &pair_table(),
    );
    // The bank turns the camera fast enough to smear most frames away.
    assert!(aided.iter().all(|flown| flown.frames_unsolved > 0));
}

/// The same bank with the camera held at its mid-exposure pose, to show
/// what the smear costs.
#[test]
fn a_frozen_exposure_solves_every_frame_of_the_bank() {
    init_tracing();
    let aided = compare(
        ScenarioProfile::CorridorAggressiveBank,
        Exposure::Frozen,
        &pair_table(),
    );
    assert!(aided.iter().all(|flown| flown.frames_unsolved == 0));
}

#[test]
fn stars_bound_attitude_in_cruise() {
    init_tracing();
    compare(
        ScenarioProfile::CorridorNominalCruise,
        Exposure::Moving,
        &pair_table(),
    );
}
