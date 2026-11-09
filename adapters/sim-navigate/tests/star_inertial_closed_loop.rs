//! Celestial navigation as its minimal unit, star camera plus INS, under
//! acceleration. The simulator renders a roof star camera at 1 Hz along a
//! corridor flight; Navigate detects, identifies with an attitude prior
//! taken from the navigator's own solution and covariance, solves, and feeds
//! each sighting as a line of sight to the inertial filter. The same IMU
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
use navigate_camera::{CameraMount as NavigateMount, ImageView, ImagerModel, Pixels};
use navigate_celestial::StarSight;
use navigate_contract::{NavigationSolution, SourceEpoch, SourceId, WrappingSequence};
use navigate_fusion::IngestOutcome;
use navigate_startrack::{
    AttitudePrior, DetectConfig, IdentifyConfig, SolveConfig, StarCentroid, catalogue_ecef,
    detect_stars, identify_stars, line_of_sight_observations, solve_attitude,
};
use sim_core::{CameraMount, SimInstant, VehicleState};
use sim_devices::ImuGrade;
use sim_geo::{LocalFrame, UtcTime, ecef_to_geodetic};
use sim_navigate::{
    InertialNavigator, InitialAlignment, MasterClockBridge, SimulatedImu, Truth, TruthGravity,
    camera_stars, compare_attitude_with_truth, compare_with_truth, imager_model, inertial_config,
    navigate_mount, solution_position_ecef, star_camera_epoch,
};
use sim_render::{CameraPose, StarCamera, StarCameraConfig};
use sim_scenario::{CorridorRun, FlightDynamics, ScenarioProfile};

/// 2026-03-20T02:00:00Z, a night over the Delaware corridor.
const START_UNIX_S: f64 = 1_773_972_000.0;
const FLIGHT_S: f64 = 60.0;
/// Simulation steps between star frames: 1 Hz at 120 Hz.
const STAR_PERIOD_STEPS: usize = 120;
/// Simulation steps between scores: 10 Hz.
const SCORE_PERIOD_STEPS: usize = 12;
const SKY_FAINTEST_V: f64 = 6.5;
const CATALOGUE_FAINTEST_V: f64 = 6.0;
/// Camera-to-body alignment knowledge, radians RMS, added to each sighting.
const MOUNT_ALIGNMENT_RAD: f64 = 2.0e-5;
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

/// A star camera on the cabin roof, a little off the zenith toward the nose
/// and right wing, and the centre of gravity in the mount's axes.
#[cfg(test)]
fn sim_mount() -> (CameraMount, DVec3) {
    let forward_body = DVec3::new(0.25, 0.12, -1.0).normalize();
    let mount = CameraMount::looking_along(DVec3::new(4.2, 0.6, -2.1), forward_body, DVec3::X);
    (mount, DVec3::new(1.1, 0.0, 0.3))
}

/// What the navigator has of one exposure: centroids from the counts and the
/// catalogue at the exposure epoch. The counts come from the true pose; that
/// is the scene, not an input to the navigator.
#[cfg(test)]
struct StarFrame {
    step: usize,
    at: SimInstant,
    centroids: Vec<StarCentroid>,
    catalogue: Vec<StarSight>,
}

#[cfg(test)]
fn star_frames(truth: &[Truth<(SimInstant, VehicleState)>]) -> Vec<StarFrame> {
    let config = StarCameraConfig::star_tracker_1024();
    let camera = StarCamera::new(StarCameraConfig { seed: 7, ..config }).unwrap();
    let (mount, cg_body_m) = sim_mount();
    let start = UtcTime::from_unix_seconds(START_UNIX_S);
    let frames = truth.iter().enumerate().skip(1).step_by(STAR_PERIOD_STEPS);
    frames
        .map(|(step, sample)| {
            let (at, vehicle) = *sample.get();
            let pose = mount.pose(&vehicle, cg_body_m);
            let sky = camera_stars(start, at, SKY_FAINTEST_V).unwrap();
            let pose = CameraPose {
                position_ecef: pose.position_ecef,
                orientation: pose.orientation,
            };
            let image = camera.expose(&pose, &sky, step as u64).image;
            let pixels = Pixels::Gray16 {
                data: &image.counts,
                stride: image.width as usize,
                significant_bits: image.significant_bits,
            };
            let view = ImageView::new(image.width, image.height, pixels).unwrap();
            let epoch = star_camera_epoch(start, at);
            StarFrame {
                step,
                at,
                centroids: detect_stars(&view, &DetectConfig::default()).unwrap_or_default(),
                catalogue: catalogue_ecef(epoch, CATALOGUE_FAINTEST_V).unwrap(),
            }
        })
        .collect()
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

/// The navigator side of the star camera: the declared camera models.
#[cfg(test)]
struct StarTracker {
    imager: ImagerModel,
    mount: NavigateMount,
    next: WrappingSequence,
}

#[cfg(test)]
impl StarTracker {
    fn new() -> Self {
        let (mount, cg_body_m) = sim_mount();
        Self {
            imager: imager_model(&StarCameraConfig::star_tracker_1024().intrinsics).unwrap(),
            mount: navigate_mount(&mount, cg_body_m),
            next: WrappingSequence::new(0),
        }
    }

    /// Sights `frame` from the navigator's own prior and offers every line
    /// of sight to it.
    fn aid(&mut self, frame: &StarFrame, navigator: &mut InertialNavigator, flown: &mut Flown) {
        let solution = navigator.solution(frame.at).unwrap().unwrap();
        let prior = own_prior(&solution);
        let matches = identify_stars(
            &frame.centroids,
            &self.imager,
            &self.mount,
            Some(&prior),
            &frame.catalogue,
            &IdentifyConfig::default(),
        )
        .unwrap();
        let Ok(sighting) = solve_attitude(
            &matches,
            &frame.centroids,
            &self.imager,
            &self.mount,
            &frame.catalogue,
            &SolveConfig::default(),
        ) else {
            flown.frames_unsolved += 1;
            return;
        };
        flown.frames_solved += 1;
        let clock = MasterClockBridge::default();
        let stamp = clock
            .stamp(STAR_SOURCE, SourceEpoch::new(0), self.next, frame.at)
            .unwrap();
        let variance = sighting.direction_sigma_rad.powi(2) + MOUNT_ALIGNMENT_RAD.powi(2);
        let frame_id = navigator.filter().frame();
        let observations =
            line_of_sight_observations(&sighting, stamp, frame_id, variance).unwrap();
        for observation in &observations {
            self.next = self.next.next();
            match navigator.observe(observation, frame.at).unwrap() {
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
    frames: Option<&[StarFrame]>,
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
    let mut tracker = StarTracker::new();
    let mut pending = frames.unwrap_or_default().iter().peekable();
    let mut flown = Flown::default();
    for (step, sample) in truth.iter().enumerate().skip(1) {
        let (at, vehicle) = *sample.get();
        let increment = imu.sense(at, &vehicle).unwrap().unwrap();
        navigator.ingest(&increment, at).unwrap();
        if let Some(frame) = pending.next_if(|frame| frame.step == step) {
            tracker.aid(frame, &mut navigator, &mut flown);
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
        admitted = flown.admitted,
        refused = flown.refused,
        "star + INS vs dead reckoning"
    );
}

/// Flies `profile` with and without stars on each grade and checks what the
/// stars can bound.
#[cfg(test)]
fn compare(profile: ScenarioProfile) {
    init_tracing();
    let truth = flight(profile);
    let frames = star_frames(&truth);
    for grade in [ImuGrade::Tactical, ImuGrade::ConsumerMems] {
        let unaided = fly(&truth, grade, None);
        let aided = fly(&truth, grade, Some(&frames));
        log(profile, grade, "dead reckoning", &unaided);
        log(profile, grade, "star + INS", &aided);
        assert!(aided.admitted > 0, "{aided:?}");
        // Stars bound attitude, heading included, whatever the gyro: within a
        // milliradian once the gyro bias is learnt, against hundreds of
        // milliradians unaided on a consumer part.
        assert_eq!(aided.frames_unsolved, 0, "{aided:?}");
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
    }
}

#[test]
fn stars_bound_attitude_through_the_aggressive_bank() {
    compare(ScenarioProfile::CorridorAggressiveBank);
}

#[test]
fn stars_bound_attitude_in_cruise() {
    compare(ScenarioProfile::CorridorNominalCruise);
}
