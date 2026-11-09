//! The star chain across the simulator–Navigate seam: the simulator renders a
//! star-tracker frame at the true camera pose, and Navigate detects,
//! identifies and solves attitude from the counts alone, then feeds the
//! sightings to its inertial filter. A disagreement in axes, signs or the
//! pixel-centre convention between `sim_render::StarCamera`, the frame link's
//! models and `navigate-startrack` shows up as an attitude error.

use std::io::Write;

use glam::{DQuat, DVec3};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use navigate_camera::{ImageView, Pixels};
use navigate_contract::{SourceEpoch, SourceId, WrappingSequence};
use navigate_fusion::{InertialConfig, IngestOutcome};
use navigate_startrack::{
    AttitudePrior, AttitudeSolution, DetectConfig, IdentifyConfig, SolveConfig, catalogue_ecef,
    detect_stars, identify_stars, line_of_sight_observations, solve_attitude,
};
use sim_core::{CameraMount, SimInstant, TickSpan, VehicleState};
use sim_geo::{Geodetic, UtcTime};
use sim_navigate::{
    Estimate, InertialNavigator, InitialAlignment, MASTER_CLOCK_DOMAIN, MasterClockBridge, Truth,
    camera_stars, imager_model, navigate_mount, star_camera_epoch,
};
use sim_render::{CameraPose, SensorCounts, StarCamera, StarCameraConfig, StarVisibility};

/// 2026-03-20T02:00:00Z, a night over the Delaware corridor.
const START_UNIX_S: f64 = 1_773_972_000.0;
/// The exposure, ten minutes into the scenario.
const EXPOSURE_S: u64 = 600;
/// Faintest star the simulator draws; fainter than Navigate's catalogue, so
/// the image holds stars the identifier must ignore.
const SKY_FAINTEST_V: f64 = 6.5;
/// Faintest star of Navigate's reference catalogue.
const CATALOGUE_FAINTEST_V: f64 = 6.0;
/// One standard deviation of the attitude prior, and the size of its error.
const PRIOR_SIGMA_RAD: f64 = 0.017_453_292_5;
/// Camera-to-body alignment knowledge, radians RMS, added to each sighting.
const MOUNT_ALIGNMENT_RAD: f64 = 2.0e-5;
const STAR_SOURCE: SourceId = SourceId::new(71);
/// Source of the inertial stream the sightings aid.
const INERTIAL_SOURCE: SourceId = SourceId::new(70);
const ARCSEC_PER_RAD: f64 = 206_264.806;

#[cfg(test)]
fn log(line: &str) {
    writeln!(std::io::stderr(), "{line}").ok();
}

#[cfg(test)]
fn to_nalgebra(q: DQuat) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z))
}

#[cfg(test)]
/// A rotation of exactly `angle_rad` about an axis drawn from `seed`.
fn seeded_rotation(seed: u64, angle_rad: f64) -> UnitQuaternion<f64> {
    let mut state = seed;
    let mut next = || {
        state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) >> 11) as f64 / (1_u64 << 53) as f64 - 0.5
    };
    let axis = Vector3::new(next(), next(), next()).normalize();
    UnitQuaternion::from_scaled_axis(axis * angle_rad)
}

#[cfg(test)]
/// A level airliner at 10 km over Delaware, heading 063°.
fn vehicle() -> VehicleState {
    let position = Geodetic {
        lat_rad: 39.2_f64.to_radians(),
        lon_rad: (-75.4_f64).to_radians(),
        height_m: 10_000.0,
    };
    VehicleState::level_flight(&position, 63.0_f64.to_radians(), 230.0)
}

#[cfg(test)]
/// A star tracker on the cabin roof, tilted off the zenith toward the nose
/// and right wing, with the image top toward the nose.
fn mount() -> (CameraMount, DVec3) {
    let offset_body_m = DVec3::new(4.2, 0.6, -2.1);
    let forward_body = DVec3::new(0.25, 0.12, -1.0).normalize();
    let mount = CameraMount::looking_along(offset_body_m, forward_body, DVec3::X);
    (mount, DVec3::new(1.1, 0.0, 0.3))
}

#[cfg(test)]
/// What the simulator renders: the counts the navigator gets, and the truth
/// it is scored against.
fn simulate(now: SimInstant) -> (SensorCounts, Truth<VehicleState>) {
    let truth = vehicle();
    let (sim_mount, cg_body_m) = mount();
    let pose = sim_mount.pose(&truth, cg_body_m);
    let camera = StarCamera::new(StarCameraConfig {
        seed: 7,
        ..StarCameraConfig::star_tracker_1024()
    })
    .unwrap();
    let sky = camera_stars(
        UtcTime::from_unix_seconds(START_UNIX_S),
        now,
        SKY_FAINTEST_V,
    )
    .unwrap();
    let exposure = camera.expose(
        &CameraPose {
            position_ecef: pose.position_ecef,
            orientation: pose.orientation,
        },
        &sky,
        0,
    );
    let visible = exposure
        .truth
        .stars
        .iter()
        .filter(|star| star.visibility == StarVisibility::Visible)
        .count();
    log(&format!(
        "simulated sky: {} stars, {visible} visible",
        sky.len()
    ));
    (exposure.image, Truth::new(truth))
}

/// What Navigate computes from the counts alone, with the declared camera
/// models, the exposure epoch, and an attitude prior.
#[cfg(test)]
struct Sighting {
    centroids: usize,
    identified: usize,
    mount: navigate_camera::CameraMount,
    solution: Estimate<AttitudeSolution>,
}

#[cfg(test)]
fn sight_stars(counts: &SensorCounts, now: SimInstant, prior: &AttitudePrior) -> Sighting {
    let imager = imager_model(&StarCameraConfig::star_tracker_1024().intrinsics).unwrap();
    let (sim_mount, cg_body_m) = mount();
    let nav_mount = navigate_mount(&sim_mount, cg_body_m);
    let pixels = Pixels::Gray16 {
        data: &counts.counts,
        stride: counts.width as usize,
        significant_bits: counts.significant_bits,
    };
    let view = ImageView::new(counts.width, counts.height, pixels).unwrap();
    let centroids = detect_stars(&view, &DetectConfig::default()).unwrap();
    let epoch = star_camera_epoch(UtcTime::from_unix_seconds(START_UNIX_S), now);
    let catalogue = catalogue_ecef(epoch, CATALOGUE_FAINTEST_V).unwrap();
    let identify = IdentifyConfig::default();
    let matches = identify_stars(
        &centroids,
        &imager,
        &nav_mount,
        Some(prior),
        &catalogue,
        &identify,
    )
    .unwrap();
    let solve = SolveConfig::default();
    let solution = solve_attitude(
        &matches, &centroids, &imager, &nav_mount, &catalogue, &solve,
    )
    .unwrap();
    Sighting {
        centroids: centroids.len(),
        identified: matches.len(),
        mount: nav_mount,
        solution: Estimate::new(solution),
    }
}

/// Offers the sightings to Navigate's inertial filter, aligned from a
/// declared in-flight alignment rather than from the truth itself, and
/// gives each outcome.
#[cfg(test)]
fn ingest_sightings(
    solution: &AttitudeSolution,
    truth: &Truth<VehicleState>,
    now: SimInstant,
) -> Vec<IngestOutcome> {
    let clock = MasterClockBridge::new(MASTER_CLOCK_DOMAIN);
    let mut navigator = InertialNavigator::new(InertialConfig::default(), clock, INERTIAL_SOURCE);
    let alignment = InitialAlignment {
        position_sigma_m: DVec3::splat(5.0),
        velocity_sigma_m_s: DVec3::splat(0.1),
        attitude_sigma_rad: DVec3::splat(1e-3),
        seed: 5,
    }
    .measure(truth.get());
    navigator.align(&alignment, now).unwrap();
    let stamp = clock
        .stamp(
            STAR_SOURCE,
            SourceEpoch::new(0),
            WrappingSequence::new(0),
            now,
        )
        .unwrap();
    let variance = solution.direction_sigma_rad.powi(2) + MOUNT_ALIGNMENT_RAD.powi(2);
    let frame = navigator.filter().frame();
    let observations = line_of_sight_observations(solution, stamp, frame, variance).unwrap();
    observations
        .iter()
        .map(|observation| navigator.observe(observation, now).unwrap())
        .collect()
}

#[test]
fn navigate_solves_the_true_attitude_from_simulated_star_counts() {
    let now = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(EXPOSURE_S, 0));
    let (counts, truth) = simulate(now);
    let truth_ecef_from_body = to_nalgebra(truth.get().attitude);
    let prior = AttitudePrior {
        ecef_from_body: truth_ecef_from_body * seeded_rotation(13, PRIOR_SIGMA_RAD),
        sigma_rad: PRIOR_SIGMA_RAD,
    };
    let sighting = sight_stars(&counts, now, &prior);

    // Navigate's convention: truth = solved · exp(δθ), δθ in body axes.
    let solved = sighting.solution.get();
    let error_body = (solved.ecef_from_body.inverse() * truth_ecef_from_body).scaled_axis();
    let boresight_body = sighting.mount.cam_from_body.inverse() * Vector3::z();
    let roll_rad = error_body.dot(&boresight_body);
    let cross_rad = (error_body - boresight_body * roll_rad).norm();
    let information = solved.covariance_body_rad2.try_inverse().unwrap();
    let nees = (error_body.transpose() * information * error_body).x;
    let prior_error_arcsec =
        (prior.ecef_from_body.inverse() * truth_ecef_from_body).angle() * ARCSEC_PER_RAD;
    log(&format!(
        "star tracker: {} centroids, {} identified, {} in solution ({} outliers), \
         rms residual {:.2}\", error {:.2}\" total = {:.2}\" across the boresight \
         and {:.2}\" roll about it, NEES {nees:.2}, from a {prior_error_arcsec:.0}\" prior",
        sighting.centroids,
        sighting.identified,
        solved.matches.len(),
        solved.rejected_outliers,
        solved.rms_residual_rad * ARCSEC_PER_RAD,
        error_body.norm() * ARCSEC_PER_RAD,
        cross_rad * ARCSEC_PER_RAD,
        roll_rad.abs() * ARCSEC_PER_RAD,
    ));
    assert!(solved.matches.len() >= 10);
    // Centroid noise is ~0.15 px (10") per star; a convention slip of half a
    // pixel or an axis sign would put tens to thousands of arcseconds across
    // the boresight. Roll about a 20° field's boresight is weaker by the
    // field's lever, so it is held to the solver's own covariance instead.
    let cross_arcsec = cross_rad * ARCSEC_PER_RAD;
    assert!(cross_arcsec < 5.0, "cross-boresight error {cross_arcsec}\"");
    assert!(
        nees < 16.27,
        "attitude NEES {nees} beyond the 3-dof 99.9% bound"
    );

    let outcomes = ingest_sightings(solved, &truth, now);
    let admitted = outcomes
        .iter()
        .filter(|outcome| matches!(outcome, IngestOutcome::Accepted))
        .count();
    log(&format!(
        "inertial filter admitted {admitted} of {} line-of-sight sightings: {outcomes:?}",
        outcomes.len()
    ));
    assert_eq!(admitted, solved.matches.len());
}
