//! Unaided inertial dead reckoning: simulated strapdown increments from the
//! simulator's truth through Navigate's 22-state inertial filter, scored
//! against truth outside the filter.

use std::sync::Arc;

use glam::DVec3;
use navigate_contract::SourceId;
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_devices::{ImuErrorSpec, ImuGrade};
use sim_geo::{BodyEllipsoid, Geodetic, GravityField, LocalFrame};
use sim_navigate::{
    InertialNavigator, InitialAlignment, MasterClockBridge, SimulatedImu, compare_with_truth,
    inertial_config,
};
use sim_scenario::{CorridorRun, FlightDynamics, ScenarioProfile};

const STEP_HZ: u64 = 120;
const STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / STEP_HZ);
const IMU_SOURCE: SourceId = SourceId::new(60);
/// Seeds per grade for the ensemble NEES; one trajectory's errors are one
/// correlated realisation, so a single run says little about consistency.
const ENSEMBLE: u64 = 6;
/// An in-flight alignment from a GNSS position, a velocity and a
/// transfer-aligned attitude: position, velocity and attitude 1-sigma, in m,
/// m/s and rad.
const FLIGHT_ALIGNMENT: [f64; 3] = [2.0, 0.1, 1e-3];
/// An alignment fine enough that each grade's own errors dominate a minute.
const FINE_ALIGNMENT: [f64; 3] = [0.1, 0.01, 2e-5];

type Truth = Vec<(SimInstant, VehicleState)>;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

#[cfg(test)]
fn stationary_truth(seconds: u64) -> Truth {
    let ground = Geodetic {
        lat_rad: 39.0_f64.to_radians(),
        lon_rad: (-75.5_f64).to_radians(),
        height_m: 10.0,
    };
    let parked = VehicleState::level_flight(&ground, 0.4, 0.0);
    (0..=seconds * STEP_HZ)
        .scan(SimInstant::START, |at, _| {
            let now = *at;
            *at = now.advanced(STEP);
            Some((now, parked))
        })
        .collect()
}

#[cfg(test)]
fn cruise_truth(seconds: f64) -> Truth {
    let mut run = CorridorRun::new(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Kinematic,
    )
    .unwrap();
    assert_eq!(run.period(), STEP);
    let mut truth = vec![(run.now(), run.vehicle())];
    while run.now().duration_since(SimInstant::START).as_seconds() < seconds {
        run.step().unwrap();
        truth.push((run.now(), run.vehicle()));
    }
    truth
}

#[cfg(test)]
#[derive(Debug, Default)]
struct Drift {
    final_horizontal_m: f64,
    final_vertical_m: f64,
    worst_horizontal_m: f64,
    nees_sum: f64,
    scored: u64,
}

/// Dead-reckons over `truth`, scoring once a second.
#[cfg(test)]
fn dead_reckon(truth: &Truth, spec: ImuErrorSpec, seed: u64, alignment: InitialAlignment) -> Drift {
    let imu = SimulatedImu::new(spec, seed).unwrap();
    dead_reckon_with(truth, imu, spec, alignment)
}

/// Dead-reckons over `truth` with `imu`, built from `spec`.
#[cfg(test)]
fn dead_reckon_with(
    truth: &Truth,
    mut imu: SimulatedImu,
    spec: ImuErrorSpec,
    alignment: InitialAlignment,
) -> Drift {
    let clock = MasterClockBridge::default();
    let mut navigator = InertialNavigator::new(inertial_config(&spec), clock, IMU_SOURCE);
    let (start, initial) = truth[0];
    navigator
        .align(&alignment.measure(&initial), start)
        .unwrap();
    assert!(imu.sense(start, &initial).unwrap().is_none());
    let mut drift = Drift::default();
    for (step, (at, state)) in truth.iter().enumerate().skip(1) {
        let increment = imu.sense(*at, state).unwrap().unwrap();
        navigator.ingest(&increment, *at).unwrap();
        if step % STEP_HZ as usize != 0 && step != truth.len() - 1 {
            continue;
        }
        let solution = navigator.solution(*at).unwrap().unwrap();
        let score = compare_with_truth(&solution, state.position_ecef).unwrap();
        drift.final_horizontal_m = score.horizontal_error_m;
        drift.final_vertical_m = score.vertical_error_m;
        drift.worst_horizontal_m = drift.worst_horizontal_m.max(score.horizontal_error_m);
        drift.nees_sum += score.position_nees;
        drift.scored += 1;
    }
    drift
}

#[cfg(test)]
fn seeded_alignment([position, velocity, attitude]: [f64; 3], seed: u64) -> InitialAlignment {
    InitialAlignment {
        position_sigma_m: DVec3::splat(position),
        velocity_sigma_m_s: DVec3::splat(velocity),
        attitude_sigma_rad: DVec3::splat(attitude),
        seed,
    }
}

#[test]
fn a_parked_ideal_imu_holds_position() {
    init_tracing();
    let truth = stationary_truth(60);
    let drift = dead_reckon(&truth, ImuErrorSpec::IDEAL, 0, InitialAlignment::EXACT);
    tracing::info!(
        horizontal_m = drift.final_horizontal_m,
        vertical_m = drift.final_vertical_m,
        "parked 60 s, ideal IMU"
    );
    assert!(drift.final_horizontal_m < 1e-2, "{drift:?}");
    assert!(drift.final_vertical_m < 1e-2, "{drift:?}");
}

/// Earth's reference field plus a uniform horizontal anomaly, a world whose
/// gravity the navigator's `J2` model does not know.
#[derive(Debug)]
struct Anomalous {
    extra_m_s2: DVec3,
}

impl GravityField for Anomalous {
    fn attraction_fixed(&self, fixed: DVec3) -> DVec3 {
        BodyEllipsoid::WGS84.gravity_fixed(fixed) + self.extra_m_s2
    }
}

#[test]
fn unmodelled_gravity_reaches_the_navigator_as_drift() {
    init_tracing();
    let truth = stationary_truth(60);
    // 10 mGal towards the north, a strong but ordinary anomaly.
    let at = sim_geo::ecef_to_geodetic(truth[0].1.position_ecef);
    let north = LocalFrame::at(&at).north();
    let anomaly = Arc::new(Anomalous {
        extra_m_s2: north * 1e-4,
    });
    let imu = SimulatedImu::new(ImuErrorSpec::IDEAL, 0)
        .unwrap()
        .with_gravity(anomaly);
    let drift = dead_reckon_with(&truth, imu, ImuErrorSpec::IDEAL, InitialAlignment::EXACT);
    tracing::info!(
        horizontal_m = drift.final_horizontal_m,
        vertical_m = drift.final_vertical_m,
        "parked 60 s, ideal IMU, 10 mGal unmodelled"
    );
    // The accelerometers cannot sense the pull, so the filter integrates
    // its reaction: ½·a·t² = 0.18 m in a minute.
    assert!(
        (0.12..0.24).contains(&drift.final_horizontal_m),
        "{drift:?}"
    );
}

#[test]
fn an_ideal_imu_follows_the_kinematic_cruise() {
    init_tracing();
    let truth = cruise_truth(120.0);
    let drift = dead_reckon(&truth, ImuErrorSpec::IDEAL, 0, InitialAlignment::EXACT);
    tracing::info!(
        seconds = 120,
        final_horizontal_m = drift.final_horizontal_m,
        worst_horizontal_m = drift.worst_horizontal_m,
        final_vertical_m = drift.final_vertical_m,
        "nominal cruise, ideal IMU"
    );
    assert!(drift.worst_horizontal_m < 2.0, "{drift:?}");
    assert!(drift.final_vertical_m < 2.0, "{drift:?}");
}

/// Each grade's RMS final 3-D error over the ensemble and its mean position
/// NEES, flown over `truth` from alignments of 1-sigma `alignment`.
#[cfg(test)]
fn grade_ensemble(truth: &Truth, alignment: [f64; 3], label: &str) -> [(f64, f64); 3] {
    ImuGrade::ALL.map(|grade| {
        let (mut squares, mut nees_sum, mut scored) = (0.0, 0.0, 0);
        for seed in 0..ENSEMBLE {
            let drift = dead_reckon(
                truth,
                grade.spec(),
                0x1_0000 + seed,
                seeded_alignment(alignment, seed),
            );
            squares += drift.final_horizontal_m.powi(2) + drift.final_vertical_m.powi(2);
            nees_sum += drift.nees_sum;
            scored += drift.scored;
        }
        let rms_final_m = (squares / ENSEMBLE as f64).sqrt();
        let mean_nees = nees_sum / scored as f64;
        tracing::info!(
            ?grade,
            rms_final_m,
            mean_nees,
            label,
            "60 s cruise, unaided"
        );
        assert!(
            (0.5..=8.0).contains(&mean_nees),
            "{grade:?} mean NEES {mean_nees}"
        );
        (rms_final_m, mean_nees)
    })
}

#[test]
fn better_grades_drift_less_from_a_fine_alignment() {
    init_tracing();
    let truth = cruise_truth(60.0);
    let [consumer, tactical, navigation] = grade_ensemble(&truth, FINE_ALIGNMENT, "fine");
    assert!(navigation.0 < tactical.0 && tactical.0 < consumer.0);
    // Sensor errors, not the alignment, set these: well apart.
    assert!(
        tactical.0 > 5.0 * navigation.0,
        "{tactical:?} {navigation:?}"
    );
}

#[test]
fn a_flight_alignment_stays_consistent_for_every_grade() {
    init_tracing();
    let truth = cruise_truth(60.0);
    let [consumer, tactical, navigation] = grade_ensemble(&truth, FLIGHT_ALIGNMENT, "flight");
    // Alignment error dominates the two better grades here.
    assert!(consumer.0 > tactical.0 && consumer.0 > navigation.0);
}
