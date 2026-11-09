//! Unaided inertial dead reckoning through manoeuvres: the corridor profiles'
//! banks, orbits and turns on both flight-dynamics sources, an ideal IMU
//! first and then each grade preset, scored against truth outside the filter.

use glam::DVec3;
use navigate_contract::SourceId;
use sim_core::{SimInstant, VehicleState};
use sim_devices::{ImuErrorSpec, ImuGrade};
use sim_navigate::{
    InertialNavigator, InitialAlignment, MasterClockBridge, SimulatedImu, Truth, TruthGravity,
    compare_attitude_with_truth, compare_with_truth, inertial_config,
};
use sim_scenario::{CorridorRun, FlightDynamics, ScenarioProfile};

const IMU_SOURCE: SourceId = SourceId::new(62);
/// Seeds per grade; one run's errors are one correlated realisation.
const ENSEMBLE: u64 = 4;
/// An alignment fine enough that each grade's own errors dominate: position,
/// velocity and attitude 1-sigma, in m, m/s and rad.
const FINE_ALIGNMENT: [f64; 3] = [0.1, 0.01, 2e-5];

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

/// The true flight of `profile` on `dynamics`, every step, for `seconds`.
#[cfg(test)]
fn flight(
    profile: ScenarioProfile,
    dynamics: FlightDynamics,
    seconds: f64,
) -> Vec<Truth<(SimInstant, VehicleState)>> {
    let mut run = CorridorRun::new(profile, dynamics).unwrap();
    let mut truth = vec![Truth::new((run.now(), run.vehicle()))];
    while run.now().duration_since(SimInstant::START).as_seconds() < seconds && !run.finished() {
        run.step().unwrap();
        truth.push(Truth::new((run.now(), run.vehicle())));
    }
    truth
}

#[cfg(test)]
#[derive(Debug, Default, Clone, Copy)]
struct Drift {
    final_horizontal_m: f64,
    final_vertical_m: f64,
    worst_horizontal_m: f64,
    worst_vertical_m: f64,
    worst_attitude_rad: f64,
    position_nees_sum: f64,
    attitude_nees_sum: f64,
    scored: u64,
    peak_rate_rad_s: f64,
    peak_accel_m_s2: f64,
}

#[cfg(test)]
impl Drift {
    fn mean_position_nees(&self) -> f64 {
        self.position_nees_sum / self.scored as f64
    }

    fn mean_attitude_nees(&self) -> f64 {
        self.attitude_nees_sum / self.scored as f64
    }
}

/// Dead-reckons over `truth`, scoring ten times a second.
#[cfg(test)]
fn dead_reckon(
    truth: &[Truth<(SimInstant, VehicleState)>],
    spec: ImuErrorSpec,
    seed: u64,
    alignment: InitialAlignment,
) -> Drift {
    let clock = MasterClockBridge::default();
    let mut imu = SimulatedImu::new(spec, seed, TruthGravity::WgsJ2).unwrap();
    let mut navigator = InertialNavigator::new(inertial_config(&spec), clock, IMU_SOURCE);
    let (start, initial) = *truth[0].get();
    navigator
        .align(&alignment.measure(&initial), start)
        .unwrap();
    assert!(imu.sense(start, &initial).unwrap().is_none());
    let mut drift = Drift::default();
    let mut previous = initial;
    for (step, sample) in truth.iter().enumerate().skip(1) {
        let (at, state) = *sample.get();
        let increment = imu.sense(at, &state).unwrap().unwrap();
        navigator.ingest(&increment, at).unwrap();
        let dt = increment.dt_s;
        drift.peak_rate_rad_s = drift.peak_rate_rad_s.max(state.angular_rate_body.length());
        drift.peak_accel_m_s2 = drift
            .peak_accel_m_s2
            .max(((state.velocity_ecef - previous.velocity_ecef) / dt).length());
        previous = state;
        if step % 12 != 0 && step != truth.len() - 1 {
            continue;
        }
        let solution = navigator.solution(at).unwrap().unwrap();
        let position = compare_with_truth(&solution, state.position_ecef).unwrap();
        let attitude = compare_attitude_with_truth(&solution, state.attitude).unwrap();
        drift.final_horizontal_m = position.horizontal_error_m;
        drift.final_vertical_m = position.vertical_error_m;
        drift.worst_horizontal_m = drift.worst_horizontal_m.max(position.horizontal_error_m);
        drift.worst_vertical_m = drift.worst_vertical_m.max(position.vertical_error_m);
        drift.worst_attitude_rad = drift.worst_attitude_rad.max(attitude.error_rad);
        drift.position_nees_sum += position.position_nees;
        drift.attitude_nees_sum += attitude.attitude_nees;
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

#[cfg(test)]
fn ideal_drift(profile: ScenarioProfile, dynamics: FlightDynamics, seconds: f64) -> Drift {
    let truth = flight(profile, dynamics, seconds);
    let drift = dead_reckon(&truth, ImuErrorSpec::IDEAL, 0, InitialAlignment::EXACT);
    tracing::info!(
        ?profile,
        ?dynamics,
        seconds,
        worst_horizontal_m = drift.worst_horizontal_m,
        worst_vertical_m = drift.worst_vertical_m,
        final_horizontal_m = drift.final_horizontal_m,
        final_vertical_m = drift.final_vertical_m,
        worst_attitude_urad = drift.worst_attitude_rad * 1e6,
        peak_rate_rad_s = drift.peak_rate_rad_s,
        peak_accel_m_s2 = drift.peak_accel_m_s2,
        "ideal IMU"
    );
    drift
}

/// An ideal IMU must leave only what a parked one leaves, the 1e-4 m of the
/// two `J2` constants: a sculling, coning or truth-integration mismatch under
/// turn rate and acceleration shows up as centimetres to metres.
#[cfg(test)]
fn assert_ideal(drift: &Drift) {
    assert!(drift.worst_horizontal_m < 1e-3, "{drift:?}");
    assert!(drift.worst_vertical_m < 1e-3, "{drift:?}");
    assert!(drift.worst_attitude_rad < 1e-9, "{drift:?}");
}

#[test]
fn an_ideal_imu_follows_kinematic_manoeuvres() {
    init_tracing();
    for (profile, seconds) in [
        (ScenarioProfile::CorridorAggressiveBank, 60.0),
        (ScenarioProfile::CorridorSameCellLoiter, 60.0),
        (ScenarioProfile::CorridorOffSeedAcquisition, 60.0),
    ] {
        assert_ideal(&ideal_drift(profile, FlightDynamics::Kinematic, seconds));
    }
}

#[test]
fn an_ideal_imu_follows_jsbsim_manoeuvres() {
    init_tracing();
    for (profile, seconds) in [
        (ScenarioProfile::CorridorAggressiveBank, 60.0),
        (ScenarioProfile::CorridorSameCellLoiter, 60.0),
    ] {
        assert_ideal(&ideal_drift(profile, FlightDynamics::Jsbsim, seconds));
    }
}

#[cfg(test)]
fn grade_runs(profile: ScenarioProfile, dynamics: FlightDynamics, seconds: f64) {
    let truth = flight(profile, dynamics, seconds);
    for grade in ImuGrade::ALL {
        let runs: Vec<Drift> = (0..ENSEMBLE)
            .map(|seed| {
                dead_reckon(
                    &truth,
                    grade.spec(),
                    0x2_0000 + seed,
                    seeded_alignment(FINE_ALIGNMENT, seed),
                )
            })
            .collect();
        let count = runs.len() as f64;
        let rms =
            |f: fn(&Drift) -> f64| (runs.iter().map(|d| f(d).powi(2)).sum::<f64>() / count).sqrt();
        let position_nees = runs.iter().map(Drift::mean_position_nees).sum::<f64>() / count;
        let attitude_nees = runs.iter().map(Drift::mean_attitude_nees).sum::<f64>() / count;
        tracing::info!(
            ?profile,
            ?dynamics,
            ?grade,
            rms_final_horizontal_m = rms(|d| d.final_horizontal_m),
            rms_final_vertical_m = rms(|d| d.final_vertical_m),
            rms_worst_attitude_mrad = rms(|d| d.worst_attitude_rad) * 1e3,
            position_nees,
            attitude_nees,
            "grade under manoeuvre"
        );
        assert!(
            (0.3..=10.0).contains(&position_nees),
            "{grade:?} {position_nees}"
        );
        assert!(
            (0.3..=10.0).contains(&attitude_nees),
            "{grade:?} {attitude_nees}"
        );
    }
}

#[test]
fn grades_stay_consistent_through_the_aggressive_bank() {
    init_tracing();
    grade_runs(
        ScenarioProfile::CorridorAggressiveBank,
        FlightDynamics::Kinematic,
        60.0,
    );
}

#[test]
fn grades_stay_consistent_in_cruise() {
    init_tracing();
    grade_runs(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Kinematic,
        60.0,
    );
}

#[test]
fn grades_stay_consistent_through_the_jsbsim_bank() {
    init_tracing();
    grade_runs(
        ScenarioProfile::CorridorAggressiveBank,
        FlightDynamics::Jsbsim,
        60.0,
    );
}
