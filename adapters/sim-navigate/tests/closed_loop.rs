//! Corridor flights through Navigate's real admission path and filter, with
//! the autopilot steering on the published solution, scored against truth.

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use navigate_contract::{
    FaultDetection, NavigationSolution, Redundancy, SensorClass, SolutionQuality, SourceEpoch,
    SourceId,
};
use navigate_visual::MapRevision;
use navigate_visual::fusion_bridge::{
    EvidenceIndependence, VerticalDatum, VisualErrorBudget, VisualFixSource, VisualSourceIdentity,
};
use navigate_visual::session::{
    DriftModel, MapReliability, RegionChange, SessionConfig, SurfaceModel,
};
use sim_core::{
    CameraMount, FusionEligibility, MapCell, SensorIntrinsics, SimInstant, TICKS_PER_SECOND,
    TickSpan, VehicleState, VisualFix,
};
use sim_devices::WhiteNoise;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_navigate::{
    FixDisposition, KinematicAiding, MasterClockBridge, NavigationLoop, VisualFixAdapter,
    VisualFixConfig, airborne_fusion_config, compare_with_truth, solution_position_ecef,
};
use sim_scenario::{CorridorRun, FlightDynamics, ScenarioProfile};
use sim_scenery::SyntheticCorridorSpec;

/// The bias a map cell's imagery carries, the same for every anchor in it,
/// per axis, metres.
const MAP_BIAS_SIGMA_M: f64 = 8.0;
const ATTITUDE_RMS_RAD: f64 = 0.002;
const AIDING_SIGMA_M_S: f64 = 0.3;
const AIDING_SEED: u64 = 0xA1D;
const ATTITUDE_SEED: u64 = 0xA77;
const MAX_SPEED_M_S: f64 = 150.0;
/// A 32 m/s orbit of 150 m radius pulls 6.8 m/s².
const MAX_ACCEL_M_S2: f64 = 10.0;
/// How often inertial aiding is sampled, in simulation steps of 1/120 s.
const AIDING_STEPS: u32 = 6;
const AIDING_PERIOD_S: f64 = AIDING_STEPS as f64 / 120.0;
/// A little over the anchor loop's 300 ms delay, so no anchor arrives after
/// the fresher aiding it must precede.
const HOLD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 100 * 32);
const VISUAL_SOURCE: SourceId = SourceId::new(40);
const AIDING_SOURCE: SourceId = SourceId::new(41);
/// The nose camera of the corridor runs.
const CAMERA_OFFSET_BODY_M: DVec3 = DVec3::new(1.0, 0.0, 0.0);
/// 99.9 % of chi-square with 3 degrees of freedom.
const NEES_GATE: f64 = 16.27;

#[cfg(test)]
fn visual_config() -> VisualFixConfig {
    let mut session = SessionConfig::standard();
    // Differencing two 2 m warm poses gives about 2.8 m of step error.
    session.drift = DriftModel {
        step_floor_m: 3.0,
        ..DriftModel::default()
    };
    VisualFixConfig {
        anchor: SyntheticCorridorSpec::delaware().south_west,
        mount: CameraMount::forward_nose(CAMERA_OFFSET_BODY_M),
        cg_body_m: DVec3::ZERO,
        intrinsics: SensorIntrinsics {
            width: 800,
            height: 600,
            fx: 600.0,
            fy: 600.0,
            cx: 399.5,
            cy: 299.5,
        },
        map: MapRevision {
            release_id: "synthetic-delaware".into(),
            manifest_sha256: "d".repeat(64),
        },
        budget: VisualErrorBudget {
            map_horizontal_m: MAP_BIAS_SIGMA_M,
            map_vertical_m: MAP_BIAS_SIGMA_M,
            calibration_m: 0.5,
            geometry_model_m: 0.5,
            vertical_datum: VerticalDatum::Wgs84Ellipsoid,
            vertical_datum_m: 0.5,
        },
        reliability: MapReliability {
            horizontal_m: MAP_BIAS_SIGMA_M,
            vertical_m: MAP_BIAS_SIGMA_M,
            imagery_age_years: Some(0.0),
            age_growth_m_per_year: 0.0,
            unknown_age_years: 0.0,
            surface: SurfaceModel::Surface,
            change: RegionChange::Unchanged,
        },
        identity: VisualSourceIdentity {
            source: VISUAL_SOURCE,
            epoch: SourceEpoch::new(0),
            clock: MasterClockBridge::default().domain(),
        },
        host_independence: EvidenceIndependence::ValidatedIndependent,
        attitude_rms_rad: ATTITUDE_RMS_RAD,
        session,
    }
}

/// Each cell's imagery registration error: fixed by the cell, so two anchors
/// in one cell repeat it, which is what makes them correlated.
#[cfg(test)]
fn map_bias(cell: MapCell) -> DVec3 {
    let seed =
        (cell.cell_x as u64).wrapping_mul(0x9E37) ^ (cell.cell_y as u64).wrapping_mul(0xC13F);
    WhiteNoise::from_seed(seed).next_vec3(DVec3::splat(MAP_BIAS_SIGMA_M))
}

/// A harness anchor as a map with registration error would give it.
#[cfg(test)]
fn with_map_bias(fix: &VisualFix) -> VisualFix {
    let mut biased = *fix;
    if let Some(cell) = fix.map_cell {
        biased.estimate.position_enu_m += map_bias(cell);
    }
    biased
}

/// The true vehicle with the solution's position and velocity in place of the
/// truth. Attitude and rates stay true: the filter estimates neither.
#[cfg(test)]
fn navigated_state(truth: VehicleState, solution: &NavigationSolution) -> VehicleState {
    let position_ecef = solution_position_ecef(solution);
    let velocity = solution.velocity;
    let ned_to_ecef = LocalFrame::at(&ecef_to_geodetic(position_ecef)).ned_to_ecef();
    VehicleState {
        position_ecef,
        velocity_ecef: ned_to_ecef
            * DVec3::new(velocity.north_mps, velocity.east_mps, velocity.down_mps),
        ..truth
    }
}

/// Adds the attitude reference's error to the truth.
#[cfg(test)]
fn measured_attitude(truth: DQuat, noise: &mut WhiteNoise) -> DQuat {
    let error = noise.next_vec3(DVec3::splat(ATTITUDE_RMS_RAD));
    (truth * DQuat::from_scaled_axis(error)).normalize()
}

/// What one closed-loop flight showed.
#[cfg(test)]
#[derive(Debug, Default)]
struct FlightResult {
    admitted: u64,
    admitted_cells: Vec<MapCell>,
    refused_shared_cell: u64,
    tracked: u64,
    not_anchored: u64,
    solutions: u64,
    nees_sum: f64,
    worst_horizontal_error_m: f64,
    worst_nees: f64,
    worst_horizontal_sigma_m: f64,
    final_horizontal_sigma_m: f64,
    last: Option<NavigationSolution>,
    saw_independent_classes: bool,
    /// A filter that takes every anchor as independent, as a host that
    /// ignored the map-cell labels would.
    naive_nees_sum: f64,
    naive_final_horizontal_sigma_m: f64,
}

#[cfg(test)]
impl FlightResult {
    fn mean_nees(&self) -> f64 {
        self.nees_sum / self.solutions as f64
    }

    fn naive_mean_nees(&self) -> f64 {
        self.naive_nees_sum / self.solutions as f64
    }
}

/// The same fixes without the discipline: every anchor goes in as independent.
#[cfg(test)]
struct NaiveFilter {
    source: VisualFixSource,
    filter: NavigationLoop,
    next_sequence: u64,
}

#[cfg(test)]
impl NaiveFilter {
    fn new() -> Self {
        let config = visual_config();
        Self {
            source: VisualFixSource::new(config.identity, config.budget).unwrap(),
            filter: new_loop(),
            next_sequence: 0,
        }
    }

    fn offer_anchor(&mut self, adapter: &VisualFixAdapter, fix: &VisualFix, attitude: DQuat) {
        let sequence = self.next_sequence;
        self.next_sequence = sequence.wrapping_add(1);
        let estimate = adapter.vehicle_estimate(fix, attitude, sequence).unwrap();
        let converted = self
            .source
            .convert(&estimate, EvidenceIndependence::ValidatedIndependent)
            .unwrap();
        self.filter.offer(converted.observation);
    }
}

#[cfg(test)]
fn new_loop() -> NavigationLoop {
    NavigationLoop::new(
        airborne_fusion_config(MAX_SPEED_M_S, MAX_ACCEL_M_S2, AIDING_PERIOD_S),
        MasterClockBridge::default(),
        HOLD,
    )
    .unwrap()
}

#[cfg(test)]
fn fly(profile: ScenarioProfile, dynamics: FlightDynamics) -> FlightResult {
    let clock = MasterClockBridge::default();
    let mut run = CorridorRun::new(profile, dynamics).unwrap();
    let mut adapter = VisualFixAdapter::new(visual_config(), clock).unwrap();
    let mut navigation = new_loop();
    let mut naive = NaiveFilter::new();
    let mut aiding =
        KinematicAiding::new(clock, AIDING_SOURCE, SourceEpoch::new(0), AIDING_SIGMA_M_S);
    let mut naive_aiding =
        KinematicAiding::new(clock, AIDING_SOURCE, SourceEpoch::new(0), AIDING_SIGMA_M_S);
    let mut velocity_noise = WhiteNoise::from_seed(AIDING_SEED);
    let mut attitude_noise = WhiteNoise::from_seed(ATTITUDE_SEED);
    let mut attitudes: BTreeMap<SimInstant, DQuat> = BTreeMap::new();
    let mut result = FlightResult::default();
    let mut navigated = None;
    let mut step = 0u32;

    while !run.finished() {
        let fixes = run.step_navigated(navigated).unwrap();
        if let Some(capture) = run.latest_capture() {
            attitudes.entry(capture.timestamp).or_insert_with(|| {
                measured_attitude(capture.vehicle.attitude, &mut attitude_noise)
            });
        }
        for fix in &fixes {
            let attitude = attitudes[&fix.captured_at];
            let fix = with_map_bias(fix);
            if fix.eligibility != FusionEligibility::CorrelatedTrack {
                naive.offer_anchor(&adapter, &fix, attitude);
            }
            match adapter.offer(&fix, attitude).unwrap() {
                FixDisposition::Tracked => result.tracked += 1,
                FixDisposition::Admitted(admitted) => {
                    result.admitted += 1;
                    result.admitted_cells.extend(fix.map_cell);
                    navigation.offer(admitted.observation);
                }
                FixDisposition::RefusedCorrelated { .. } => result.refused_shared_cell += 1,
                FixDisposition::NotAnchored(_) => result.not_anchored += 1,
            }
        }
        let truth = run.vehicle();
        step += 1;
        if step.is_multiple_of(AIDING_STEPS) {
            let measured =
                truth.velocity_ecef + velocity_noise.next_vec3(DVec3::splat(AIDING_SIGMA_M_S));
            let at = run.now();
            navigation.offer(aiding.observe(at, truth.position_ecef, measured).unwrap());
            naive.filter.offer(
                naive_aiding
                    .observe(at, truth.position_ecef, measured)
                    .unwrap(),
            );
        }
        let Some(solution) = navigation.advance(run.now()).unwrap() else {
            continue;
        };
        let naive_solution = naive.filter.advance(run.now()).unwrap();
        score(&mut result, &solution, naive_solution.as_ref(), truth);
        navigated = Some(navigated_state(truth, &solution));
    }
    result
}

#[cfg(test)]
fn score(
    result: &mut FlightResult,
    solution: &NavigationSolution,
    naive: Option<&NavigationSolution>,
    truth: VehicleState,
) {
    let comparison = compare_with_truth(solution, truth.position_ecef).unwrap();
    let sigma = solution.integrity.horizontal_1sigma_m;
    result.solutions += 1;
    result.nees_sum += comparison.position_nees;
    result.worst_nees = result.worst_nees.max(comparison.position_nees);
    result.worst_horizontal_error_m = result
        .worst_horizontal_error_m
        .max(comparison.horizontal_error_m);
    result.worst_horizontal_sigma_m = result.worst_horizontal_sigma_m.max(sigma);
    result.final_horizontal_sigma_m = sigma;
    result.last = Some(*solution);
    result.saw_independent_classes |=
        solution.integrity.redundancy == Redundancy::IndependentClasses;
    if let Some(naive) = naive {
        let naive_comparison = compare_with_truth(naive, truth.position_ecef).unwrap();
        result.naive_nees_sum += naive_comparison.position_nees;
        result.naive_final_horizontal_sigma_m = naive.integrity.horizontal_1sigma_m;
    }
}

/// Crossing the corridor earns fresh map cells: each is admitted once, and
/// the solution holds truth with a covariance that means what it says.
#[test]
fn a_corridor_cruise_is_tracked_on_new_cells_with_an_honest_covariance() {
    for dynamics in [FlightDynamics::Kinematic, FlightDynamics::Jsbsim] {
        let result = fly(ScenarioProfile::CorridorNominalCruise, dynamics);

        assert!(result.admitted >= 8, "{dynamics:?}: {result:?}");
        let mut cells = result.admitted_cells.clone();
        cells.sort_unstable();
        cells.dedup();
        assert_eq!(cells.len(), result.admitted_cells.len(), "{result:?}");
        assert!(result.refused_shared_cell > 0, "{result:?}");
        assert!(
            result.worst_horizontal_error_m < 3.0 * result.worst_horizontal_sigma_m,
            "{dynamics:?}: {result:?}"
        );
        let mean = result.mean_nees();
        assert!(
            mean < 3.0 * 3.0,
            "{dynamics:?}: mean NEES {mean}: {result:?}"
        );
        assert!(
            mean > 3.0 / 100.0,
            "{dynamics:?}: covariance is far too wide: {mean}"
        );
    }
}

/// An orbit stays in one cell, so every anchor after the first repeats its map
/// error. The disciplined filter must refuse them and stay consistent, where a
/// filter that took them as independent collapses onto the bias.
#[test]
fn a_same_cell_loiter_refuses_revisits_and_its_covariance_does_not_collapse() {
    for dynamics in [FlightDynamics::Kinematic, FlightDynamics::Jsbsim] {
        let result = fly(ScenarioProfile::CorridorSameCellLoiter, dynamics);

        assert_eq!(result.admitted, 1, "{dynamics:?}: {result:?}");
        // The rest wait on the session's agreement between frames.
        assert!(
            result.refused_shared_cell >= 100,
            "{dynamics:?}: {result:?}"
        );
        assert!(
            result.refused_shared_cell + result.not_anchored >= 200,
            "{dynamics:?}: {result:?}"
        );
        assert!(result.tracked >= 3_000, "{dynamics:?}: {result:?}");
        assert!(
            result.final_horizontal_sigma_m > 3.0 * result.naive_final_horizontal_sigma_m,
            "{dynamics:?}: {result:?}"
        );
        assert!(
            result.mean_nees() < NEES_GATE,
            "{dynamics:?}: {}: {result:?}",
            result.mean_nees()
        );
        assert!(
            result.naive_mean_nees() > 3.0 * result.mean_nees(),
            "{dynamics:?}: naive {} against {}",
            result.naive_mean_nees(),
            result.mean_nees()
        );
    }
}

/// The contract states what the solution rests on and no more.
#[test]
fn the_published_integrity_claims_only_what_the_sources_support() {
    let result = fly(
        ScenarioProfile::CorridorNominalCruise,
        FlightDynamics::Kinematic,
    );
    let last = result.last.unwrap();
    let integrity = last.integrity;

    // Visual and inertial cross-check only while a fix is inside the five
    // second window; between cells the filter must say nothing can.
    assert!(result.saw_independent_classes, "{result:?}");
    assert_eq!(integrity.redundancy, Redundancy::None);
    assert_eq!(integrity.fault_detection, FaultDetection::Unavailable);
    assert!(last.composition.contains(SensorClass::VisualLandmark));
    assert!(last.composition.contains(SensorClass::Inertial));
    assert!(!last.composition.is_estimator_derived());
    assert_ne!(integrity.quality, SolutionQuality::Unusable);
    // ADR-0004 reserves protection levels until an implementation earns them.
    assert_eq!(integrity.protection_levels, None);
    let (north, east, down) = last.position_cov.diagonal();
    assert!(north > 0.0 && east > 0.0 && down > 0.0);
}

/// Two flights of the loop are the same bits.
#[test]
fn the_loop_replays_bit_for_bit() {
    let solutions = |_: ()| {
        let result = fly(
            ScenarioProfile::CorridorSameCellLoiter,
            FlightDynamics::Kinematic,
        );
        let last = result.last.unwrap();
        (
            last.position.latitude_rad.to_bits(),
            last.position.longitude_rad.to_bits(),
            result.nees_sum.to_bits(),
            result.admitted,
        )
    };
    assert_eq!(solutions(()), solutions(()));
}
