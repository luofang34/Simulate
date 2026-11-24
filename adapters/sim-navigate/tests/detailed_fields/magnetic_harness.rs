use glam::DVec3;
use navigate_contract::{
    GeodeticPosition, NavigationSolution, SensorClass, SourceComposition, SourceEpoch, SourceId,
    Waypoint,
};
use navigate_fusion::{FogmBias, IngestOutcome, MeasurementKind, Observation, ObservationValue};
use sim_atmosphere::{AtmosphereField, AtmosphereSample, GeoidStandardAtmosphere};
use sim_bodies::OutsideCoverage;
use sim_core::{
    Block, KinematicInput, KinematicModel, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_devices::{AirDataSample, AirDataSystem, AirFlow, ImuGrade};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, egm96_undulation_m};
use sim_navigate::trial::{
    Autopilot, GuidanceSource, PilotInput, QualityFloor, VehicleSpec, WaypointSpec, plan_waypoints,
    trial_director,
};
use sim_navigate::{
    Estimate, InertialNavigator, InitialAlignment, MasterClockBridge, compare_with_truth,
    inertial_config,
};

use super::magnetic_world::{
    MagneticAidingMode, Sensors, build_sensors, corridor_crustal_overlay, pressure_altitude_m,
};

const STEP_HZ: u64 = 120;
const STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / STEP_HZ);
const AID_STEPS: usize = 120;
const PITOT_STEPS: usize = 5;

const IMU_SOURCE: SourceId = SourceId::new(72);
const BARO_SOURCE: SourceId = SourceId::new(73);

#[derive(Debug, Clone)]
pub struct MagNavSummary {
    pub completed: bool,
    pub horizontal_rmse_m: f64,
    pub final_horizontal_error_m: f64,
    pub final_waypoint_miss_m: f64,
    pub mean_position_nees: f64,
    pub mag_admitted: u64,
    pub mag_uninformative: u64,
    pub composition: SourceComposition,
    pub sigma_series_m: Vec<(f64, f64)>,
}

pub fn corridor_waypoints() -> Vec<WaypointSpec> {
    let wp = |ident: &str, lat_deg: f64, lon_deg: f64, altitude_m: f64| WaypointSpec {
        ident: ident.to_owned(),
        lat_deg,
        lon_deg,
        altitude_m,
        speed_m_s: None,
        fly_over: false,
    };
    vec![
        wp("M0", 39.00, -75.60, 1_000.0),
        wp("M1", 39.00, -75.5075, 1_000.0),
        wp("M2", 39.0592, -75.4798, 1_300.0),
        wp("M3", 39.0483, -75.4001, 1_300.0),
    ]
}

struct LoopState {
    clock: MasterClockBridge,
    model: KinematicModel,
    autopilot: Autopilot,
    sensors: Sensors,
    navigator: InertialNavigator,
    air_data: Option<AirDataSample>,
    sensed_rate: DVec3,
    last_solution: Option<NavigationSolution>,
    fallback_pos: GeodeticPosition,
    at: SimInstant,
    sq_err_sum: f64,
    nees_sum: f64,
    scored: u64,
    final_horiz: f64,
    sigma_series_m: Vec<(f64, f64)>,
    switched: bool,
    completed: bool,
}

fn initial_heading(first: &Waypoint, second: &Waypoint) -> f64 {
    let f = Geodetic {
        lat_rad: first.position.latitude_rad,
        lon_rad: first.position.longitude_rad,
        height_m: first.position.altitude_m,
    };
    let frame = LocalFrame::at(&f);
    let d = BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic {
        lat_rad: second.position.latitude_rad,
        lon_rad: second.position.longitude_rad,
        height_m: second.position.altitude_m,
    }) - BodyEllipsoid::WGS84.geodetic_to_fixed(&f);
    d.dot(frame.east()).atan2(d.dot(frame.north()))
}

fn init_loop_state(
    waypoints: &[Waypoint],
    vehicle_spec: &VehicleSpec,
    mode: MagneticAidingMode,
    seed: u64,
) -> LoopState {
    let clock = MasterClockBridge::default();
    let autopilot = Autopilot::new(
        trial_director(waypoints, vehicle_spec).unwrap(),
        vehicle_spec,
    );
    let first = &waypoints[0];
    let initial = VehicleState::level_flight(
        &Geodetic {
            lat_rad: first.position.latitude_rad,
            lon_rad: first.position.longitude_rad,
            height_m: first.position.altitude_m,
        },
        initial_heading(first, &waypoints[1]),
        vehicle_spec.cruise_speed_m_s,
    );
    let mut sensors = build_sensors(
        mode,
        seed,
        clock,
        &corridor_crustal_overlay(OutsideCoverage::Zero),
        &corridor_crustal_overlay(OutsideCoverage::Refuse),
    );
    let mut cfg = inertial_config(&ImuGrade::Tactical.spec());
    cfg.baro_bias = FogmBias::new(600.0, 1.85);
    let mut navigator = InertialNavigator::new(cfg, clock, IMU_SOURCE);
    let mut align_fix = InitialAlignment {
        position_sigma_m: DVec3::splat(5.0),
        velocity_sigma_m_s: DVec3::splat(0.05),
        attitude_sigma_rad: DVec3::new(3e-4, 3e-4, 1e-3),
        seed: 31_u64.wrapping_add(seed),
    }
    .measure(&initial);
    // Transfer-aligned GNSS-denied start so output composition reports only
    // Inertial, Barometric, and (when admitted) MagneticAnomaly.
    align_fix.composition = SourceComposition::of(SensorClass::Inertial);
    navigator.align(&align_fix, SimInstant::START).unwrap();
    sensors.imu.sense(SimInstant::START, &initial).unwrap();
    let air0 = GeoidStandardAtmosphere::new()
        .sample(
            &sim_geo::ecef_to_geodetic(initial.position_ecef),
            sensors.utc,
        )
        .unwrap();
    let air_data = Some(
        sensors
            .pitot
            .sense(&AirFlow::of(&air0, &initial), PITOT_STEPS as f64 / 120.0),
    );
    LoopState {
        clock,
        model: KinematicModel::new(initial),
        autopilot,
        sensors,
        navigator,
        air_data,
        sensed_rate: DVec3::ZERO,
        last_solution: None,
        fallback_pos: align_fix.position,
        at: SimInstant::START,
        sq_err_sum: 0.0,
        nees_sum: 0.0,
        scored: 0,
        final_horiz: 0.0,
        sigma_series_m: Vec::new(),
        switched: false,
        completed: false,
    }
}

fn aid_at_1hz(
    state: &mut LoopState,
    mode: MagneticAidingMode,
    truth: &VehicleState,
    air: &AtmosphereSample,
) {
    let est_pos = state
        .navigator
        .solution(state.at)
        .unwrap()
        .map_or(state.fallback_pos, |s| s.position);
    let baro_sample = state
        .sensors
        .static_port
        .sense(&AirFlow::of(air, truth), AID_STEPS as f64 / 120.0);
    let altitude_m = pressure_altitude_m(baro_sample.static_pressure_pa)
        + egm96_undulation_m(est_pos.latitude_rad, est_pos.longitude_rad);
    let baro_obs = Observation::new(
        state
            .clock
            .stamp(
                BARO_SOURCE,
                SourceEpoch::new(0),
                state.sensors.baro_seq,
                state.at,
            )
            .unwrap(),
        ObservationValue::Altitude {
            altitude_m,
            variance_m2: state.sensors.baro_variance_m2,
        },
        SourceComposition::of(SensorClass::Barometric),
    );
    state.sensors.baro_seq = state.sensors.baro_seq.next();
    let baro_outcome = state.navigator.observe(&baro_obs, state.at).unwrap();
    assert!(
        matches!(
            baro_outcome,
            IngestOutcome::Accepted | IngestOutcome::Rejected(_)
        ),
        "barometric altitude update had unexpected outcome: {baro_outcome:?}"
    );

    if matches!(mode, MagneticAidingMode::None) {
        return;
    }
    let total_t = state.sensors.mag_sensor.sense_total_field_t(truth).unwrap();
    let est_after_baro = state
        .navigator
        .solution(state.at)
        .unwrap()
        .map_or(est_pos, |s| s.position);
    if let Some(mag_obs) = state
        .sensors
        .mag_map
        .observe_total_field(state.at, &Estimate::new(est_after_baro), total_t)
        .unwrap()
    {
        let outcome = state.navigator.observe(&mag_obs, state.at).unwrap();
        if matches!(mode, MagneticAidingMode::MainFieldOnly) {
            assert!(
                !matches!(outcome, IngestOutcome::Accepted),
                "main field only should not be admitted: {outcome:?}"
            );
        }
    }
}

fn step_loop_once(
    state: &mut LoopState,
    step: usize,
    mode: MagneticAidingMode,
    revision_switch: Option<(f64, u64, f64)>,
) -> bool {
    let pilot_out = state.autopilot.step(
        &PilotInput {
            estimate: state.last_solution.map(Estimate::new),
            sensed_rate_body: state.sensed_rate,
            sensed_air_data: state.air_data,
            now: state.clock.to_monotonic(state.at).unwrap(),
        },
        STEP.as_seconds(),
    );
    if pilot_out.event.is_some_and(|e| e.plan_complete) || pilot_out.active_index.is_none() {
        state.completed = true;
        return false;
    }
    state
        .model
        .step(&KinematicInput {
            controls: pilot_out.controls,
        })
        .unwrap();
    state.at = state.at.advanced(STEP);
    let t_s = state.at.duration_since(SimInstant::START).as_seconds();
    let truth = state.model.output();
    let inc = state.sensors.imu.sense(state.at, &truth).unwrap().unwrap();
    state.sensed_rate = inc.delta_theta_rad / inc.dt_s;
    state.navigator.ingest(&inc, state.at).unwrap();

    let air = GeoidStandardAtmosphere::new()
        .sample(
            &sim_geo::ecef_to_geodetic(truth.position_ecef),
            state.sensors.utc.advanced_by(t_s),
        )
        .unwrap();
    if step.is_multiple_of(PITOT_STEPS) {
        state.air_data = Some(
            state
                .sensors
                .pitot
                .sense(&AirFlow::of(&air, &truth), PITOT_STEPS as f64 / 120.0),
        );
    }
    if let Some((switch_s, rev, bias_t)) = revision_switch
        && !state.switched
        && t_s >= switch_s
    {
        state
            .sensors
            .mag_map
            .set_map_revision(Some(rev), bias_t)
            .unwrap();
        state.switched = true;
    }
    if step.is_multiple_of(AID_STEPS) {
        aid_at_1hz(state, mode, &truth, &air);
    }
    state.last_solution = state.navigator.solution(state.at).unwrap();
    if step.is_multiple_of(AID_STEPS)
        && let Some(sol) = &state.last_solution
    {
        let cmp = compare_with_truth(sol, truth.position_ecef).unwrap();
        state.sq_err_sum += cmp.horizontal_error_m * cmp.horizontal_error_m;
        state.nees_sum += cmp.position_nees;
        state.final_horiz = cmp.horizontal_error_m;
        state.scored += 1;
        state
            .sigma_series_m
            .push((t_s, sol.integrity.horizontal_1sigma_m));
    }
    true
}

pub fn fly_magnav_route(
    waypoints_spec: &[WaypointSpec],
    mode: MagneticAidingMode,
    min_quality: QualityFloor,
    seed: u64,
    max_seconds: f64,
    revision_switch: Option<(f64, u64, f64)>,
) -> MagNavSummary {
    let vehicle_spec = VehicleSpec {
        cruise_speed_m_s: 60.0,
        bank_limit_deg: 18.0,
        max_climb_m_s: 5.0,
        capture_radius_m: 100.0,
        guidance: GuidanceSource::Estimate,
        guidance_min_quality: Some(min_quality),
    };
    let waypoints = plan_waypoints(waypoints_spec).unwrap();
    let mut state = init_loop_state(&waypoints, &vehicle_spec, mode, seed);
    let max_steps = (max_seconds * STEP_HZ as f64).round() as usize;
    for step in 1..=max_steps {
        if !step_loop_once(&mut state, step, mode, revision_switch) {
            break;
        }
    }
    let final_truth = state.model.output();
    let last_wp = waypoints.last().unwrap().position;
    let last_wp_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic {
        lat_rad: last_wp.latitude_rad,
        lon_rad: last_wp.longitude_rad,
        height_m: last_wp.altitude_m,
    });
    let frame = LocalFrame::at(&sim_geo::ecef_to_geodetic(last_wp_ecef));
    let miss_ned = frame.ecef_to_ned_vector(final_truth.position_ecef - last_wp_ecef);
    let sol = state.last_solution.unwrap();
    let mag_admitted = state
        .navigator
        .filter()
        .innovation_stats(MeasurementKind::MagneticAnomaly)
        .map_or(0, |s| s.admitted_count);
    MagNavSummary {
        completed: state.completed,
        horizontal_rmse_m: (state.sq_err_sum / state.scored.max(1) as f64).sqrt(),
        final_horizontal_error_m: state.final_horiz,
        final_waypoint_miss_m: miss_ned.x.hypot(miss_ned.y),
        mean_position_nees: state.nees_sum / state.scored.max(1) as f64,
        mag_admitted,
        mag_uninformative: state.navigator.filter().rejections().uninformative_gradient,
        composition: sol.composition,
        sigma_series_m: state.sigma_series_m,
    }
}
