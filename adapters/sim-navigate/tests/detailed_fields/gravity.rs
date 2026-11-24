use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use navigate_contract::{NavigationSolution, SourceId};
use sim_atmosphere::VacuumAtmosphere;
use sim_bodies::{
    AnomalyOverlay, GravityWithAnomaly, GridGeometry, HarmonicGravity, OutsideCoverage,
};
use sim_core::{
    Block, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, SimInstant, TICKS_PER_SECOND,
    ThrusterWrench, TickSpan, VehicleState,
};
use sim_devices::ImuErrorSpec;
use sim_geo::{BodyEllipsoid, Geodetic, GravityField, LocalFrame, UtcTime, ecef_to_geodetic};
use sim_navigate::{
    InertialNavigator, InitialAlignment, MasterClockBridge, SimulatedImu, compare_with_truth,
    inertial_config, solution_position_ecef,
};

const STEP_HZ: u64 = 120;
const STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / STEP_HZ);
const IMU_SOURCE: SourceId = SourceId::new(71);
const VEHICLE_MASS_KG: f64 = 1_000.0;
const EPOCH_UNIX_S: f64 = 1_773_972_000.0;

#[derive(Debug, Clone, Copy)]
struct FlightSummary {
    actual_dp: DVec3,
    actual_dv: DVec3,
    raw_dp: DVec3,
    raw_dv: DVec3,
    schuler_dp: DVec3,
    schuler_dv: DVec3,
    uniform_dp: DVec3,
    max_disturbance_delta_m_s2: f64,
    final_horizontal_error_m: f64,
    final_vertical_error_m: f64,
    max_specific_force_m_s2: f64,
}

#[derive(Default)]
struct DisturbanceAccum {
    p_prev: DVec3,
    dg_prev: DVec3,
    dg0: DVec3,
    raw_dp: DVec3,
    raw_dv: DVec3,
    schuler_dp: DVec3,
    schuler_dv: DVec3,
    max_delta: f64,
}

impl DisturbanceAccum {
    fn new(p0: DVec3, dg0: DVec3) -> Self {
        Self {
            p_prev: p0,
            dg_prev: dg0,
            dg0,
            ..Self::default()
        }
    }

    fn update(&mut self, p: DVec3, dg: DVec3, dt_s: f64) {
        self.max_delta = self.max_delta.max((dg - self.dg0).length());
        let dv_next = self.raw_dv + 0.5 * (self.dg_prev + dg) * dt_s;
        self.raw_dp += 0.5 * (self.raw_dv + dv_next) * dt_s;
        self.raw_dv = dv_next;
        (self.schuler_dp, self.schuler_dv) = step_schuler_perturbation(
            self.schuler_dp,
            self.schuler_dv,
            self.p_prev,
            p,
            self.dg_prev,
            dg,
            dt_s,
        );
        self.p_prev = p;
        self.dg_prev = dg;
    }
}

fn rigid_block(initial: VehicleState, gravity: Arc<dyn GravityField>) -> RigidBody6DofBlock {
    let spec = ComponentSpec {
        name: "core".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: VEHICLE_MASS_KG,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(500.0)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(290.0))]).unwrap();
    let config = RigidBodyConfig {
        body: BodyEllipsoid::WGS84,
        period: STEP,
        epoch: UtcTime::from_unix_seconds(EPOCH_UNIX_S),
        initial,
    };
    RigidBody6DofBlock::new(
        config,
        tree,
        Vec::new(),
        Box::new(VacuumAtmosphere),
        gravity,
    )
    .unwrap()
}

fn rotation_vector(q: DQuat) -> DVec3 {
    let short = if q.w < 0.0 { -q } else { q };
    let axis = short.xyz();
    let sine = axis.length();
    if sine == 0.0 {
        DVec3::ZERO
    } else {
        axis * (2.0 * sine.atan2(short.w) / sine)
    }
}

fn prime_thruster(initial: &VehicleState) -> RigidBodyInput {
    let omega = DVec3::new(0.0, 0.0, BodyEllipsoid::WGS84.omega_rad_s);
    let g_app = BodyEllipsoid::WGS84.gravity_fixed(initial.position_ecef)
        - omega.cross(omega.cross(initial.position_ecef));
    let f_body = initial.attitude.inverse() * (2.0 * omega.cross(initial.velocity_ecef) - g_app);
    RigidBodyInput {
        thrusters: vec![ThrusterWrench {
            application_point_body_m: DVec3::ZERO,
            force_body_n: VEHICLE_MASS_KG * f_body,
            moment_body_nm: DVec3::ZERO,
        }],
        ..RigidBodyInput::default()
    }
}

/// Closed-loop thruster & attitude command from the navigator's estimate alone.
fn closed_loop_command(
    solution: &NavigationSolution,
    sensed_rate_body: DVec3,
    target_speed_m_s: f64,
    target_heading_rad: f64,
    target_height_m: f64,
) -> RigidBodyInput {
    let est_ecef = solution_position_ecef(solution);
    let est_geo = ecef_to_geodetic(est_ecef);
    let frame = LocalFrame::at(&est_geo);
    let v = solution.velocity;
    let est_vel_ecef = frame.ned_to_ecef() * DVec3::new(v.north_mps, v.east_mps, v.down_mps);
    let att = solution.attitude.unwrap();
    let est_body_to_ned = DQuat::from_xyzw(att.x, att.y, att.z, att.w);
    let est_body_to_ecef = (DQuat::from_mat3(&frame.ned_to_ecef()) * est_body_to_ned).normalize();

    let (sin_h, cos_h) = target_heading_rad.sin_cos();
    let a_n = 1.5 * (target_speed_m_s * cos_h - v.north_mps);
    let a_e = 1.5 * (target_speed_m_s * sin_h - v.east_mps);
    let a_d = -(2.0 * (target_height_m - est_geo.height_m) + 2.0 * v.down_mps);
    let a_cmd_ecef = frame.ned_to_ecef() * DVec3::new(a_n, a_e, a_d);

    let omega = DVec3::new(0.0, 0.0, BodyEllipsoid::WGS84.omega_rad_s);
    let g_app_est =
        BodyEllipsoid::WGS84.gravity_fixed(est_ecef) - omega.cross(omega.cross(est_ecef));
    let f_cmd_body =
        est_body_to_ecef.inverse() * (a_cmd_ecef + 2.0 * omega.cross(est_vel_ecef) - g_app_est);

    let att_err =
        rotation_vector(est_body_to_ned.inverse() * DQuat::from_rotation_z(target_heading_rad));
    let moment_body_nm = 500.0 * (16.0 * att_err - 8.0 * sensed_rate_body);
    RigidBodyInput {
        thrusters: vec![ThrusterWrench {
            application_point_body_m: DVec3::ZERO,
            force_body_n: VEHICLE_MASS_KG * f_cmd_body,
            moment_body_nm,
        }],
        ..RigidBodyInput::default()
    }
}

fn j2_gradient(p: DVec3) -> DMat3 {
    let h = 25.0;
    let col = |axis: DVec3| {
        (BodyEllipsoid::WGS84.gravity_fixed(p + axis * h)
            - BodyEllipsoid::WGS84.gravity_fixed(p - axis * h))
            / (2.0 * h)
    };
    DMat3::from_cols(col(DVec3::X), col(DVec3::Y), col(DVec3::Z))
}

fn step_schuler_perturbation(
    dp: DVec3,
    dv: DVec3,
    p0: DVec3,
    p1: DVec3,
    dg0: DVec3,
    dg1: DVec3,
    dt_s: f64,
) -> (DVec3, DVec3) {
    let omega = DVec3::new(0.0, 0.0, BodyEllipsoid::WGS84.omega_rad_s);
    let accel = |p: DVec3, dg: DVec3, x: DVec3, v: DVec3| {
        dg + j2_gradient(p) * x - 2.0 * omega.cross(v) - omega.cross(omega.cross(x))
    };
    let a0 = accel(p0, dg0, dp, dv);
    let dp_mid = dp + dv * (0.5 * dt_s);
    let dv_mid = dv + a0 * (0.5 * dt_s);
    let a_mid = accel(0.5 * (p0 + p1), 0.5 * (dg0 + dg1), dp_mid, dv_mid);
    let dv_next = dv + a_mid * dt_s;
    let dp_next = dp + (dv + dv_next) * (0.5 * dt_s);
    (dp_next, dv_next)
}

fn solution_velocity_ecef(solution: &NavigationSolution) -> DVec3 {
    let pos = solution_position_ecef(solution);
    let frame = LocalFrame::at(&ecef_to_geodetic(pos));
    let v = solution.velocity;
    frame.ned_to_ecef() * DVec3::new(v.north_mps, v.east_mps, v.down_mps)
}

fn fly_closed_loop(
    start: &Geodetic,
    heading_rad: f64,
    speed_m_s: f64,
    seconds: f64,
    dynamics_gravity: Arc<dyn GravityField>,
    imu_gravity: Arc<dyn GravityField>,
) -> FlightSummary {
    let initial = VehicleState::level_flight(start, heading_rad, speed_m_s);
    let mut body = rigid_block(initial, Arc::clone(&dynamics_gravity));
    assert!(std::ptr::addr_eq(
        Arc::as_ptr(body.gravity()),
        Arc::as_ptr(&dynamics_gravity)
    ));
    let mut imu = SimulatedImu::new(ImuErrorSpec::IDEAL, 0, imu_gravity).unwrap();
    let mut navigator = InertialNavigator::new(
        inertial_config(&ImuErrorSpec::IDEAL),
        MasterClockBridge::default(),
        IMU_SOURCE,
    );
    let mut at = SimInstant::START;
    navigator
        .align(&InitialAlignment::EXACT.measure(&initial), at)
        .unwrap();
    assert!(imu.sense(at, &initial).unwrap().is_none());

    let disturbance =
        |p: DVec3| dynamics_gravity.attraction_fixed(p) - BodyEllipsoid::WGS84.gravity_fixed(p);
    let mut accum =
        DisturbanceAccum::new(initial.position_ecef, disturbance(initial.position_ecef));
    let mut max_sf = 0.0_f64;
    let mut sensed_rate = DVec3::ZERO;
    let mut last_solution: Option<NavigationSolution> = None;
    let dt_s = STEP.as_seconds();
    let steps = (seconds * STEP_HZ as f64).round() as usize;

    for _ in 0..steps {
        let input = last_solution.as_ref().map_or_else(
            || prime_thruster(&initial),
            |sol| closed_loop_command(sol, sensed_rate, speed_m_s, heading_rad, start.height_m),
        );
        body.step(&input).unwrap();
        max_sf = max_sf.max(body.specific_force_cg_body().length());
        at = at.advanced(STEP);
        let truth = body.output();
        let inc = imu.sense(at, &truth).unwrap().unwrap();
        sensed_rate = inc.delta_theta_rad / inc.dt_s;
        navigator.ingest(&inc, at).unwrap();
        last_solution = navigator.solution(at).unwrap();
        accum.update(truth.position_ecef, disturbance(truth.position_ecef), dt_s);
    }

    let truth = body.output();
    let sol = last_solution.unwrap();
    let score = compare_with_truth(&sol, truth.position_ecef).unwrap();
    FlightSummary {
        actual_dp: truth.position_ecef - solution_position_ecef(&sol),
        actual_dv: truth.velocity_ecef - solution_velocity_ecef(&sol),
        raw_dp: accum.raw_dp,
        raw_dv: accum.raw_dv,
        schuler_dp: accum.schuler_dp,
        schuler_dv: accum.schuler_dv,
        uniform_dp: 0.5 * accum.dg0 * seconds * seconds,
        max_disturbance_delta_m_s2: accum.max_delta,
        final_horizontal_error_m: score.horizontal_error_m,
        final_vertical_error_m: score.vertical_error_m,
        max_specific_force_m_s2: max_sf,
    }
}

fn corridor_gravity_overlay() -> AnomalyOverlay {
    let geometry = GridGeometry {
        south_deg: 38.90,
        west_deg: -75.70,
        latitude_step_deg: 0.005,
        longitude_step_deg: 0.005,
        rows: 81,
        columns: 101,
    };
    let origin = Geodetic::from_degrees(39.10, -75.48, 0.0);
    let origin_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&origin);
    let frame = LocalFrame::at(&origin);
    let mut values = Vec::with_capacity(geometry.rows * geometry.columns * 3);
    for row in 0..geometry.rows {
        let lat_deg = geometry.south_deg + row as f64 * geometry.latitude_step_deg;
        for col in 0..geometry.columns {
            let lon_deg = geometry.west_deg + col as f64 * geometry.longitude_step_deg;
            let p = BodyEllipsoid::WGS84
                .geodetic_to_fixed(&Geodetic::from_degrees(lat_deg, lon_deg, 0.0));
            let ned = frame.ecef_to_ned_vector(p - origin_ecef);
            let r2 = (ned.x * ned.x + ned.y * ned.y) / (7_000.0 * 7_000.0);
            let env = (-r2).exp();
            let gn = 35.0e-5 * env * (ned.x / 5_000.0).sin();
            let ge = -25.0e-5 * env * (ned.y / 5_000.0).cos();
            let gd = 45.0e-5 * env;
            values.extend_from_slice(&[gn, ge, gd]);
        }
    }
    AnomalyOverlay::new(
        geometry,
        values,
        &BodyEllipsoid::WGS84,
        OutsideCoverage::Zero,
    )
    .unwrap()
}

#[test]
fn egm96_degree_36_closed_loop_drift_matches_integrated_disturbance() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    // Fly 210 km across the Himalayan gravity gradient so EGM96 degree-36
    // disturbance varies strongly along the path (unlike a uniform vector).
    let start = Geodetic::from_degrees(26.90, 86.50, 10_000.0);
    let j2: Arc<dyn GravityField> =
        Arc::new(HarmonicGravity::reference(&BodyEllipsoid::WGS84).unwrap());
    let egm96 = HarmonicGravity::egm96_degree_36().unwrap();
    assert!(egm96.truncation().degree >= 20);
    let detailed: Arc<dyn GravityField> = Arc::new(egm96);

    let baseline = fly_closed_loop(&start, 0.0, 1_500.0, 140.0, Arc::clone(&j2), j2);
    assert!(baseline.final_horizontal_error_m < 0.05, "{baseline:?}");
    assert!(baseline.final_vertical_error_m < 0.05, "{baseline:?}");
    assert!(baseline.max_specific_force_m_s2 > 9.0, "{baseline:?}");

    let flown = fly_closed_loop(&start, 0.0, 1_500.0, 140.0, Arc::clone(&detailed), detailed);
    tracing::info!(
        horiz_m = flown.final_horizontal_error_m,
        vert_m = flown.final_vertical_error_m,
        actual_dp_m = flown.actual_dp.length(),
        raw_dp_m = flown.raw_dp.length(),
        schuler_dp_m = flown.schuler_dp.length(),
        uniform_dp_m = flown.uniform_dp.length(),
        max_dg_delta_mgal = flown.max_disturbance_delta_m_s2 * 1e5,
        "EGM96 degree-36 closed-loop drift"
    );
    assert!(flown.actual_dp.length() > 5.0, "{flown:?}");
    assert!(flown.max_disturbance_delta_m_s2 > 3.0e-4, "{flown:?}");
    let non_uniform_diff = (flown.raw_dp - flown.uniform_dp).length() / flown.raw_dp.length();
    assert!(
        non_uniform_diff > 0.20 && (flown.raw_dp - flown.uniform_dp).length() > 1.0,
        "diff {non_uniform_diff}: {flown:?}"
    );

    let raw_p_rel = (flown.actual_dp - flown.raw_dp).length() / flown.raw_dp.length();
    let raw_v_rel = (flown.actual_dv - flown.raw_dv).length() / flown.raw_dv.length();
    let schuler_p_rel = (flown.actual_dp - flown.schuler_dp).length() / flown.schuler_dp.length();
    let schuler_v_rel = (flown.actual_dv - flown.schuler_dv).length() / flown.schuler_dv.length();
    assert!(raw_p_rel < 0.05, "raw_p_rel={raw_p_rel} {flown:?}");
    assert!(raw_v_rel < 0.05, "raw_v_rel={raw_v_rel} {flown:?}");
    assert!(
        schuler_p_rel < 0.01,
        "schuler_p_rel={schuler_p_rel} {flown:?}"
    );
    assert!(
        schuler_v_rel < 0.01,
        "schuler_v_rel={schuler_v_rel} {flown:?}"
    );
}

#[test]
fn gravity_with_anomaly_overlay_adds_spatially_varying_drift_consistently() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let start = Geodetic::from_degrees(38.98, -75.60, 1_500.0);
    let egm96: Arc<dyn GravityField> = Arc::new(HarmonicGravity::egm96_degree_36().unwrap());
    let overlaid: Arc<dyn GravityField> = Arc::new(GravityWithAnomaly {
        base: Arc::clone(&egm96),
        overlays: vec![corridor_gravity_overlay()],
    });

    let heading = 0.72;
    let harmonic_only = fly_closed_loop(&start, heading, 400.0, 60.0, Arc::clone(&egm96), egm96);
    let with_overlay = fly_closed_loop(
        &start,
        heading,
        400.0,
        60.0,
        Arc::clone(&overlaid),
        overlaid,
    );

    let delta_actual_dp = with_overlay.actual_dp - harmonic_only.actual_dp;
    let delta_raw_dp = with_overlay.raw_dp - harmonic_only.raw_dp;
    let delta_schuler_dp = with_overlay.schuler_dp - harmonic_only.schuler_dp;
    tracing::info!(
        harmonic_dp_m = harmonic_only.actual_dp.length(),
        overlaid_dp_m = with_overlay.actual_dp.length(),
        delta_actual_dp_m = delta_actual_dp.length(),
        delta_raw_dp_m = delta_raw_dp.length(),
        "EGM96 + AnomalyOverlay closed-loop drift"
    );
    assert!(delta_actual_dp.length() > 0.2, "{delta_actual_dp:?}");
    assert!(
        (with_overlay.actual_dp - with_overlay.schuler_dp).length()
            / with_overlay.schuler_dp.length()
            < 0.01
    );
    assert!((delta_actual_dp - delta_raw_dp).length() / delta_raw_dp.length() < 0.05);
    assert!((delta_actual_dp - delta_schuler_dp).length() / delta_schuler_dp.length() < 0.01);
}

#[test]
fn ballistic_free_fall_and_mismatched_imu_control_verify_shared_gravity() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let start = Geodetic::from_degrees(39.00, -75.58, 25_000.0);
    let initial = VehicleState::level_flight(&start, 0.7, 350.0);
    let egm96: Arc<dyn GravityField> = Arc::new(HarmonicGravity::egm96_degree_36().unwrap());
    let overlaid: Arc<dyn GravityField> = Arc::new(GravityWithAnomaly {
        base: Arc::clone(&egm96),
        overlays: vec![corridor_gravity_overlay()],
    });
    let j2: Arc<dyn GravityField> =
        Arc::new(HarmonicGravity::reference(&BodyEllipsoid::WGS84).unwrap());

    let run_ballistic = |imu_field: Arc<dyn GravityField>| {
        let mut body = rigid_block(initial, Arc::clone(&overlaid));
        let mut imu = SimulatedImu::new(ImuErrorSpec::IDEAL, 0, imu_field).unwrap();
        let mut nav = InertialNavigator::new(
            inertial_config(&ImuErrorSpec::IDEAL),
            MasterClockBridge::default(),
            IMU_SOURCE,
        );
        let mut at = SimInstant::START;
        nav.align(&InitialAlignment::EXACT.measure(&initial), at)
            .unwrap();
        imu.sense(at, &initial).unwrap();
        let (mut raw_dv, mut raw_dp, mut max_sf) = (DVec3::ZERO, DVec3::ZERO, 0.0_f64);
        let mut dg_prev = overlaid.attraction_fixed(initial.position_ecef)
            - BodyEllipsoid::WGS84.gravity_fixed(initial.position_ecef);
        let dt_s = STEP.as_seconds();
        for _ in 0..(45 * STEP_HZ as usize) {
            body.step(&RigidBodyInput::default()).unwrap();
            at = at.advanced(STEP);
            let truth = body.output();
            let inc = imu.sense(at, &truth).unwrap().unwrap();
            max_sf = max_sf.max((inc.delta_velocity_m_s / inc.dt_s).length());
            nav.ingest(&inc, at).unwrap();
            let dg = overlaid.attraction_fixed(truth.position_ecef)
                - BodyEllipsoid::WGS84.gravity_fixed(truth.position_ecef);
            let dv_next = raw_dv + 0.5 * (dg_prev + dg) * dt_s;
            raw_dp += 0.5 * (raw_dv + dv_next) * dt_s;
            raw_dv = dv_next;
            dg_prev = dg;
        }
        let sol = nav.solution(at).unwrap().unwrap();
        (
            body.output().position_ecef - solution_position_ecef(&sol),
            raw_dp,
            max_sf,
        )
    };

    let (matched_dp, expected_dp, matched_sf) = run_ballistic(Arc::clone(&overlaid));
    let (mismatched_dp, _, mismatched_sf) = run_ballistic(j2);
    assert!(
        matched_sf < 1e-8,
        "free-fall should have zero specific force: {matched_sf}"
    );
    assert!(
        mismatched_sf > 1e-4,
        "mismatched IMU sees spurious gravity: {mismatched_sf}"
    );
    assert!(matched_dp.length() > 0.15, "{matched_dp:?}");
    assert!((matched_dp - expected_dp).length() / expected_dp.length() < 0.03);
    assert!(mismatched_dp.length() < 1e-2, "{mismatched_dp:?}");
}
