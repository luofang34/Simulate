//! WP 5 integration tests:
//! 1. End-to-end GMAT-style `Initial State -> Finite Burn -> Propagate to Event -> Targeter Solve -> Ground Station Access Analysis`
//!    workflow (`evaluate_orbital_maneuver_and_access`) with candidate comparison and chain integrity verification.
//! 2. Multi-variable / multi-goal scaled differential corrector (`solve_multivariable_differential_correction`)
//!    with variable scaling, box bounds, and central-difference Jacobian sensitivity diagnostics.
//! 3. Assisted $L_2$ Planner (`plan_assisted_l2_bounded_arc`) verifying seed generation, finite-horizon
//!    targeted arc, sensitivity $\partial x(t_f) / \partial \dot{y}_0$, and explicit periodic-closure check
//!    preventing a finite-horizon arc from being misclassified as a closed periodic Halo orbit.

use glam::{DQuat, DVec3};
use sim_analysis::{
    DesignRevisionIdentity, GroundStationModel, OccultationEnvironmentModel, VehicleSensorModel,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_mission::{
    ApoapsisTargetProblem, BurnSteeringFrame, CrossingDirection, MissionPlan, MissionSegmentNode,
    OrbitalScalarEventKind, PlanBudgetScope, ScaledControlVariable, ScaledEqualityGoal,
    SegmentBurnLaw, SegmentStoppingCondition, SolverConvergenceStatus, TargetGoalSpec,
    TargetVariableSpec, evaluate_orbital_maneuver_and_access, plan_assisted_l2_bounded_arc,
    solve_multivariable_differential_correction,
};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContinuousPlantState, Dop853Tolerances,
    DynamicalMode, ForceModel, L2BoundedArcTargetSpec, L2OrbitRegimeKind, L2SingleShootingStatus,
    MassPropertiesModel, NaifId, PropellantMode, ReactionWheelMode, SimClockAdapter, TdbEpoch,
    VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{DynamicalModelIdentity, SimInstant};

#[cfg(test)]
fn sample_leo_plant_and_config() -> (ContinuousPlantState, VesselConfig) {
    let cfg = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(2_000.0, [4_000.0, 2_000.0, 4_000.0], [0.0; 3]),
        propellant_mode: PropellantMode::FiniteVariableMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 5.0e6,
            tau_max_nm: 5.0e4,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 3.0,
        auto_soi_switch: false,
    };
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_685.0, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 500.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    (plant, cfg)
}

#[cfg(test)]
fn build_apoapsis_problem(fm: &ForceModel, target_alt_m: f64) -> ApoapsisTargetProblem {
    let base_plan = MissionPlan {
        plan_id: 77,
        version: 1,
        vessel_id: VesselId(1),
        model_identity: DynamicalModelIdentity::SolarSystemEphemerisNBody {
            force_model_hash: fm.content_hash(),
        },
        segments: vec![
            MissionSegmentNode {
                segment_id: 1,
                label: "Prograde Apoapsis Raising Burn".to_owned(),
                burn_law: SegmentBurnLaw::ConstantThrust {
                    thrust_n: 15_000.0,
                    exhaust_vel_m_s: 3_000.0,
                    steering: BurnSteeringFrame::InertialIcrfUnit { dir_icrf: DVec3::Y },
                },
                stopping: SegmentStoppingCondition::FixedDuration { duration_s: 15.0 },
            },
            MissionSegmentNode {
                segment_id: 2,
                label: "Coast to Apoapsis".to_owned(),
                burn_law: SegmentBurnLaw::Coast,
                stopping: SegmentStoppingCondition::ScalarEvent {
                    event: OrbitalScalarEventKind::ApoapsisPassage {
                        body: NaifId::EARTH,
                    },
                    direction: CrossingDirection::Falling,
                    max_duration_s: 5_000.0,
                    min_search_offset_s: 60.0,
                    time_tolerance_s: 1.0e-6,
                },
            },
        ],
        max_fuel_budget_kg: Some(250.0),
        delta_v_budget_m_s: None,
        budget_scope: PlanBudgetScope::FullPlanCumulative,
    };
    ApoapsisTargetProblem {
        base_plan,
        vary: TargetVariableSpec::VarySegmentBurnDuration {
            segment_id: 1,
            min_burn_duration_s: 1.0,
            max_burn_duration_s: 45.0,
            fd_perturbation_s: 0.25,
        },
        goal: TargetGoalSpec::TerminalRadius {
            body: NaifId::EARTH,
            target_radius_m: 6_378_137.0 + target_alt_m,
            tolerance_m: 5.0,
        },
        request_generation: 1,
        max_iterations: 10,
    }
}

#[test]
fn wp5_phase1_end_to_end_maneuver_targeting_and_station_access_candidate_comparison() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg) = sample_leo_plant_and_config();
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);
    let rot0 = solar
        .orientation
        .body_rotation(NaifId::EARTH, t0)
        .expect("earth rot");
    let sub_sat_geo =
        BodyEllipsoid::WGS84.fixed_to_geodetic(rot0.r_icrf_to_fixed * init_plant.pos_rel_icrf_m);
    let station = GroundStationModel::earth_wgs84(
        101,
        "Downrange Tracking Station",
        Geodetic {
            lat_rad: sub_sat_geo.lat_rad,
            lon_rad: sub_sat_geo.lon_rad + 0.35,
            height_m: 120.0,
        },
        5.0_f64.to_radians(),
    );
    let omni_sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni S-Band");
    let occ_env = OccultationEnvironmentModel::earth_wgs84_and_moon();
    let run_candidate = |id: &str, rev_num: u64, digest: &str, alt_m: f64| {
        let rev = DesignRevisionIdentity {
            design_id: "leo-apoapsis-raise".to_owned(),
            revision_number: rev_num,
            design_digest_sha256: digest.to_owned(),
        };
        let prob = build_apoapsis_problem(&fm, alt_m);
        let art = evaluate_orbital_maneuver_and_access(
            id,
            rev.clone(),
            &prob,
            &cp0,
            &cfg,
            &clock,
            std::slice::from_ref(&station),
            &omni_sensor,
            &occ_env,
            15.0,
            1.0e-3,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("candidate workflow");
        (rev, prob, art)
    };
    let (rev1, prob_800, cand_800) = run_candidate("cand-800km", 1, "rev1-800km", 800_000.0);
    assert!(cand_800.verify_chain_integrity(&rev1, &prob_800.base_plan, &cp0, &clock));
    assert!(cand_800.metrics.target_residual_m <= 5.0);
    assert!(cand_800.metrics.propellant_consumed_kg > 10.0);
    assert!(cand_800.metrics.station_access_window_count >= 1);

    let (rev2, prob_950, cand_950) = run_candidate("cand-950km", 2, "rev2-950km", 950_000.0);
    assert!(cand_950.verify_chain_integrity(&rev2, &prob_950.base_plan, &cp0, &clock));
    assert!(!cand_950.verify_chain_integrity(&rev1, &prob_950.base_plan, &cp0, &clock));
    assert!(cand_950.metrics.propellant_consumed_kg > cand_800.metrics.propellant_consumed_kg);
    assert!(cand_950.metrics.mission_duration_s > cand_800.metrics.mission_duration_s);
    assert_ne!(
        cand_800.metrics.artifact_chain_digest,
        cand_950.metrics.artifact_chain_digest
    );
}

#[test]
fn wp5_phase2_multivariable_differential_corrector_scaling_bounds_and_diagnostics() {
    let variables = vec![
        ScaledControlVariable {
            name: "burn_duration_s".to_owned(),
            initial_value: 15.0,
            scale: 20.0,
            perturbation_step: 0.05,
            min_value: 2.0,
            max_value: 60.0,
            max_step: 15.0,
        },
        ScaledControlVariable {
            name: "pitch_offset_rad".to_owned(),
            initial_value: 0.12,
            scale: 0.1,
            perturbation_step: 1.0e-4,
            min_value: -0.5,
            max_value: 0.5,
            max_step: 0.2,
        },
    ];
    let goals = vec![
        ScaledEqualityGoal {
            name: "apoapsis_radius_m".to_owned(),
            target_value: 7_200_000.0,
            scale: 100_000.0,
            tolerance: 1.0,
        },
        ScaledEqualityGoal {
            name: "flight_path_angle_rad".to_owned(),
            target_value: 0.0,
            scale: 0.01,
            tolerance: 1.0e-6,
        },
    ];

    let report = solve_multivariable_differential_correction(&variables, &goals, 12, 1.0e-6, |u| {
        let dt = u.first().copied().unwrap_or(0.0);
        let pitch = u.get(1).copied().unwrap_or(0.0);
        let r_a = 6_778_137.0 + 16_500.0 * dt * pitch.cos() + 120.0 * dt * dt;
        let fpa = pitch - 0.0015 * (dt - 24.0);
        Ok(vec![r_a, fpa])
    })
    .expect("2x2 differential correction");

    assert_eq!(report.status, SolverConvergenceStatus::Converged);
    assert!(report.iterations >= 1 && report.iterations <= 8);
    assert!((report.final_achieved_goals[0] - 7_200_000.0).abs() <= 1.0);
    assert!(report.final_achieved_goals[1].abs() <= 1.0e-6);
    assert!(!report.history.is_empty());
    assert_eq!(report.final_jacobian_row_major.len(), 4);
}

#[test]
fn wp5_phase2_assisted_l2_planner_distinguishes_finite_horizon_arc_from_periodic_halo() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::new(
        solar.constants.clone(),
        vec![NaifId::SUN, NaifId::EARTH, NaifId::MOON],
        Vec::new(),
        DynamicalMode::PrescribedEphemeris,
        0,
    )
    .expect("force model");
    let t0 = TdbEpoch::new(840_000_000, 0);
    let spec = L2BoundedArcTargetSpec::sun_earth_southern(t0, 14.0 * 86_400.0, 1.2e8);

    let report = plan_assisted_l2_bounded_arc(
        &spec,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        1_000.0,
        0.1,
    )
    .expect("assisted L2 plan");

    assert_eq!(
        report.targeted_arc.status,
        L2SingleShootingStatus::Converged
    );
    assert_eq!(
        report.targeted_arc.regime,
        L2OrbitRegimeKind::FiniteHorizonTargetedArc
    );
    assert!(report.targeted_arc.terminal_x_residual_m.abs() <= spec.tolerance_x_m);
    assert!(report.sensitivity.dx_terminal_dvy0_s.is_finite());
    assert!(report.sensitivity.dx_terminal_dvy0_s.abs() > 1.0);

    // Crucial WP 5 requirement: a 14-day finite-horizon targeted arc does NOT close on itself in 6D
    // synodic space, so periodic_closure MUST report satisfies_periodic_closure == false and
    // certified_regime == FiniteHorizonTargetedArc (never PeriodicHaloOrbitCr3bp).
    assert!(!report.periodic_closure.satisfies_periodic_closure);
    assert_eq!(
        report.periodic_closure.certified_regime,
        L2OrbitRegimeKind::FiniteHorizonTargetedArc
    );
    assert!(report.periodic_closure.synodic_position_closure_residual_m > 1.0e6);
}
