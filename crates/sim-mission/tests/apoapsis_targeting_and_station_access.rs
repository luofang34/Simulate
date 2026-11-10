//! Verification suite for `solve_apoapsis_target` differential-correction targeting,
//! full `SolverRequestIdentity` commit validation (`VersionedSolverResult::verify_applicable`),
//! solver failure taxonomy (`SolverFailureReason`), and direct interoperability of
//! `MissionExecutionReport::trajectory_product` with `sim_analysis::evaluate_station_access`.

use glam::{DQuat, DVec3};
use sim_analysis::{
    AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, VehicleSensorModel,
    evaluate_station_access,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_mission::{
    ApoapsisTargetProblem, BurnSteeringFrame, CrossingDirection, MissionDesignError, MissionPlan,
    MissionSegmentNode, OrbitalScalarEventKind, PlanBudgetScope, SegmentBurnLaw,
    SegmentStoppingCondition, SolverConvergenceStatus, SolverFailureReason, TargetGoalSpec,
    TargetVariableSpec, execute_mission_plan, solve_apoapsis_target,
};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContinuousPlantState, Dop853Tolerances,
    ForceModel, MassPropertiesModel, NaifId, PropellantMode, ReactionWheelMode, SimClockAdapter,
    TdbEpoch, VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{DynamicalModelIdentity, SimInstant, TickSpan, TranslationalTrajectorySource};

#[cfg(test)]
fn sample_leo_plant_and_config(
    propellant_mode: PropellantMode,
) -> (ContinuousPlantState, VesselConfig) {
    let cfg = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(2_000.0, [4_000.0, 2_000.0, 4_000.0], [0.0; 3]),
        propellant_mode,
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
fn sample_apoapsis_problem(fm: &ForceModel) -> ApoapsisTargetProblem {
    let base_plan = MissionPlan {
        plan_id: 42,
        version: 5,
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
            target_radius_m: 6_378_137.0 + 850_000.0,
            tolerance_m: 5.0,
        },
        request_generation: 11,
        max_iterations: 10,
    }
}

#[test]
fn apoapsis_targeter_convergence_and_full_request_identity_verification() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg_fin) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);

    let problem = sample_apoapsis_problem(&fm);
    let res = solve_apoapsis_target(
        &problem,
        &cp0,
        &cfg_fin,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("targeter converges");
    assert_eq!(res.status, SolverConvergenceStatus::Converged);
    assert!(res.residual_m.abs() <= 5.0);
    assert!(res.solved_burn_duration_s > 15.0 && res.solved_burn_duration_s < 35.0);

    res.verify_applicable(&res.request_identity, &problem.base_plan, &cp0, &clock)
        .expect("applicable");

    let mut mutated_plan = problem.base_plan.clone();
    mutated_plan.max_fuel_budget_kg = Some(249.0);
    assert!(matches!(
        res.verify_applicable(&res.request_identity, &mutated_plan, &cp0, &clock),
        Err(MissionDesignError::PlanContentMutatedWithoutVersionBump { .. })
    ));

    let shifted_clock =
        SimClockAdapter::new(t0.add_span(TickSpan::from_seconds_and_subsec_ticks(10, 0)));
    assert!(matches!(
        res.verify_applicable(
            &res.request_identity,
            &problem.base_plan,
            &cp0,
            &shifted_clock
        ),
        Err(MissionDesignError::TimeAnchorMismatch { .. })
    ));
    let mut mutated_cp = cp0.clone();
    mutated_cp.plant.propellant_mass_kg -= 1.0;
    assert!(matches!(
        res.verify_applicable(
            &res.request_identity,
            &problem.base_plan,
            &mutated_cp,
            &clock
        ),
        Err(MissionDesignError::InitialCheckpointMismatch { .. })
    ));
    let mut stale_req = res.request_identity;
    stale_req.request_generation = 12;
    assert!(matches!(
        res.verify_applicable(&stale_req, &problem.base_plan, &cp0, &clock),
        Err(MissionDesignError::IdentityFieldMismatch {
            field: "request_generation"
        })
    ));
    let exec_rep = execute_mission_plan(
        &res.updated_plan,
        &cp0,
        &cfg_fin,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("execute updated plan");
    assert_eq!(
        exec_rep.final_checkpoint.checkpoint_hash(),
        res.final_execution.final_checkpoint.checkpoint_hash()
    );
}

#[test]
fn apoapsis_targeter_rejects_infinite_mass_clamping_and_fuel_exhaustion() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg_inf) = sample_leo_plant_and_config(PropellantMode::InfiniteConstantMass);
    let (_, cfg_fin) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);
    let base_prob = sample_apoapsis_problem(&fm);

    assert_eq!(
        solve_apoapsis_target(
            &base_prob,
            &cp0,
            &cfg_inf,
            &clock,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        ),
        Err(MissionDesignError::RequiresFiniteVariableMass)
    );

    let mut oob_prob = base_prob.clone();
    oob_prob.vary = TargetVariableSpec::VarySegmentBurnDuration {
        segment_id: 1,
        min_burn_duration_s: 1.0,
        max_burn_duration_s: 16.0,
        fd_perturbation_s: 0.25,
    };
    assert!(matches!(
        solve_apoapsis_target(
            &oob_prob,
            &cp0,
            &cfg_fin,
            &clock,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        ),
        Err(MissionDesignError::TargeterDidNotConverge {
            reason: SolverFailureReason::ClampedAtBoundsWithoutConvergence,
            ..
        })
    ));

    let mut fuel_prob = base_prob;
    fuel_prob.base_plan.max_fuel_budget_kg = Some(40.0);
    assert!(matches!(
        solve_apoapsis_target(
            &fuel_prob,
            &cp0,
            &cfg_fin,
            &clock,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        ),
        Err(MissionDesignError::TargeterDidNotConverge {
            reason: SolverFailureReason::FuelExhausted,
            ..
        })
    ));
}

#[test]
fn apoapsis_targeter_reports_singular_sensitivity_and_max_iterations() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg_fin) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);
    let base_prob = sample_apoapsis_problem(&fm);

    let mut sing_prob = base_prob.clone();
    sing_prob
        .base_plan
        .segments
        .get_mut(0)
        .expect("seg0")
        .burn_law = SegmentBurnLaw::ConstantThrust {
        thrust_n: 0.0,
        exhaust_vel_m_s: 3_000.0,
        steering: BurnSteeringFrame::InertialIcrfUnit { dir_icrf: DVec3::Y },
    };
    assert!(matches!(
        solve_apoapsis_target(
            &sing_prob,
            &cp0,
            &cfg_fin,
            &clock,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        ),
        Err(MissionDesignError::TargeterDidNotConverge {
            reason: SolverFailureReason::SingularSensitivity,
            ..
        })
    ));

    let mut max_iter_prob = base_prob;
    max_iter_prob.max_iterations = 1;
    assert!(matches!(
        solve_apoapsis_target(
            &max_iter_prob,
            &cp0,
            &cfg_fin,
            &clock,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        ),
        Err(MissionDesignError::TargeterDidNotConverge {
            reason: SolverFailureReason::MaxIterationsExceeded,
            iterations: 1,
            ..
        })
    ));
}

#[test]
fn mission_trajectory_product_plugs_directly_into_sim_analysis_station_access() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg_fin) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);

    let problem = sample_apoapsis_problem(&fm);
    let res = solve_apoapsis_target(
        &problem,
        &cp0,
        &cfg_fin,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("converges");
    let traj = &res.final_execution.trajectory_product;

    let rot0 = solar
        .orientation
        .body_rotation(NaifId::EARTH, t0)
        .expect("rot0");
    let (pos_fixed_0, _) = rot0.icrf_to_fixed_pv(init_plant.pos_rel_icrf_m, DVec3::ZERO);
    let sub_sat_geo = BodyEllipsoid::WGS84.fixed_to_geodetic(pos_fixed_0);
    let station = GroundStationModel::earth_wgs84(
        1,
        "Sub-Satellite Tracking Station",
        Geodetic {
            lat_rad: sub_sat_geo.lat_rad,
            lon_rad: sub_sat_geo.lon_rad,
            height_m: 0.0,
        },
        10.0_f64.to_radians(),
    );
    let sensor = VehicleSensorModel::omnidirectional_at_cg(1, "Omni S-Band");
    let env = OccultationEnvironmentModel::earth_wgs84_and_moon();
    let query = AccessQuerySpec {
        query_interval: TranslationalTrajectorySource::valid_interval(traj),
        sample_step_s: 60.0,
        boundary_time_tolerance_s: 1.0e-3,
        min_range_m: None,
        max_range_m: Some(2_500_000.0),
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };
    let access = evaluate_station_access(
        traj,
        Some(traj),
        &station,
        &sensor,
        &env,
        &query,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("access report");
    assert!(!access.windows.is_empty());
    assert!(access.samples.first().expect("s0").accessible);
}

fn assert_binding_mismatch(
    tampered: &sim_mission::VersionedSolverResult,
    baseline: &sim_mission::VersionedSolverResult,
    problem: &ApoapsisTargetProblem,
    cp0: &VesselCheckpoint,
    clock: &SimClockAdapter,
    expected_field: &'static str,
) {
    assert!(matches!(
        tampered.verify_applicable(&baseline.request_identity, &problem.base_plan, cp0, clock),
        Err(MissionDesignError::SolverArtifactBindingMismatch { field, .. }) if field == expected_field
    ));
}

#[test]
fn solver_result_artifact_tampering_and_unconverged_status_are_rejected() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let clock = SimClockAdapter::new(TdbEpoch::new(835_000_000, 0));
    let (init_plant, cfg_fin) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);
    let problem = sample_apoapsis_problem(&fm);
    let res = solve_apoapsis_target(
        &problem,
        &cp0,
        &cfg_fin,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("converges");

    let mut tampered_plan_res = res.clone();
    tampered_plan_res
        .updated_plan
        .segments
        .get_mut(0)
        .expect("seg0")
        .stopping = SegmentStoppingCondition::FixedDuration { duration_s: 12.0 };
    assert_binding_mismatch(
        &tampered_plan_res,
        &res,
        &problem,
        &cp0,
        &clock,
        "updated_plan",
    );

    let mut tampered_ckpt_res = res.clone();
    tampered_ckpt_res
        .final_execution
        .final_checkpoint
        .plant
        .propellant_mass_kg -= 0.5;
    assert_binding_mismatch(
        &tampered_ckpt_res,
        &res,
        &problem,
        &cp0,
        &clock,
        "final_execution.final_checkpoint",
    );

    let mut tampered_residual_res = res.clone();
    tampered_residual_res.residual_m = 500.0;
    assert_binding_mismatch(
        &tampered_residual_res,
        &res,
        &problem,
        &cp0,
        &clock,
        "residual_m",
    );

    let mut unconverged_res = res;
    unconverged_res.status =
        SolverConvergenceStatus::Failed(SolverFailureReason::MaxIterationsExceeded);
    let mut plan_to_commit = problem.base_plan.clone();
    assert!(matches!(
        unconverged_res.commit_to_plan(
            &unconverged_res.request_identity,
            &mut plan_to_commit,
            &cp0,
            &clock
        ),
        Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "status",
            ..
        })
    ));
    assert_eq!(plan_to_commit.version, problem.base_plan.version);
}
