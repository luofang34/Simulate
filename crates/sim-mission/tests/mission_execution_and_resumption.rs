//! Verification suite for `sim-mission` execution contracts, independent `MissionExecutionState`,
//! cryptographic `MissionPlan::content_hash()` and `SimClockAdapter` time-anchor verification,
//! `pause_after_s` enforcement on both `FixedDuration` and `ScalarEvent` segments,
//! and explicit `ResumptionGuarantee` (`BitwiseIdenticalAtSegmentBoundary` vs `IntegratorToleranceConsistentMidSegment`).

use glam::{DQuat, DVec3};
use sim_mission::{
    BurnSteeringFrame, CrossingDirection, ExecutionLimits, MissionDesignError,
    MissionExecutionState, MissionLifecycleStatus, MissionPlan, MissionSegmentNode,
    OrbitalScalarEventKind, PlanBudgetScope, ResumptionGuarantee, SegmentBurnLaw,
    SegmentStoppingCondition, cancel_mission_execution, execute_mission_plan,
    resume_mission_execution, start_mission_execution,
};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, ContinuousPlantState, DerivativeOrder, Dop853Tolerances,
    EphemerisCatalog, EphemerisError, ForceModel, KinematicStateIcrf, MassPropertiesModel, NaifId,
    PropellantMode, ReactionWheelMode, SimClockAdapter, TdbEpoch, VesselCheckpoint, VesselConfig,
    VesselId,
};
use sim_spacetime::{DynamicalModelIdentity, SimInstant, TickSpan};

#[cfg(test)]
struct BoundedHorizonEphemeris<'a> {
    inner: &'a dyn EphemerisCatalog,
    cutoff_epoch: TdbEpoch,
}

#[cfg(test)]
impl EphemerisCatalog for BoundedHorizonEphemeris<'_> {
    fn content_hash(&self) -> sim_orbital::ContentHash {
        self.inner.content_hash()
    }

    fn coverage(
        &self,
        target: NaifId,
        center: NaifId,
        order: DerivativeOrder,
    ) -> Vec<sim_orbital::TdbSpanNs> {
        self.inner.coverage(target, center, order)
    }

    fn geometric_state_icrf(
        &self,
        target: NaifId,
        center: NaifId,
        epoch: TdbEpoch,
        order: DerivativeOrder,
    ) -> Result<KinematicStateIcrf, EphemerisError> {
        if epoch > self.cutoff_epoch {
            return Err(EphemerisError::OutOfCoverage {
                target,
                center,
                epoch_j2000_s: epoch.j2000_seconds() as f64,
            });
        }
        self.inner
            .geometric_state_icrf(target, center, epoch, order)
    }

    fn segment_knots_in(
        &self,
        target: NaifId,
        center: NaifId,
        start: TdbEpoch,
        end: TdbEpoch,
    ) -> Vec<TdbEpoch> {
        self.inner
            .segment_knots_in(target, center, start, end.min(self.cutoff_epoch))
    }
}

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
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_668.6, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 500.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    (plant, cfg)
}

#[cfg(test)]
fn build_leo_altitude_raising_plan(fm: &ForceModel) -> MissionPlan {
    MissionPlan {
        plan_id: 7,
        version: 3,
        vessel_id: VesselId(1),
        model_identity: DynamicalModelIdentity::SolarSystemEphemerisNBody {
            force_model_hash: fm.content_hash(),
        },
        segments: vec![
            MissionSegmentNode {
                segment_id: 10,
                label: "Prograde Burn".to_owned(),
                burn_law: SegmentBurnLaw::ConstantThrust {
                    thrust_n: 15_000.0,
                    exhaust_vel_m_s: 3_000.0,
                    steering: BurnSteeringFrame::InertialIcrfUnit { dir_icrf: DVec3::Y },
                },
                stopping: SegmentStoppingCondition::FixedDuration { duration_s: 20.0 },
            },
            MissionSegmentNode {
                segment_id: 20,
                label: "Coast to 405 km".to_owned(),
                burn_law: SegmentBurnLaw::Coast,
                stopping: SegmentStoppingCondition::ScalarEvent {
                    event: OrbitalScalarEventKind::RadiusCrossing {
                        body: NaifId::EARTH,
                        target_radius_m: 6_378_137.0 + 405_000.0,
                    },
                    direction: CrossingDirection::Rising,
                    max_duration_s: 1_800.0,
                    min_search_offset_s: 1.0,
                    time_tolerance_s: 1.0e-6,
                },
            },
        ],
        max_fuel_budget_kg: Some(250.0),
        delta_v_budget_m_s: Some(500.0),
        budget_scope: PlanBudgetScope::FullPlanCumulative,
    }
}

#[test]
fn plan_content_mutation_without_version_bump_rejected_on_resume() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let plan = build_leo_altitude_raising_plan(&fm);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);

    let paused = start_mission_execution(
        &plan,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits {
            max_accepted_steps: 200_000,
            pause_after_s: Some(20.0),
        },
    )
    .expect("pause at segment boundary");
    assert_eq!(paused.status, MissionLifecycleStatus::Paused);

    // Mutate segment 1 target radius while keeping (plan_id=7, version=3) unchanged:
    let mut mutated_plan = plan.clone();
    mutated_plan.segments.get_mut(1).expect("seg1").stopping =
        SegmentStoppingCondition::ScalarEvent {
            event: OrbitalScalarEventKind::RadiusCrossing {
                body: NaifId::EARTH,
                target_radius_m: 6_378_137.0 + 410_000.0,
            },
            direction: CrossingDirection::Rising,
            max_duration_s: 1_800.0,
            min_search_offset_s: 1.0,
            time_tolerance_s: 1.0e-6,
        };
    assert_eq!(mutated_plan.plan_id, plan.plan_id);
    assert_eq!(mutated_plan.version, plan.version);
    assert_ne!(mutated_plan.content_hash(), plan.content_hash());

    let err = resume_mission_execution(
        &mutated_plan,
        &paused.final_checkpoint,
        paused.execution_state,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits::default(),
    )
    .expect_err("must reject unversioned plan content mutation");
    assert!(matches!(
        err,
        MissionDesignError::PlanContentMutatedWithoutVersionBump {
            plan_id: 7,
            version: 3,
            ..
        }
    ));
}

#[test]
fn time_anchor_mutation_rejected_on_resume() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let plan = build_leo_altitude_raising_plan(&fm);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);

    let paused = start_mission_execution(
        &plan,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits {
            max_accepted_steps: 200_000,
            pause_after_s: Some(20.0),
        },
    )
    .expect("pause at segment boundary");

    let shifted_clock =
        SimClockAdapter::new(t0.add_span(TickSpan::from_seconds_and_subsec_ticks(60, 0)));
    let err = resume_mission_execution(
        &plan,
        &paused.final_checkpoint,
        paused.execution_state,
        &cfg,
        &shifted_clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits::default(),
    )
    .expect_err("must reject mutated time anchor on resume");
    assert!(matches!(err, MissionDesignError::TimeAnchorMismatch { .. }));
}

#[test]
fn scalar_event_segment_obeys_pause_after_s_and_resumes_consistently() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let plan = build_leo_altitude_raising_plan(&fm);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);

    let full_rep = execute_mission_plan(
        &plan,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("full uninterrupted execution");
    assert_eq!(full_rep.status, MissionLifecycleStatus::Completed);
    assert_eq!(
        full_rep.resumption_guarantee,
        ResumptionGuarantee::BitwiseIdenticalAtSegmentBoundary
    );

    // Pause at 35.0 s: 20 s burn completes (seg 0), and ScalarEvent coast (seg 1) pauses mid-segment after 15 s!
    let paused = start_mission_execution(
        &plan,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits {
            max_accepted_steps: 200_000,
            pause_after_s: Some(35.0),
        },
    )
    .expect("pause inside ScalarEvent segment");
    assert_eq!(paused.status, MissionLifecycleStatus::Paused);
    assert_eq!(paused.execution_state.active_segment_index, 1);
    assert!(paused.execution_state.segment_elapsed_ticks > 0);
    assert_eq!(
        paused.resumption_guarantee,
        ResumptionGuarantee::IntegratorToleranceConsistentMidSegment
    );

    let cp_json = serde_json::to_string(&paused.final_checkpoint).expect("ser cp");
    let st_json = serde_json::to_string(&paused.execution_state).expect("ser exec_state");
    let restored_cp: VesselCheckpoint = serde_json::from_str(&cp_json).expect("de cp");
    let restored_state: MissionExecutionState = serde_json::from_str(&st_json).expect("de state");

    let resumed = resume_mission_execution(
        &plan,
        &restored_cp,
        restored_state,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits::default(),
    )
    .expect("resume ScalarEvent segment to completion");
    assert_eq!(resumed.status, MissionLifecycleStatus::Completed);
    assert_eq!(
        resumed.resumption_guarantee,
        ResumptionGuarantee::IntegratorToleranceConsistentMidSegment
    );
    assert!(
        (resumed.final_checkpoint.plant.pos_rel_icrf_m
            - full_rep.final_checkpoint.plant.pos_rel_icrf_m)
            .length()
            < 1.0e-2
    );
}

#[test]
fn segment_boundary_pause_and_resume_is_bitwise_identical_to_unpaused_run() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let plan = build_leo_altitude_raising_plan(&fm);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);

    let full_rep = execute_mission_plan(
        &plan,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("uninterrupted execution");

    let boundary_paused = start_mission_execution(
        &plan,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits {
            max_accepted_steps: 200_000,
            pause_after_s: Some(20.0),
        },
    )
    .expect("pause at 20s segment boundary");
    assert_eq!(boundary_paused.status, MissionLifecycleStatus::Paused);
    assert_eq!(boundary_paused.execution_state.active_segment_index, 1);
    assert_eq!(boundary_paused.execution_state.segment_elapsed_ticks, 0);
    assert_eq!(
        boundary_paused.resumption_guarantee,
        ResumptionGuarantee::BitwiseIdenticalAtSegmentBoundary
    );

    let resumed = resume_mission_execution(
        &plan,
        &boundary_paused.final_checkpoint,
        boundary_paused.execution_state,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        ExecutionLimits::default(),
    )
    .expect("resume from segment boundary");
    assert_eq!(resumed.status, MissionLifecycleStatus::Completed);
    assert_eq!(
        resumed.resumption_guarantee,
        ResumptionGuarantee::BitwiseIdenticalAtSegmentBoundary
    );
    assert_eq!(
        resumed.final_checkpoint.checkpoint_hash(),
        full_rep.final_checkpoint.checkpoint_hash()
    );
}

#[test]
fn early_event_termination_and_lifecycle_cancellation_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let (init_plant, cfg) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);

    let cutoff = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(5, 0));
    let bounded_eph = BoundedHorizonEphemeris {
        inner: &solar.ephemeris,
        cutoff_epoch: cutoff,
    };
    let plan_60s = MissionPlan {
        plan_id: 99,
        version: 1,
        vessel_id: VesselId(1),
        model_identity: DynamicalModelIdentity::SolarSystemEphemerisNBody {
            force_model_hash: fm.content_hash(),
        },
        segments: vec![MissionSegmentNode {
            segment_id: 1,
            label: "Burn until 495 kg".to_owned(),
            burn_law: SegmentBurnLaw::ConstantThrust {
                thrust_n: 15_000.0,
                exhaust_vel_m_s: 3_000.0,
                steering: BurnSteeringFrame::InertialIcrfUnit { dir_icrf: DVec3::Y },
            },
            stopping: SegmentStoppingCondition::ScalarEvent {
                event: OrbitalScalarEventKind::RemainingPropellantCrossing {
                    target_propellant_kg: 495.0,
                },
                direction: CrossingDirection::Falling,
                max_duration_s: 60.0,
                min_search_offset_s: 0.1,
                time_tolerance_s: 1.0e-6,
            },
        }],
        max_fuel_budget_kg: Some(100.0),
        delta_v_budget_m_s: None,
        budget_scope: PlanBudgetScope::FullPlanCumulative,
    };
    let rep = execute_mission_plan(
        &plan_60s,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &bounded_eph,
        &solar.orientation,
    )
    .expect("stops at 1.0s event before 5.0s ephemeris cutoff");
    let end_ep = clock.instant_to_tdb(rep.final_checkpoint.instant);
    assert!((end_ep.signed_offset_seconds_from(t0) - 1.0).abs() < 1.0e-4);

    // Lifecycle cancellation: cannot cancel Completed, can cancel Paused, and cannot resume Cancelled:
    let mut completed_state = rep.execution_state;
    assert!(matches!(
        cancel_mission_execution(&mut completed_state),
        Err(MissionDesignError::InvalidLifecycleTransition {
            status: MissionLifecycleStatus::Completed,
            ..
        })
    ));
}

#[test]
fn vnb_steering_origin_conversion_failure_fails_closed_without_silent_fallback() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let clock = SimClockAdapter::new(TdbEpoch::new(835_000_000, 0));
    let (init_plant, cfg) = sample_leo_plant_and_config(PropellantMode::FiniteVariableMass);
    let cp0 = VesselCheckpoint::new(VesselId(1), SimInstant::START, init_plant, 1);
    let mut bad_vnb_plan = build_leo_altitude_raising_plan(&fm);
    bad_vnb_plan.segments.get_mut(0).expect("seg0").burn_law = SegmentBurnLaw::ConstantThrust {
        thrust_n: 15_000.0,
        exhaust_vel_m_s: 3_000.0,
        steering: BurnSteeringFrame::LocalVnbUnit {
            primary: NaifId(999_999),
            dir_vnb: DVec3::X,
        },
    };
    let err = execute_mission_plan(
        &bad_vnb_plan,
        &cp0,
        &cfg,
        &clock,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect_err("LocalVnbUnit with missing ephemeris body must fail closed");
    assert!(matches!(err, MissionDesignError::Ephemeris(_)));
}
