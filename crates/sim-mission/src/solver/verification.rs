//! Cryptographic and structural verification helpers for [`VersionedSolverResult`](super::VersionedSolverResult).

use sim_orbital::{
    ContentHash, DomainHasher, OrbitalTrajectoryProduct, SimClockAdapter, VesselCheckpoint,
};

use crate::{
    error::{MissionDesignError, MissionLifecycleStatus},
    plan::{MissionPlan, SegmentStoppingCondition},
    solver::{
        SolverConvergenceStatus, SolverRequestIdentity, TargetGoalSpec, TargetVariableSpec,
        VersionedSolverResult,
    },
};

#[allow(clippy::too_many_arguments)]
pub(super) fn compute_conclusion_binding_hash(
    id: &SolverRequestIdentity,
    status: SolverConvergenceStatus,
    iterations: u32,
    solved_burn_duration_s: f64,
    achieved_radius_m: f64,
    residual_m: f64,
    updated_plan_hash: ContentHash,
    final_checkpoint_hash: ContentHash,
    trajectory_hash: ContentHash,
) -> ContentHash {
    let mut h = DomainHasher::new(b"sim-mission:versioned-solver-result:v1");
    h.write_u64(id.target_plan_id);
    h.write_u64(id.base_plan_version);
    h.write_hash(id.base_plan_content_hash);
    h.write_hash(id.initial_checkpoint_hash);
    h.write_hash(id.vary_spec_hash);
    h.write_hash(id.goal_spec_hash);
    h.write_hash(id.input_fingerprint.combined_hash);
    h.write_u64(id.request_generation);
    h.write_u8(match status {
        SolverConvergenceStatus::Converged => 1,
        SolverConvergenceStatus::Failed(_) => 2,
    });
    h.write_u32(iterations);
    h.write_f64_canonical(solved_burn_duration_s);
    h.write_f64_canonical(achieved_radius_m);
    h.write_f64_canonical(residual_m);
    h.write_hash(updated_plan_hash);
    h.write_hash(final_checkpoint_hash);
    h.write_hash(trajectory_hash);
    h.finish()
}

pub(super) fn verify_request_and_context(
    res: &VersionedSolverResult,
    expected_request: &SolverRequestIdentity,
    current_plan: &MissionPlan,
    current_checkpoint: &VesselCheckpoint,
    current_clock: &SimClockAdapter,
) -> Result<(), MissionDesignError> {
    let id = &res.request_identity;
    if current_plan.plan_id != id.target_plan_id || current_plan.version != id.base_plan_version {
        return Err(MissionDesignError::PlanVersionMismatch {
            expected_plan_id: id.target_plan_id,
            expected_version: id.base_plan_version,
            actual_plan_id: current_plan.plan_id,
            actual_version: current_plan.version,
        });
    }
    let actual_plan_hash = current_plan.content_hash();
    if actual_plan_hash != id.base_plan_content_hash {
        return Err(MissionDesignError::PlanContentMutatedWithoutVersionBump {
            plan_id: current_plan.plan_id,
            version: current_plan.version,
            expected_hash: id.base_plan_content_hash,
            actual_hash: actual_plan_hash,
        });
    }
    if current_clock.anchor_tdb() != id.anchor_tdb {
        return Err(MissionDesignError::TimeAnchorMismatch {
            expected_anchor: id.anchor_tdb,
            actual_anchor: current_clock.anchor_tdb(),
        });
    }
    if current_checkpoint.instant != id.initial_instant {
        return Err(MissionDesignError::InitialInstantMismatch {
            expected_instant: id.initial_instant,
            actual_instant: current_checkpoint.instant,
        });
    }
    let actual_ckpt_hash = current_checkpoint.checkpoint_hash();
    if actual_ckpt_hash != id.initial_checkpoint_hash {
        return Err(MissionDesignError::InitialCheckpointMismatch {
            expected_hash: id.initial_checkpoint_hash,
            actual_hash: actual_ckpt_hash,
        });
    }
    verify_expected_request_fields(id, expected_request)
}

fn verify_expected_request_fields(
    id: &SolverRequestIdentity,
    expected: &SolverRequestIdentity,
) -> Result<(), MissionDesignError> {
    if id.target_plan_id != expected.target_plan_id
        || id.base_plan_version != expected.base_plan_version
    {
        return Err(MissionDesignError::PlanVersionMismatch {
            expected_plan_id: expected.target_plan_id,
            expected_version: expected.base_plan_version,
            actual_plan_id: id.target_plan_id,
            actual_version: id.base_plan_version,
        });
    }
    if id.base_plan_content_hash != expected.base_plan_content_hash {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "base_plan_content_hash",
        });
    }
    if id.initial_checkpoint_hash != expected.initial_checkpoint_hash {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "initial_checkpoint_hash",
        });
    }
    if id.anchor_tdb != expected.anchor_tdb {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "anchor_tdb",
        });
    }
    if id.initial_instant != expected.initial_instant {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "initial_instant",
        });
    }
    if id.request_generation != expected.request_generation {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "request_generation",
        });
    }
    if id.vary_spec_hash != expected.vary_spec_hash {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "vary_spec_hash",
        });
    }
    if id.goal_spec_hash != expected.goal_spec_hash {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "goal_spec_hash",
        });
    }
    if id.input_fingerprint != expected.input_fingerprint {
        return Err(MissionDesignError::IdentityFieldMismatch {
            field: "input_fingerprint",
        });
    }
    Ok(())
}

pub(super) fn verify_conclusion_and_plan_binding(
    res: &VersionedSolverResult,
    current_plan: &MissionPlan,
) -> Result<(), MissionDesignError> {
    let segment_id = verify_convergence_metrics(res)?;
    let mut expected_plan = current_plan.clone();
    expected_plan.version = current_plan.version.wrapping_add(1);
    let Some(seg) = expected_plan
        .segments
        .iter_mut()
        .find(|s| s.segment_id == segment_id)
    else {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "updated_plan",
            reason: format!("target segment {segment_id} missing in plan"),
        });
    };
    seg.stopping = SegmentStoppingCondition::FixedDuration {
        duration_s: res.solved_burn_duration_s,
    };
    let actual_updated_hash = res.updated_plan.content_hash();
    if res.updated_plan != expected_plan
        || actual_updated_hash != expected_plan.content_hash()
        || actual_updated_hash != res.verified_updated_plan_hash
    {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "updated_plan",
            reason: "updated_plan does not match solved decision variable and base plan".to_owned(),
        });
    }
    Ok(())
}

fn verify_convergence_metrics(res: &VersionedSolverResult) -> Result<u32, MissionDesignError> {
    let id = &res.request_identity;
    if res.status != SolverConvergenceStatus::Converged {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "status",
            reason: format!("cannot apply unconverged solver status {:?}", res.status),
        });
    }
    if res.vary_spec.content_hash() != id.vary_spec_hash {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "vary_spec",
            reason: "vary_spec does not match request_identity.vary_spec_hash".to_owned(),
        });
    }
    if res.goal_spec.content_hash() != id.goal_spec_hash {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "goal_spec",
            reason: "goal_spec does not match request_identity.goal_spec_hash".to_owned(),
        });
    }
    let TargetVariableSpec::VarySegmentBurnDuration {
        segment_id,
        min_burn_duration_s,
        max_burn_duration_s,
        ..
    } = res.vary_spec;
    if !res.solved_burn_duration_s.is_finite()
        || res.solved_burn_duration_s < min_burn_duration_s
        || res.solved_burn_duration_s > max_burn_duration_s
    {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "solved_burn_duration_s",
            reason: format!(
                "solved_burn_duration_s ({}) is outside [{min_burn_duration_s}, {max_burn_duration_s}]",
                res.solved_burn_duration_s
            ),
        });
    }
    let TargetGoalSpec::TerminalRadius {
        target_radius_m,
        tolerance_m,
        ..
    } = res.goal_spec;
    let expected_res = res.achieved_radius_m - target_radius_m;
    if !res.achieved_radius_m.is_finite()
        || !res.residual_m.is_finite()
        || (expected_res - res.residual_m).abs() > 1.0e-6
        || res.residual_m.abs() > tolerance_m
    {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "residual_m",
            reason: format!(
                "residual_m ({}) / achieved_radius_m ({}) violates goal tolerance ({tolerance_m})",
                res.residual_m, res.achieved_radius_m
            ),
        });
    }
    Ok(segment_id)
}

pub(super) fn verify_execution_artifact_binding(
    res: &VersionedSolverResult,
) -> Result<(), MissionDesignError> {
    let id = &res.request_identity;
    let rep = &res.final_execution;
    let st = &rep.execution_state;
    if rep.plan_id != res.updated_plan.plan_id
        || rep.plan_version != res.updated_plan.version
        || rep.plan_content_hash != res.verified_updated_plan_hash
        || rep.status != MissionLifecycleStatus::Completed
    {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "final_execution",
            reason: "final_execution header does not match verified updated_plan".to_owned(),
        });
    }
    if st.plan_id != res.updated_plan.plan_id
        || st.plan_version != res.updated_plan.version
        || st.plan_content_hash != res.verified_updated_plan_hash
        || st.status != MissionLifecycleStatus::Completed
        || st.anchor_tdb != id.anchor_tdb
        || st.initial_instant != id.initial_instant
        || st.initial_checkpoint_hash != id.initial_checkpoint_hash
        || st.input_fingerprint != id.input_fingerprint
    {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "final_execution.execution_state",
            reason: "execution_state identity fields do not match updated_plan or request"
                .to_owned(),
        });
    }
    verify_checkpoint_and_trajectory_product(res)
}

fn verify_checkpoint_and_trajectory_product(
    res: &VersionedSolverResult,
) -> Result<(), MissionDesignError> {
    let id = &res.request_identity;
    let rep = &res.final_execution;
    let st = &rep.execution_state;
    let actual_final_ckpt_hash = rep.final_checkpoint.checkpoint_hash();
    if actual_final_ckpt_hash != res.verified_final_checkpoint_hash
        || rep.trajectory_product.terminal_plant != rep.final_checkpoint.plant
    {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "final_execution.final_checkpoint",
            reason: "final_checkpoint was mutated or detached from trajectory_product".to_owned(),
        });
    }
    let rebuilt_prod = OrbitalTrajectoryProduct::from_segments(
        res.updated_plan.plan_id,
        id.input_fingerprint,
        rep.trajectory_product.dry_mass_kg,
        rep.trajectory_product.initial_plant,
        rep.final_checkpoint.plant,
        st.anchor_tdb,
        rep.trajectory_product.segments.clone(),
    );
    let actual_traj_hash = rep.trajectory_product.provenance.content_hash;
    if actual_traj_hash != res.verified_trajectory_hash
        || actual_traj_hash != rebuilt_prod.provenance.content_hash
        || rep.trajectory_product.product_id != res.updated_plan.plan_id
        || rep.trajectory_product.input_fingerprint != id.input_fingerprint
    {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "final_execution.trajectory_product",
            reason: "trajectory_product provenance or segments do not match verified execution"
                .to_owned(),
        });
    }
    let expected_seal = compute_conclusion_binding_hash(
        id,
        res.status,
        res.iterations,
        res.solved_burn_duration_s,
        res.achieved_radius_m,
        res.residual_m,
        res.verified_updated_plan_hash,
        actual_final_ckpt_hash,
        actual_traj_hash,
    );
    if expected_seal != res.conclusion_binding_hash {
        return Err(MissionDesignError::SolverArtifactBindingMismatch {
            field: "conclusion_binding_hash",
            reason: "solver conclusion seal mismatch".to_owned(),
        });
    }
    Ok(())
}
