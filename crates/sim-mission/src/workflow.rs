//! End-to-end GMAT-style orbital maneuver targeting and ground-station access workflow
//! (`Initial State -> Finite Burn -> Propagate to Event -> Targeter Solve -> Ground Station Access Analysis`),
//! plus multi-variable differential correction and the Assisted $L_2$ Planner.

mod differential_corrector;
mod l2_planner;
mod station_keeping;

use serde::{Deserialize, Serialize};
use sim_analysis::{
    AccessQuerySpec, DesignRevisionIdentity, GroundStationModel, OccultationEnvironmentModel,
    StationAccessReport, VehicleSensorModel, evaluate_station_access,
};
use sim_orbital::{
    BodyOrientationCatalog, EphemerisCatalog, ForceModel, SimClockAdapter, VesselCheckpoint,
    VesselConfig,
};
use sim_spacetime::{ContentHash, DomainHasher, TranslationalTrajectorySource};

pub use differential_corrector::{
    DifferentialCorrectorIterationRecord, DifferentialCorrectorReport, ScaledControlVariable,
    ScaledEqualityGoal, solve_multivariable_differential_correction,
};
pub use l2_planner::{
    AssistedL2PlanReport, L2PeriodicClosureCheck, L2SensitivitySummary,
    plan_assisted_l2_bounded_arc, verify_l2_periodic_closure,
};
pub use station_keeping::{
    L2StationKeepingReport, L2StationKeepingSpec, StationKeepingBurn, StationKeepingError,
    fly_l2_station_keeping,
};

use crate::{
    ApoapsisTargetProblem, MissionDesignError, MissionExecutionReport, MissionPlan,
    SolverConvergenceStatus, SolverRequestIdentity, VersionedSolverResult, solve_apoapsis_target,
};

/// Summary comparison metrics for a solved orbital maneuver + ground-station access candidate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OrbitalManeuverCandidateMetrics {
    /// Candidate identifier.
    pub candidate_id: String,
    /// Design revision identity bound to this candidate.
    pub design_revision: DesignRevisionIdentity,
    /// Solved burn duration (`s`).
    pub solved_burn_duration_s: f64,
    /// Propellant mass consumed across the mission (`kg`).
    pub propellant_consumed_kg: f64,
    /// Total propagated mission duration (`s`).
    pub mission_duration_s: f64,
    /// Absolute terminal apoapsis radius residual $|r_a - r_{a,\text{target}}|$ (`m`).
    pub target_residual_m: f64,
    /// Total ground-station access duration summed across all evaluated stations (`s`).
    pub total_station_access_duration_s: f64,
    /// Total number of ground-station access windows across all evaluated stations.
    pub station_access_window_count: usize,
    /// Cryptographic digest binding the solver result, verified plan, trajectory product, and access reports.
    pub artifact_chain_digest: ContentHash,
}

/// Complete end-to-end orbital maneuver + ground-station access artifact linking:
/// `DesignRevisionIdentity -> SolverRequestIdentity -> VersionedSolverResult -> MissionPlan -> OrbitalTrajectoryProduct -> Vec<StationAccessReport>`.
#[derive(Clone, Debug)]
pub struct OrbitalManeuverAccessArtifact {
    /// Candidate identifier.
    pub candidate_id: String,
    /// Bound design revision identity.
    pub design_revision: DesignRevisionIdentity,
    /// Versioned solver request identity.
    pub solver_request_identity: SolverRequestIdentity,
    /// Converged differential-correction solver result.
    pub solver_result: VersionedSolverResult,
    /// Verified post-solve `MissionPlan` with updated finite-burn duration.
    pub verified_plan: MissionPlan,
    /// Propagated mission execution report containing the `OrbitalTrajectoryProduct`.
    pub execution_report: MissionExecutionReport,
    /// Ground-station access reports evaluated directly over `execution_report.trajectory_product`.
    pub station_access_reports: Vec<StationAccessReport>,
    /// Candidate comparison summary metrics and cryptographic chain digest.
    pub metrics: OrbitalManeuverCandidateMetrics,
}

impl OrbitalManeuverAccessArtifact {
    /// Verifies that the entire cryptographic chain (`design_revision -> solver_result -> verified_plan -> execution_report -> station_access_reports`) is intact.
    #[must_use]
    pub fn verify_chain_integrity(
        &self,
        expected_revision: &DesignRevisionIdentity,
        base_plan: &MissionPlan,
        initial_checkpoint: &VesselCheckpoint,
        clock: &SimClockAdapter,
    ) -> bool {
        if &self.design_revision != expected_revision {
            return false;
        }
        if self
            .solver_result
            .verify_applicable(
                &self.solver_request_identity,
                base_plan,
                initial_checkpoint,
                clock,
            )
            .is_err()
        {
            return false;
        }
        if self.verified_plan != self.solver_result.updated_plan {
            return false;
        }
        let expected_traj_prov =
            TranslationalTrajectorySource::provenance(&self.execution_report.trajectory_product);
        for rep in &self.station_access_reports {
            if rep.provenance.first() != Some(expected_traj_prov) {
                return false;
            }
        }
        compute_chain_digest(
            &self.candidate_id,
            &self.design_revision,
            &self.solver_result,
            &self.verified_plan,
            &self.execution_report,
            &self.station_access_reports,
        ) == self.metrics.artifact_chain_digest
    }
}

/// Executes the complete GMAT-style workflow:
/// `Initial State -> Finite Burn -> Propagate to Event -> Targeter Solve -> Ground Station Access Analysis`.
///
/// Fails closed if the targeter does not converge or if any ground-station access evaluation fails.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_orbital_maneuver_and_access(
    candidate_id: impl Into<String>,
    design_revision: DesignRevisionIdentity,
    problem: &ApoapsisTargetProblem,
    initial_checkpoint: &VesselCheckpoint,
    vessel_cfg: &VesselConfig,
    clock: &SimClockAdapter,
    stations: &[GroundStationModel],
    vehicle_sensor: &VehicleSensorModel,
    occultation_env: &OccultationEnvironmentModel,
    access_sample_step_s: f64,
    access_boundary_tolerance_s: f64,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<OrbitalManeuverAccessArtifact, MissionDesignError> {
    let candidate_id = candidate_id.into();
    let solver_result = solve_apoapsis_target(
        problem,
        initial_checkpoint,
        vessel_cfg,
        clock,
        force_model,
        ephemeris,
        orientation,
    )?;
    if solver_result.status != SolverConvergenceStatus::Converged {
        return Err(MissionDesignError::InvalidPlan {
            plan_id: problem.base_plan.plan_id,
            reason: format!(
                "targeter did not converge for candidate `{candidate_id}`: {:?}",
                solver_result.status
            ),
        });
    }
    let mut verified_plan = problem.base_plan.clone();
    solver_result.commit_to_plan(
        &solver_result.request_identity,
        &mut verified_plan,
        initial_checkpoint,
        clock,
    )?;
    let execution_report = solver_result.final_execution.clone();
    let traj = &execution_report.trajectory_product;
    let valid_interval = traj.valid_interval;
    let query = AccessQuerySpec {
        query_interval: valid_interval,
        sample_step_s: access_sample_step_s,
        boundary_time_tolerance_s: access_boundary_tolerance_s,
        min_range_m: None,
        max_range_m: None,
        signal_time: sim_analysis::SignalTimeModel::Geometric,
    };
    let (station_access_reports, total_access_s, window_count) = evaluate_all_station_reports(
        verified_plan.plan_id,
        traj,
        stations,
        vehicle_sensor,
        occultation_env,
        &query,
        ephemeris,
        orientation,
    )?;
    Ok(assemble_maneuver_access_artifact(
        candidate_id,
        design_revision,
        initial_checkpoint,
        solver_result,
        verified_plan,
        execution_report,
        station_access_reports,
        valid_interval.duration_s(),
        total_access_s,
        window_count,
    ))
}

#[allow(clippy::too_many_arguments)]
fn assemble_maneuver_access_artifact(
    candidate_id: String,
    design_revision: DesignRevisionIdentity,
    initial_checkpoint: &VesselCheckpoint,
    solver_result: VersionedSolverResult,
    verified_plan: MissionPlan,
    execution_report: MissionExecutionReport,
    station_access_reports: Vec<StationAccessReport>,
    mission_duration_s: f64,
    total_access_s: f64,
    window_count: usize,
) -> OrbitalManeuverAccessArtifact {
    let propellant_consumed_kg = (initial_checkpoint.plant.propellant_mass_kg
        - execution_report.final_checkpoint.plant.propellant_mass_kg)
        .max(0.0);
    let artifact_chain_digest = compute_chain_digest(
        &candidate_id,
        &design_revision,
        &solver_result,
        &verified_plan,
        &execution_report,
        &station_access_reports,
    );
    let metrics = OrbitalManeuverCandidateMetrics {
        candidate_id: candidate_id.clone(),
        design_revision: design_revision.clone(),
        solved_burn_duration_s: solver_result.solved_burn_duration_s,
        propellant_consumed_kg,
        mission_duration_s,
        target_residual_m: solver_result.residual_m.abs(),
        total_station_access_duration_s: total_access_s,
        station_access_window_count: window_count,
        artifact_chain_digest,
    };
    OrbitalManeuverAccessArtifact {
        candidate_id,
        design_revision,
        solver_request_identity: solver_result.request_identity,
        solver_result,
        verified_plan,
        execution_report,
        station_access_reports,
        metrics,
    }
}

#[allow(clippy::too_many_arguments)]
fn evaluate_all_station_reports(
    plan_id: u64,
    traj: &sim_orbital::OrbitalTrajectoryProduct,
    stations: &[GroundStationModel],
    vehicle_sensor: &VehicleSensorModel,
    occultation_env: &OccultationEnvironmentModel,
    query: &AccessQuerySpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(Vec<StationAccessReport>, f64, usize), MissionDesignError> {
    let mut reports = Vec::with_capacity(stations.len());
    let mut total_access_s = 0.0_f64;
    let mut window_count = 0_usize;
    for station in stations {
        let rep = evaluate_station_access(
            traj,
            Some(traj),
            station,
            vehicle_sensor,
            occultation_env,
            query,
            ephemeris,
            orientation,
        )
        .map_err(|e| MissionDesignError::InvalidPlan {
            plan_id,
            reason: format!("station access error: {e}"),
        })?;
        for w in &rep.windows {
            total_access_s += w.duration_s;
        }
        window_count = window_count.wrapping_add(rep.windows.len());
        reports.push(rep);
    }
    Ok((reports, total_access_s, window_count))
}

fn compute_chain_digest(
    candidate_id: &str,
    design_revision: &DesignRevisionIdentity,
    solver_result: &VersionedSolverResult,
    verified_plan: &MissionPlan,
    execution_report: &MissionExecutionReport,
    station_access_reports: &[StationAccessReport],
) -> ContentHash {
    let mut h = DomainHasher::new(b"sim-mission:orbital-maneuver-access-chain:v1");
    h.write_bytes(candidate_id.as_bytes());
    design_revision.write_to_hasher(&mut h);
    h.write_hash(solver_result.request_identity.base_plan_content_hash);
    h.write_hash(verified_plan.content_hash());
    h.write_hash(execution_report.trajectory_product.provenance.content_hash);
    h.write_u64(station_access_reports.len() as u64);
    for rep in station_access_reports {
        h.write_u32(rep.station_id);
        h.write_u64(rep.windows.len() as u64);
        for w in &rep.windows {
            h.write_i64(w.start_epoch.j2000_seconds());
            h.write_u64(w.start_epoch.subsec_ticks());
            h.write_i64(w.end_epoch.j2000_seconds());
            h.write_u64(w.end_epoch.subsec_ticks());
        }
    }
    h.finish()
}
