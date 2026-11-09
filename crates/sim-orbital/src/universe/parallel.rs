//! Multi-thread speculative worker pool over [`OrbitalUniverse`] and hardware-tagged
//! `10 / 100 / 1,000 / 10,000` fleet-scale benchmark harness.
//!
//! Guarantees bitwise-identical committed states across `1`, `2`, `4`, and `8` threads
//! and under arbitrary worker completion permutations because workers are pure functions
//! of `(VesselCheckpoint, VesselConfig, SimInstant)` and `ingest_proposals` sorts canonically
//! by `(VesselId, from_instant)` before mutating universe state.

use std::collections::BTreeMap;
use std::sync::Arc;

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

use crate::{
    builtin_solar::BuiltinSolarSystem2026_2028,
    checkpoint::{PropagatorKind, VesselCheckpoint, VesselConfig},
    dop853::Dop853Tolerances,
    ephemeris::EphemerisCatalog,
    gravity::ForceModel,
    hash::ContentHash,
    ids::{NaifId, VesselId},
    manifest::SimulationManifestDigest,
    orientation::BodyOrientationCatalog,
    plant::{ContinuousPlantState, MassPropertiesModel, PropellantMode, ReactionWheelMode},
    scalar_math::{cos, sin, sqrt},
    time::{SimClockAdapter, TdbEpoch},
    universe::{
        MAX_SPECULATIVE_SEGMENTS, OrbitalUniverse, RollbackMetrics, SchedulerError,
        SpeculativeProposal,
    },
};

/// Hardware and runtime environment descriptor attached to [`FleetScaleBenchmarkReport`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareProfile {
    /// Target CPU architecture (`aarch64`, `x86_64`, `wasm32`, etc.).
    pub arch: String,
    /// Target operating system (`macos`, `linux`, `windows`, `unknown`).
    pub os: String,
    /// Host logical CPU parallelism reported by the runtime.
    pub available_parallelism: usize,
    /// Number of worker threads used during the benchmark run.
    pub worker_threads_used: usize,
}

impl HardwareProfile {
    /// Captures the current host's [`HardwareProfile`] for `worker_threads_used`.
    #[must_use]
    pub fn detect(worker_threads_used: usize) -> Self {
        let par = std::thread::available_parallelism().map_or(1, usize::from);
        Self {
            arch: std::env::consts::ARCH.to_owned(),
            os: std::env::consts::OS.to_owned(),
            available_parallelism: par,
            worker_threads_used: worker_threads_used.max(1),
        }
    }
}

/// Frame-step latency distribution and final state hash for one fleet size tier.
///
/// Explicitly scoped to warm-cache `1.0 s` real-time fleet scheduler commit steps
/// (`FullCoupled6Dof` active vessels + `DormantKepler` background fleet), NOT multi-month
/// trajectory optimization.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FleetTierMetrics {
    /// Total vessels in the universe (`10`, `100`, `1_000`, `10_000`).
    pub vessel_count: usize,
    /// Number of active `FullCoupled6Dof` / `ReducedManifoldThrust` vessels.
    pub active_vessel_count: usize,
    /// Number of `Dormant` / `PinnedSurfaceRest` background fleet vessels.
    pub background_vessel_count: usize,
    /// Real-time commit step duration measured per frame (`s`).
    pub step_horizon_s: f64,
    /// Cache condition (`"warm_speculative_segment_ring"`).
    pub cache_regime: String,
    /// Relative integrator tolerance (`rtol`).
    pub tolerance_rtol: f64,
    /// Median frame-step wall-clock time (`ms`).
    pub p50_ms: f64,
    /// 95th-percentile frame-step wall-clock time (`ms`).
    pub p95_ms: f64,
    /// 99th-percentile frame-step wall-clock time (`ms`).
    pub p99_ms: f64,
    /// Canonical SHA-256 [`ContentHash`] of the final committed [`crate::UniverseSnapshot`].
    pub state_hash: ContentHash,
}

/// Hardware-tagged benchmark report across fleet tiers (`10`, `100`, `1_000`, `10_000`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FleetScaleBenchmarkReport {
    /// Host hardware and thread-pool descriptor.
    pub hardware: HardwareProfile,
    /// Per-tier latency percentiles and deterministic state hashes.
    pub tiers: Vec<FleetTierMetrics>,
}

impl OrbitalUniverse {
    /// Advances `committed_instant` to `target` using a parallel worker pool of `worker_threads`
    /// threads (producing bitwise-identical state to [`Self::advance_committed_to`]).
    pub fn advance_committed_to_parallel(
        &mut self,
        target: SimInstant,
        worker_threads: usize,
    ) -> Result<RollbackMetrics, SchedulerError> {
        if target < self.committed_instant {
            return Err(SchedulerError::BackwardCommitTarget {
                requested: target,
                committed: self.committed_instant,
            });
        }
        let mut metrics = RollbackMetrics::default();
        let mut same_tick_counts: BTreeMap<(VesselId, u128), u32> = BTreeMap::new();

        loop {
            let next_ev_inst = self.event_queue.keys().next().map(|k| k.instant);
            let step_goal = match next_ev_inst {
                Some(ev_t) if ev_t <= target => ev_t,
                _ => target,
            };

            if !self.all_vessels_covered_through(step_goal) {
                self.speculate_missing_vessels_parallel(step_goal, worker_threads)?;
                continue;
            }

            self.committed_instant = step_goal;
            let clock = self.clock;
            let orient = self.orientation.clone();
            for tl in self.vessels.values_mut() {
                tl.drain_committed_through(step_goal, &clock, orient.as_ref())?;
            }

            if self.has_events_at_committed() {
                let m = self.process_one_committed_event(&mut same_tick_counts)?;
                metrics.accumulate(m);
                continue;
            }

            if self.committed_instant >= target {
                break;
            }
        }
        Ok(metrics)
    }

    fn speculate_missing_vessels_parallel(
        &mut self,
        goal: SimInstant,
        worker_threads: usize,
    ) -> Result<(), SchedulerError> {
        let clock = self.clock;
        let orient = self.orientation.clone();
        for tl in self.vessels.values_mut() {
            if tl.frontier_instant() < goal && tl.speculative.len() >= MAX_SPECULATIVE_SEGMENTS {
                tl.drain_committed_through(self.committed_instant, &clock, orient.as_ref())?;
            }
        }
        let reqs = self.build_worker_requests(goal);
        if reqs.is_empty() {
            return Ok(());
        }
        let prev_frontiers: Vec<(VesselId, SimInstant)> = reqs
            .iter()
            .map(|(cp, ..)| (cp.vessel_id, cp.instant))
            .collect();
        let proposals = self.dispatch_worker_batch(&reqs, worker_threads.max(1))?;
        self.ingest_proposals(proposals);
        for (vid, prev_inst) in prev_frontiers {
            if let Some(tl) = self.vessels.get(&vid)
                && tl.frontier_instant() <= prev_inst
            {
                return Err(SchedulerError::SpeculationStalled {
                    vessel_id: vid,
                    frontier: tl.frontier_instant(),
                    step_goal: goal,
                });
            }
        }
        Ok(())
    }

    fn dispatch_worker_batch(
        &self,
        reqs: &[(VesselCheckpoint, VesselConfig, SimInstant, usize)],
        worker_threads: usize,
    ) -> Result<Vec<SpeculativeProposal>, SchedulerError> {
        #[cfg(not(target_arch = "wasm32"))]
        if worker_threads > 1 && reqs.len() > 1 {
            let chunk_size = reqs.len().div_ceil(worker_threads).max(1);
            let mut results: Vec<Result<Vec<SpeculativeProposal>, SchedulerError>> =
                Vec::with_capacity(worker_threads);
            let mut join_err: Option<SchedulerError> = None;
            std::thread::scope(|s| {
                let mut handles = Vec::new();
                for chunk in reqs.chunks(chunk_size) {
                    handles.push(s.spawn(move || {
                        let mut out = Vec::with_capacity(chunk.len());
                        for (cp, cfg, horizon, len) in chunk {
                            out.push(self.evaluate_worker_request(cp, cfg, *horizon, *len)?);
                        }
                        Ok(out)
                    }));
                }
                for (worker_index, h) in handles.into_iter().enumerate() {
                    match h.join() {
                        Ok(res) => results.push(res),
                        Err(_) => {
                            if join_err.is_none() {
                                join_err = Some(SchedulerError::WorkerBatchFailed {
                                    worker_index,
                                    expected_proposals: reqs.len(),
                                    received_proposals: 0,
                                });
                            }
                        }
                    }
                }
            });
            if let Some(err) = join_err {
                return Err(err);
            }
            let mut merged = Vec::with_capacity(reqs.len());
            for r in results {
                merged.extend(r?);
            }
            if merged.len() != reqs.len() {
                return Err(SchedulerError::WorkerBatchFailed {
                    worker_index: worker_threads,
                    expected_proposals: reqs.len(),
                    received_proposals: merged.len(),
                });
            }
            return Ok(merged);
        }
        let _ = worker_threads;
        let mut out = Vec::with_capacity(reqs.len());
        for (cp, cfg, horizon, len) in reqs {
            out.push(self.evaluate_worker_request(cp, cfg, *horizon, *len)?);
        }
        Ok(out)
    }
}

/// Runs a deterministic fleet-scale benchmark across `vessel_counts` (e.g. `&[10, 100, 1_000]`)
/// for `frames_per_tier` steps using `worker_threads` threads.
pub fn run_fleet_scale_benchmark(
    solar: &BuiltinSolarSystem2026_2028,
    vessel_counts: &[usize],
    frames_per_tier: usize,
    worker_threads: usize,
) -> Result<FleetScaleBenchmarkReport, SchedulerError> {
    let mut tiers = Vec::with_capacity(vessel_counts.len());
    for &count in vessel_counts {
        tiers.push(run_single_fleet_tier(
            solar,
            count,
            frames_per_tier.max(1),
            worker_threads.max(1),
        )?);
    }
    Ok(FleetScaleBenchmarkReport {
        hardware: HardwareProfile::detect(worker_threads),
        tiers,
    })
}

fn run_single_fleet_tier(
    solar: &BuiltinSolarSystem2026_2028,
    vessel_count: usize,
    frames: usize,
    worker_threads: usize,
) -> Result<FleetTierMetrics, SchedulerError> {
    let mut univ = build_fleet_universe(solar, vessel_count)?;
    let step_span = TickSpan::from_seconds_and_subsec_ticks(1, 0);
    let mut cur = SimInstant::START.advanced(step_span);
    // Warm up speculative segment cache before measuring steady-state frame times.
    let _ = univ.advance_committed_to_parallel(cur, worker_threads)?;

    let mut samples_ms = Vec::with_capacity(frames);
    let mut f_idx = 0_usize;
    while f_idx < frames {
        cur = cur.advanced(step_span);
        let t_start = std::time::Instant::now();
        let _ = univ.advance_committed_to_parallel(cur, worker_threads)?;
        samples_ms.push(t_start.elapsed().as_secs_f64() * 1_000.0);
        f_idx = f_idx.wrapping_add(1);
    }
    samples_ms.sort_by(f64::total_cmp);
    let p50 = percentile_sorted(&samples_ms, 0.50);
    let p95 = percentile_sorted(&samples_ms, 0.95);
    let p99 = percentile_sorted(&samples_ms, 0.99);
    let active_count = vessel_count.min(2);
    Ok(FleetTierMetrics {
        vessel_count,
        active_vessel_count: active_count,
        background_vessel_count: vessel_count.saturating_sub(active_count),
        step_horizon_s: 1.0,
        cache_regime: "warm_speculative_segment_ring".to_owned(),
        tolerance_rtol: Dop853Tolerances::default().rtol,
        p50_ms: p50,
        p95_ms: p95,
        p99_ms: p99,
        state_hash: univ.capture_snapshot().snapshot_hash(),
    })
}

fn build_fleet_universe(
    solar: &BuiltinSolarSystem2026_2028,
    vessel_count: usize,
) -> Result<OrbitalUniverse, SchedulerError> {
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).map_err(|s| {
        SchedulerError::Propagation {
            vessel_id: VesselId(1),
            source: crate::dop853::Dop853Error::Rhs(s),
        }
    })?;
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let manifest = SimulationManifestDigest::compute(
        t0,
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"sim-scenery:terrain-empty:v1"),
        fm.content_hash(),
        Dop853Tolerances::default().content_hash(),
    );
    let mut univ = OrbitalUniverse::new(
        manifest,
        clock,
        fm,
        Arc::new(solar.ephemeris.clone()),
        Arc::new(solar.orientation.clone()),
    )
    .with_horizon_quantum_ticks(60 * (TICKS_PER_SECOND as u128));

    let mu = 3.986_004_418e14_f64;
    let mut i = 0_usize;
    while i < vessel_count {
        let raw_id = (i as u64).wrapping_add(1);
        let vid = VesselId(raw_id);
        let r = 6_778_137.0 + f64::from((i % 40) as u32) * 15_000.0;
        let phase = f64::from(i as u32) * 0.137_5;
        let v = sqrt(mu / r);
        let prop_kind = if i < 2 {
            PropagatorKind::FullCoupled6Dof
        } else {
            PropagatorKind::DormantKepler
        };
        let (cfg, cp) = make_fleet_vessel(vid, r, v, phase, prop_kind);
        univ.insert_vessel(cfg, cp)?;
        i = i.wrapping_add(1);
    }
    Ok(univ)
}

fn make_fleet_vessel(
    vid: VesselId,
    r_m: f64,
    v_m_s: f64,
    phase_rad: f64,
    prop_kind: PropagatorKind,
) -> (VesselConfig, VesselCheckpoint) {
    let cfg = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(
            12_000.0,
            [85_000.0, 18_000.0, 85_000.0],
            [0.0; 3],
        ),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 5.0e6,
            tau_max_nm: 5.0e4,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 6.0,
        auto_soi_switch: false,
    };
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(r_m * cos(phase_rad), r_m * sin(phase_rad), 0.0),
        vel_rel_icrf_m_s: DVec3::new(-v_m_s * sin(phase_rad), v_m_s * cos(phase_rad), 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let mut cp = VesselCheckpoint::new(vid, SimInstant::START, plant, vid.0);
    cp.integrator_hint.propagator = prop_kind;
    cp.integrator_hint.h_trial_s = Some(5.0);
    (cfg, cp)
}

fn percentile_sorted(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = ((sorted.len().saturating_sub(1) as f64) * q.clamp(0.0, 1.0)).round() as usize;
    sorted.get(idx).copied().unwrap_or(0.0)
}
