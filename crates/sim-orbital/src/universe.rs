//! Transactional multi-vessel orbital scheduler (`OrbitalUniverse`).
//!
//! Enforces pure worker speculation, frame-rate-independent `DOP853` step boundaries,
//! worker-completion-order invariance, total `EventOrderKey` ordering, generation-based
//! stale-event invalidation, zero-time loop protection, and manifest-verified snapshots.

mod error;
mod mutations;
mod parallel;
mod queries;
mod rollback;
mod snapshot;
pub mod worker;

use std::{collections::BTreeMap, sync::Arc};

use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

use crate::{
    checkpoint::{PropagatorKind, VesselCheckpoint, VesselConfig},
    ephemeris::EphemerisCatalog,
    events::{
        AuthoritativeCommandLog, EventOrderKey, PlayerCommandPayload, ScheduledEvent,
        StampedPlayerCommand, UniverseEventPayload, phase_ranks,
    },
    gravity::ForceModel,
    ids::VesselId,
    manifest::SimulationManifestDigest,
    orientation::BodyOrientationCatalog,
    time::SimClockAdapter,
};

use mutations::apply_checkpoint_event_mutation;

pub use error::SchedulerError;
pub use parallel::{
    FleetScaleBenchmarkReport, FleetTierMetrics, HardwareProfile, run_fleet_scale_benchmark,
};
pub use rollback::RollbackMetrics;
use rollback::VesselTimeline;
pub use snapshot::{UniverseSnapshot, VesselSnapshotRecord};
pub use worker::{
    MAX_PROPOSAL_CHUNK_SECONDS, MAX_SPECULATIVE_SEGMENTS, SpeculativeProposal,
    SpeculativeSegmentEntry, propose_vessel_horizon,
};

/// Maximum same-tick mutating discrete transitions allowed per vessel before failing closed.
pub const MAX_SAME_TICK_TRANSITIONS: u32 = 32;

/// Default canonical speculation quantum (`60 s`) decoupling step boundaries from render FPS.
pub const DEFAULT_HORIZON_QUANTUM_SECONDS: u64 = 60;

/// Authoritative multi-vessel transactional orbital scheduler.
pub struct OrbitalUniverse {
    manifest: SimulationManifestDigest,
    clock: SimClockAdapter,
    force_model: ForceModel,
    ephemeris: Arc<dyn EphemerisCatalog>,
    orientation: Arc<dyn BodyOrientationCatalog>,
    committed_instant: SimInstant,
    horizon_quantum_ticks: u128,
    vessels: BTreeMap<VesselId, VesselTimeline>,
    event_queue: BTreeMap<EventOrderKey, ScheduledEvent>,
    command_log: AuthoritativeCommandLog,
    fired_events_log: Vec<EventOrderKey>,
    active_camera_vessel: Option<VesselId>,
}

impl OrbitalUniverse {
    /// Creates an empty universe anchored at [`SimInstant::START`] (`clock.base_tdb()`).
    #[must_use]
    pub fn new(
        manifest: SimulationManifestDigest,
        clock: SimClockAdapter,
        force_model: ForceModel,
        ephemeris: Arc<dyn EphemerisCatalog>,
        orientation: Arc<dyn BodyOrientationCatalog>,
    ) -> Self {
        Self {
            manifest,
            clock,
            force_model,
            ephemeris,
            orientation,
            committed_instant: SimInstant::START,
            horizon_quantum_ticks: (DEFAULT_HORIZON_QUANTUM_SECONDS as u128)
                * (TICKS_PER_SECOND as u128),
            vessels: BTreeMap::new(),
            event_queue: BTreeMap::new(),
            command_log: AuthoritativeCommandLog::default(),
            fired_events_log: Vec::new(),
            active_camera_vessel: None,
        }
    }

    /// Configures the canonical speculation horizon quantum (`> 0` ticks).
    #[must_use]
    pub fn with_horizon_quantum_ticks(mut self, quantum_ticks: u128) -> Self {
        self.horizon_quantum_ticks = quantum_ticks.max(1);
        self
    }

    /// Registers a vessel at `committed_instant`, scheduling its initial discrete GNC tick if configured.
    pub fn insert_vessel(
        &mut self,
        config: VesselConfig,
        mut checkpoint: VesselCheckpoint,
    ) -> Result<(), SchedulerError> {
        let vid = checkpoint.vessel_id;
        if vid == VesselId::NONE {
            return Err(SchedulerError::UnknownVessel(vid));
        }
        if self.vessels.contains_key(&vid) {
            return Err(SchedulerError::DuplicateVessel(vid));
        }
        checkpoint.instant = self.committed_instant;
        if checkpoint.integrator_hint.propagator == PropagatorKind::PinnedSurfaceRest {
            checkpoint = checkpoint
                .at_pinned_surface_instant(
                    self.committed_instant,
                    &self.clock,
                    self.orientation.as_ref(),
                )
                .map_err(|source| SchedulerError::SurfaceRest {
                    vessel_id: vid,
                    source,
                })?;
        }
        if checkpoint.gnc.gnc_period_ticks > 0 && checkpoint.gnc.next_gnc_ticks.is_none() {
            checkpoint.gnc.next_gnc_ticks = Some(
                self.committed_instant
                    .ticks_u128()
                    .wrapping_add(checkpoint.gnc.gnc_period_ticks),
            );
        }
        self.vessels
            .insert(vid, VesselTimeline::new(config, checkpoint));
        Ok(())
    }

    /// Stamps a player command onto the authoritative timeline (`instant >= committed_instant`)
    /// and enqueues its execution event.
    pub fn stamp_player_command(
        &mut self,
        instant: SimInstant,
        vessel_id: VesselId,
        payload: PlayerCommandPayload,
    ) -> Result<StampedPlayerCommand, SchedulerError> {
        if instant < self.committed_instant {
            return Err(SchedulerError::CommandInPast {
                command_instant: instant,
                committed: self.committed_instant,
            });
        }
        if !self.vessels.contains_key(&vessel_id) {
            return Err(SchedulerError::UnknownVessel(vessel_id));
        }
        let stamped = self.command_log.record(instant, vessel_id, payload);
        let key = EventOrderKey::single(
            instant,
            phase_ranks::PLAYER_COMMAND,
            vessel_id,
            0,
            stamped.command_seq,
        );
        self.event_queue.insert(
            key,
            ScheduledEvent::single(
                key,
                vessel_id,
                None,
                UniverseEventPayload::PlayerCommand(payload),
            ),
        );
        Ok(stamped)
    }

    /// Enqueues a deterministic scheduled event at `event.key.instant >= committed_instant`.
    pub fn enqueue_event(&mut self, event: ScheduledEvent) -> Result<(), SchedulerError> {
        if event.key.instant < self.committed_instant {
            return Err(SchedulerError::BackwardCommitTarget {
                requested: event.key.instant,
                committed: self.committed_instant,
            });
        }
        self.event_queue.insert(event.key, event);
        Ok(())
    }

    /// Builds immutable worker inputs `(frontier_checkpoint, config, canonical_horizon, speculative_len)`
    /// for all vessels that have not yet speculated through `min_coverage_instant`.
    #[must_use]
    pub fn build_worker_requests(
        &self,
        min_coverage_instant: SimInstant,
    ) -> Vec<(VesselCheckpoint, VesselConfig, SimInstant, usize)> {
        let mut reqs = Vec::new();
        for tl in self.vessels.values() {
            if tl.frontier_instant() < min_coverage_instant
                && tl.speculative.len() < MAX_SPECULATIVE_SEGMENTS
            {
                let horizon = self.next_canonical_horizon(tl.frontier_instant());
                reqs.push((
                    tl.frontier_checkpoint.clone(),
                    tl.config,
                    horizon,
                    tl.speculative.len(),
                ));
            }
        }
        reqs
    }

    /// Evaluates one pure worker request without mutating `self` (callable in any thread or order).
    pub fn evaluate_worker_request(
        &self,
        checkpoint: &VesselCheckpoint,
        config: &VesselConfig,
        horizon_target: SimInstant,
        existing_speculative_len: usize,
    ) -> Result<SpeculativeProposal, SchedulerError> {
        propose_vessel_horizon(
            checkpoint,
            config,
            horizon_target,
            existing_speculative_len,
            &self.clock,
            &self.force_model,
            self.ephemeris.as_ref(),
            self.orientation.as_ref(),
        )
    }

    /// Deterministically ingests worker proposals regardless of thread completion order.
    pub fn ingest_proposals(&mut self, proposals: impl IntoIterator<Item = SpeculativeProposal>) {
        let mut sorted: Vec<SpeculativeProposal> = proposals.into_iter().collect();
        sorted.sort_by_key(|p| (p.vessel_id, p.from_instant));
        for prop in sorted {
            let vid = prop.vessel_id;
            let events = prop.proposed_events.clone();
            if let Some(tl) = self.vessels.get_mut(&vid)
                && tl.apply_proposal(prop)
            {
                for ev in events {
                    if ev.key.instant >= self.committed_instant {
                        self.event_queue.insert(ev.key, ev);
                    }
                }
            }
        }
    }

    /// Advances authoritative `committed_instant` to `target`, resolving all events up to `target`
    /// in strict `EventOrderKey` order while keeping integration steps independent of `target`.
    pub fn advance_committed_to(
        &mut self,
        target: SimInstant,
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

            let step_goal = if self.all_vessels_covered_through(step_goal) {
                step_goal
            } else if self.speculate_missing_vessels_once(step_goal)? {
                continue;
            } else {
                // Every lagging vessel's speculative buffer is full. Committing up to
                // the earliest frontier drains it, and no event lies before that instant.
                self.earliest_frontier().min(step_goal)
            };

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

    fn next_canonical_horizon(&self, from: SimInstant) -> SimInstant {
        let q = self.horizon_quantum_ticks.max(1);
        let next_grid = ((from.ticks_u128() / q).wrapping_add(1)).saturating_mul(q);
        SimInstant::START.advanced(TickSpan::from_ticks_u128(next_grid))
    }

    fn all_vessels_covered_through(&self, goal: SimInstant) -> bool {
        self.vessels
            .values()
            .all(|tl| tl.frontier_instant() >= goal)
    }

    fn earliest_frontier(&self) -> SimInstant {
        self.vessels
            .values()
            .map(VesselTimeline::frontier_instant)
            .min()
            .unwrap_or(self.committed_instant)
    }

    /// Speculates each vessel short of `goal` one horizon further. `false` when no vessel
    /// had room in its speculative buffer.
    fn speculate_missing_vessels_once(&mut self, goal: SimInstant) -> Result<bool, SchedulerError> {
        let clock = self.clock;
        let orient = self.orientation.clone();
        for tl in self.vessels.values_mut() {
            if tl.frontier_instant() < goal && tl.speculative.len() >= MAX_SPECULATIVE_SEGMENTS {
                tl.drain_committed_through(self.committed_instant, &clock, orient.as_ref())?;
            }
        }
        let reqs = self.build_worker_requests(goal);
        if reqs.is_empty() {
            return Ok(false);
        }
        let prev_frontiers: Vec<(VesselId, SimInstant)> = reqs
            .iter()
            .map(|(cp, ..)| (cp.vessel_id, cp.instant))
            .collect();
        let mut proposals = Vec::with_capacity(reqs.len());
        for (cp, cfg, horizon, len) in &reqs {
            proposals.push(self.evaluate_worker_request(cp, cfg, *horizon, *len)?);
        }
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
        Ok(true)
    }

    fn has_events_at_committed(&self) -> bool {
        self.event_queue
            .keys()
            .next()
            .is_some_and(|k| k.instant <= self.committed_instant)
    }

    fn process_one_committed_event(
        &mut self,
        same_tick_counts: &mut BTreeMap<(VesselId, u128), u32>,
    ) -> Result<RollbackMetrics, SchedulerError> {
        let Some((&first_key, _)) = self.event_queue.iter().next() else {
            return Ok(RollbackMetrics::default());
        };
        let Some(ev) = self.event_queue.remove(&first_key) else {
            return Ok(RollbackMetrics::default());
        };
        if !self.generations_still_valid(&ev) {
            return Ok(RollbackMetrics::default());
        }
        self.fired_events_log.push(ev.key);
        let [vid, _] = ev.key.participants;
        if matches!(ev.payload, UniverseEventPayload::Marker { .. }) {
            return Ok(RollbackMetrics::default());
        }

        let tick_key = (vid, self.committed_instant.ticks_u128());
        let count = same_tick_counts.get(&tick_key).copied().unwrap_or(0);
        if count >= MAX_SAME_TICK_TRANSITIONS {
            return Err(SchedulerError::ZeroTimeEventLoop {
                vessel_id: vid,
                instant: self.committed_instant,
                limit: MAX_SAME_TICK_TRANSITIONS,
            });
        }
        same_tick_counts.insert(tick_key, count.wrapping_add(1));

        let m = self.rollback_vessel(vid, self.committed_instant)?;
        self.apply_vessel_event_mutation(vid, ev.payload)?;
        Ok(m)
    }

    fn generations_still_valid(&self, ev: &ScheduledEvent) -> bool {
        for guard in ev.expected_generations.into_iter().flatten() {
            let (vid, expected_gen) = guard;
            let Some(tl) = self.vessels.get(&vid) else {
                return false;
            };
            let current_gen = if tl.committed_checkpoint.instant == self.committed_instant {
                tl.committed_checkpoint.generations.state_gen
            } else {
                tl.frontier_checkpoint.generations.state_gen
            };
            if current_gen != expected_gen {
                return false;
            }
        }
        true
    }

    fn rollback_vessel(
        &mut self,
        vid: VesselId,
        instant: SimInstant,
    ) -> Result<RollbackMetrics, SchedulerError> {
        let tl = self
            .vessels
            .get_mut(&vid)
            .ok_or(SchedulerError::UnknownVessel(vid))?;
        tl.rollback_to_instant(
            instant,
            &self.clock,
            &self.force_model,
            self.ephemeris.as_ref(),
            self.orientation.as_ref(),
        )
    }

    fn apply_vessel_event_mutation(
        &mut self,
        vid: VesselId,
        payload: UniverseEventPayload,
    ) -> Result<(), SchedulerError> {
        let ep = self.clock.instant_to_tdb(self.committed_instant);
        let committed = self.committed_instant;
        let follow_up = {
            let tl = self
                .vessels
                .get_mut(&vid)
                .ok_or(SchedulerError::UnknownVessel(vid))?;
            let next_ev = apply_checkpoint_event_mutation(
                &mut tl.committed_checkpoint,
                committed,
                ep,
                payload,
                &self.force_model,
                self.ephemeris.as_ref(),
                vid,
            )?;
            tl.frontier_checkpoint = tl.committed_checkpoint.clone();
            next_ev
        };
        if let Some(ev) = follow_up {
            self.event_queue.insert(ev.key, ev);
        }
        Ok(())
    }
}
