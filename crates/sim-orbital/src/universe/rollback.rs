//! Per-vessel speculative segment ring, read-only sampling, and measured rollback.

use glam::DVec3;
use sim_spacetime::SimInstant;

use crate::{
    checkpoint::{PropagatorKind, VesselCheckpoint, VesselConfig},
    contact::PinnedSurfaceState,
    ephemeris::EphemerisCatalog,
    gravity::ForceModel,
    orientation::BodyOrientationCatalog,
    plant::ContinuousPlantState,
    time::SimClockAdapter,
    universe::{
        SchedulerError,
        snapshot::VesselSnapshotRecord,
        worker::{
            MAX_SPECULATIVE_SEGMENTS, SpeculativeProposal, SpeculativeSegmentEntry,
            propagate_checkpoint_interval,
        },
    },
};

/// Honest, measured cost accounting for speculative invalidation and rollback (`O(log S + S_drop)`
/// plus at most one containing-segment replay).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RollbackMetrics {
    /// Number of vessels whose speculative horizon was truncated or replayed.
    pub vessels_rolled_back: usize,
    /// Total speculative segments discarded across affected vessels.
    pub segments_dropped: usize,
    /// Number of integration substeps replayed to split a containing segment (`0` at boundaries).
    pub replayed_steps: u32,
    /// Number of binary-search segment probes executed.
    pub binary_search_probes: usize,
}

impl RollbackMetrics {
    pub(crate) fn accumulate(&mut self, other: Self) {
        self.vessels_rolled_back = self
            .vessels_rolled_back
            .saturating_add(other.vessels_rolled_back);
        self.segments_dropped = self.segments_dropped.saturating_add(other.segments_dropped);
        self.replayed_steps = self.replayed_steps.saturating_add(other.replayed_steps);
        self.binary_search_probes = self
            .binary_search_probes
            .saturating_add(other.binary_search_probes);
    }
}

#[derive(Clone, Debug)]
pub(crate) struct VesselTimeline {
    pub(crate) config: VesselConfig,
    pub(crate) committed_checkpoint: VesselCheckpoint,
    pub(crate) speculative: Vec<SpeculativeSegmentEntry>,
    pub(crate) frontier_checkpoint: VesselCheckpoint,
}

impl VesselTimeline {
    pub(crate) fn new(config: VesselConfig, initial: VesselCheckpoint) -> Self {
        Self {
            config,
            frontier_checkpoint: initial.clone(),
            committed_checkpoint: initial,
            speculative: Vec::new(),
        }
    }

    pub(crate) fn from_snapshot_record(rec: VesselSnapshotRecord) -> Self {
        let mut speculative = Vec::new();
        let frontier_checkpoint = match rec.spanning_segment {
            Some(seg) => {
                let end_cp = seg.end_checkpoint.clone();
                speculative.push(seg);
                end_cp
            }
            None => rec.committed_checkpoint.clone(),
        };
        Self {
            config: rec.config,
            committed_checkpoint: rec.committed_checkpoint,
            speculative,
            frontier_checkpoint,
        }
    }

    pub(crate) fn to_snapshot_record(&self, committed_instant: SimInstant) -> VesselSnapshotRecord {
        let spanning = self
            .speculative
            .first()
            .filter(|e| e.start_instant <= committed_instant && committed_instant < e.end_instant)
            .cloned();
        VesselSnapshotRecord {
            vessel_id: self.committed_checkpoint.vessel_id,
            config: self.config,
            committed_checkpoint: self.committed_checkpoint.clone(),
            spanning_segment: spanning,
        }
    }

    pub(crate) const fn frontier_instant(&self) -> SimInstant {
        self.frontier_checkpoint.instant
    }

    pub(crate) fn apply_proposal(&mut self, proposal: SpeculativeProposal) -> bool {
        if proposal.base_state_gen != self.frontier_checkpoint.generations.state_gen
            || proposal.from_instant != self.frontier_checkpoint.instant
        {
            return false;
        }
        let avail = MAX_SPECULATIVE_SEGMENTS.saturating_sub(self.speculative.len());
        if proposal.entries.is_empty() {
            self.frontier_checkpoint = proposal.end_checkpoint;
            return true;
        }
        let take = proposal.entries.len().min(avail);
        if take == 0 {
            return false;
        }
        for entry in proposal.entries.into_iter().take(take) {
            self.frontier_checkpoint = entry.end_checkpoint.clone();
            self.speculative.push(entry);
        }
        true
    }

    pub(crate) fn drain_committed_through(
        &mut self,
        committed_instant: SimInstant,
        clock: &SimClockAdapter,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<(), SchedulerError> {
        let mut drain_n: usize = 0;
        for entry in &self.speculative {
            if entry.end_instant <= committed_instant {
                self.committed_checkpoint = entry.end_checkpoint.clone();
                drain_n = drain_n.wrapping_add(1);
            } else {
                break;
            }
        }
        if drain_n > 0 {
            self.speculative.drain(..drain_n);
        }
        if self.speculative.is_empty() {
            if self.frontier_checkpoint.instant <= committed_instant {
                self.committed_checkpoint = self.frontier_checkpoint.clone();
            } else if self.committed_checkpoint.instant < committed_instant
                && self.committed_checkpoint.integrator_hint.propagator
                    == PropagatorKind::PinnedSurfaceRest
            {
                self.committed_checkpoint = self
                    .committed_checkpoint
                    .at_pinned_surface_instant(committed_instant, clock, orientation)
                    .map_err(|source| SchedulerError::SurfaceRest {
                        vessel_id: self.committed_checkpoint.vessel_id,
                        source,
                    })?;
            }
        }
        Ok(())
    }

    pub(crate) fn sample_plant_and_accel(
        &self,
        instant: SimInstant,
        clock: &SimClockAdapter,
        orientation: &dyn BodyOrientationCatalog,
    ) -> (ContinuousPlantState, DVec3) {
        if self.committed_checkpoint.integrator_hint.propagator == PropagatorKind::PinnedSurfaceRest
        {
            let pinned_opt = self.committed_checkpoint.pinned_surface.or_else(|| {
                PinnedSurfaceState::from_icrf_plant_state(
                    &self.committed_checkpoint.plant,
                    clock.instant_to_tdb(self.committed_checkpoint.instant),
                    orientation,
                )
                .ok()
            });
            if let Some(pinned) = pinned_opt
                && let Ok(res) = pinned.to_icrf_plant_and_accel(
                    clock.instant_to_tdb(instant),
                    self.committed_checkpoint.plant.propellant_mass_kg,
                    self.committed_checkpoint.plant.h_rw_body_nms,
                    orientation,
                )
            {
                return res;
            }
        }
        if instant <= self.committed_checkpoint.instant {
            return (self.committed_checkpoint.plant, DVec3::ZERO);
        }
        let (idx_opt, _) = find_containing_segment(&self.speculative, instant);
        if let Some(idx) = idx_opt
            && let Some(entry) = self.speculative.get(idx)
        {
            if instant == entry.end_instant {
                return (entry.end_checkpoint.plant, DVec3::ZERO);
            }
            return entry
                .segment
                .sample_plant_and_accel(clock.instant_to_tdb(instant));
        }
        (self.frontier_checkpoint.plant, DVec3::ZERO)
    }

    pub(crate) fn rollback_to_instant(
        &mut self,
        target: SimInstant,
        clock: &SimClockAdapter,
        force_model: &ForceModel,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<RollbackMetrics, SchedulerError> {
        if target >= self.frontier_checkpoint.instant && self.speculative.is_empty() {
            return Ok(RollbackMetrics::default());
        }
        if self.speculative.is_empty() {
            if target < self.frontier_checkpoint.instant
                && self.committed_checkpoint.integrator_hint.propagator
                    == PropagatorKind::PinnedSurfaceRest
            {
                let cp = self
                    .committed_checkpoint
                    .at_pinned_surface_instant(target, clock, orientation)
                    .map_err(|source| SchedulerError::SurfaceRest {
                        vessel_id: self.committed_checkpoint.vessel_id,
                        source,
                    })?;
                self.committed_checkpoint = cp.clone();
                self.frontier_checkpoint = cp;
                return Ok(RollbackMetrics {
                    vessels_rolled_back: 1,
                    ..RollbackMetrics::default()
                });
            }
            return Ok(RollbackMetrics::default());
        }

        let (idx_opt, probes) = find_containing_segment(&self.speculative, target);
        if target <= self.committed_checkpoint.instant {
            let dropped = self.speculative.len();
            self.speculative.clear();
            self.frontier_checkpoint = self.committed_checkpoint.clone();
            return Ok(RollbackMetrics {
                vessels_rolled_back: usize::from(dropped > 0),
                segments_dropped: dropped,
                replayed_steps: 0,
                binary_search_probes: probes,
            });
        }

        let Some(idx) = idx_opt else {
            return Ok(RollbackMetrics {
                binary_search_probes: probes,
                ..RollbackMetrics::default()
            });
        };
        self.truncate_and_replay_from(
            idx,
            target,
            probes,
            clock,
            force_model,
            ephemeris,
            orientation,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn truncate_and_replay_from(
        &mut self,
        idx: usize,
        target: SimInstant,
        probes: usize,
        clock: &SimClockAdapter,
        force_model: &ForceModel,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<RollbackMetrics, SchedulerError> {
        let Some(containing) = self.speculative.get(idx).cloned() else {
            return Ok(RollbackMetrics::default());
        };
        if target == containing.end_instant {
            let keep = idx.wrapping_add(1);
            let dropped = self.speculative.len().saturating_sub(keep);
            self.speculative.truncate(keep);
            self.committed_checkpoint = containing.end_checkpoint.clone();
            self.speculative.clear();
            self.frontier_checkpoint = self.committed_checkpoint.clone();
            return Ok(RollbackMetrics {
                vessels_rolled_back: usize::from(dropped > 0),
                segments_dropped: dropped,
                replayed_steps: 0,
                binary_search_probes: probes,
            });
        }
        let dropped = self.speculative.len().saturating_sub(idx);
        self.speculative.truncate(idx);
        for entry in self.speculative.drain(..) {
            self.committed_checkpoint = entry.end_checkpoint;
        }
        if target == containing.start_instant {
            self.committed_checkpoint = containing.start_checkpoint.clone();
            self.frontier_checkpoint = containing.start_checkpoint;
            return Ok(RollbackMetrics {
                vessels_rolled_back: 1,
                segments_dropped: dropped,
                replayed_steps: 0,
                binary_search_probes: probes,
            });
        }

        let out = propagate_checkpoint_interval(
            &containing.start_checkpoint,
            clock.instant_to_tdb(containing.start_instant),
            clock.instant_to_tdb(target),
            Some(containing.segment.duration_s),
            &self.config.tolerances,
            &self.config,
            force_model,
            ephemeris,
            orientation,
        )?;
        let cp = containing.start_checkpoint.advanced_continuous(
            target,
            out.final_state,
            out.stats.next_h_hint_s,
            out.audit,
        );
        self.committed_checkpoint = cp.clone();
        self.frontier_checkpoint = cp;
        Ok(RollbackMetrics {
            vessels_rolled_back: 1,
            segments_dropped: dropped,
            replayed_steps: out.stats.accepted_steps,
            binary_search_probes: probes,
        })
    }
}

fn find_containing_segment(
    segs: &[SpeculativeSegmentEntry],
    target: SimInstant,
) -> (Option<usize>, usize) {
    let mut lo: usize = 0;
    let mut hi: usize = segs.len();
    let mut probes: usize = 0;
    while lo < hi {
        probes = probes.wrapping_add(1);
        let mid = lo + (hi - lo) / 2;
        let Some(entry) = segs.get(mid) else {
            break;
        };
        if target < entry.start_instant {
            hi = mid;
        } else if target > entry.end_instant {
            lo = mid.wrapping_add(1);
        } else {
            return (Some(mid), probes);
        }
    }
    (None, probes)
}
