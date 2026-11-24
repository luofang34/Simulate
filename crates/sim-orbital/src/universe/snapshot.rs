//! Manifest-verified save/restore snapshots (`UniverseSnapshot`) for [`crate::OrbitalUniverse`].

use serde::{Deserialize, Serialize};
use sim_spacetime::SimInstant;

use crate::{
    checkpoint::{VesselCheckpoint, VesselConfig, sim_instant_serde},
    events::{AuthoritativeCommandLog, ScheduledEvent},
    hash::{ContentHash, DomainHasher},
    ids::VesselId,
    universe::{
        OrbitalUniverse, SchedulerError, SpeculativeSegmentEntry,
        manifest::SimulationManifestDigest, rollback::VesselTimeline,
    },
};

/// Saved per-vessel configuration, committed checkpoint and speculative lookahead in a
/// [`UniverseSnapshot`].
///
/// A proposal integrates from its frontier to its horizon and clips its last step there, so
/// where later steps begin depends on where the buffered lookahead ends. The record keeps the
/// whole buffer and its frontier, which makes a restored vessel continue on exactly the step
/// boundaries the saved one would have.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VesselSnapshotRecord {
    /// Authoritative vessel identifier.
    pub vessel_id: VesselId,
    /// Static mass, propellant, reaction-wheel, and tolerance configuration.
    pub config: VesselConfig,
    /// Last completed step-boundary checkpoint at or before `committed_instant`.
    pub committed_checkpoint: VesselCheckpoint,
    /// Buffered speculative integration steps in time order, the first spanning
    /// `committed_instant` when the vessel is mid-step.
    pub speculative_segments: Vec<SpeculativeSegmentEntry>,
    /// Checkpoint the next speculative proposal starts from: the end of the last buffered
    /// segment, or further on for regimes that advance without dense segments.
    pub frontier_checkpoint: VesselCheckpoint,
}

/// Complete, serializable authoritative state of [`crate::OrbitalUniverse`] at `committed_instant`.
///
/// Identity boundaries:
/// - [`Self::physics_state_hash`]: instantaneous physical and actuator state of all vessels at `committed_instant`.
/// - [`Self::snapshot_hash`]: full replay identity covering `manifest`, `committed_instant`, `horizon_quantum_ticks`,
///   every vessel's `VesselConfig`, full `VesselCheckpoint`, every buffered speculative segment (bounds, checkpoints and
///   `PlantTrajectorySegment` polynomial coefficients) and frontier checkpoint,
///   all `pending_events` (keys, generation guards, and payloads), and the full `AuthoritativeCommandLog`.
///   Pure display/UI state (`active_camera_vessel`) is excluded from [`Self::snapshot_hash`].
/// - [`Self::display_state_hash`]: non-authoritative camera/view selection state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct UniverseSnapshot {
    /// Complete model manifest digest; `restore_snapshot` fails closed on any mismatch.
    pub manifest: SimulationManifestDigest,
    /// Global authoritative committed simulation timestamp.
    #[serde(with = "sim_instant_serde")]
    pub committed_instant: SimInstant,
    /// Canonical horizon quantum in master-clock ticks used for frame-independent speculation.
    pub horizon_quantum_ticks: u128,
    /// Canonical `VesselId`-sorted vessel records.
    pub vessels: Vec<VesselSnapshotRecord>,
    /// Pending future scheduled events in strict `EventOrderKey` order.
    pub pending_events: Vec<ScheduledEvent>,
    /// Authoritative stamped player command log.
    pub command_log: AuthoritativeCommandLog,
    /// Currently selected active camera vessel (pure UI/view state; never alters physics).
    pub active_camera_vessel: Option<VesselId>,
}

impl UniverseSnapshot {
    /// Canonical SHA-256 hash of instantaneous physical and actuator state across all vessels at `committed_instant`.
    #[must_use]
    pub fn physics_state_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:universe-physics-state:v1");
        h.write_hash(self.manifest.root_hash);
        h.write_u128(self.committed_instant.ticks_u128());
        h.write_u64(self.vessels.len() as u64);
        for v in &self.vessels {
            h.write_u64(v.vessel_id.0);
            h.write_hash(v.committed_checkpoint.physics_state_hash());
        }
        h.finish()
    }

    /// Canonical SHA-256 full replay snapshot identity covering all state, configuration,
    /// buffered speculative segments and frontier, pending events (including guards and payloads),
    /// and command logs that can affect future deterministic evolution. Equal hashes mean equal
    /// futures; two universes that buffered different lookahead hash differently even where
    /// their futures happen to agree. Excludes pure display state (`active_camera_vessel`).
    #[must_use]
    pub fn snapshot_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:universe-snapshot:v3");
        h.write_hash(self.manifest.root_hash);
        h.write_u128(self.committed_instant.ticks_u128());
        h.write_u128(self.horizon_quantum_ticks);
        h.write_u64(self.vessels.len() as u64);
        for v in &self.vessels {
            h.write_u64(v.vessel_id.0);
            v.config.write_to_hasher(&mut h);
            h.write_hash(v.committed_checkpoint.checkpoint_hash());
            h.write_u64(v.speculative_segments.len() as u64);
            for seg in &v.speculative_segments {
                h.write_u128(seg.start_instant.ticks_u128());
                h.write_u128(seg.end_instant.ticks_u128());
                h.write_hash(seg.start_checkpoint.checkpoint_hash());
                h.write_hash(seg.end_checkpoint.checkpoint_hash());
                h.write_hash(seg.segment.content_hash());
            }
            h.write_hash(v.frontier_checkpoint.checkpoint_hash());
        }
        h.write_u64(self.pending_events.len() as u64);
        for ev in &self.pending_events {
            ev.write_to_hasher(&mut h);
        }
        h.write_hash(self.command_log.content_hash());
        h.finish()
    }

    /// Canonical SHA-256 hash of non-authoritative display/camera state (`active_camera_vessel`).
    #[must_use]
    pub fn display_state_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:universe-display-state:v1");
        match self.active_camera_vessel {
            Some(vid) => {
                h.write_u8(1);
                h.write_u64(vid.0);
            }
            None => h.write_u8(0),
        }
        h.finish()
    }
}

impl OrbitalUniverse {
    /// Captures a complete, manifest-tagged [`UniverseSnapshot`] at `committed_instant`.
    #[must_use]
    pub fn capture_snapshot(&self) -> UniverseSnapshot {
        let vessels = self
            .vessels
            .values()
            .map(VesselTimeline::to_snapshot_record)
            .collect();
        let pending_events = self.event_queue.values().copied().collect();
        UniverseSnapshot {
            manifest: self.manifest,
            committed_instant: self.committed_instant,
            horizon_quantum_ticks: self.horizon_quantum_ticks,
            vessels,
            pending_events,
            command_log: self.command_log.clone(),
            active_camera_vessel: self.active_camera_vessel,
        }
    }

    /// Restores authoritative state from `snapshot` after verifying exact `SimulationManifestDigest` match.
    pub fn restore_snapshot(&mut self, snapshot: &UniverseSnapshot) -> Result<(), SchedulerError> {
        self.manifest.verify_matches(&snapshot.manifest)?;
        self.committed_instant = snapshot.committed_instant;
        self.horizon_quantum_ticks = snapshot.horizon_quantum_ticks.max(1);
        self.vessels.clear();
        for rec in &snapshot.vessels {
            self.vessels.insert(
                rec.vessel_id,
                VesselTimeline::from_snapshot_record(rec.clone()),
            );
        }
        self.event_queue.clear();
        for ev in &snapshot.pending_events {
            self.event_queue.insert(ev.key, *ev);
        }
        self.command_log = snapshot.command_log.clone();
        self.fired_events_log.clear();
        self.active_camera_vessel = snapshot.active_camera_vessel;
        Ok(())
    }

    /// Canonical SHA-256 content hash of the committed universe state.
    #[must_use]
    pub fn universe_hash(&self) -> ContentHash {
        self.capture_snapshot().snapshot_hash()
    }
}
