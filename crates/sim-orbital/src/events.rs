//! Strictly unique total event ordering, generation-guarded scheduled events,
//! and authoritative stamped player-command logs.

use serde::{Deserialize, Serialize};
use sim_spacetime::SimInstant;

use crate::{
    checkpoint::{LatchedBurnLaw, sim_instant_serde},
    hash::{ContentHash, DomainHasher},
    ids::{NaifId, VesselId},
};

/// Deterministic phase precedence ranks within a single `SimInstant` tick.
pub mod phase_ranks {
    /// Authoritative stamped player input latch (`highest` priority within a tick).
    pub const PLAYER_COMMAND: u8 = 10;
    /// Discrete GNC computer step / maneuver plan cursor advance.
    pub const GNC_DISCRETE_UPDATE: u8 = 20;
    /// Primary coordinate-origin (`SOI` / Hill-sphere) frame switch.
    pub const PRIMARY_FRAME_SWITCH: u8 = 30;
    /// Proximity envelope / ground-contact / docking transition.
    pub const PROXIMITY_OR_CONTACT: u8 = 40;
    /// Generic state-transition or root-found detector callback.
    pub const CUSTOM_DETECTOR: u8 = 50;
}

/// Strictly unique total ordering key for all discrete and root-found events across the universe.
///
/// Lexical ordering on `(instant, phase_rank, participants, detector_id, sequence_no)` guarantees
/// zero ties even when the same vessel emits multiple same-rank events on the exact same tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct EventOrderKey {
    /// Master-clock tick of the event.
    #[serde(with = "sim_instant_serde")]
    pub instant: SimInstant,
    /// Deterministic phase precedence within the same tick (`lower` fires first).
    pub phase_rank: u8,
    /// Canonical sorted vessel pair `[min_id, max_id]` (`[id, VesselId::NONE]` for single-vessel events).
    pub participants: [VesselId; 2],
    /// Registered static detector or subsystem slot identifier.
    pub detector_id: u32,
    /// Monotonic per-emitter sequence number disambiguating multiple same-tick emissions.
    pub sequence_no: u64,
}

impl EventOrderKey {
    /// Constructs a canonical single-vessel event key `[vessel_id, VesselId::NONE]`.
    #[must_use]
    pub const fn single(
        instant: SimInstant,
        phase_rank: u8,
        vessel_id: VesselId,
        detector_id: u32,
        sequence_no: u64,
    ) -> Self {
        Self {
            instant,
            phase_rank,
            participants: VesselId::canonical_pair(vessel_id, VesselId::NONE),
            detector_id,
            sequence_no,
        }
    }

    /// Constructs a canonical two-vessel interaction event key `[min(a, b), max(a, b)]`.
    #[must_use]
    pub const fn pair(
        instant: SimInstant,
        phase_rank: u8,
        vessel_a: VesselId,
        vessel_b: VesselId,
        detector_id: u32,
        sequence_no: u64,
    ) -> Self {
        Self {
            instant,
            phase_rank,
            participants: VesselId::canonical_pair(vessel_a, vessel_b),
            detector_id,
            sequence_no,
        }
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        let [p0, p1] = self.participants;
        h.write_u128(self.instant.ticks_u128());
        h.write_u8(self.phase_rank);
        h.write_u64(p0.0);
        h.write_u64(p1.0);
        h.write_u32(self.detector_id);
        h.write_u64(self.sequence_no);
    }
}

/// Authoritative player or crew command payload stamped onto the simulation timeline.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PlayerCommandPayload {
    /// Latches a new continuous burn/steering law (`Coast`, `ConstantBody`, or `InertialThrust`).
    SetBurnLaw(LatchedBurnLaw),
    /// Switches coordinate origin to `new_primary` while preserving barycentric state.
    SwitchPrimary(NaifId),
    /// Updates the discrete GNC mode code.
    SetGncMode(u32),
    /// Commands aerodynamic fin deflections and pad/landing zero-lock flag.
    SetFins {
        /// Target fin deflections `Fin_0..3/Rudder` ($\text{rad}$).
        fin_angle_rad: [f64; 4],
        /// Whether fins are mechanically locked at `0°`.
        fins_locked_zero: bool,
    },
    /// Commands conformal nosecone `Carrier` and `Wrist` angles ($\text{rad}$).
    SetNosecone {
        /// `Carrier` hinge angle ($\text{rad}$).
        carrier_rad: f64,
        /// `Wrist` hinge angle ($\text{rad}$).
        wrist_rad: f64,
    },
}

impl PlayerCommandPayload {
    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::SetBurnLaw(ref law) => {
                h.write_u8(1);
                law.write_to_hasher(h);
            }
            Self::SwitchPrimary(body) => {
                h.write_u8(2);
                h.write_i32(body.0);
            }
            Self::SetGncMode(mode) => {
                h.write_u8(3);
                h.write_u32(mode);
            }
            Self::SetFins {
                fin_angle_rad,
                fins_locked_zero,
            } => {
                h.write_u8(4);
                for v in fin_angle_rad {
                    h.write_f64_canonical(v);
                }
                h.write_u8(u8::from(fins_locked_zero));
            }
            Self::SetNosecone {
                carrier_rad,
                wrist_rad,
            } => {
                h.write_u8(5);
                h.write_f64_canonical(carrier_rad);
                h.write_f64_canonical(wrist_rad);
            }
        }
    }
}

/// Event action executed when a [`ScheduledEvent`] fires at `committed_instant`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum UniverseEventPayload {
    /// Applies a stamped player command to `participants[0]`.
    PlayerCommand(PlayerCommandPayload),
    /// Runs one discrete GNC tick on `participants[0]` (draws 1 deterministic PRNG sample for audit/noise and schedules next GNC tick).
    DiscreteGncUpdate,
    /// Switches `participants[0]`'s coordinate origin to `new_primary`.
    SwitchPrimaryBody {
        /// Target `NaifId` coordinate origin.
        new_primary: NaifId,
    },
    /// Discrete state-machine transition on `participants[0]`; if `reemit_same_tick` is `true`,
    /// re-enqueues another same-tick transition (used to verify both valid chains and zero-time loop caps).
    ModeTransition {
        /// New discrete mode code.
        next_mode_code: u32,
        /// Remaining chained same-tick transitions to emit (`u32::MAX` for an unbounded pathological loop).
        remaining_chain: u32,
    },
    /// Records a pure observation / detector trigger without invalidating trajectory segments (`state_gen` unchanged).
    Marker {
        /// User/scenario marker tag.
        tag: u32,
    },
}

impl UniverseEventPayload {
    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        match *self {
            Self::PlayerCommand(ref cmd) => {
                h.write_u8(1);
                cmd.write_to_hasher(h);
            }
            Self::DiscreteGncUpdate => h.write_u8(2),
            Self::SwitchPrimaryBody { new_primary } => {
                h.write_u8(3);
                h.write_i32(new_primary.0);
            }
            Self::ModeTransition {
                next_mode_code,
                remaining_chain,
            } => {
                h.write_u8(4);
                h.write_u32(next_mode_code);
                h.write_u32(remaining_chain);
            }
            Self::Marker { tag } => {
                h.write_u8(5);
                h.write_u32(tag);
            }
        }
    }
}

/// Generation-guarded event entry in [`crate::OrbitalUniverse`]'s authoritative priority queue.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScheduledEvent {
    /// Unique total-order key.
    pub key: EventOrderKey,
    /// Expected `(VesselId, state_gen)` preconditions; if any participant's current `state_gen`
    /// differs, this event came from a superseded trajectory and is deterministically skipped.
    pub expected_generations: [Option<(VesselId, u64)>; 2],
    /// Event payload.
    pub payload: UniverseEventPayload,
}

impl ScheduledEvent {
    /// Single-vessel event guarded by `expected_state_gen` (or `None` for unguarded player commands).
    #[must_use]
    pub const fn single(
        key: EventOrderKey,
        vessel_id: VesselId,
        expected_state_gen: Option<u64>,
        payload: UniverseEventPayload,
    ) -> Self {
        let guard = match expected_state_gen {
            Some(g) => Some((vessel_id, g)),
            None => None,
        };
        Self {
            key,
            expected_generations: [guard, None],
            payload,
        }
    }

    /// Canonical SHA-256 content hash of the full scheduled event (`key`, `expected_generations`, and `payload`).
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:scheduled-event:v1");
        self.write_to_hasher(&mut h);
        h.finish()
    }

    pub(crate) fn write_to_hasher(&self, h: &mut DomainHasher) {
        self.key.write_to_hasher(h);
        for guard in self.expected_generations {
            match guard {
                Some((vid, state_gen)) => {
                    h.write_u8(1);
                    h.write_u64(vid.0);
                    h.write_u64(state_gen);
                }
                None => h.write_u8(0),
            }
        }
        self.payload.write_to_hasher(h);
    }
}

/// Authoritative timeline-stamped player command recorded for deterministic replay.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StampedPlayerCommand {
    /// Monotonic universe-wide command sequence number.
    pub command_seq: u64,
    /// Authoritative simulation instant at which the command takes effect.
    #[serde(with = "sim_instant_serde")]
    pub instant: SimInstant,
    /// Target vessel identifier.
    pub vessel_id: VesselId,
    /// Command payload.
    pub payload: PlayerCommandPayload,
}

/// Append-only log of stamped player commands for save files and deterministic replay.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AuthoritativeCommandLog {
    entries: Vec<StampedPlayerCommand>,
    next_seq: u64,
}

impl AuthoritativeCommandLog {
    /// Allocates the next command sequence number, appends to the log, and returns the stamped entry.
    pub fn record(
        &mut self,
        instant: SimInstant,
        vessel_id: VesselId,
        payload: PlayerCommandPayload,
    ) -> StampedPlayerCommand {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        let cmd = StampedPlayerCommand {
            command_seq: seq,
            instant,
            vessel_id,
            payload,
        };
        self.entries.push(cmd);
        cmd
    }

    /// Recorded stamped commands in exact emission order.
    #[must_use]
    pub fn entries(&self) -> &[StampedPlayerCommand] {
        &self.entries
    }

    /// Canonical SHA-256 digest of the command log entries, sequence metadata, and full payloads.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:command-log:v2");
        h.write_u64(self.next_seq);
        h.write_u64(self.entries.len() as u64);
        for cmd in &self.entries {
            h.write_u64(cmd.command_seq);
            h.write_u128(cmd.instant.ticks_u128());
            h.write_u64(cmd.vessel_id.0);
            cmd.payload.write_to_hasher(&mut h);
        }
        h.finish()
    }
}
