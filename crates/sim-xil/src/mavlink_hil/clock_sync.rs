use std::collections::VecDeque;

use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan};

#[cfg(test)]
mod tests;

const TICKS_PER_MICROSECOND: u64 = TICKS_PER_SECOND / 1_000_000;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(1_000_000));

/// Sensor frames remembered for reply correlation; older replies than this
/// many frames are counted as uncorrelated.
const SENT_FRAME_HISTORY: usize = 64;

/// Default peer-clock jump that forces a re-anchor: one second.
pub const DEFAULT_CLOCK_RESYNC_THRESHOLD: TickSpan = TickSpan::SECOND;

/// Suggested [`HilCorrelationTracker`] reply-age limit: 100 ms.
pub const DEFAULT_MAX_REPLY_AGE: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 10);

fn usec_to_ticks(usec: u64) -> u128 {
    u128::from(usec) * u128::from(TICKS_PER_MICROSECOND)
}

/// Maps a peer's `time_usec` clock (PX4 / ArduPilot boot-monotonic uptime, or
/// an echo of the simulator's own stamps) onto the master clock.
///
/// The first peer stamp is anchored to the simulation instant it arrives at;
/// later stamps map by their offset from that anchor, so a peer that booted
/// a day ago lands on the simulation timeline rather than a day ahead of it.
/// A stamp that moves backwards or forwards by more than the resync threshold
/// (a peer reboot, a paused process) re-anchors to the arrival instant, never
/// earlier than any instant already mapped, and counts a resync. Smaller
/// backward steps keep the anchor and map before the newest instant, which is
/// how a late, reordered frame reveals itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeerClockSynchronizer {
    resync_threshold_ticks: u128,
    anchor: Option<(u64, SimInstant)>,
    newest_peer_usec: u64,
    newest_mapped: SimInstant,
    clock_resyncs: u64,
}

impl PeerClockSynchronizer {
    /// An unanchored synchronizer re-anchoring on jumps beyond
    /// `resync_threshold`.
    #[must_use]
    pub const fn new(resync_threshold: TickSpan) -> Self {
        Self {
            resync_threshold_ticks: resync_threshold.ticks_u128(),
            anchor: None,
            newest_peer_usec: 0,
            newest_mapped: SimInstant::START,
            clock_resyncs: 0,
        }
    }

    /// Maps `peer_usec`, arriving when the simulation stands at `sim_now`.
    pub fn map(&mut self, peer_usec: u64, sim_now: SimInstant) -> SimInstant {
        let Some((peer_origin, sim_origin)) = self.anchor else {
            return self.reanchor(peer_usec, sim_now);
        };
        let jump_ticks = usec_to_ticks(peer_usec.abs_diff(self.newest_peer_usec));
        if jump_ticks > self.resync_threshold_ticks {
            self.clock_resyncs = self.clock_resyncs.wrapping_add(1);
            return self.reanchor(peer_usec, sim_now);
        }
        let mapped = if peer_usec >= peer_origin {
            sim_origin.advanced(TickSpan::from_ticks_u128(usec_to_ticks(
                peer_usec - peer_origin,
            )))
        } else {
            let back = usec_to_ticks(peer_origin - peer_usec);
            SimInstant::START.advanced(TickSpan::from_ticks_u128(
                sim_origin.ticks_u128().saturating_sub(back),
            ))
        };
        self.newest_peer_usec = self.newest_peer_usec.max(peer_usec);
        self.newest_mapped = self.newest_mapped.max(mapped);
        mapped
    }

    fn reanchor(&mut self, peer_usec: u64, sim_now: SimInstant) -> SimInstant {
        let sim_origin = sim_now.max(self.newest_mapped);
        self.anchor = Some((peer_usec, sim_origin));
        self.newest_peer_usec = peer_usec;
        self.newest_mapped = sim_origin;
        sim_origin
    }

    /// The current `(peer_origin_usec, sim_origin)` anchor, once a stamp has
    /// arrived.
    #[must_use]
    pub const fn anchor(&self) -> Option<(u64, SimInstant)> {
        self.anchor
    }

    /// Re-anchors forced by peer clock jumps (the first anchor is not one).
    #[must_use]
    pub const fn clock_resyncs(&self) -> u64 {
        self.clock_resyncs
    }
}

/// Link-quality counters of a MAVLink HIL endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HilLinkStats {
    /// `HIL_SENSOR` frames handed to the transport.
    pub sensor_frames_sent: u64,
    /// Accepted actuator frames matched to a remembered sensor frame.
    pub correlated_actuator_frames: u64,
    /// Accepted actuator frames stamped before every remembered sensor frame.
    pub uncorrelated_actuator_frames: u64,
    /// Actuator frames rejected as older than the reply-age limit or than a
    /// frame already accepted.
    pub stale_actuator_drops: u64,
    /// Peer clock re-anchors (see [`PeerClockSynchronizer`]).
    pub clock_resyncs: u64,
    /// Master-clock ticks from sending the sensor frame the newest accepted
    /// reply answers to the simulation instant it arrived at.
    pub last_round_trip_ticks: u64,
}

/// What [`HilCorrelationTracker::on_actuator`] decided about one reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorVerdict {
    /// The reply is fresh and should replace the controls in force.
    Accepted {
        /// The reply's peer stamp mapped onto the master clock.
        deliver_at: SimInstant,
        /// Sequence of the sensor frame it answers, if still remembered.
        answers_sensor: Option<u64>,
    },
    /// The reply is too old and must not overwrite fresher controls.
    Stale {
        /// The reply's peer stamp mapped onto the master clock.
        mapped_at: SimInstant,
        /// How far it lags the simulation instant it arrived at.
        age: TickSpan,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SentFrame {
    sequence: u64,
    sim_time: SimInstant,
}

/// Correlates `HIL_ACTUATOR_CONTROLS` replies with the `HIL_SENSOR` frames
/// that provoked them and rejects stale replies.
///
/// A transport only learns simulation time from the instants it sends at, so
/// "now" here is the instant of the newest sensor frame sent: a reply's age
/// is that instant minus the reply's mapped stamp, and its round trip is that
/// instant minus the send instant of the newest sensor frame stamped no later
/// than the reply, the one it can have answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HilCorrelationTracker {
    clock: PeerClockSynchronizer,
    max_reply_age: Option<TickSpan>,
    sent: VecDeque<SentFrame>,
    newest_accepted: Option<SimInstant>,
    stats: HilLinkStats,
}

impl HilCorrelationTracker {
    /// A tracker re-anchoring the peer clock on jumps beyond
    /// `resync_threshold` and, when `max_reply_age` is set, rejecting replies
    /// older than it.
    #[must_use]
    pub fn new(resync_threshold: TickSpan, max_reply_age: Option<TickSpan>) -> Self {
        Self {
            clock: PeerClockSynchronizer::new(resync_threshold),
            max_reply_age,
            sent: VecDeque::with_capacity(SENT_FRAME_HISTORY),
            newest_accepted: None,
            stats: HilLinkStats::default(),
        }
    }

    /// Records a sensor frame sent at `at` and returns its sequence number.
    pub fn on_sensor_sent(&mut self, at: SimInstant) -> u64 {
        let sequence = self.stats.sensor_frames_sent;
        if self.sent.len() >= SENT_FRAME_HISTORY {
            self.sent.pop_front();
        }
        self.sent.push_back(SentFrame {
            sequence,
            sim_time: at,
        });
        self.stats.sensor_frames_sent = sequence.wrapping_add(1);
        sequence
    }

    /// The simulation instant of the newest sensor frame sent.
    #[must_use]
    pub fn sim_now(&self) -> SimInstant {
        self.sent
            .back()
            .map_or(SimInstant::START, |frame| frame.sim_time)
    }

    /// Judges a reply stamped `peer_usec` on the peer's clock.
    pub fn on_actuator(&mut self, peer_usec: u64) -> ActuatorVerdict {
        let now = self.sim_now();
        let mapped_at = self.clock.map(peer_usec, now);
        self.stats.clock_resyncs = self.clock.clock_resyncs();
        let age = now.duration_since(mapped_at);
        let reordered = self.newest_accepted.is_some_and(|n| mapped_at < n);
        let too_old = self.max_reply_age.is_some_and(|max| age > max);
        if reordered || too_old {
            self.stats.stale_actuator_drops = self.stats.stale_actuator_drops.wrapping_add(1);
            return ActuatorVerdict::Stale { mapped_at, age };
        }
        self.newest_accepted = Some(mapped_at);
        let answered = self.sent.iter().rev().find(|f| f.sim_time <= mapped_at);
        if let Some(frame) = answered {
            self.stats.correlated_actuator_frames =
                self.stats.correlated_actuator_frames.wrapping_add(1);
            self.stats.last_round_trip_ticks = now.duration_since(frame.sim_time).ticks();
        } else {
            self.stats.uncorrelated_actuator_frames =
                self.stats.uncorrelated_actuator_frames.wrapping_add(1);
        }
        ActuatorVerdict::Accepted {
            deliver_at: mapped_at,
            answers_sensor: answered.map(|frame| frame.sequence),
        }
    }

    /// The peer clock mapping.
    #[must_use]
    pub const fn clock(&self) -> &PeerClockSynchronizer {
        &self.clock
    }

    /// The reply-age limit, if enforced.
    #[must_use]
    pub const fn max_reply_age(&self) -> Option<TickSpan> {
        self.max_reply_age
    }

    /// Link-quality counters.
    #[must_use]
    pub const fn stats(&self) -> HilLinkStats {
        self.stats
    }
}
