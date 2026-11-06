use sim_core::{SimInstant, StampedResponse};

use super::MavlinkHilConfig;
use crate::mavlink_hil::clock_sync::{ActuatorVerdict, HilCorrelationTracker, HilLinkStats};
use crate::mavlink_hil::messages::HilActuatorControls;
use crate::mavlink_hil::reader::{MavlinkHilFrame, MavlinkHilReader};

/// The inbound half every transport shares: frame parsing, the peer's armed
/// state, and reply correlation against the sensor frames sent.
#[derive(Debug)]
pub(super) struct PeerLink {
    pub(super) reader: MavlinkHilReader,
    pub(super) peer_armed: bool,
    tracker: HilCorrelationTracker,
}

impl PeerLink {
    pub(super) fn new(config: &MavlinkHilConfig) -> Self {
        Self {
            reader: MavlinkHilReader::new(),
            peer_armed: false,
            tracker: HilCorrelationTracker::new(
                config.clock_resync_threshold,
                config.max_reply_age,
            ),
        }
    }

    pub(super) fn note_sensor_sent(&mut self, at: SimInstant) {
        self.tracker.on_sensor_sent(at);
    }

    /// The next fresh actuator frame already parsed, stamped on the master
    /// clock; stale frames are consumed and counted, never returned.
    pub(super) fn next_fresh_controls(&mut self) -> Option<StampedResponse<HilActuatorControls>> {
        while let Some(frame) = self.reader.pop_frame() {
            match frame {
                MavlinkHilFrame::Heartbeat { armed, .. } => {
                    self.peer_armed = armed;
                }
                MavlinkHilFrame::ActuatorControls { controls, .. } => {
                    if let ActuatorVerdict::Accepted { deliver_at, .. } =
                        self.tracker.on_actuator(controls.time_usec)
                    {
                        self.peer_armed = controls.is_armed();
                        return Some(StampedResponse {
                            deliver_at,
                            response: controls,
                        });
                    }
                }
            }
        }
        None
    }

    pub(super) const fn stats(&self) -> HilLinkStats {
        self.tracker.stats()
    }
}
