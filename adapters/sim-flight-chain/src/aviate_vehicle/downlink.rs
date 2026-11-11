//! The flight controller's telemetry, through Pilotage's own receive path:
//! its codec parses the frames, its link cache applies the acquisition
//! discipline, and its sampling builds the samples the host reads.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use pilotage_adapter_api::TelemetrySample;
use pilotage_mavlink::codec::{FcMessage, FrameSource, parse_datagram};
use pilotage_mavlink::link::apply_messages_at;
use pilotage_mavlink::link::estimator::QUALITY_DEGRADED;
use pilotage_mavlink::sampling::{
    ArmTracker, OperationalPose, estimate_batch, fc_state_sample, operational_pose,
    report_only_sample,
};
use pilotage_mavlink::{LinkState, ResetPolicy};
use pilotage_protocol::VehicleId;
use pilotage_timing::SimTick;
use sim_core::SimInstant;

/// Source-clock lag admitted between the attitude and position streams,
/// which Aviate publishes at different rates.
const INTER_GROUP_SKEW_MS: u32 = 300;
/// Attitude, as Pilotage numbers the estimate groups.
const ATTITUDE_VALID: u32 = 0b0001;
/// How far ahead of the wall clock received frames are stamped.
const RECEIVE_LEAD: Duration = Duration::from_secs(10 * 365 * 86_400);

/// Pilotage's link cache, fed from the flight controller's transmit ring.
pub(super) struct Downlink {
    state: Arc<Mutex<LinkState>>,
    /// Receive instant of a frame arriving at [`SimInstant::START`].
    ///
    /// Pilotage's sampling withholds a group three wall-clock seconds after it
    /// was received. A run is as fast or as slow as its host, so receive
    /// stamps are placed ahead of the wall clock, where they never age, and
    /// staleness is judged on the simulation clock by whoever reads the
    /// measurement stamps.
    epoch: Instant,
    arm: ArmTracker,
    messages: Vec<(FrameSource, FcMessage)>,
}

impl Downlink {
    pub(super) fn new() -> Self {
        let now = Instant::now();
        let epoch = now.checked_add(RECEIVE_LEAD).unwrap_or(now);
        let state = LinkState {
            started_at: epoch,
            reset_policy: ResetPolicy::Conservative,
            maximum_inter_group_skew_ms: INTER_GROUP_SKEW_MS,
            ..LinkState::default()
        };
        Self {
            state: Arc::new(Mutex::new(state)),
            epoch,
            arm: ArmTracker::default(),
            messages: Vec::new(),
        }
    }

    /// Folds the frames in `bytes`, received at `at`, into the cache.
    pub(super) fn receive(&mut self, bytes: &[u8], at: SimInstant) {
        self.messages.clear();
        let stats = parse_datagram(bytes, &mut self.messages);
        let elapsed = Duration::from_secs_f64(at.duration_since(SimInstant::START).as_seconds());
        let received = self.epoch.checked_add(elapsed).unwrap_or(self.epoch);
        apply_messages_at(
            &self.state,
            &self.messages,
            stats.crc_failures,
            stats.unknown_ids,
            received,
        );
    }

    /// The flight controller's authorized pose, when it reports one.
    pub(super) fn pose(&self) -> Option<OperationalPose> {
        let latest = self.state.lock().ok()?;
        operational_pose(&latest)
    }

    /// Whether the flight controller authorizes its attitude estimate in
    /// the present source epoch, whatever it says of its position.
    pub(super) fn attitude_authorized(&self) -> bool {
        let Ok(latest) = self.state.lock() else {
            return false;
        };
        let current = |epoch: u32| epoch == latest.source_epoch;
        let status = latest.estimator_status_stamp();
        latest.attitude.is_some_and(|attitude| {
            attitude.valid_flags & ATTITUDE_VALID != 0
                && attitude.quality <= QUALITY_DEGRADED
                && current(attitude.stamp.source_epoch)
                && status.is_some_and(|stamp| current(stamp.source_epoch))
        })
    }

    /// Whether the flight controller's last heartbeat reported it armed.
    pub(super) fn reports_armed(&self) -> Option<bool> {
        self.state.lock().ok()?.heartbeat_armed
    }

    /// The estimate and the flight controller's own state as one sample.
    pub(super) fn sample(&mut self, vehicle: VehicleId, tick: SimTick) -> TelemetrySample {
        let mut sample = estimate_batch(vehicle, &self.state)
            .samples
            .pop()
            .unwrap_or_else(|| report_only_sample(vehicle, tick));
        sample.tick = tick;
        if let Ok(latest) = self.state.lock() {
            self.arm.observe(&latest);
        }
        sample.fc_state = fc_state_sample(self.arm.report(), None, None);
        sample
    }
}
