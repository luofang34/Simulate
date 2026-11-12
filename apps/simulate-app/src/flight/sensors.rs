//! The deterministic `30 Hz` optical sensor channel every flight exposes, clocked by
//! physics steps so a 30 FPS window, a 144 FPS window, and a headless run see the same
//! exposures.

use sim_core::{
    RateAdapter, SensorChannelId, SensorExposureEvent, SimInstant, TICKS_PER_SECOND, TickSpan,
    VehicleState,
};

/// Optical sensor frame rate.
pub const OPTICAL_SENSOR_HZ: u64 = 30;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(OPTICAL_SENSOR_HZ));
/// Channel of the window-independent machine-vision camera.
pub const OPTICAL_SENSOR_CHANNEL: SensorChannelId = SensorChannelId(0);
/// Exposure period of [`OPTICAL_SENSOR_CHANNEL`].
pub const OPTICAL_SENSOR_PERIOD: TickSpan =
    TickSpan::from_ticks(TICKS_PER_SECOND / OPTICAL_SENSOR_HZ);
/// Undrained exposures kept (`8 s` at `30 Hz`); a shell that stops draining loses the oldest.
pub const MAX_PENDING_EXPOSURES: usize = 240;

/// Sensor clock for scripted scenarios, which step outside a `SimSession`: each completed
/// physics step advances it by one plant period and every due exposure is stamped at the
/// step's exact end with the step's uninterpolated truth state.
#[derive(Debug, Clone, Copy)]
pub(super) struct ScenarioSensorClock {
    rate: RateAdapter,
    sequence: u64,
}

impl ScenarioSensorClock {
    pub(super) const fn new() -> Self {
        Self {
            rate: RateAdapter::new(OPTICAL_SENSOR_PERIOD),
            sequence: 0,
        }
    }

    /// Re-phases after a time jump, keeping sequence numbers monotonic.
    pub(super) const fn rephase(&mut self) {
        self.rate = RateAdapter::new(OPTICAL_SENSOR_PERIOD);
    }

    /// Records one physics step of `period` ending at `at` in state `state`.
    pub(super) fn on_step(
        &mut self,
        period: TickSpan,
        at: SimInstant,
        state: VehicleState,
        out: &mut Vec<SensorExposureEvent>,
    ) {
        for _ in 0..self.rate.steps_for(period) {
            out.push(SensorExposureEvent {
                channel: OPTICAL_SENSOR_CHANNEL,
                sequence: self.sequence,
                exposure_at: at,
                state,
            });
            self.sequence = self.sequence.wrapping_add(1);
        }
    }
}

/// Drops the oldest exposures beyond [`MAX_PENDING_EXPOSURES`], returning how many.
pub(super) fn trim_pending(pending: &mut Vec<SensorExposureEvent>) -> usize {
    let excess = pending.len().saturating_sub(MAX_PENDING_EXPOSURES);
    if excess > 0 {
        pending.drain(..excess);
    }
    excess
}

impl super::Flight {
    /// Hands over the `30 Hz` optical sensor exposures emitted since the last call, oldest
    /// first, each carrying the exact exposure instant and truth state (never the
    /// display-interpolated pose).
    pub fn drain_sensor_exposures(&mut self) -> Vec<SensorExposureEvent> {
        let drained = std::mem::take(&mut self.exposures);
        self.exposures_emitted = self
            .exposures_emitted
            .wrapping_add(u64::try_from(drained.len()).unwrap_or(u64::MAX));
        drained
    }

    /// Exposures handed over by [`Self::drain_sensor_exposures`] since the app started.
    #[cfg(test)]
    #[must_use]
    pub const fn sensor_exposures_drained(&self) -> u64 {
        self.exposures_emitted
    }
}
