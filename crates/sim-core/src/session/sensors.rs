use crate::{SimInstant, TickSpan, VehicleState};

use super::{SessionError, interpolate_vehicle_state};

/// Identifies one fixed-rate sensor channel within a session.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SensorChannelId(pub u16);

/// One sensor exposure falling due on the physics master clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorExposureEvent {
    /// The channel that exposed.
    pub channel: SensorChannelId,
    /// Exposures of this channel before this one since it was registered.
    pub sequence: u64,
    /// The exact master-clock instant of the exposure.
    pub exposure_at: SimInstant,
    /// Vehicle state at `exposure_at`, interpolated within the physics step
    /// that spans it.
    pub state: VehicleState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SensorChannel {
    id: SensorChannelId,
    period: TickSpan,
    next_due: SimInstant,
    sequence: u64,
}

/// Fixed-rate sensor channels clocked by physics steps, never by display
/// redraws, so exposures are identical at any frame rate.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct SensorSchedule {
    channels: Vec<SensorChannel>,
}

impl SensorSchedule {
    /// A schedule with no channels.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers channel `id` exposing every `period`, first at
    /// `origin + period`.
    pub fn register(
        &mut self,
        id: SensorChannelId,
        period: TickSpan,
        origin: SimInstant,
    ) -> Result<(), SessionError> {
        if period == TickSpan::ZERO {
            return Err(SessionError::ZeroSensorPeriod { channel: id });
        }
        if self.channels.iter().any(|channel| channel.id == id) {
            return Err(SessionError::DuplicateSensorChannel { channel: id });
        }
        self.channels.push(SensorChannel {
            id,
            period,
            next_due: origin.advanced(period),
            sequence: 0,
        });
        Ok(())
    }

    /// Number of registered channels.
    #[must_use]
    pub fn len(&self) -> usize {
        self.channels.len()
    }

    /// Whether no channels are registered.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.channels.is_empty()
    }

    /// Re-phases every channel so its next exposure is one period after
    /// `origin`, keeping sequence numbers monotonic across the discontinuity.
    pub fn rephase(&mut self, origin: SimInstant) {
        for channel in &mut self.channels {
            channel.next_due = origin.advanced(channel.period);
        }
    }

    /// Re-phases every channel to `origin` and restarts sequence numbers.
    pub fn restart(&mut self, origin: SimInstant) {
        self.rephase(origin);
        for channel in &mut self.channels {
            channel.sequence = 0;
        }
    }

    /// Appends to `out` every exposure due in `(step_start, step_end]`, in
    /// channel-registration then time order.
    pub fn on_step(
        &mut self,
        step_start: SimInstant,
        step_end: SimInstant,
        prev: &VehicleState,
        curr: &VehicleState,
        out: &mut Vec<SensorExposureEvent>,
    ) {
        let step_ticks = step_end.duration_since(step_start).ticks_u128();
        for channel in &mut self.channels {
            while channel.next_due <= step_end {
                let into_step = channel.next_due.duration_since(step_start).ticks_u128();
                let phase = if step_ticks == 0 {
                    1.0
                } else {
                    into_step as f64 / step_ticks as f64
                };
                out.push(SensorExposureEvent {
                    channel: channel.id,
                    sequence: channel.sequence,
                    exposure_at: channel.next_due,
                    state: interpolate_vehicle_state(prev, curr, phase),
                });
                channel.sequence = channel.sequence.wrapping_add(1);
                channel.next_due = channel.next_due.advanced(channel.period);
            }
        }
    }
}
