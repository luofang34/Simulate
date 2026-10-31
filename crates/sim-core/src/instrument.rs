use crate::{SimInstant, TickSpan};

#[cfg(test)]
mod tests;

/// How far a cockpit reading can be trusted, in the spirit of `Indicate`'s
/// source monitors: a display shows the reading and its health together, so a
/// frozen or failed source never passes for a live one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceValidity {
    /// Live, from the primary source, inside its limits.
    Valid,
    /// Live but of reduced quality, such as a backup source or a saturated
    /// sensor.
    Degraded,
    /// Was valid, but the source has stopped updating.
    Stale,
    /// The source has nothing to report: out of range, or never heard from.
    Missing,
    /// The source has failed and its value must not be used.
    Failed,
}

impl SourceValidity {
    /// Whether a display or a control loop may use the value.
    pub const fn is_usable(self) -> bool {
        matches!(self, Self::Valid | Self::Degraded)
    }
}

/// A reading with the instant it was taken and the health of its source.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StampedReading<T> {
    /// The reading. Holds the last value when the source is not usable.
    pub value: T,
    /// When the source produced it.
    pub timestamp: SimInstant,
    /// The source's health.
    pub validity: SourceValidity,
}

impl<T> StampedReading<T> {
    /// A reading taken `timestamp`.
    pub const fn new(value: T, timestamp: SimInstant, validity: SourceValidity) -> Self {
        Self {
            value,
            timestamp,
            validity,
        }
    }

    /// This reading as seen at `now`: one older than `max_age` that would
    /// otherwise pass as usable is [`SourceValidity::Stale`].
    #[must_use]
    pub fn aged(mut self, now: SimInstant, max_age: TickSpan) -> Self {
        if self.validity.is_usable()
            && now > self.timestamp
            && now.duration_since(self.timestamp) > max_age
        {
            self.validity = SourceValidity::Stale;
        }
        self
    }
}
