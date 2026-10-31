/// A UTC instant as seconds since the Unix epoch.
///
/// Weather fields, ephemerides, and NOTAM validity are all keyed by wall-clock
/// UTC, independent of how fast or how deterministically the simulation steps.
/// `f64` seconds resolve better than a microsecond for centuries around now.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct UtcTime {
    unix_seconds: f64,
}

impl UtcTime {
    /// An instant `unix_seconds` after 1970-01-01T00:00:00Z.
    pub fn from_unix_seconds(unix_seconds: f64) -> Self {
        Self { unix_seconds }
    }

    /// Seconds since the Unix epoch.
    pub fn unix_seconds(&self) -> f64 {
        self.unix_seconds
    }

    /// This instant advanced by `seconds` (negative moves backwards).
    pub fn advanced_by(&self, seconds: f64) -> Self {
        Self {
            unix_seconds: self.unix_seconds + seconds,
        }
    }
}
