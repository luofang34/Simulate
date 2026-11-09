#[cfg(test)]
mod tests;

/// One raw frame from a simulated detector: digital numbers as the sensor's
/// ADC writes them, with nothing of the simulation's truth in them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SensorCounts {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// ADC resolution: every count is below `2^significant_bits`.
    pub significant_bits: u8,
    /// Row-major counts, top row first.
    pub counts: Vec<u16>,
}

impl SensorCounts {
    /// The largest count the ADC can write.
    pub fn full_scale(&self) -> u16 {
        full_scale(self.significant_bits)
    }

    /// The count at pixel `(x, y)`, or `None` outside the image.
    pub fn count(&self, x: u32, y: u32) -> Option<u16> {
        if x >= self.width || y >= self.height {
            return None;
        }
        self.counts
            .get(y as usize * self.width as usize + x as usize)
            .copied()
    }
}

/// The largest count of a `bits`-bit ADC; sixteen bits at most.
pub(super) fn full_scale(bits: u8) -> u16 {
    match bits {
        0 => 0,
        1..=15 => (1_u16 << bits) - 1,
        _ => u16::MAX,
    }
}

/// Rounds an analogue level to the nearest count an ADC of `full_scale` can
/// write, clipping below zero and above full scale.
pub(super) fn digitise(level_dn: f64, full_scale: u16) -> u16 {
    if level_dn.is_nan() {
        return 0;
    }
    level_dn.round().clamp(0.0, f64::from(full_scale)) as u16
}
