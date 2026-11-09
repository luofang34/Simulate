//! Cockpit-panel state the airframe model draws: instrument needle angles and
//! the gauges of a second engine.

/// Hands one instrument-pack layout slot may drive; an altimeter has three.
pub const NEEDLE_HANDS_PER_SLOT: usize = 3;
/// Instrument-pack layout slots whose needles an airframe can draw.
pub const NEEDLE_SLOTS: usize = 16;

const CHANNELS: usize = NEEDLE_SLOTS * NEEDLE_HANDS_PER_SLOT;

/// Needle angles of an instrument panel, rad clockwise from each dial's zero
/// graduation as the pilot sees it, keyed by `(layout slot, hand)` in the
/// order of the instrument pack's `layout`. A channel nobody drives reads zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InstrumentNeedles {
    angle_rad: [f32; CHANNELS],
}

impl Default for InstrumentNeedles {
    fn default() -> Self {
        Self {
            angle_rad: [0.0; CHANNELS],
        }
    }
}

impl InstrumentNeedles {
    /// The channel of `hand` on layout `slot`, or `None` beyond the panel's capacity.
    #[must_use]
    pub const fn channel(slot: usize, hand: usize) -> Option<usize> {
        if slot < NEEDLE_SLOTS && hand < NEEDLE_HANDS_PER_SLOT {
            Some(slot * NEEDLE_HANDS_PER_SLOT + hand)
        } else {
            None
        }
    }

    /// The angle of `channel`, rad; zero for a channel beyond capacity.
    #[must_use]
    pub fn angle_rad(&self, channel: usize) -> f64 {
        self.angle_rad.get(channel).copied().map_or(0.0, f64::from)
    }

    /// Sets `hand` on layout `slot` to `angle_rad`; ignored beyond capacity or
    /// when the angle is not finite.
    pub fn set(&mut self, slot: usize, hand: usize, angle_rad: f64) {
        if let Some(channel) = Self::channel(slot, hand)
            && let Some(cell) = self.angle_rad.get_mut(channel)
            && angle_rad.is_finite()
        {
            *cell = angle_rad as f32;
        }
    }
}

/// Gauge readings of the second (starboard) engine of a twin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SecondEngineGauges {
    /// Crankshaft speed, rpm.
    pub engine_rpm: f64,
    /// Intake manifold absolute pressure, Pa.
    pub manifold_pressure_pa: f64,
}

impl Default for SecondEngineGauges {
    fn default() -> Self {
        Self {
            engine_rpm: 0.0,
            manifold_pressure_pa: 101_325.0,
        }
    }
}
