//! Cockpit-panel state the airframe model draws and its gauges read:
//! instrument pointer poses, and each engine's and fuel tank's readings.

/// Pointers one instrument-pack layout slot may carry: as many as one of
/// Indicate's analog dials holds, an altimeter's three hands and a spare.
pub const NEEDLE_HANDS_PER_SLOT: usize = 4;
/// Instrument-pack layout slots whose needles an airframe can draw.
pub const NEEDLE_SLOTS: usize = 24;

const CHANNELS: usize = NEEDLE_SLOTS * NEEDLE_HANDS_PER_SLOT;
/// Digit wheels of the drum window one layout slot may carry, such as a
/// Kollsman window.
pub const DRUM_WHEELS_PER_SLOT: usize = 6;
const WHEELS: usize = NEEDLE_SLOTS * DRUM_WHEELS_PER_SLOT;
/// Moving cards one layout slot may carry, such as a gyro horizon's bar.
pub const CARDS_PER_SLOT: usize = 2;
const CARDS: usize = NEEDLE_SLOTS * CARDS_PER_SLOT;

/// Pointer poses of an instrument panel, keyed by `(layout slot, hand)` in
/// the order of the instrument pack's `layout`, a hand being the index of the
/// pointer in its dial. Each angle is rad clockwise from twelve o'clock as the
/// pilot sees the face, the whole rotation a drawn needle needs. A channel
/// nobody drives reads zero and is shown. Each slot's drum wheels carry
/// their position in digits, and each slot its failure flag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InstrumentNeedles {
    angle_rad: [f32; CHANNELS],
    /// One bit per channel whose pointer is not drawn.
    hidden: u128,
    wheels: [f32; WHEELS],
    /// One bit per slot whose drum shows no digits.
    drum_blank: u32,
    /// One bit per slot whose failure flag shows.
    flagged: u32,
    /// Each card's turn, rad clockwise as the pilot sees the face, and its
    /// travel down the turned card, m.
    cards: [[f32; 2]; CARDS],
    /// One bit per card that is not drawn.
    cards_hidden: u64,
}

const _: () = assert!(CHANNELS <= u128::BITS as usize);
const _: () = assert!(NEEDLE_SLOTS <= u32::BITS as usize);
const _: () = assert!(CARDS <= u64::BITS as usize);

impl Default for InstrumentNeedles {
    fn default() -> Self {
        Self {
            angle_rad: [0.0; CHANNELS],
            hidden: 0,
            wheels: [0.0; WHEELS],
            drum_blank: 0,
            flagged: 0,
            cards: [[0.0; 2]; CARDS],
            cards_hidden: 0,
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

    /// Shows or hides `hand` on layout `slot`; ignored beyond capacity.
    pub fn set_visible(&mut self, slot: usize, hand: usize, visible: bool) {
        if let Some(channel) = Self::channel(slot, hand) {
            let bit = 1_u128 << channel;
            if visible {
                self.hidden &= !bit;
            } else {
                self.hidden |= bit;
            }
        }
    }

    /// Whether `channel`'s pointer is drawn; a channel beyond capacity is.
    #[must_use]
    pub const fn visible(&self, channel: usize) -> bool {
        channel >= CHANNELS || self.hidden & (1_u128 << channel) == 0
    }

    /// The channel of drum `wheel` (rightmost first) on layout `slot`, or
    /// `None` beyond the panel's capacity.
    #[must_use]
    pub const fn wheel_channel(slot: usize, wheel: usize) -> Option<usize> {
        if slot < NEEDLE_SLOTS && wheel < DRUM_WHEELS_PER_SLOT {
            Some(slot * DRUM_WHEELS_PER_SLOT + wheel)
        } else {
            None
        }
    }

    /// The position of wheel `channel` in digits: digit `floor(p)` turning
    /// toward the next by the fraction; zero beyond capacity.
    #[must_use]
    pub fn wheel(&self, channel: usize) -> f64 {
        self.wheels.get(channel).copied().map_or(0.0, f64::from)
    }

    /// Sets drum `wheel` on layout `slot` to `position` digits; ignored
    /// beyond capacity or when not finite.
    pub fn set_wheel(&mut self, slot: usize, wheel: usize, position: f64) {
        if let Some(channel) = Self::wheel_channel(slot, wheel)
            && let Some(cell) = self.wheels.get_mut(channel)
            && position.is_finite()
        {
            *cell = position as f32;
        }
    }

    /// Whether `slot`'s drum shows digits; one beyond capacity does.
    #[must_use]
    pub const fn drum_live(&self, slot: usize) -> bool {
        slot >= NEEDLE_SLOTS || self.drum_blank & (1_u32 << slot) == 0
    }

    /// Whether `slot`'s failure flag shows; one beyond capacity does not.
    #[must_use]
    pub const fn flagged(&self, slot: usize) -> bool {
        slot < NEEDLE_SLOTS && self.flagged & (1_u32 << slot) != 0
    }

    /// Sets whether `slot`'s drum shows digits and its flag shows; ignored
    /// beyond capacity.
    pub fn set_slot_state(&mut self, slot: usize, drum_live: bool, flagged: bool) {
        if slot < NEEDLE_SLOTS {
            let bit = 1_u32 << slot;
            self.drum_blank = if drum_live {
                self.drum_blank & !bit
            } else {
                self.drum_blank | bit
            };
            self.flagged = if flagged {
                self.flagged | bit
            } else {
                self.flagged & !bit
            };
        }
    }
}

impl InstrumentNeedles {
    /// The channel of `card` on layout `slot`, or `None` beyond capacity.
    #[must_use]
    pub const fn card_channel(slot: usize, card: usize) -> Option<usize> {
        if slot < NEEDLE_SLOTS && card < CARDS_PER_SLOT {
            Some(slot * CARDS_PER_SLOT + card)
        } else {
            None
        }
    }

    /// Card `channel`'s turn, rad, and travel down the turned card, m; zero
    /// beyond capacity.
    #[must_use]
    pub fn card(&self, channel: usize) -> (f64, f64) {
        self.cards
            .get(channel)
            .map_or((0.0, 0.0), |[turn, travel]| {
                (f64::from(*turn), f64::from(*travel))
            })
    }

    /// Whether card `channel` is drawn; one beyond capacity is.
    #[must_use]
    pub const fn card_visible(&self, channel: usize) -> bool {
        channel >= CARDS || self.cards_hidden & (1_u64 << channel) == 0
    }

    /// Sets `card` on layout `slot` to `turn_rad` and `travel_m`, shown or
    /// not; ignored beyond capacity or when either is not finite.
    pub fn set_card(
        &mut self,
        slot: usize,
        card: usize,
        turn_rad: f64,
        travel_m: f64,
        visible: bool,
    ) {
        let Some(channel) = Self::card_channel(slot, card) else {
            return;
        };
        if let Some(cell) = self.cards.get_mut(channel)
            && turn_rad.is_finite()
            && travel_m.is_finite()
        {
            *cell = [turn_rad as f32, travel_m as f32];
        }
        let bit = 1_u64 << channel;
        self.cards_hidden = if visible {
            self.cards_hidden & !bit
        } else {
            self.cards_hidden | bit
        };
    }
}

/// Fuel tanks whose contents an airframe reports.
pub const FUEL_TANKS: usize = 4;

/// What one engine's gauges read from the plant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineReadout {
    /// Crankshaft speed, rpm.
    pub rpm: f64,
    /// Intake manifold absolute pressure, Pa.
    pub manifold_pressure_pa: f64,
    /// Fuel consumption, US gallons per hour.
    pub fuel_flow_gph: f64,
    /// Exhaust gas temperature, °F.
    pub egt_deg_f: f64,
    /// Cylinder head temperature, °F.
    pub cht_deg_f: f64,
    /// Oil temperature, °F.
    pub oil_temp_deg_f: f64,
    /// Oil pressure, psi.
    pub oil_pressure_psi: f64,
}

impl Default for EngineReadout {
    fn default() -> Self {
        Self {
            rpm: 0.0,
            manifold_pressure_pa: 101_325.0,
            fuel_flow_gph: 0.0,
            egt_deg_f: 70.0,
            cht_deg_f: 70.0,
            oil_temp_deg_f: 70.0,
            oil_pressure_psi: 0.0,
        }
    }
}

/// The engine and fuel readings a cockpit's gauges show, for up to a twin.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EngineGauges {
    /// The left, or only, engine first. A single engine's second entry
    /// mirrors its first gauges but burns no fuel.
    pub engines: [EngineReadout; 2],
    /// Contents of each fuel tank in the flight model's order, lb; `None`
    /// past the airframe's last tank.
    pub fuel_tanks_lbs: [Option<f64>; FUEL_TANKS],
}

impl EngineGauges {
    /// Fuel consumption of every engine together, US gallons per hour.
    #[must_use]
    pub fn total_fuel_flow_gph(&self) -> f64 {
        self.engines.iter().map(|e| e.fuel_flow_gph).sum()
    }
}

#[cfg(test)]
mod tests;
