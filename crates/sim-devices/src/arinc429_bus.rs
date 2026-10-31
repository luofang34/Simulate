use std::collections::VecDeque;

use mbd_core::DigitalBusFrame;
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan};

use crate::DeviceError;

#[cfg(test)]
mod tests;

/// Bits in an ARINC 429 word.
pub const ARINC429_WORD_BITS: u64 = 32;
const HIGH_SPEED_BPS: u64 = 100_000;
const LOW_SPEED_BPS: u64 = 12_500;
// A bit rate that does not divide the master clock would round every word.
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(HIGH_SPEED_BPS));
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(LOW_SPEED_BPS));

/// An ARINC 429 bit rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arinc429Speed {
    /// 100 kbps; a word takes 320 µs.
    High,
    /// 12.5 kbps; a word takes 2.56 ms.
    Low,
}

impl Arinc429Speed {
    /// One bit on the wire.
    pub const fn bit_time(self) -> TickSpan {
        TickSpan::from_ticks(TICKS_PER_SECOND / self.bits_per_second())
    }

    /// `bits` bits on the wire.
    pub const fn bits_time(self, bits: u64) -> TickSpan {
        TickSpan::from_ticks_u128(self.bit_time().ticks_u128() * bits as u128)
    }

    /// One whole word on the wire.
    pub const fn word_time(self) -> TickSpan {
        self.bits_time(ARINC429_WORD_BITS)
    }

    const fn bits_per_second(self) -> u64 {
        match self {
            Self::High => HIGH_SPEED_BPS,
            Self::Low => LOW_SPEED_BPS,
        }
    }
}

/// A word that crossed the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Arinc429Delivery {
    /// When its first bit went onto the wire.
    pub started_at: SimInstant,
    /// When its last bit arrived, the instant a receiver has the word.
    pub completed_at: SimInstant,
    /// The word.
    pub frame: DigitalBusFrame,
}

/// One ARINC 429 channel: words go out strictly one at a time, each taking an
/// exact whole number of master-clock ticks.
///
/// By default words run back to back, 32 bit times apart. The standard
/// requires at least 4 idle bits between words, which
/// [`Arinc429Bus::with_gap_bits`] adds. The whole bus is plain data; a clone
/// is its snapshot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Arinc429Bus {
    channel: u8,
    speed: Arinc429Speed,
    gap: TickSpan,
    wire_free_at: SimInstant,
    latest_request: Option<SimInstant>,
    on_wire: VecDeque<Arinc429Delivery>,
}

impl Arinc429Bus {
    /// An idle channel.
    pub fn new(channel: u8, speed: Arinc429Speed) -> Self {
        Self {
            channel,
            speed,
            gap: TickSpan::ZERO,
            wire_free_at: SimInstant::START,
            latest_request: None,
            on_wire: VecDeque::new(),
        }
    }

    /// The same channel, keeping `bits` idle bits between words.
    #[must_use]
    pub fn with_gap_bits(self, bits: u64) -> Self {
        Self {
            gap: self.speed.bits_time(bits),
            ..self
        }
    }

    /// Queues `frame` for transmission no earlier than `at`, returning when
    /// its last bit will arrive. Requests must come in time order.
    pub fn transmit(
        &mut self,
        at: SimInstant,
        frame: DigitalBusFrame,
    ) -> Result<SimInstant, DeviceError> {
        frame
            .validate()
            .map_err(|source| DeviceError::InvalidFrame { source })?;
        if !matches!(frame, DigitalBusFrame::Arinc429 { channel, .. } if channel == self.channel) {
            return Err(DeviceError::WrongChannel {
                channel: self.channel,
                frame: format!("{frame:?}"),
            });
        }
        if let Some(previous) = self.latest_request
            && at < previous
        {
            return Err(DeviceError::OutOfOrder { at, previous });
        }
        self.latest_request = Some(at);
        let started_at = at.max(self.wire_free_at);
        let completed_at = started_at.advanced(self.speed.word_time());
        self.wire_free_at = completed_at.advanced(self.gap);
        self.on_wire.push_back(Arinc429Delivery {
            started_at,
            completed_at,
            frame,
        });
        Ok(completed_at)
    }

    /// Hands over every word fully received by `now`, oldest first.
    pub fn take_due(&mut self, now: SimInstant) -> Vec<Arinc429Delivery> {
        let waiting = self
            .on_wire
            .iter()
            .take_while(|delivery| delivery.completed_at <= now)
            .count();
        self.on_wire.drain(..waiting).collect()
    }
}
