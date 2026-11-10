//! A pack's whole panel of dials, and the state its mechanisms feed them.

use indicate_instrument_analog::DialPose;
use syspack::InstrumentPackDocument;

use super::{DialError, DialSource, GaugeState, PanelLettering, PointerValues};

/// One layout slot's dial.
#[derive(Debug, Clone, PartialEq)]
pub struct SlotDial {
    /// Index of the slot in the pack's `layout`.
    pub slot: usize,
    /// The slot's name, such as `flight_left`.
    pub name: String,
    /// The instrument's id.
    pub instrument: String,
    /// The dial.
    pub dial: DialSource,
}

/// Every layout slot of a pack that makes a dial.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PanelDials {
    slots: Vec<SlotDial>,
    lettering: PanelLettering,
}

impl PanelDials {
    /// The dials of `pack`'s layout, and why any instrument made none.
    /// `tanks` names the aircraft's fuel tanks in its tank order.
    #[must_use]
    pub fn from_pack(pack: &InstrumentPackDocument, tanks: &[&str]) -> (Self, Vec<DialError>) {
        let mut slots = Vec::new();
        let mut refused = Vec::new();
        let mut lettering = PanelLettering::default();
        for (slot, place) in pack.layout.iter().enumerate() {
            let Some(spec) = pack.instrument(&place.instrument) else {
                continue;
            };
            match DialSource::from_instrument(spec, tanks) {
                Ok(mut dial) => {
                    if let Some(spec) = &spec.lettering {
                        dial.lettering = indicate_instrument_analog::Lettering {
                            numerals: lettering.slot_for(&spec.numerals),
                            legends: lettering.slot_for(spec.legends()),
                        };
                    }
                    slots.push(SlotDial {
                        slot,
                        name: place.slot.clone(),
                        instrument: place.instrument.clone(),
                        dial,
                    });
                }
                Err(err) => refused.push(err),
            }
        }
        (Self { slots, lettering }, refused)
    }

    /// The faces the panel's dials are lettered in, slot by slot.
    #[must_use]
    pub const fn lettering(&self) -> &PanelLettering {
        &self.lettering
    }

    /// The dials, in layout order.
    #[must_use]
    pub fn slots(&self) -> &[SlotDial] {
        &self.slots
    }

    /// The dial at layout slot `slot`.
    #[must_use]
    pub fn slot(&self, slot: usize) -> Option<&SlotDial> {
        self.slots.iter().find(|s| s.slot == slot)
    }

    /// Each dial's values and pose by layout slot: state lanes read from
    /// `state`, direct lanes from `direct(slot, pointer)` in dial units.
    pub fn poses<'a, F>(
        &'a self,
        state: &'a GaugeState,
        direct: F,
    ) -> impl Iterator<Item = (usize, PointerValues, DialPose)> + 'a
    where
        F: Fn(usize, usize) -> Option<f32> + 'a,
    {
        let beside = state.beside();
        self.slots.iter().map(move |slot| {
            let (values, pose) = slot
                .dial
                .read_and_pose(&state.data, &beside, |pointer| direct(slot.slot, pointer));
            (slot.slot, values, pose)
        })
    }
}
