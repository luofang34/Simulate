//! Round analog instruments of an instrument pack, drawn by Indicate.
//!
//! [`DialSource`] turns one pack instrument into the dial Indicate's analog
//! set draws: the pack's bezel, units, scale, markings, Kollsman range and
//! printed face, and Indicate's built-in gauge of the same kind for whatever
//! the pack leaves out. [`PanelDials`] holds one per layout slot, feeds the
//! indicated values the pack's mechanisms show into Indicate's state, and
//! poses every pointer from Indicate's own value-to-angle laws, so a needle
//! and its painted scale cannot disagree. The mechanism's lag and
//! instrument error stay with the mechanism; the law stays with Indicate.

mod bake;
mod convert;
mod house;
mod indications;
mod lettering;
mod panel;
mod source;
mod tape;

pub use bake::{FaceBake, OFF_FLAG_TEXT, bake_drum_strip, bake_face, bake_off_flag, bake_placard};
pub use indications::{EngineGauges, GaugeIndications, GaugeState};
pub use lettering::PanelLettering;
pub use panel::{PanelDials, SlotDial};
pub use source::{DialSource, Lane, PointerValues};
pub use tape::{TapeBake, bake_tape_strip};

#[cfg(test)]
mod tests;

use indicate_instrument_analog::SpecError;
use thiserror::Error;

/// Why an instrument of a pack does not become a dial.
#[derive(Debug, Error, PartialEq)]
pub enum DialError {
    /// A face the pack letters its dials in cannot be bound.
    #[error("typeface {face} cannot letter the panel: {detail}")]
    Lettering {
        /// The typeface pack's id.
        face: String,
        /// Why.
        detail: String,
    },
    /// The converted dial cannot place its values.
    #[error("instrument {instrument} does not make a usable dial")]
    Spec {
        /// The pack instrument.
        instrument: String,
        /// Indicate's reason.
        #[source]
        source: SpecError,
    },
}
