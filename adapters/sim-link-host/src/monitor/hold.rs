//! The holding pattern a vehicle flies after its plan, as the flight's host
//! reports it: the fix, the part of the pattern flown and the circuits
//! completed, in the words a page shows.

use sim_navigate::trial::{
    EventKind, HoldingEntryName, HoldingLegName, HoldingPatternRecord, HoldingTurnName,
};

#[cfg(test)]
mod tests;

/// Where the vehicle is in its holding pattern.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HoldState {
    /// The identifier of the holding fix.
    pub fix_ident: String,
    /// The part of the pattern flown now.
    pub leg: HoldingLegName,
    /// The circuits completed since the entry.
    pub circuits: u32,
}

impl HoldState {
    /// The hold after `event`, from `hold` before it, or `None` when the event says
    /// nothing of a hold and there is none.
    pub(super) fn after(hold: Option<Self>, event: &EventKind) -> Option<Self> {
        match event {
            EventKind::HoldingStarted { pattern } => Some(Self {
                fix_ident: pattern.fix_ident.clone(),
                leg: HoldingLegName::ToFix,
                circuits: 0,
            }),
            EventKind::HoldingEntered { entry } => hold.map(|hold| Self {
                leg: HoldingLegName::Entry(*entry),
                ..hold
            }),
            EventKind::HoldingLegChanged { leg, circuits } => hold.map(|hold| Self {
                leg: *leg,
                circuits: *circuits,
                ..hold
            }),
            _ => hold,
        }
    }

    /// The hold for a page's status line: "HOLD at ZA5: inbound, 2 circuits flown".
    #[must_use]
    pub fn words(&self) -> String {
        format!(
            "HOLD at {}: {}, {} {} flown",
            self.fix_ident,
            leg_words(self.leg),
            self.circuits,
            if self.circuits == 1 {
                "circuit"
            } else {
                "circuits"
            }
        )
    }
}

/// The part of a pattern in words.
#[must_use]
pub const fn leg_words(leg: HoldingLegName) -> &'static str {
    match leg {
        HoldingLegName::ToFix => "to the fix",
        HoldingLegName::Entry(entry) => entry_words(entry),
        HoldingLegName::Inbound => "inbound",
        HoldingLegName::FixTurn => "turning at the fix",
        HoldingLegName::Outbound => "outbound",
        HoldingLegName::OutboundTurn => "turning inbound",
    }
}

/// An entry in words.
#[must_use]
pub const fn entry_words(entry: HoldingEntryName) -> &'static str {
    match entry {
        HoldingEntryName::Direct => "direct entry",
        HoldingEntryName::Parallel => "parallel entry",
        HoldingEntryName::Teardrop => "teardrop entry",
    }
}

/// A pattern in words: "holding at ZA5, inbound 090°, right turns, 60 s legs".
#[must_use]
pub fn pattern_words(pattern: &HoldingPatternRecord) -> String {
    let turns = match pattern.turn {
        HoldingTurnName::Right => "right",
        HoldingTurnName::Left => "left",
    };
    format!(
        "holding at {}, inbound {:03.0}°, {turns} turns, {:.0} s legs",
        pattern.fix_ident,
        pattern.inbound_course_rad.to_degrees().rem_euclid(360.0),
        pattern.leg_time_s
    )
}
