//! The holding pattern Pilotage flies after the plan, in the progress
//! stream: when it starts, which entry it takes, and each change of the
//! part flown.

#[cfg(test)]
mod tests;

use sim_core::SimInstant;
use sim_flight_chain::{
    HoldingEntry, HoldingLeg, HoldingPattern, HoldingTurn, HostEvent, MissionEvent,
    NavigationDisplay,
};
use sim_navigate::trial::{
    EventKind, EventRecord, HoldingEntryName, HoldingFix, HoldingLegName, HoldingPatternRecord,
    HoldingTurnName, ProgressRecord,
};

/// The part of the pattern last reported.
#[derive(Debug, Default)]
pub(super) struct HoldingWatch {
    leg: Option<HoldingLegName>,
}

impl HoldingWatch {
    /// The records of Pilotage's holding events among `events`, and of a
    /// change of the part flown that `display` shows at `t_s`.
    pub(super) fn records(
        &mut self,
        t_s: f64,
        events: &[(SimInstant, HostEvent)],
        display: Option<&NavigationDisplay>,
    ) -> Vec<ProgressRecord> {
        let mut records: Vec<ProgressRecord> = events
            .iter()
            .filter_map(|(at, event)| {
                let kind = match event {
                    HostEvent::Mission(MissionEvent::HoldingPatternStarted { pattern }) => {
                        EventKind::HoldingStarted {
                            pattern: pattern_record(pattern),
                        }
                    }
                    HostEvent::Mission(MissionEvent::HoldingPatternEntered { entry }) => {
                        EventKind::HoldingEntered {
                            entry: entry_name(*entry),
                        }
                    }
                    _ => return None,
                };
                Some(event_record(seconds(*at), kind))
            })
            .collect();
        let status = display.and_then(|d| d.holding_pattern.as_ref());
        let leg = status.map(|s| leg_name(s.leg));
        if let (Some(status), Some(leg)) = (status, leg)
            && self.leg != Some(leg)
        {
            records.push(event_record(
                t_s,
                EventKind::HoldingLegChanged {
                    leg,
                    circuits: status.circuits,
                },
            ));
        }
        self.leg = leg;
        records
    }
}

fn event_record(t_s: f64, kind: EventKind) -> ProgressRecord {
    ProgressRecord::Event(EventRecord {
        t_s,
        set: None,
        kind,
    })
}

fn seconds(at: SimInstant) -> f64 {
    at.duration_since(SimInstant::START).as_seconds()
}

/// `pattern` with the same parts, names and units.
pub(super) fn pattern_record(pattern: &HoldingPattern) -> HoldingPatternRecord {
    HoldingPatternRecord {
        fix_ident: pattern.fix_ident.clone(),
        fix: HoldingFix {
            latitude_rad: pattern.fix.latitude_rad,
            longitude_rad: pattern.fix.longitude_rad,
            altitude_m: pattern.fix.altitude_m,
        },
        inbound_course_rad: pattern.inbound_course_rad,
        turn: match pattern.turn {
            HoldingTurn::Right => HoldingTurnName::Right,
            HoldingTurn::Left => HoldingTurnName::Left,
        },
        leg_time_s: pattern.leg_time_s,
        turn_radius_m: pattern.turn_radius_m,
        leg_length_m: pattern.leg_length_m,
    }
}

const fn entry_name(entry: HoldingEntry) -> HoldingEntryName {
    match entry {
        HoldingEntry::Direct => HoldingEntryName::Direct,
        HoldingEntry::Parallel => HoldingEntryName::Parallel,
        HoldingEntry::Teardrop => HoldingEntryName::Teardrop,
    }
}

const fn leg_name(leg: HoldingLeg) -> HoldingLegName {
    match leg {
        HoldingLeg::ToFix => HoldingLegName::ToFix,
        HoldingLeg::Entry(entry) => HoldingLegName::Entry(entry_name(entry)),
        HoldingLeg::Inbound => HoldingLegName::Inbound,
        HoldingLeg::FixTurn => HoldingLegName::FixTurn,
        HoldingLeg::Outbound => HoldingLegName::Outbound,
        HoldingLeg::OutboundTurn => HoldingLegName::OutboundTurn,
    }
}
