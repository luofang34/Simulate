//! The flight's status as a page shows it: a fixed number of short lines,
//! one fact each, so the block keeps its height while the flight runs, holds,
//! ends or stops, and nothing under it moves. A fact the flight has not
//! reported yet keeps its line with a dash.

use sim_navigate::trial::{HoldingEntryName, HoldingLegName};

use super::{Monitor, Pacer, clock_words};

#[cfg(test)]
mod tests;

/// Lines in a flight's status block, in every state.
pub const RUN_LINES: usize = 5;
/// What stands in a line whose fact is not known yet.
pub const ABSENT: &str = "—";
/// Share of the page's time above which the page's other drawing is named as
/// slowing the pace.
const OTHERS_NAMED_ABOVE: f64 = 0.5;

/// A pace as a multiple: whole above ten, one decimal below.
fn times(speed: f64) -> String {
    if speed >= 10.0 {
        format!("{speed:.0}×")
    } else {
        format!("{speed:.1}×")
    }
}

/// The block while the vehicle's software starts, under `head`, what flies.
#[must_use]
pub fn starting_lines(head: &str) -> [String; RUN_LINES] {
    [
        head.to_owned(),
        "Starting the vehicle's software".to_owned(),
        "Trim · align navigators · camera world".to_owned(),
        format!("Hold {ABSENT}"),
        format!("Pace {ABSENT}"),
    ]
}

/// The block of a flight that cannot start or has stopped; the reason
/// stands in the section's lines below.
#[must_use]
pub fn stopped_lines(head: &str) -> [String; RUN_LINES] {
    [
        head.to_owned(),
        "Stopped · reason below".to_owned(),
        format!("Sequenced {ABSENT}"),
        format!("Hold {ABSENT}"),
        format!("Pace {ABSENT}"),
    ]
}

/// The block of a flying flight: `head`, what flies; route time and phase;
/// the plan's progress; the hold; the pace, with `others`, the page's other
/// drawing and its share of the page's time when measured.
#[must_use]
pub fn run_lines(
    head: &str,
    monitor: &Monitor,
    pacer: &Pacer,
    others: Option<(&str, f64)>,
) -> [String; RUN_LINES] {
    [
        head.to_owned(),
        route_line(monitor),
        plan_line(monitor),
        hold_line(monitor),
        pace_line(pacer, others),
    ]
}

/// "Route 2:09 / 9:39 · Climb".
fn route_line(monitor: &Monitor) -> String {
    let planned = monitor
        .planned_duration_s()
        .map_or_else(|| ABSENT.to_owned(), clock_words);
    let phase = if monitor.finished() {
        "ended"
    } else {
        monitor.phase().unwrap_or(ABSENT)
    };
    format!(
        "Route {} / {planned} · {phase}",
        clock_words(monitor.sim_time_s())
    )
}

/// "Sequenced WP1 · 1 of 3", or "Plan done at WP3 · 8:51".
fn plan_line(monitor: &Monitor) -> String {
    if let Some(end) = monitor.route_end() {
        return format!("Plan done at {} · {}", end.ident, clock_words(end.t_s));
    }
    match monitor.last_waypoint() {
        Some((index, ident)) => format!(
            "Sequenced {ident} · {} of {}",
            index.saturating_add(1),
            monitor.waypoints().len()
        ),
        None => format!("Sequenced {ABSENT}"),
    }
}

/// "Hold WP3 · outbound · 2 circuits".
fn hold_line(monitor: &Monitor) -> String {
    monitor.hold().map_or_else(
        || format!("Hold {ABSENT}"),
        |hold| {
            let circuits = if hold.circuits == 1 {
                "circuit"
            } else {
                "circuits"
            };
            format!(
                "Hold {} · {} · {} {circuits}",
                hold.fix_ident,
                leg_line(hold.leg),
                hold.circuits
            )
        },
    )
}

/// "Pace 4×", or "Pace 1.1× of 4× · held by flight cost": the page names
/// its `others`, its other drawing, in place of the cost when they take
/// most of the page's time.
fn pace_line(pacer: &Pacer, others: Option<(&str, f64)>) -> String {
    let chosen = pacer.speed().map_or_else(|| "max".to_owned(), times);
    let others = others
        .filter(|(_, share)| *share > OTHERS_NAMED_ABOVE)
        .map(|(name, share)| format!("{name} {:.0} %", share * 100.0));
    match (pacer.limit(), others) {
        (Some(limit), others) => {
            let by = others.unwrap_or_else(|| {
                if limit.by_camera {
                    "camera".to_owned()
                } else {
                    "flight cost".to_owned()
                }
            });
            format!("Pace {} of {chosen} · held by {by}", times(limit.speed))
        }
        (None, Some(others)) => format!("Pace {chosen} · {others}"),
        (None, None) => format!("Pace {chosen}"),
    }
}

/// A part of a hold, short enough for a status line.
const fn leg_line(leg: HoldingLegName) -> &'static str {
    match leg {
        HoldingLegName::ToFix => "to the fix",
        HoldingLegName::Entry(HoldingEntryName::Direct) => "direct entry",
        HoldingLegName::Entry(HoldingEntryName::Parallel) => "parallel entry",
        HoldingLegName::Entry(HoldingEntryName::Teardrop) => "teardrop entry",
        HoldingLegName::Inbound => "inbound",
        HoldingLegName::FixTurn => "turn at fix",
        HoldingLegName::Outbound => "outbound",
        HoldingLegName::OutboundTurn => "turn inbound",
    }
}
