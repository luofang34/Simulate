//! What the page hands the app's map from the monitor: the plan at the mission host's
//! progress, and the tracks, rings, fix marks and coverage laid over it.

use std::collections::BTreeSet;

use sim_link_host::Monitor;
use sim_maplibre::{
    EfbTracks, FixId, FlightPlan, HoldPattern, PointMark, RouteFix, RouteProgress, SigmaRing,
    TRUTH_MARK, TrackLine,
};
use sim_navigate::trial::{HoldingPatternRecord, HoldingTurnName};

use super::super::series_colour;

/// The plan from `monitor`'s route, flown from `origin`, at the mission host's progress.
pub(super) fn plan_of(monitor: &Monitor, origin: Option<[f64; 2]>) -> (FlightPlan, RouteProgress) {
    let altitudes = monitor.waypoint_altitudes_m();
    let fixes = monitor
        .waypoints()
        .iter()
        .enumerate()
        .map(|(index, (ident, (lat_deg, lon_deg)))| RouteFix {
            id: FixId(u32::try_from(index).unwrap_or(u32::MAX)),
            ident: ident.clone(),
            lat_deg: *lat_deg,
            lon_deg: *lon_deg,
            altitude_m: altitudes.get(index).copied(),
        })
        .collect();
    let progress = RouteProgress {
        hold: monitor.hold_pattern().map(hold_of),
        ..RouteProgress::after_sequenced(monitor.sequenced())
    };
    (FlightPlan { origin, fixes }, progress)
}

/// The map's drawing of the pattern the mission host reported.
fn hold_of(pattern: &HoldingPatternRecord) -> HoldPattern {
    HoldPattern {
        fix_deg: [
            pattern.fix.latitude_rad.to_degrees(),
            pattern.fix.longitude_rad.to_degrees(),
        ],
        inbound_course_deg: pattern.inbound_course_rad.to_degrees(),
        right_turns: matches!(pattern.turn, HoldingTurnName::Right),
        turn_radius_m: pattern.turn_radius_m,
        leg_length_m: pattern.leg_length_m,
    }
}

/// The true track, each shown set's track, ring and fix marks, and the coverage. The
/// `steered` set has no ring here: its estimate is the map's aircraft, whose ring the
/// app draws at each frame's own instant.
pub(super) fn tracks_of(
    monitor: &Monitor,
    (hidden, steered): (&BTreeSet<String>, Option<&str>),
    coverage: &[Vec<[f64; 2]>],
) -> EfbTracks {
    let shown = |index: usize| {
        monitor
            .traces()
            .get(index)
            .is_some_and(|trace| !hidden.contains(&trace.name))
    };
    let mut tracks = EfbTracks {
        outlines: coverage.to_vec(),
        ..EfbTracks::default()
    };
    for (index, trace) in monitor.traces().iter().enumerate() {
        if !shown(index) {
            continue;
        }
        let colour = rgb(series_colour(index));
        tracks.lines.push(TrackLine {
            colour,
            width_px: 1.5,
            points: trace
                .points()
                .iter()
                .map(|p| [p.estimate_deg.0, p.estimate_deg.1])
                .collect(),
        });
        if let Some((latest, sigma_m)) = trace
            .latest()
            .filter(|_| Some(trace.name.as_str()) != steered)
            .and_then(|latest| Some((latest, latest.sigma_horizontal_m?)))
        {
            tracks.rings.push(SigmaRing {
                centre_deg: [latest.estimate_deg.0, latest.estimate_deg.1],
                sigma_m,
                colour,
            });
        }
    }
    tracks.lines.push(TrackLine {
        colour: TRUTH_MARK,
        width_px: 2.0,
        points: monitor
            .truth()
            .iter()
            .map(|(_, (lat, lon))| [*lat, *lon])
            .collect(),
    });
    tracks.marks = monitor
        .camera()
        .marks()
        .iter()
        .filter(|mark| shown(mark.set))
        .map(|mark| PointMark {
            at_deg: [mark.at_deg.0, mark.at_deg.1],
            colour: rgb(series_colour(mark.set)),
            filled: mark.accepted,
        })
        .collect();
    tracks
}

/// `[r, g, b]` of a `#rrggbb` colour; grey for anything else.
pub(super) fn rgb(css: &str) -> [u8; 3] {
    let channel = |range: std::ops::Range<usize>| {
        css.get(range)
            .and_then(|hex| u8::from_str_radix(hex, 16).ok())
    };
    match (channel(1..3), channel(3..5), channel(5..7)) {
        (Some(r), Some(g), Some(b)) if css.len() == 7 => [r, g, b],
        _ => [0x8f, 0x98, 0xad],
    }
}
