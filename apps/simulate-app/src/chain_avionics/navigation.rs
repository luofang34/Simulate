//! Pilotage's navigation display as the HSI's guidance lane: the desired
//! track as the course, the cross-track deviation on the CDI, the distance
//! to the active fix and its name. Indicate draws the lane on its terminal
//! scale, 25 m a dot, and shows TO whenever the leg has a track. In the
//! holding pattern after the plan the course is the inbound course to the
//! holding fix and the FROM name reads `HOLD`.

use sim_core::SimInstant;
use sim_flight_chain::{NavQuality, NavigationDisplay};
use sim_indicate::{
    CLOCK_HOST_MONOTONIC, HostNavGuidance, IdentStr, LaneGuidance, ROLE_NAVIGATION_SOLUTION,
    RawStamp,
};

/// The lane's source identity: Pilotage's mission on this vehicle.
const SOURCE_ID: u64 = 0x5049_4c4f_5441_4745;
/// The source's attachment identity, fixed for the vehicle's one host.
const INCARNATION: [u8; 16] = [0x50; 16];
/// The FROM name the HSI shows in the holding pattern.
const HOLD_IDENT: &str = "HOLD";

/// The guidance lane of `display` at `at`, Pilotage's report of that
/// instant, in the host's guidance epoch `source_epoch`; none while no leg
/// is flown.
#[must_use]
pub fn navigation_lane(
    display: &NavigationDisplay,
    at: SimInstant,
    source_epoch: u32,
) -> HostNavGuidance {
    let (Some(leg), Some(at_ns)) = (display.leg.as_ref(), at.to_nanos()) else {
        return HostNavGuidance::None;
    };
    let holding = display.holding_pattern.is_some();
    let from_ident = match (holding, leg.from.as_ref()) {
        (true, _) => ident(HOLD_IDENT),
        (false, Some(from)) => ident(&from.ident),
        (false, None) => IdentStr::default(),
    };
    let leg_index = display.active_waypoint.unwrap_or(display.waypoints.len());
    let guidance = LaneGuidance {
        to_ident: ident(&leg.to.ident),
        from_ident,
        course_rad: leg.desired_track_rad as f32,
        lateral_deviation_m: leg.cross_track_m.map_or(f32::NAN, |m| m as f32),
        vertical_deviation_m: leg.vertical_deviation_m.map_or(f32::NAN, |m| m as f32),
        distance_to_waypoint_m: leg.distance_to_go_m as f32,
        leg_index: u32::try_from(leg_index).unwrap_or(u32::MAX),
        waypoint_count: u32::try_from(display.waypoints.len()).unwrap_or(u32::MAX),
        solution_quality: quality_code(leg.quality),
    };
    let at_ms = at_ns / 1_000_000;
    HostNavGuidance::PilotageLane {
        stamp: RawStamp {
            role: ROLE_NAVIGATION_SOLUTION,
            integrity: 1,
            source_id: SOURCE_ID,
            incarnation: INCARNATION,
            epoch: source_epoch,
            // A new report each millisecond of the host's time; the same
            // instant again is the same report.
            sequence: at_ms as u32,
            acquired_at_ns: at_ns,
            clock: CLOCK_HOST_MONOTONIC,
        },
        guidance,
        now_ms: at_ms as f64,
    }
}

/// The lane's solution quality code: Indicate presents 0 and 1 and removes
/// every other code's guidance.
const fn quality_code(quality: NavQuality) -> u32 {
    match quality {
        NavQuality::Good => 0,
        NavQuality::Degraded => 1,
        NavQuality::Unusable => 2,
    }
}

fn ident(name: &str) -> IdentStr {
    IdentStr::new(name).unwrap_or_default()
}
