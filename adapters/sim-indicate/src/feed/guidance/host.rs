//! The guidance contract between the flight-guidance host and the display.
//!
//! Autopilot modes, procedures, and the pilot's selections belong to the host
//! that flies them, Pilotage in real flight. The feed only presents what the
//! host reports: it never derives a course, a mode, or a deviation of its own,
//! and with nothing reported the HSI shows no deviation rather than a centred
//! one (ADR-0031).

use glam::DVec3;
use indicate_instrument_feeder::RawStamp;
use indicate_instrument_feeder::nav_display::{DeviationScaleProfile, nav_display_state};
use indicate_instrument_feeder::nav_guidance::{Guidance, NavGuidanceTracker};
use indicate_instrument_state::{IdentStr, NavData, Stamped};
use sim_geo::BodyEllipsoid;

use super::{PilotSelections, RunwayNavGuidance, runway_nav};

#[cfg(test)]
mod tests;

/// Autopilot modes as the host reports them, shown as context only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AutopilotModeSummary {
    /// Active lateral mode label, when the host reports one.
    pub lateral: Option<IdentStr>,
    /// Active vertical mode label, when the host reports one.
    pub vertical: Option<IdentStr>,
    /// Whether an approach mode is armed.
    pub armed_approach: bool,
    /// Whether the autopilot is engaged.
    pub engaged: bool,
}

/// The navigation guidance the host has active, if any.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum HostNavGuidance {
    /// No active procedure or course: the HSI shows no deviation at all.
    #[default]
    None,
    /// A Pilotage lane sample, admitted through the feed's guidance tracker
    /// so a duplicate, replayed, or foreign sample never refreshes the display.
    PilotageLane {
        /// The sample's identity and acquisition stamp.
        stamp: RawStamp,
        /// The guidance values, in the lane's wire units.
        guidance: Guidance,
        /// The host-monotonic clock the tracker ages the sample against,
        /// milliseconds. It must not run backwards; see
        /// [`crate::StateFeed::reset_guidance_clock`].
        now_ms: f64,
    },
    /// A localizer and glide slope receiver the host has tuned to a runway.
    RunwayReceiver(RunwayNavGuidance),
}

/// Everything the flight-guidance host reports for one displayed instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HostFlightGuidance {
    /// Heading, altitude, and altimeter selections on the guidance panel.
    pub selections: PilotSelections,
    /// The active navigation guidance.
    pub nav: HostNavGuidance,
    /// The autopilot modes the host reports.
    pub autopilot: AutopilotModeSummary,
    /// The host's rewind/reset generation. A host that rewinds advances it
    /// with `wrapping_add(1)` alongside the lane's stamp epoch, and calls
    /// [`crate::StateFeed::reset_guidance_clock`] when its clock goes back.
    pub source_epoch: u32,
}

/// The nav group for `nav`, absent unless the host reports admissible guidance.
pub(in crate::feed) fn host_nav(
    tracker: &mut NavGuidanceTracker,
    nav: &HostNavGuidance,
    deviation_scale: &DeviationScaleProfile,
    body: &BodyEllipsoid,
    position_fixed_m: DVec3,
) -> Stamped<NavData> {
    match nav {
        HostNavGuidance::None => Stamped::default(),
        HostNavGuidance::PilotageLane {
            stamp,
            guidance,
            now_ms,
        } => {
            let snapshot = tracker.observe(Some(&(*stamp, *guidance)), *now_ms);
            nav_display_state(snapshot.as_ref(), deviation_scale).unwrap_or_default()
        }
        HostNavGuidance::RunwayReceiver(receiver) => runway_nav(receiver, body, position_fixed_m),
    }
}
