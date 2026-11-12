//! A holding pattern as the mission host flies it, drawn as its racetrack: the inbound
//! leg ends at the fix, the turns lie on the holding side, and the outbound leg runs
//! parallel two turn radii off the inbound one.

use crate::geodesy::destination;

/// Points on each half-circle turn of a drawn racetrack.
const TURN_STEPS: u32 = 16;

/// A holding pattern.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldPattern {
    /// `[lat, lon]` of the holding fix, degrees.
    pub fix_deg: [f64; 2],
    /// True course of the inbound leg, degrees.
    pub inbound_course_deg: f64,
    /// Whether the turns are to the right, the standard pattern.
    pub right_turns: bool,
    /// Radius of the turns, metres.
    pub turn_radius_m: f64,
    /// Length of the inbound leg, metres. The outbound leg is timed, so its flown
    /// length varies with the wind; the drawing takes it as the inbound one's.
    pub leg_length_m: f64,
}

impl HoldPattern {
    /// The racetrack as a closed line of `[lat, lon]` points, degrees, starting and
    /// ending where the inbound leg starts.
    #[must_use]
    pub fn racetrack(&self) -> Vec<[f64; 2]> {
        let [lat, lon] = self.fix_deg;
        let fix = [lon, lat];
        let course = self.inbound_course_deg;
        let (side, sign) = if self.right_turns {
            (course + 90.0, 1.0)
        } else {
            (course - 90.0, -1.0)
        };
        let radius = self.turn_radius_m.max(0.0);
        let inbound_start = destination(fix, course + 180.0, self.leg_length_m.max(0.0));
        let turn = |centre: [f64; 2], from_deg: f64| {
            (0..=TURN_STEPS).map(move |step| {
                let bearing = from_deg + sign * 180.0 * f64::from(step) / f64::from(TURN_STEPS);
                destination(centre, bearing, radius)
            })
        };
        let at_fix = turn(destination(fix, side, radius), side + 180.0);
        let at_start = turn(destination(inbound_start, side, radius), side);
        std::iter::once(inbound_start)
            .chain(at_fix)
            .chain(at_start)
            .map(|[lon, lat]| [lat, lon])
            .collect()
    }
}
