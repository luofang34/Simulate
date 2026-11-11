//! The route's vertical plan and the vertical profile flown.
//!
//! Navigate's guidance steers each leg toward the `At` altitude of the
//! waypoint it flies to, and the route follower climbs or descends against
//! that deviation within the vehicle's climb limit. So the plan is a step:
//! from the start of a leg the vehicle climbs or descends at the limit until
//! it reaches the waypoint's altitude, then holds it, closing the last tens
//! of metres exponentially. A leg is reachable when that climb at the limit
//! ends before the executor captures the waypoint.

use navigate_contract::Waypoint;
use navigate_geodesy::{BodyEllipsoid, distance_m};
use serde::{Deserialize, Serialize};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{LocalFrame, ecef_to_geodetic};

use super::route::{capture_threshold_m, leg_speed, plan_waypoints};
use super::{TrialError, VehicleSpec, WaypointSpec};
use crate::Truth;

/// One leg's vertical demand against the vehicle's climb limit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LegVertical {
    /// Index of the waypoint the leg flies to.
    pub leg: usize,
    /// The waypoint it leaves.
    pub from: String,
    /// The waypoint it flies to.
    pub to: String,
    /// Geodesic length, metres.
    pub length_m: f64,
    /// Altitude to gain, negative to lose, metres.
    pub altitude_change_m: f64,
    /// Speed on the leg, m/s.
    pub speed_m_s: f64,
    /// Time from the start of the leg to the waypoint's capture threshold,
    /// seconds: the time the altitude change has.
    pub available_s: f64,
    /// The steady climb or descent rate that changes the altitude in that
    /// time, m/s, unsigned.
    pub required_rate_m_s: f64,
    /// The vehicle's climb and descent limit, m/s.
    pub limit_m_s: f64,
}

/// One point of a planned vertical profile.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProfilePoint {
    /// Distance along the route's legs from the first waypoint, metres.
    pub along_track_m: f64,
    /// Height above the ellipsoid, metres.
    pub altitude_m: f64,
}

/// One point of a flown vertical profile.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlownPoint {
    /// Route time, seconds.
    pub time_s: f64,
    /// Horizontal distance flown since the start, metres: the path, so
    /// turns flown inside their corners make it a little shorter than the
    /// legs.
    pub along_track_m: f64,
    /// True height above the ellipsoid, metres.
    pub altitude_m: f64,
}

/// The route's vertical plan and, in open loop, the profile flown.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VerticalReport {
    /// Each leg's demand.
    pub legs: Vec<LegVertical>,
    /// The planned profile: a climb or descent at the limit from each
    /// leg's start, then level.
    pub planned: Vec<ProfilePoint>,
    /// The true profile; empty in closed loop, where each set reports its
    /// own.
    pub flown: Vec<FlownPoint>,
}

/// Each leg's vertical demand and the planned profile of `specs` flown by
/// `vehicle`.
pub fn vertical_plan(
    specs: &[WaypointSpec],
    vehicle: &VehicleSpec,
) -> Result<(Vec<LegVertical>, Vec<ProfilePoint>), TrialError> {
    let waypoints = plan_waypoints(specs)?;
    let mut legs = Vec::new();
    let mut planned = Vec::new();
    let mut along_m = 0.0;
    if let Some(first) = waypoints.first() {
        planned.push(ProfilePoint {
            along_track_m: 0.0,
            altitude_m: first.position.altitude_m,
        });
    }
    for (index, pair) in waypoints.windows(2).enumerate() {
        let [from, to] = pair else { continue };
        let leg = leg_vertical(&waypoints, index.wrapping_add(1), (from, to), vehicle);
        let climb_m = leg.altitude_change_m.abs() / vehicle.max_climb_m_s * leg.speed_m_s;
        if climb_m > 0.0 && climb_m < leg.length_m {
            planned.push(ProfilePoint {
                along_track_m: along_m + climb_m,
                altitude_m: to.position.altitude_m,
            });
        }
        along_m += leg.length_m;
        planned.push(ProfilePoint {
            along_track_m: along_m,
            altitude_m: to.position.altitude_m,
        });
        legs.push(leg);
    }
    Ok((legs, planned))
}

fn leg_vertical(
    waypoints: &[Waypoint],
    index: usize,
    (from, to): (&Waypoint, &Waypoint),
    vehicle: &VehicleSpec,
) -> LegVertical {
    let length_m = distance_m(&from.position, &to.position, BodyEllipsoid::WGS84);
    let speed_m_s = leg_speed(waypoints, index, vehicle);
    let capture_m = capture_threshold_m(waypoints, index, vehicle);
    let available_s = (length_m - capture_m).max(0.0) / speed_m_s;
    let altitude_change_m = to.position.altitude_m - from.position.altitude_m;
    let required_rate_m_s = if altitude_change_m == 0.0 {
        0.0
    } else if available_s > 0.0 {
        altitude_change_m.abs() / available_s
    } else {
        f64::INFINITY
    };
    LegVertical {
        leg: index,
        from: from.ident.as_str().to_owned(),
        to: to.ident.as_str().to_owned(),
        length_m,
        altitude_change_m,
        speed_m_s,
        available_s,
        required_rate_m_s,
        limit_m_s: vehicle.max_climb_m_s,
    }
}

/// Refuses the first leg whose altitude change needs more than the
/// vehicle's climb limit.
pub(super) fn check_reachable(legs: &[LegVertical]) -> Result<(), TrialError> {
    match legs
        .iter()
        .find(|leg| leg.required_rate_m_s > leg.limit_m_s)
    {
        Some(leg) => Err(TrialError::UnreachableAltitude {
            leg: leg.leg,
            from: leg.from.clone(),
            to: leg.to.clone(),
            altitude_change_m: leg.altitude_change_m,
            required_rate_m_s: leg.required_rate_m_s,
            limit_m_s: leg.limit_m_s,
        }),
        None => Ok(()),
    }
}

/// The true vertical profile of `truth`, a point every `period_steps` and
/// at the end; the horizontal path is summed over every step.
pub(super) fn flown_profile(
    truth: &[Truth<(SimInstant, VehicleState)>],
    period_steps: usize,
) -> Vec<FlownPoint> {
    let mut points = Vec::new();
    let mut along_m = 0.0;
    let mut previous: Option<glam::DVec3> = None;
    let last = truth.len().saturating_sub(1);
    for (step, sample) in truth.iter().enumerate() {
        let (at, state) = *sample.get();
        let geo = ecef_to_geodetic(state.position_ecef);
        if let Some(before) = previous {
            let moved = state.position_ecef - before;
            let up = LocalFrame::at(&geo).up();
            along_m += (moved - up * moved.dot(up)).length();
        }
        previous = Some(state.position_ecef);
        if step.is_multiple_of(period_steps.max(1)) || step == last {
            points.push(FlownPoint {
                time_s: at.duration_since(SimInstant::START).as_seconds(),
                along_track_m: along_m,
                altitude_m: geo.height_m,
            });
        }
    }
    points
}
