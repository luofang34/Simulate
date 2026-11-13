//! What of a link's legs the vehicle's software flies today.
//!
//! A link states ARINC 424 legs. Pilotage's mission plan flies `IF`, `TF`,
//! `DF`, `CF`, and `CA` legs, `HA`/`HF`/`HM` holding patterns (time or
//! distance, true or WMM2025 magnetic `P4` courses), altitude windows (`At`,
//! `AtOrAbove`, `AtOrBelow`, `Between`), mandatory turn directions (`L`/`R`),
//! CIFP flight-phase roles, and per-leg RNP bounds. Every other path
//! terminator or unflown attribute (`RF`, `AF`, `PI`, flight levels, speed,
//! and vertical-angle data) is refused by its leg number and code so a link
//! never flies a different route than it states.

use sim_link::{
    Fix, HeightUnit, HoldLength, Leg, Limit, Micro, North, PathTerminator, Role, SimulationLink,
    TurnDirection,
};

use crate::HostError;

mod cifp;
#[cfg(test)]
mod tests;

pub use cifp::{FlyableAltitude, FlyableHold, FlyableHoldLength, FlyableLegRole, FlyablePath};
use cifp::{
    PhaseRoleTracker, flyable_altitude, flyable_hold, flyable_rnp_nm, project_course_to_altitude,
    require_course,
};

/// How far a terminal hold's inbound course may differ from the course of the
/// last leg (`degrees`) to be folded into [`FlyableRoute::hold`].
pub const HOLD_COURSE_TOLERANCE_DEG: f64 = 1.0;

/// One fix (or course-to-altitude termination) the route flies to.
#[derive(Debug, Clone, PartialEq)]
pub struct FlyableFix {
    /// The leg it ends, counted from one.
    pub leg: usize,
    /// Latitude, degrees north.
    pub lat: Micro,
    /// Longitude, degrees east.
    pub lon: Micro,
    /// Reference 3D altitude to cross at, metres in the link's datum; absent
    /// for the link's default height.
    pub height_m: Option<f64>,
    /// Full altitude constraint (`At`, `AtOrAbove`, `AtOrBelow`, `Window`),
    /// metres in the link's datum.
    pub altitude: Option<FlyableAltitude>,
    /// Cross the fix before turning.
    pub fly_over: bool,
    /// Optional mandatory turn direction (`L` or `R`) onto this leg.
    pub turn_direction: Option<TurnDirection>,
    /// ARINC 424 path terminator of this leg.
    pub path: FlyablePath,
    /// The fix's name in the link.
    pub name: Option<String>,
    /// The fix's CIFP roles in the procedure, for display.
    pub roles: Vec<Role>,
    /// Resolved operational flight-phase role of this leg.
    pub leg_role: Option<FlyableLegRole>,
    /// Procedure-published RNP on this leg, nautical miles.
    pub rnp_nm: Option<f64>,
}

/// The holding pattern at the last fix when folded into `TerminalHoldingPattern`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerminalHold {
    /// The direction of the turns.
    pub turn: TurnDirection,
    /// The inbound course, degrees true.
    pub inbound_course_deg: f64,
    /// The inbound leg time, seconds.
    pub leg_time_s: f64,
    /// The holding altitude, metres in the link's datum; absent for the
    /// last fix's.
    pub height_m: Option<f64>,
}

/// The route the vehicle's software flies.
#[derive(Debug, Clone, PartialEq)]
pub struct FlyableRoute {
    /// The fixes in fly order, spanning at least two distinct positions.
    pub fixes: Vec<FlyableFix>,
    /// The hold the link asks for at the last fix when aligned with the last
    /// leg's course; absent for the standard right-hand hold or when carried
    /// explicitly as a [`FlyablePath::HoldManual`] leg in [`Self::fixes`].
    pub hold: Option<TerminalHold>,
}

/// The route `link` asks the vehicle's software to fly.
///
/// # Errors
///
/// [`HostError::Unflyable`] for the first leg or attribute the vehicle's
/// software does not fly yet, or [`HostError::TooFewFixes`] when fewer than
/// two distinct positions are flown.
pub fn flyable_route(link: &SimulationLink) -> Result<FlyableRoute, HostError> {
    let mut fixes: Vec<FlyableFix> = Vec::with_capacity(link.route.len());
    let mut hold = None;
    let mut roles = PhaseRoleTracker::default();
    for (index, leg) in link.route.iter().enumerate() {
        let number = index.saturating_add(1);
        let refuse = |reason: String| unflyable(number, leg, reason);
        unflown_attributes(leg).map_err(refuse)?;
        let leg_role = roles.classify(leg);
        if leg.path == PathTerminator::Hm && number == link.route.len() {
            if fixes.len() < 2 {
                return Err(refuse("a hold needs two fixes before it".to_owned()));
            }
            if let Some(terminal) = try_terminal_hold(leg, &fixes).map_err(refuse)? {
                hold = Some(terminal);
                continue;
            }
        }
        let built = build_leg_fix(link, number, leg, leg_role, &fixes).map_err(refuse)?;
        fixes.push(built);
    }
    let distinct = distinct_fix_count(&fixes);
    if distinct < 2 {
        return Err(HostError::TooFewFixes { count: distinct });
    }
    Ok(FlyableRoute { fixes, hold })
}

fn build_leg_fix(
    link: &SimulationLink,
    number: usize,
    leg: &Leg,
    leg_role: Option<FlyableLegRole>,
    fixes: &[FlyableFix],
) -> Result<FlyableFix, String> {
    let altitude = flyable_altitude(leg)?;
    if leg.path == PathTerminator::Ca {
        return build_ca_fix(link, number, leg, leg_role, altitude, fixes);
    }
    let path = match leg.path {
        PathTerminator::If => FlyablePath::InitialFix,
        PathTerminator::Tf => FlyablePath::TrackToFix,
        PathTerminator::Df => FlyablePath::DirectToFix,
        PathTerminator::Cf => {
            let (course_deg, magnetic) = require_course(leg)?;
            FlyablePath::CourseToFix {
                course_deg,
                magnetic,
            }
        }
        PathTerminator::Ha => {
            if altitude.is_none() {
                return Err("a hold-to-altitude leg needs an altitude".to_owned());
            }
            FlyablePath::HoldToAltitude(flyable_hold(leg)?)
        }
        PathTerminator::Hf => FlyablePath::HoldToFix(flyable_hold(leg)?),
        PathTerminator::Hm => FlyablePath::HoldManual(flyable_hold(leg)?),
        other => return Err(format!("{} legs are not flown yet", path_words(other))),
    };
    let fix: &Fix = leg
        .fix
        .as_ref()
        .ok_or_else(|| "the leg has no fix".to_owned())?;
    let turn_direction = match leg.path {
        PathTerminator::Ha | PathTerminator::Hf | PathTerminator::Hm => None,
        _ => leg.turn,
    };
    Ok(FlyableFix {
        leg: number,
        lat: fix.lat,
        lon: fix.lon,
        height_m: altitude.map(FlyableAltitude::reference_m),
        altitude,
        fly_over: leg.fly_over,
        turn_direction,
        path,
        name: fix.name.clone(),
        roles: leg.roles.clone(),
        leg_role,
        rnp_nm: flyable_rnp_nm(leg),
    })
}

fn build_ca_fix(
    link: &SimulationLink,
    number: usize,
    leg: &Leg,
    leg_role: Option<FlyableLegRole>,
    altitude: Option<FlyableAltitude>,
    fixes: &[FlyableFix],
) -> Result<FlyableFix, String> {
    let prev = fixes
        .last()
        .ok_or_else(|| "a course-to-altitude leg needs a fix before it".to_owned())?;
    let target_alt =
        altitude.ok_or_else(|| "a course-to-altitude leg needs an altitude".to_owned())?;
    let course = leg
        .course
        .ok_or_else(|| "the leg has no course".to_owned())?;
    let (lat, lon) = project_course_to_altitude(
        link,
        (prev.lat, prev.lon, prev.height_m),
        course,
        target_alt,
    );
    Ok(FlyableFix {
        leg: number,
        lat,
        lon,
        height_m: Some(target_alt.reference_m()),
        altitude: Some(target_alt),
        fly_over: true,
        turn_direction: leg.turn,
        path: FlyablePath::CourseToAltitude {
            course_deg: course.degrees.as_f64(),
            magnetic: course.north == North::Magnetic,
        },
        name: Some(format!("CA{number}")),
        roles: leg.roles.clone(),
        leg_role,
        rnp_nm: flyable_rnp_nm(leg),
    })
}

fn distinct_fix_count(fixes: &[FlyableFix]) -> usize {
    let mut count = 0_usize;
    let mut last: Option<(Micro, Micro)> = None;
    for fix in fixes {
        let here = (fix.lat, fix.lon);
        if last != Some(here) {
            count = count.saturating_add(1);
            last = Some(here);
        }
    }
    count
}

fn unflyable(leg: usize, stated: &Leg, reason: String) -> HostError {
    HostError::Unflyable {
        leg,
        code: stated.path.code(),
        fix: stated.fix.as_ref().and_then(|fix| fix.name.clone()),
        reason,
    }
}

/// Refuses attributes that the vehicle's software does not fly yet (`s`, `g`, `v`, `x`).
fn unflown_attributes(leg: &Leg) -> Result<(), String> {
    if leg.speed.is_some() {
        return Err("speed constraints are not flown yet".to_owned());
    }
    if leg.second_altitude.is_some()
        || leg.vertical_angle.is_some()
        || leg.crossing_height.is_some()
    {
        return Err("vertical path data (g, v, x) is not flown yet".to_owned());
    }
    Ok(())
}

/// Returns `Ok(Some(TerminalHold))` when the final `HM` leg matches the last
/// leg's true course and uses a time leg at an `At` (or default) height; returns
/// `Ok(None)` when it should instead be flown as an explicit `HoldManual` leg.
fn try_terminal_hold(leg: &Leg, fixes: &[FlyableFix]) -> Result<Option<TerminalHold>, String> {
    let (Some(before), Some(last)) = (
        fixes
            .len()
            .checked_sub(2)
            .and_then(|index| fixes.get(index)),
        fixes.last(),
    ) else {
        return Ok(None);
    };
    if before.lat == last.lat && before.lon == last.lon {
        return Ok(None);
    }
    let at = leg.fix.as_ref().map(|fix| (fix.lat, fix.lon));
    if at != Some((last.lat, last.lon)) {
        return Ok(None);
    }
    let height_m = match leg.altitude {
        None => None,
        Some(Limit::At(h)) if h.unit != HeightUnit::FlightLevel => Some(h.metres()),
        Some(Limit::At(_)) => {
            return Err("flight levels need a pressure altitude and are not flown yet".to_owned());
        }
        Some(_) => return Ok(None),
    };
    let Some(course) = leg.course else {
        return Err("the hold has no inbound course".to_owned());
    };
    if course.north == North::Magnetic {
        return Ok(None);
    }
    let final_course = final_course_deg(before, last);
    let given = course.degrees.as_f64();
    let off = (given - final_course + 540.0).rem_euclid(360.0) - 180.0;
    if off.abs() > HOLD_COURSE_TOLERANCE_DEG {
        return Ok(None);
    }
    let Some(HoldLength::Time(time)) = leg.hold_length else {
        if leg.hold_length.is_none() {
            return Err("the hold has no leg length".to_owned());
        }
        return Ok(None);
    };
    let turn = leg
        .turn
        .ok_or_else(|| "the hold has no turn direction".to_owned())?;
    Ok(Some(TerminalHold {
        turn,
        inbound_course_deg: given,
        leg_time_s: time.seconds(),
        height_m,
    }))
}

/// The true course of the great circle from `from` to `to` where it reaches
/// `to`, degrees: the course the mission's hold takes as its inbound course.
pub fn final_course_deg(from: &FlyableFix, to: &FlyableFix) -> f64 {
    let (lat1, lon1) = (
        from.lat.as_f64().to_radians(),
        from.lon.as_f64().to_radians(),
    );
    let (lat2, lon2) = (to.lat.as_f64().to_radians(), to.lon.as_f64().to_radians());
    let y = (lon1 - lon2).sin() * lat1.cos();
    let x = lat2.cos() * lat1.sin() - lat2.sin() * lat1.cos() * (lon1 - lon2).cos();
    (y.atan2(x).to_degrees() + 180.0).rem_euclid(360.0)
}

/// The path terminator in words.
pub const fn path_words(path: PathTerminator) -> &'static str {
    match path {
        PathTerminator::If => "initial-fix",
        PathTerminator::Tf => "track-to-fix",
        PathTerminator::Cf => "course-to-fix",
        PathTerminator::Df => "direct-to-fix",
        PathTerminator::Fa => "course-from-fix-to-altitude",
        PathTerminator::Fc => "course-from-fix-for-a-distance",
        PathTerminator::Fd => "course-from-fix-to-a-DME-distance",
        PathTerminator::Fm => "course-from-fix-to-a-manual-end",
        PathTerminator::Ca => "course-to-altitude",
        PathTerminator::Cd => "course-to-a-DME-distance",
        PathTerminator::Ci => "course-to-intercept",
        PathTerminator::Cr => "course-to-a-radial",
        PathTerminator::Rf => "radius-to-fix arc",
        PathTerminator::Af => "DME-arc",
        PathTerminator::Va => "heading-to-altitude",
        PathTerminator::Vd => "heading-to-a-DME-distance",
        PathTerminator::Vi => "heading-to-intercept",
        PathTerminator::Vm => "heading-to-a-manual-end",
        PathTerminator::Vr => "heading-to-a-radial",
        PathTerminator::Pi => "procedure-turn",
        PathTerminator::Ha => "hold-to-altitude",
        PathTerminator::Hf => "hold-to-fix",
        PathTerminator::Hm => "manual-hold",
    }
}
