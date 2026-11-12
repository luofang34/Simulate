//! What of a link's legs the vehicle's software flies today.
//!
//! A link states ARINC 424 legs. Pilotage's mission plan flies a list of
//! fixes joined by great-circle tracks, each crossed at one altitude, fly-by
//! or fly-over, and one holding pattern at the last fix. This module takes
//! the legs that say exactly that and refuses every other leg and attribute
//! by its leg number and code, so a link never flies a different route than
//! it states. The link itself stays valid: the refusal is the host's.

use sim_link::{
    Fix, HeightUnit, HoldLength, Leg, Limit, Micro, North, PathTerminator, Role, SimulationLink,
    TurnDirection,
};

use crate::HostError;

#[cfg(test)]
mod tests;

/// How far a hold's inbound course may differ from the course of the last
/// leg, degrees: the course is written with one decimal, and the hold's
/// own geometry follows the last leg.
pub const HOLD_COURSE_TOLERANCE_DEG: f64 = 1.0;

/// One fix the route flies to.
#[derive(Debug, Clone, PartialEq)]
pub struct FlyableFix {
    /// The leg it ends, counted from one.
    pub leg: usize,
    /// Latitude, degrees north.
    pub lat: Micro,
    /// Longitude, degrees east.
    pub lon: Micro,
    /// The altitude to cross at, metres in the link's datum; absent for the
    /// link's default height.
    pub height_m: Option<f64>,
    /// Cross the fix before turning.
    pub fly_over: bool,
    /// The fix's name in the link.
    pub name: Option<String>,
    /// The fix's roles in the procedure, for display.
    pub roles: Vec<Role>,
}

/// The holding pattern at the last fix.
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
    /// The fixes in fly order, at least two.
    pub fixes: Vec<FlyableFix>,
    /// The hold the link asks for at the last fix; absent for the
    /// standard right-hand hold the mission flies when its plan ends.
    pub hold: Option<TerminalHold>,
}

/// The route `link` asks the vehicle's software to fly.
///
/// # Errors
///
/// [`HostError::Unflyable`] for the first leg or attribute the vehicle's
/// software does not fly yet.
pub fn flyable_route(link: &SimulationLink) -> Result<FlyableRoute, HostError> {
    let mut fixes: Vec<FlyableFix> = Vec::with_capacity(link.route.len());
    let mut hold = None;
    for (index, leg) in link.route.iter().enumerate() {
        let number = index.saturating_add(1);
        let refuse = |reason: String| unflyable(number, leg, reason);
        match leg.path {
            PathTerminator::If | PathTerminator::Tf => {}
            PathTerminator::Df if index == 0 => {}
            PathTerminator::Df => {
                return Err(refuse(
                    "a direct-to after the first leg is not flown yet".to_owned(),
                ));
            }
            PathTerminator::Hm if number < link.route.len() => {
                return Err(refuse(
                    "a hold before the end of the route is not flown yet".to_owned(),
                ));
            }
            PathTerminator::Hm => {
                hold = Some(terminal_hold(leg, &fixes).map_err(refuse)?);
                continue;
            }
            other => {
                return Err(refuse(format!(
                    "{} legs are not flown yet",
                    path_words(other)
                )));
            }
        }
        let fix = leg
            .fix
            .as_ref()
            .ok_or_else(|| refuse("the leg has no fix".to_owned()))?;
        fixes.push(flyable_fix(number, leg, fix).map_err(refuse)?);
    }
    if fixes.len() < 2 {
        return Err(HostError::TooFewFixes { count: fixes.len() });
    }
    Ok(FlyableRoute { fixes, hold })
}

fn unflyable(leg: usize, stated: &Leg, reason: String) -> HostError {
    HostError::Unflyable {
        leg,
        code: stated.path.code(),
        fix: stated.fix.as_ref().and_then(|fix| fix.name.clone()),
        reason,
    }
}

fn flyable_fix(number: usize, leg: &Leg, fix: &Fix) -> Result<FlyableFix, String> {
    unflown_attributes(leg)?;
    Ok(FlyableFix {
        leg: number,
        lat: fix.lat,
        lon: fix.lon,
        height_m: at_height(leg)?,
        fly_over: leg.fly_over,
        name: fix.name.clone(),
        roles: leg.roles.clone(),
    })
}

/// Refuses an attribute that would change how the leg is flown when the
/// vehicle's software does not fly it yet. Roles, the missed-approach mark,
/// the navaid reference (`n`, `b`, `d`) and the required navigation
/// performance do not change the path of a track to a fix.
fn unflown_attributes(leg: &Leg) -> Result<(), String> {
    if leg.speed.is_some() {
        return Err("speed constraints are not flown yet".to_owned());
    }
    if leg.turn.is_some() {
        return Err("a forced turn direction is not flown yet".to_owned());
    }
    if leg.second_altitude.is_some()
        || leg.vertical_angle.is_some()
        || leg.crossing_height.is_some()
    {
        return Err("vertical path data (g, v, x) is not flown yet".to_owned());
    }
    Ok(())
}

/// The altitude of an `a` that says "at" in metres or feet.
fn at_height(leg: &Leg) -> Result<Option<f64>, String> {
    match leg.altitude {
        None => Ok(None),
        Some(Limit::At(height)) if height.unit != HeightUnit::FlightLevel => {
            Ok(Some(height.metres()))
        }
        Some(Limit::At(_)) => {
            Err("flight levels need a pressure altitude and are not flown yet".to_owned())
        }
        Some(_) => Err(
            "only an altitude to cross at is flown yet, not at-or-above, at-or-below or a window"
                .to_owned(),
        ),
    }
}

/// The `HM` leg as the hold the mission flies at its last fix: at that fix,
/// on the last leg's course, with a leg time.
fn terminal_hold(leg: &Leg, fixes: &[FlyableFix]) -> Result<TerminalHold, String> {
    let (Some(before), Some(last)) = (
        fixes
            .len()
            .checked_sub(2)
            .and_then(|index| fixes.get(index)),
        fixes.last(),
    ) else {
        return Err("a hold needs two fixes before it".to_owned());
    };
    let at = leg.fix.as_ref().map(|fix| (fix.lat, fix.lon));
    if at != Some((last.lat, last.lon)) {
        return Err("the hold is flown only at the last fix of the route".to_owned());
    }
    if leg.speed.is_some() {
        return Err("speed constraints are not flown yet".to_owned());
    }
    let course = leg
        .course
        .ok_or_else(|| "the hold has no inbound course".to_owned())?;
    if course.north == North::Magnetic {
        return Err(
            "a magnetic course is not flown yet: give the inbound course true (T)".to_owned(),
        );
    }
    let final_course = final_course_deg(before, last);
    let given = course.degrees.as_f64();
    let off = (given - final_course + 540.0).rem_euclid(360.0) - 180.0;
    if off.abs() > HOLD_COURSE_TOLERANCE_DEG {
        return Err(format!(
            "the hold is flown on the last leg's course, {final_course:.1}° true, not {}° true",
            course.degrees.text(0)
        ));
    }
    let leg_time_s = match leg.hold_length {
        Some(HoldLength::Time(time)) => time.seconds(),
        Some(HoldLength::Distance(_)) => {
            return Err(
                "a hold leg given as a distance is not flown yet: give a leg time".to_owned(),
            );
        }
        None => return Err("the hold has no leg length".to_owned()),
    };
    let turn = leg
        .turn
        .ok_or_else(|| "the hold has no turn direction".to_owned())?;
    Ok(TerminalHold {
        turn,
        inbound_course_deg: given,
        leg_time_s,
        height_m: at_height(leg)?,
    })
}

/// The true course of the great circle from `from` to `to` where it reaches
/// `to`, degrees: the course the mission's hold takes as its inbound course.
pub fn final_course_deg(from: &FlyableFix, to: &FlyableFix) -> f64 {
    let (lat1, lon1) = (
        from.lat.as_f64().to_radians(),
        from.lon.as_f64().to_radians(),
    );
    let (lat2, lon2) = (to.lat.as_f64().to_radians(), to.lon.as_f64().to_radians());
    // The bearing from `to` back to `from`, turned around.
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
