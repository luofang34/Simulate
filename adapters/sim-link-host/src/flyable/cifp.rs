//! ARINC 424 / CIFP leg paths, altitude windows, holding patterns, flight-phase
//! roles, and WMM2025 course-to-altitude projection for `#v=2` links.

use sim_link::{
    Bearing, Height, HeightUnit, HoldLength, Leg, Limit, Micro, Milli, North, Role, SimulationLink,
    TurnDirection,
};
use world_core::{DecimalYear, GeodeticPosition};
use world_geomag::wmm2025;

/// Mean Earth radius for nominal `CA` corridor projection, metres.
const MEAN_EARTH_RADIUS_M: f64 = 6_371_008.8;
/// Exact ICAO nautical mile in metres.
const METRES_PER_NM: f64 = 1_852.0;
/// Seconds in one mean Julian year for WMM2025 epoch conversion.
const SECONDS_PER_YEAR: f64 = 31_557_600.0;

/// Altitude constraint stated on a flyable leg, metres in the link's datum.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlyableAltitude {
    /// Cross at `height_m`.
    At(f64),
    /// Cross at or above `height_m`.
    AtOrAbove(f64),
    /// Cross at or below `height_m`.
    AtOrBelow(f64),
    /// Cross within `[lower_m, upper_m]`.
    Window {
        /// Lower bound, metres in the link's datum.
        lower_m: f64,
        /// Upper bound, metres in the link's datum.
        upper_m: f64,
    },
}

impl FlyableAltitude {
    /// Reference 3D height in the link's datum, metres.
    #[must_use]
    pub fn reference_m(self) -> f64 {
        match self {
            Self::At(h) | Self::AtOrAbove(h) | Self::AtOrBelow(h) => h,
            Self::Window { lower_m, upper_m } => 0.5 * (lower_m + upper_m),
        }
    }
}

/// Inbound leg dimension of a flyable holding pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlyableHoldLength {
    /// Inbound leg time in minutes (`> 0`).
    Minutes(f64),
    /// Inbound leg distance in metres (`> 0`).
    DistanceM(f64),
}

/// Holding racetrack geometry (`HA`, `HF`, or `HM`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlyableHold {
    /// Published inbound course to the hold fix, degrees `[0, 360)`.
    pub inbound_course_deg: f64,
    /// Whether `inbound_course_deg` is referenced to magnetic north (`M`).
    pub magnetic: bool,
    /// Turn direction of the holding racetrack (`L` or `R`).
    pub turn: TurnDirection,
    /// Inbound leg time or distance.
    pub length: FlyableHoldLength,
}

/// ARINC 424 path terminator flown by the vehicle's software.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlyablePath {
    /// `IF`: initial fix.
    InitialFix,
    /// `TF`: great-circle track from the preceding fix.
    TrackToFix,
    /// `DF`: direct-to track from present position.
    DirectToFix,
    /// `CF`: published course to fix (`True` or `Magnetic`).
    CourseToFix {
        /// Course in degrees `[0, 360)`.
        course_deg: f64,
        /// Whether `course_deg` is magnetic (`M`).
        magnetic: bool,
    },
    /// `CA`: published course until altitude constraint (`True` or `Magnetic`).
    CourseToAltitude {
        /// Course in degrees `[0, 360)`.
        course_deg: f64,
        /// Whether `course_deg` is magnetic (`M`).
        magnetic: bool,
    },
    /// `HA`: hold at fix until altitude constraint.
    HoldToAltitude(FlyableHold),
    /// `HF`: single circuit in hold terminating at fix.
    HoldToFix(FlyableHold),
    /// `HM`: manual hold at fix.
    HoldManual(FlyableHold),
}

/// Operational flight-phase role derived from a leg's CIFP roles and missed-approach mark.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlyableLegRole {
    /// Departure segment (`TERM 1.0 NM`).
    Departure,
    /// En-route segment (`ENR 2.0 NM`).
    Enroute,
    /// Terminal arrival / initial or intermediate approach (`TERM 1.0 NM`).
    Arrival,
    /// Final approach segment (`APR 0.3 NM`).
    Approach,
    /// Missed approach segment (`TERM 1.0 NM`).
    MissedApproach,
}

/// Tracks CIFP procedure segment progression (`IAF`/`IF`/`FAF`/`MAP`/`MAHF` + `m`)
/// across consecutive legs of a `#v=2` route.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct PhaseRoleTracker {
    in_arrival: bool,
    passed_faf: bool,
    in_missed: bool,
}

impl PhaseRoleTracker {
    /// Classifies `leg` and advances procedure state past `leg`'s terminating fix.
    pub(super) fn classify(&mut self, leg: &Leg) -> Option<FlyableLegRole> {
        if leg.missed_approach || leg.roles.contains(&Role::Mahf) {
            self.in_missed = true;
        }
        let has_final_fix_role = leg
            .roles
            .iter()
            .any(|r| matches!(r, Role::Sdf | Role::Map | Role::Fep | Role::Rwy));
        let has_initial_fix_role = leg
            .roles
            .iter()
            .any(|r| matches!(r, Role::Iaf | Role::If | Role::Facf | Role::Faf));
        let role = if self.in_missed {
            Some(FlyableLegRole::MissedApproach)
        } else if has_final_fix_role || self.passed_faf {
            Some(FlyableLegRole::Approach)
        } else if has_initial_fix_role || self.in_arrival {
            Some(FlyableLegRole::Arrival)
        } else {
            None
        };
        if has_initial_fix_role {
            self.in_arrival = true;
        }
        if leg.roles.contains(&Role::Faf) {
            self.passed_faf = true;
        }
        if leg
            .roles
            .iter()
            .any(|r| matches!(r, Role::Map | Role::Fep | Role::Rwy))
        {
            self.in_missed = true;
        }
        role
    }
}

/// Extracts `leg.altitude` as a [`FlyableAltitude`], refusing `FL` flight levels.
pub(super) fn flyable_altitude(leg: &Leg) -> Result<Option<FlyableAltitude>, String> {
    let datum_height = |height: Height| -> Result<f64, String> {
        if height.unit == HeightUnit::FlightLevel {
            Err("flight levels need a pressure altitude and are not flown yet".to_owned())
        } else {
            Ok(height.metres())
        }
    };
    match leg.altitude {
        None => Ok(None),
        Some(Limit::At(h)) => Ok(Some(FlyableAltitude::At(datum_height(h)?))),
        Some(Limit::AtOrAbove(h)) => Ok(Some(FlyableAltitude::AtOrAbove(datum_height(h)?))),
        Some(Limit::AtOrBelow(h)) => Ok(Some(FlyableAltitude::AtOrBelow(datum_height(h)?))),
        Some(Limit::Between { lower, upper }) => Ok(Some(FlyableAltitude::Window {
            lower_m: datum_height(lower)?,
            upper_m: datum_height(upper)?,
        })),
    }
}

/// Extracts `leg.rnp` in nautical miles.
pub(super) fn flyable_rnp_nm(leg: &Leg) -> Option<f64> {
    leg.rnp.map(|distance| distance.metres() / METRES_PER_NM)
}

/// Extracts a required [`Bearing`] course from `leg` (`CF`, `CA`).
pub(super) fn require_course(leg: &Leg) -> Result<(f64, bool), String> {
    let course = leg
        .course
        .ok_or_else(|| "the leg has no course".to_owned())?;
    Ok((course.degrees.as_f64(), course.north == North::Magnetic))
}

/// Extracts [`FlyableHold`] geometry from an `HA`, `HF`, or `HM` leg.
pub(super) fn flyable_hold(leg: &Leg) -> Result<FlyableHold, String> {
    let (inbound_course_deg, magnetic) = require_course(leg)?;
    let length = match leg.hold_length {
        Some(HoldLength::Time(time)) => FlyableHoldLength::Minutes(time.seconds() / 60.0),
        Some(HoldLength::Distance(distance)) => FlyableHoldLength::DistanceM(distance.metres()),
        None => return Err("the hold has no leg length".to_owned()),
    };
    let turn = leg
        .turn
        .ok_or_else(|| "the hold has no turn direction".to_owned())?;
    Ok(FlyableHold {
        inbound_course_deg,
        magnetic,
        turn,
        length,
    })
}

/// Projects a nominal geodetic endpoint for a `CA` (`CourseToAltitude`) leg
/// along `course` from `(prev_lat, prev_lon)` so corridor planning and summary
/// profiles cover the climb segment cleanly.
pub(super) fn project_course_to_altitude(
    link: &SimulationLink,
    prev: (Micro, Micro, Option<f64>),
    course: Bearing,
    target_alt: FlyableAltitude,
) -> (Micro, Micro) {
    let (lat1_deg, lon1_deg) = (prev.0.as_f64(), prev.1.as_f64());
    let prev_h = prev
        .2
        .or_else(|| link.default_height.map(Milli::as_f64))
        .unwrap_or(0.0);
    let dh_m = (target_alt.reference_m() - prev_h).abs();
    let distance_m = (dh_m * 28.0).clamp(2_000.0, 25_000.0);
    let mut true_rad = course.degrees.as_f64().to_radians();
    if course.north == North::Magnetic {
        let year =
            (1970.0 + link.time.unix_seconds() as f64 / SECONDS_PER_YEAR).clamp(2025.0, 2030.0);
        if let Ok(field) = wmm2025(
            GeodeticPosition::wgs84(lat1_deg, lon1_deg, prev_h),
            DecimalYear(year),
        ) {
            true_rad += field.declination_rad();
        }
    }
    let lat1 = lat1_deg.to_radians();
    let lon1 = lon1_deg.to_radians();
    let angular = distance_m / MEAN_EARTH_RADIUS_M;
    let sin_lat2 = lat1.sin() * angular.cos() + lat1.cos() * angular.sin() * true_rad.cos();
    let lat2 = sin_lat2.clamp(-1.0, 1.0).asin();
    let y = true_rad.sin() * angular.sin() * lat1.cos();
    let x = angular.cos() - lat1.sin() * lat2.sin();
    let lon2 = lon1 + y.atan2(x);
    let lon2_deg = ((lon2.to_degrees() + 540.0).rem_euclid(360.0)) - 180.0;
    let to_micro = |deg: f64, limit: i64| {
        let scaled = (deg * 1_000_000.0).round() as i64;
        Micro::from_scaled(scaled.clamp(-limit, limit))
    };
    (
        to_micro(lat2.to_degrees(), 90_000_000),
        to_micro(lon2_deg, 180_000_000),
    )
}
