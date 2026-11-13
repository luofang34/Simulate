//! Per-leg CIFP procedure metadata carried on [`super::ChainTrialSpec`] when a
//! route declares non-default ARINC 424 leg paths, altitude windows, holds,
//! mandatory turns, flight-phase roles, or RNP bounds.

use serde::{Deserialize, Serialize};
use sim_flight_chain::{
    LegRole, PlannedAltitude, PlannedCourseReference, PlannedHold, PlannedHoldLength, PlannedPath,
    RouteFix, WaypointTurnDirection,
};

use super::HoldTurn;

/// Altitude constraint on a planned leg, metres above the WGS84 ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ChainAltitudeSpec {
    /// Cross at `altitude_m`.
    At {
        /// Target ellipsoidal height, metres.
        altitude_m: f64,
    },
    /// Cross at or above `altitude_m`.
    AtOrAbove {
        /// Minimum ellipsoidal height, metres.
        altitude_m: f64,
    },
    /// Cross at or below `altitude_m`.
    AtOrBelow {
        /// Maximum ellipsoidal height, metres.
        altitude_m: f64,
    },
    /// Cross between `lower_m` and `upper_m`.
    Window {
        /// Lower ellipsoidal height bound, metres.
        lower_m: f64,
        /// Upper ellipsoidal height bound, metres.
        upper_m: f64,
    },
    /// No crossing constraint; `reference_m` supplies the 3D fix height.
    Unconstrained {
        /// Reference ellipsoidal height, metres.
        reference_m: f64,
    },
}

impl ChainAltitudeSpec {
    /// Converts to Pilotage's [`PlannedAltitude`].
    #[must_use]
    pub const fn to_planned(self) -> PlannedAltitude {
        match self {
            Self::At { altitude_m } => PlannedAltitude::At(altitude_m),
            Self::AtOrAbove { altitude_m } => PlannedAltitude::AtOrAbove(altitude_m),
            Self::AtOrBelow { altitude_m } => PlannedAltitude::AtOrBelow(altitude_m),
            Self::Window { lower_m, upper_m } => PlannedAltitude::Window { lower_m, upper_m },
            Self::Unconstrained { reference_m } => PlannedAltitude::Unconstrained { reference_m },
        }
    }

    /// Reference 3D altitude above the WGS84 ellipsoid, metres.
    #[must_use]
    pub fn reference_m(self) -> f64 {
        self.to_planned().reference_m()
    }
}

/// Inbound leg dimension of a planned holding racetrack.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ChainHoldLengthSpec {
    /// Inbound leg duration in minutes.
    Minutes {
        /// Leg duration, minutes (`> 0`).
        minutes: f64,
    },
    /// Inbound leg distance in metres.
    DistanceM {
        /// Leg length, metres (`> 0`).
        meters: f64,
    },
}

/// Racetrack geometry of a planned in-route or terminal hold (`HA`, `HF`, `HM`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChainHoldSpec {
    /// Inbound course to the holding fix, degrees `[0, 360)`.
    pub inbound_course_deg: f64,
    /// Whether `inbound_course_deg` is referenced to magnetic north (`P4`).
    #[serde(default)]
    pub magnetic: bool,
    /// Turn direction of the holding racetrack (`left` or `right`).
    pub turn: HoldTurn,
    /// Inbound leg time or distance.
    pub length: ChainHoldLengthSpec,
}

impl ChainHoldSpec {
    /// Converts to Pilotage's [`PlannedHold`].
    #[must_use]
    pub fn to_planned(self) -> PlannedHold {
        PlannedHold {
            inbound_course_rad: self.inbound_course_deg.to_radians(),
            reference: course_reference(self.magnetic),
            turn: turn_direction(self.turn),
            length: match self.length {
                ChainHoldLengthSpec::Minutes { minutes } => PlannedHoldLength::Minutes(minutes),
                ChainHoldLengthSpec::DistanceM { meters } => PlannedHoldLength::DistanceM(meters),
            },
        }
    }
}

/// ARINC 424 path terminator of a planned leg.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ChainPathSpec {
    /// Initial fix (`IF`).
    InitialFix,
    /// Great-circle track from the preceding fix (`TF`).
    TrackToFix,
    /// Great-circle track from present position (`DF`).
    DirectToFix,
    /// Published course terminating at the fix (`CF`).
    CourseToFix {
        /// Course in degrees `[0, 360)`.
        course_deg: f64,
        /// Whether `course_deg` is magnetic (`P4`).
        #[serde(default)]
        magnetic: bool,
    },
    /// Published course terminating at the leg's altitude constraint (`CA`).
    CourseToAltitude {
        /// Course in degrees `[0, 360)`.
        course_deg: f64,
        /// Whether `course_deg` is magnetic (`P4`).
        #[serde(default)]
        magnetic: bool,
    },
    /// Hold-to-altitude (`HA`).
    HoldToAltitude {
        /// Racetrack geometry.
        hold: ChainHoldSpec,
    },
    /// Hold-to-fix (`HF`, single circuit).
    HoldToFix {
        /// Racetrack geometry.
        hold: ChainHoldSpec,
    },
    /// Hold-to-manual exit (`HM`).
    HoldManual {
        /// Racetrack geometry.
        hold: ChainHoldSpec,
    },
}

impl ChainPathSpec {
    /// Converts to Pilotage's [`PlannedPath`].
    #[must_use]
    pub fn to_planned(self) -> PlannedPath {
        match self {
            Self::InitialFix => PlannedPath::InitialFix,
            Self::TrackToFix => PlannedPath::TrackToFix,
            Self::DirectToFix => PlannedPath::DirectToFix,
            Self::CourseToFix {
                course_deg,
                magnetic,
            } => PlannedPath::CourseToFix {
                course_rad: course_deg.to_radians(),
                reference: course_reference(magnetic),
            },
            Self::CourseToAltitude {
                course_deg,
                magnetic,
            } => PlannedPath::CourseToAltitude {
                course_rad: course_deg.to_radians(),
                reference: course_reference(magnetic),
            },
            Self::HoldToAltitude { hold } => PlannedPath::HoldToAltitude(hold.to_planned()),
            Self::HoldToFix { hold } => PlannedPath::HoldToFix(hold.to_planned()),
            Self::HoldManual { hold } => PlannedPath::HoldManual(hold.to_planned()),
        }
    }
}

/// Operational flight-phase role of a planned leg.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChainLegRole {
    /// Standard instrument departure (`SID` / `TERM 1.0 NM`).
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

impl ChainLegRole {
    /// Converts to Pilotage's [`LegRole`].
    #[must_use]
    pub const fn to_planned(self) -> LegRole {
        match self {
            Self::Departure => LegRole::Departure,
            Self::Enroute => LegRole::Enroute,
            Self::Arrival => LegRole::Arrival,
            Self::Approach => LegRole::Approach,
            Self::MissedApproach => LegRole::MissedApproach,
        }
    }
}

/// Full per-leg specification forwarded into Pilotage's `WaypointPlan`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChainLegSpec {
    /// Waypoint identifier (`1..=32` ASCII characters).
    pub ident: String,
    /// Geodetic latitude, degrees north.
    pub lat_deg: f64,
    /// Geodetic longitude, degrees east.
    pub lon_deg: f64,
    /// Altitude constraint above the WGS84 ellipsoid.
    pub altitude: ChainAltitudeSpec,
    /// Cross the fix before turning (`true`) rather than anticipating (`false`).
    #[serde(default)]
    pub fly_over: bool,
    /// Optional mandatory turn direction onto this leg.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turn_direction: Option<HoldTurn>,
    /// ARINC 424 leg path ending at this waypoint.
    pub path: ChainPathSpec,
    /// Optional flight-phase role of this leg.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<ChainLegRole>,
    /// Optional procedure-published RNP on this leg, nautical miles (`> 0`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rnp_nm: Option<f64>,
}

impl ChainLegSpec {
    /// Converts to a [`RouteFix`] for [`sim_flight_chain::ChainRoute`].
    #[must_use]
    pub fn to_route_fix(&self) -> RouteFix {
        RouteFix {
            ident: Some(self.ident.clone()),
            lat_deg: self.lat_deg,
            lon_deg: self.lon_deg,
            altitude_m: self.altitude.reference_m(),
            altitude: Some(self.altitude.to_planned()),
            fly_over: self.fly_over,
            turn_direction: self.turn_direction.map(turn_direction),
            path: Some(self.path.to_planned()),
            role: self.role.map(ChainLegRole::to_planned),
            rnp_nm: self.rnp_nm,
        }
    }
}

const fn course_reference(magnetic: bool) -> PlannedCourseReference {
    if magnetic {
        PlannedCourseReference::Magnetic
    } else {
        PlannedCourseReference::True
    }
}

const fn turn_direction(turn: HoldTurn) -> WaypointTurnDirection {
    match turn {
        HoldTurn::Left => WaypointTurnDirection::Left,
        HoldTurn::Right => WaypointTurnDirection::Right,
    }
}
