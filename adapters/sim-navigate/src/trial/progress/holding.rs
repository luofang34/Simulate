//! The holding pattern a flight's host flies after its plan, as the
//! progress stream records it: the same parts, names and units as the
//! host's own holding pattern, so a map draws the racetrack from the record.

use serde::{Deserialize, Serialize};

/// One holding pattern.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HoldingPatternRecord {
    /// The identifier of the holding fix.
    pub fix_ident: String,
    /// The holding fix. Its altitude is the holding altitude.
    pub fix: HoldingFix,
    /// The inbound course to the fix, radians clockwise from true north
    /// in `[0, 2π)`.
    pub inbound_course_rad: f64,
    /// The direction of the turns.
    pub turn: HoldingTurnName,
    /// The standard time of the inbound leg, seconds.
    pub leg_time_s: f64,
    /// The radius of the two turns over the ground, metres.
    pub turn_radius_m: f64,
    /// The length of the inbound leg over the ground in the wind known when
    /// the pattern started, metres.
    pub leg_length_m: f64,
}

/// A position on the WGS84 ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HoldingFix {
    /// Geodetic latitude, radians.
    pub latitude_rad: f64,
    /// Longitude, radians east.
    pub longitude_rad: f64,
    /// Height above the ellipsoid, metres.
    pub altitude_m: f64,
}

/// The direction of a pattern's turns.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldingTurnName {
    /// Right turns: the standard pattern.
    Right,
    /// Left turns.
    Left,
}

/// The entry into a pattern, as the course at the fix selects it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldingEntryName {
    /// Turn at the fix onto the outbound leg.
    Direct,
    /// Outbound on the non-holding side, then back to the inbound track.
    Parallel,
    /// 30° off the outbound course into the holding side, then inbound.
    Teardrop,
}

/// The part of a pattern flown.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "leg", content = "entry", rename_all = "snake_case")]
pub enum HoldingLegName {
    /// Direct to the fix, before the entry.
    ToFix,
    /// The entry the course at the fix selected.
    Entry(HoldingEntryName),
    /// The inbound leg to the fix.
    Inbound,
    /// The turn from the fix to the outbound leg.
    FixTurn,
    /// The outbound leg.
    Outbound,
    /// The turn from the outbound leg to the inbound leg.
    OutboundTurn,
}
