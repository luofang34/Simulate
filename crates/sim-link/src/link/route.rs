//! The route's waypoints and the start state.

use crate::{Micro, Milli};

/// How the vehicle turns at a waypoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Turn {
    /// Anticipate the turn and cut the corner; `b` or nothing in a link.
    #[default]
    FlyBy,
    /// Cross the fix before turning, `o`.
    FlyOver,
}

/// One waypoint of the route.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Waypoint {
    /// Latitude, degrees north, -90 to 90.
    pub lat: Micro,
    /// Longitude, degrees east, -180 to 180.
    pub lon: Micro,
    /// Height to cross at, metres in the link's datum, -500 to 30000;
    /// absent means the link's default height.
    pub height: Option<Milli>,
    /// How the vehicle turns here.
    pub turn: Turn,
    /// Name, 1 to 32 of `[A-Za-z0-9_-]`. Names may repeat: an out-and-back
    /// route names a fix twice.
    pub name: Option<String>,
}

impl Waypoint {
    /// Latitude, degrees north.
    pub fn lat_deg(&self) -> f64 {
        self.lat.as_f64()
    }

    /// Longitude, degrees east.
    pub fn lon_deg(&self) -> f64 {
        self.lon.as_f64()
    }
}

/// Where and how the flight begins, airborne.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Start {
    /// Latitude, degrees north, -90 to 90.
    pub lat: Micro,
    /// Longitude, degrees east, -180 to 180.
    pub lon: Micro,
    /// Height, metres in the link's datum, -500 to 30000; absent means the
    /// link's default height.
    pub height: Option<Milli>,
    /// True heading, degrees, 0 up to but not 360; absent means toward the
    /// first waypoint.
    pub heading: Option<Milli>,
}

impl Start {
    /// Latitude, degrees north.
    pub fn lat_deg(&self) -> f64 {
        self.lat.as_f64()
    }

    /// Longitude, degrees east.
    pub fn lon_deg(&self) -> f64 {
        self.lon.as_f64()
    }
}
