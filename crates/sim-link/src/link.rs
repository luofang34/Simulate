//! What a version 1 link says, value for value.

mod nav;
mod route;

pub use nav::{Camera, Means, NavSet, Outage, Sensor, Span};
pub use route::{Start, Turn, Waypoint};

use crate::Milli;

/// Longest whole link, base and fragment, in characters.
pub const MAX_LINK_CHARS: usize = 8192;
/// Most waypoints a route carries.
pub const MAX_WAYPOINTS: usize = 300;

/// One navigation simulation as a link describes it.
///
/// The fields are open so a host can edit a run, for example its outages,
/// and share it again. [`SimulationLink::fragment`] refuses a value no link
/// can carry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SimulationLink {
    /// The route in fly order, 2 to [`MAX_WAYPOINTS`] waypoints.
    pub route: Vec<Waypoint>,
    /// Flight height, metres in `datum`, of waypoints and a start that give
    /// none; required when one of them gives none.
    pub default_height: Option<Milli>,
    /// What every height is measured from.
    pub datum: HeightDatum,
    /// Where the flight begins; absent means over the first waypoint
    /// heading for the second.
    pub start: Option<Start>,
    /// UTC at the start of the flight.
    pub time: StartTime,
    /// The vehicle flown.
    pub vehicle: Vehicle,
    /// Cruise speed, m/s; absent means the vehicle's own.
    pub cruise_speed: Option<Milli>,
    /// The master seed every simulated sensor's seed derives from.
    pub seed: u64,
    /// The navigation sets compared, each with distinct means.
    pub sets: Vec<NavSet>,
    /// Index into `sets` of the set that steers the flight.
    pub steering_set: usize,
    /// Scheduled outages, at most one entry a sensor.
    pub outages: Vec<Outage>,
    /// The terrain camera; absent means the vehicle's own.
    pub camera: Option<Camera>,
    /// Build the map package and start without asking.
    pub auto_start: bool,
    /// Storage budget of the map package, megabytes, 1 to 65536.
    pub budget_mb: u32,
}

impl SimulationLink {
    /// The set that steers the flight, when `steering_set` names one.
    pub fn steering(&self) -> Option<&NavSet> {
        self.sets.get(self.steering_set)
    }

    /// The height of `own`, or the link's default, metres in `datum`.
    pub fn height_or_default(&self, own: Option<Milli>) -> Option<Milli> {
        own.or(self.default_height)
    }
}

/// What heights are measured from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HeightDatum {
    /// Mean sea level, `msl`.
    MeanSeaLevel,
    /// The WGS84 ellipsoid, `hae`.
    Ellipsoid,
}

impl HeightDatum {
    /// Every datum.
    pub const ALL: [Self; 2] = [Self::MeanSeaLevel, Self::Ellipsoid];

    /// The datum's word in a link.
    pub const fn key(self) -> &'static str {
        match self {
            Self::MeanSeaLevel => "msl",
            Self::Ellipsoid => "hae",
        }
    }
}

/// The vehicle flown.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Vehicle {
    /// The Cessna 172, `c172`.
    C172,
    /// The X500 quadrotor, `x500`.
    X500,
}

impl Vehicle {
    /// Every vehicle.
    pub const ALL: [Self; 2] = [Self::C172, Self::X500];

    /// The vehicle's word in a link.
    pub const fn key(self) -> &'static str {
        match self {
            Self::C172 => "c172",
            Self::X500 => "x500",
        }
    }
}

/// A UTC calendar second. A leap second is not a start time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct StartTime {
    /// Year, 1 to 9999.
    pub year: u16,
    /// Month, 1 to 12.
    pub month: u8,
    /// Day of the month, from 1.
    pub day: u8,
    /// Hour, 0 to 23.
    pub hour: u8,
    /// Minute, 0 to 59.
    pub minute: u8,
    /// Second, 0 to 59.
    pub second: u8,
}

impl StartTime {
    /// Days in this time's month.
    pub const fn days_in_month(year: u16, month: u8) -> u8 {
        let leap =
            year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400));
        match month {
            1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
            4 | 6 | 9 | 11 => 30,
            2 if leap => 29,
            2 => 28,
            _ => 0,
        }
    }

    /// The link form, `YYYYMMDDTHHMMSSZ`.
    pub fn compact(&self) -> String {
        format!(
            "{:04}{:02}{:02}T{:02}{:02}{:02}Z",
            self.year, self.month, self.day, self.hour, self.minute, self.second
        )
    }

    /// ISO 8601, `YYYY-MM-DDTHH:MM:SSZ`.
    pub fn iso8601(&self) -> String {
        format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
            self.year, self.month, self.day, self.hour, self.minute, self.second
        )
    }
}
