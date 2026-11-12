//! Navigation sets, sensor outages and the terrain camera.

use crate::Milli;

/// A navigation means a set may use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Means {
    /// The strapdown inertial navigator, `ins`; every set has it.
    Ins,
    /// Barometric height, `baro`.
    Baro,
    /// Satellite fixes, `gnss`.
    Gnss,
    /// Star sightings, `stars`.
    Stars,
    /// Terrain-relative visual fixes, `visual`.
    Visual,
}

impl Means {
    /// Every means.
    pub const ALL: [Self; 5] = [Self::Ins, Self::Baro, Self::Gnss, Self::Stars, Self::Visual];

    /// The means' word in a link.
    pub const fn key(self) -> &'static str {
        match self {
            Self::Ins => "ins",
            Self::Baro => "baro",
            Self::Gnss => "gnss",
            Self::Stars => "stars",
            Self::Visual => "visual",
        }
    }

    const fn bit(self) -> u8 {
        match self {
            Self::Ins => 1,
            Self::Baro => 2,
            Self::Gnss => 4,
            Self::Stars => 8,
            Self::Visual => 16,
        }
    }
}

/// One navigation set: its means in the order the link names them.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct NavSet(pub Vec<Means>);

impl NavSet {
    /// Whether the set uses `means`.
    pub fn contains(&self, means: Means) -> bool {
        self.0.contains(&means)
    }

    /// Whether both sets use the same means, in any order.
    pub fn same_means(&self, other: &Self) -> bool {
        self.mask() == other.mask()
    }

    /// The link form, means joined by `.`; it also names the set.
    pub fn text(&self) -> String {
        let keys: Vec<&str> = self.0.iter().map(|means| means.key()).collect();
        keys.join(".")
    }

    fn mask(&self) -> u8 {
        self.0.iter().fold(0, |mask, means| mask | means.bit())
    }
}

/// A simulated sensor an outage withholds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Sensor {
    /// The GNSS receiver, `gnss`.
    Gnss,
    /// The barometric altimeter, `baro`.
    Baro,
    /// The star camera, `stars`.
    Stars,
    /// The terrain camera, `cam`.
    Cam,
}

impl Sensor {
    /// Every sensor an outage can name.
    pub const ALL: [Self; 4] = [Self::Gnss, Self::Baro, Self::Stars, Self::Cam];

    /// The sensor's word in a link.
    pub const fn key(self) -> &'static str {
        match self {
            Self::Gnss => "gnss",
            Self::Baro => "baro",
            Self::Stars => "stars",
            Self::Cam => "cam",
        }
    }
}

/// One span in which a sensor delivers nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Span {
    /// Seconds from the start of the flight.
    pub start: Milli,
    /// Seconds from the start, after `start`; absent runs to the end of the
    /// flight.
    pub end: Option<Milli>,
}

/// The outages of one sensor: spans in increasing order that do not
/// overlap; one may begin where the one before ends, and only the last may
/// run to the end of the flight.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Outage {
    /// The sensor withheld.
    pub sensor: Sensor,
    /// When; at least one span.
    pub spans: Vec<Span>,
}

/// A spoofer dragging the GNSS receiver's fixes away from the truth. At a
/// span's start it captures the receiver with no offset and the reported
/// position leaves the true one at `rate` toward `bearing`, the reported
/// velocity carrying the same drag; a span's end returns the true sky at
/// once. A span that begins where the one before ends carries its drag on.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Spoof {
    /// The drag, m/s, above 0 and at most 100.
    pub rate: Milli,
    /// The direction of the drag, degrees from true north, 0 to under 360.
    pub bearing: Milli,
    /// When, as an [`Outage`]'s spans; at least one.
    pub spans: Vec<Span>,
}

/// The terrain camera.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Camera {
    /// Image width, pixels, 16 to 8192.
    pub width_px: u32,
    /// Image height, pixels, 16 to 8192.
    pub height_px: u32,
    /// Horizontal field of view, degrees, 1 to 179.
    pub fov_deg: Milli,
    /// Angle of the optical axis below the horizon, degrees, 0 to 90; 90
    /// looks straight down.
    pub depression_deg: Milli,
    /// Frames a second, 0.1 to 120.
    pub rate_hz: Milli,
}
