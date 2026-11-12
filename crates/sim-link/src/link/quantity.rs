//! The measured values a leg carries, each with the unit it was written in
//! so a value from a chart in feet or knots survives the link digit for
//! digit.

use crate::Milli;

/// Metres in one international foot.
const METRES_PER_FOOT: f64 = 0.3048;
/// Metres in one international nautical mile.
const METRES_PER_NAUTICAL_MILE: f64 = 1852.0;

/// The unit of a [`Height`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HeightUnit {
    /// Metres in the link's datum; written as a bare number.
    Metre,
    /// Feet in the link's datum, `ft`.
    Foot,
    /// A flight level, `FL`: hundreds of feet of standard pressure
    /// altitude, not a height in the link's datum.
    FlightLevel,
}

/// A vertical position or height.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Height {
    /// The number as written.
    pub value: Milli,
    /// What the number counts.
    pub unit: HeightUnit,
}

impl Height {
    /// Metres; a flight level counts as its standard pressure altitude.
    pub fn metres(self) -> f64 {
        match self.unit {
            HeightUnit::Metre => self.value.as_f64(),
            HeightUnit::Foot => self.value.as_f64() * METRES_PER_FOOT,
            HeightUnit::FlightLevel => self.value.as_f64() * 100.0 * METRES_PER_FOOT,
        }
    }

    /// The link form: `1500`, `5000ft`, `FL120`.
    pub fn text(self) -> String {
        match self.unit {
            HeightUnit::Metre => self.value.text(0),
            HeightUnit::Foot => format!("{}ft", self.value.text(0)),
            HeightUnit::FlightLevel => format!("FL{}", self.value.text(0)),
        }
    }
}

/// The unit of a [`Distance`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DistanceUnit {
    /// Metres; written as a bare number, or with `m` where a time could
    /// stand instead.
    Metre,
    /// Nautical miles, `nm`.
    NauticalMile,
}

/// A horizontal distance, more than zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Distance {
    /// The number as written.
    pub value: Milli,
    /// What the number counts.
    pub unit: DistanceUnit,
}

impl Distance {
    /// Metres.
    pub fn metres(self) -> f64 {
        match self.unit {
            DistanceUnit::Metre => self.value.as_f64(),
            DistanceUnit::NauticalMile => self.value.as_f64() * METRES_PER_NAUTICAL_MILE,
        }
    }

    /// The link form: `7400` or `4nm`.
    pub fn text(self) -> String {
        match self.unit {
            DistanceUnit::Metre => self.value.text(0),
            DistanceUnit::NauticalMile => format!("{}nm", self.value.text(0)),
        }
    }
}

/// The unit of a [`Speed`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpeedUnit {
    /// Metres a second; written as a bare number.
    MetrePerSecond,
    /// Knots, `kt`.
    Knot,
}

/// An airspeed, more than zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Speed {
    /// The number as written.
    pub value: Milli,
    /// What the number counts.
    pub unit: SpeedUnit,
}

impl Speed {
    /// Metres a second.
    pub fn metres_per_second(self) -> f64 {
        match self.unit {
            SpeedUnit::MetrePerSecond => self.value.as_f64(),
            SpeedUnit::Knot => self.value.as_f64() * METRES_PER_NAUTICAL_MILE / 3600.0,
        }
    }

    /// The link form: `60` or `120kt`.
    pub fn text(self) -> String {
        match self.unit {
            SpeedUnit::MetrePerSecond => self.value.text(0),
            SpeedUnit::Knot => format!("{}kt", self.value.text(0)),
        }
    }
}

/// The unit of a [`Duration`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimeUnit {
    /// Seconds, `s`.
    Second,
    /// Minutes, `min`.
    Minute,
}

/// A span of time, more than zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Duration {
    /// The number as written.
    pub value: Milli,
    /// What the number counts.
    pub unit: TimeUnit,
}

impl Duration {
    /// Seconds.
    pub fn seconds(self) -> f64 {
        match self.unit {
            TimeUnit::Second => self.value.as_f64(),
            TimeUnit::Minute => self.value.as_f64() * 60.0,
        }
    }

    /// The link form, always with its unit: `60s` or `1.5min`.
    pub fn text(self) -> String {
        match self.unit {
            TimeUnit::Second => format!("{}s", self.value.text(0)),
            TimeUnit::Minute => format!("{}min", self.value.text(0)),
        }
    }
}

/// The length of a holding pattern's inbound leg: a time or a distance,
/// as the procedure publishes it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HoldLength {
    /// A leg time.
    Time(Duration),
    /// A leg distance.
    Distance(Distance),
}

impl HoldLength {
    /// The link form, always with its unit: `1min`, `60s`, `4nm`, `7400m`.
    pub fn text(self) -> String {
        match self {
            Self::Time(time) => time.text(),
            Self::Distance(Distance {
                value,
                unit: DistanceUnit::Metre,
            }) => format!("{}m", value.text(0)),
            Self::Distance(distance) => distance.text(),
        }
    }
}

/// The north a direction is measured from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum North {
    /// True north, `T`.
    True,
    /// Magnetic north at the place and time of the flight, `M`.
    Magnetic,
}

/// A course, heading or radial, degrees clockwise from its north, 0 up to
/// but not 360.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Bearing {
    /// Degrees.
    pub degrees: Milli,
    /// What north the degrees count from.
    pub north: North,
}

impl Bearing {
    /// The link form: `271.5T` or `90M`.
    pub fn text(self) -> String {
        let north = match self.north {
            North::True => "T",
            North::Magnetic => "M",
        };
        format!("{}{north}", self.degrees.text(0))
    }
}

/// A constraint on a value: at it, at or above it, at or below it, or
/// between two of them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Limit<T> {
    /// Exactly this value: `1500`.
    At(T),
    /// This value or more: `1500+`.
    AtOrAbove(T),
    /// This value or less: `1500-`.
    AtOrBelow(T),
    /// From `lower` to `upper`: `1500..3000`.
    Between {
        /// The least value.
        lower: T,
        /// The greatest value.
        upper: T,
    },
}

impl<T: Copy> Limit<T> {
    /// The link form, each value written by `text`.
    pub fn text(self, text: impl Fn(T) -> String) -> String {
        match self {
            Self::At(value) => text(value),
            Self::AtOrAbove(value) => format!("{}+", text(value)),
            Self::AtOrBelow(value) => format!("{}-", text(value)),
            Self::Between { lower, upper } => format!("{}..{}", text(lower), text(upper)),
        }
    }
}

/// What the second altitude of an approach fix is (ARINC 424 altitude
/// descriptions G, H, I, J, V, X and Y pair it with the first).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SecondAltitudeKind {
    /// The glide slope or glide path altitude at the fix, `G`.
    GlidePath,
    /// The altitude the glide slope or glide path is intercepted at, `I`.
    Intercept,
    /// The altitude on the published vertical angle at the fix, `V`.
    VerticalAngle,
}

impl SecondAltitudeKind {
    /// Every kind.
    pub const ALL: [Self; 3] = [Self::GlidePath, Self::Intercept, Self::VerticalAngle];

    /// The kind's letter in a link.
    pub const fn letter(self) -> char {
        match self {
            Self::GlidePath => 'G',
            Self::Intercept => 'I',
            Self::VerticalAngle => 'V',
        }
    }
}

/// The second altitude of an approach fix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SecondAltitude {
    /// What the altitude is.
    pub kind: SecondAltitudeKind,
    /// The altitude.
    pub height: Height,
}
