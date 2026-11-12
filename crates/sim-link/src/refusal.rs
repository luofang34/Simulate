//! Why a link is refused: the parameter, where in it, and the reason.

use std::fmt;

use thiserror::Error;

use crate::NumberFault;

/// A parameter of the link, in the order a link is written.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Parameter {
    /// The link as a whole.
    Link,
    /// `v`, the format version.
    V,
    /// `r`, the route.
    R,
    /// `h`, the default flight height.
    H,
    /// `hd`, the height datum.
    Hd,
    /// `start`, where the flight begins.
    Start,
    /// `t`, the start time.
    T,
    /// `veh`, the vehicle.
    Veh,
    /// `spd`, the cruise speed.
    Spd,
    /// `seed`, the master sensor seed.
    Seed,
    /// `nav`, the navigation sets.
    Nav,
    /// `fly`, the steering set.
    Fly,
    /// `out`, the outages.
    Out,
    /// `spoof`, the spoofing of the GNSS receiver.
    Spoof,
    /// `cam`, the terrain camera.
    Cam,
    /// `go`, start without asking.
    Go,
    /// `mb`, the storage budget.
    Mb,
}

impl Parameter {
    /// Every key a link may carry, in the order a link is written.
    pub const KEYS: [Self; 16] = [
        Self::V,
        Self::R,
        Self::H,
        Self::Hd,
        Self::Start,
        Self::T,
        Self::Veh,
        Self::Spd,
        Self::Seed,
        Self::Nav,
        Self::Fly,
        Self::Out,
        Self::Spoof,
        Self::Cam,
        Self::Go,
        Self::Mb,
    ];

    /// The parameter's key; `link` for the link as a whole.
    pub const fn key(self) -> &'static str {
        match self {
            Self::Link => "link",
            Self::V => "v",
            Self::R => "r",
            Self::H => "h",
            Self::Hd => "hd",
            Self::Start => "start",
            Self::T => "t",
            Self::Veh => "veh",
            Self::Spd => "spd",
            Self::Seed => "seed",
            Self::Nav => "nav",
            Self::Fly => "fly",
            Self::Out => "out",
            Self::Spoof => "spoof",
            Self::Cam => "cam",
            Self::Go => "go",
            Self::Mb => "mb",
        }
    }

    /// The parameter with `key`.
    pub fn from_key(key: &str) -> Option<Self> {
        Self::KEYS
            .into_iter()
            .find(|parameter| parameter.key() == key)
    }
}

impl fmt::Display for Parameter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.key())
    }
}

/// A link that is not a version 2 link, and where it stops being one.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub struct Refusal {
    /// The parameter at fault.
    pub parameter: Parameter,
    /// Which entry of the parameter's list, from 1: the leg of `r`,
    /// the set of `nav`, the sensor entry of `out`.
    pub item: Option<usize>,
    /// Characters before the fault in the fragment, after its percent
    /// escapes are resolved; the fragment's length for a missing parameter.
    pub offset: usize,
    /// What is wrong.
    pub reason: Reason,
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.parameter)?;
        if let Some(item) = self.item {
            write!(f, ", entry {item}")?;
        }
        write!(
            f,
            ", character {}: {}",
            self.offset.saturating_add(1),
            self.reason
        )
    }
}

/// What is wrong with a link.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum Reason {
    /// The version is not one this build reads.
    #[error("version {found:?} is not supported; this build reads version 2")]
    UnsupportedVersion {
        /// The version given.
        found: String,
    },
    /// A version 1 link, whose route held points and not legs.
    #[error(
        "version 1 links are retired: regenerate the link with fpl_to_simulate_url.py, \
         which writes version 2"
    )]
    RetiredVersion,
    /// The link is longer than a link may be.
    #[error("{length} characters; a link has at most {limit}")]
    TooLong {
        /// Characters in the link.
        length: usize,
        /// The most a link has.
        limit: usize,
    },
    /// A required parameter is absent.
    #[error("the parameter is missing")]
    Missing,
    /// A parameter appears twice.
    #[error("the parameter appears twice")]
    Repeated,
    /// A key no version 2 link has.
    #[error("unknown parameter {name:?}")]
    UnknownParameter {
        /// The key given.
        name: String,
    },
    /// A value or field that must say something is empty.
    #[error("empty; expected {expected}")]
    Empty {
        /// What belongs there.
        expected: &'static str,
    },
    /// The text does not have the parameter's shape.
    #[error("{text:?} is not {expected}")]
    Malformed {
        /// The text given.
        text: String,
        /// The shape it must have.
        expected: &'static str,
    },
    /// A percent escape that is not `%` and two hex digits of an ASCII
    /// character, or a character outside ASCII.
    #[error("{text:?} is not an ASCII character or a percent escape of one")]
    BadEscape {
        /// The text given.
        text: String,
    },
    /// The text is not a number the parameter takes.
    #[error("{text:?} is not a number here: {}", number_fault(*.fault, *.decimals))]
    BadNumber {
        /// The text given.
        text: String,
        /// Most fractional digits the quantity keeps.
        decimals: u32,
        /// What is wrong with it.
        fault: NumberFault,
    },
    /// A number outside what the parameter allows.
    #[error("{what} {value} is outside {range}")]
    OutOfRange {
        /// The quantity.
        what: &'static str,
        /// The value given.
        value: String,
        /// The allowed range.
        range: &'static str,
    },
    /// A word that is not one of the parameter's.
    #[error("unknown {what} {name:?}; choose from {choices}")]
    UnknownWord {
        /// What the word should name: vehicle, means, sensor, datum, turn.
        what: &'static str,
        /// The word given.
        name: String,
        /// The words allowed.
        choices: &'static str,
    },
    /// A fix name that is not 1 to 32 of `[A-Za-z0-9_-]`.
    #[error("name {text:?} is not 1 to 32 of A-Z a-z 0-9 _ -")]
    BadName {
        /// The name given.
        text: String,
    },
    /// More legs than a route carries.
    #[error("{count} legs; a route has at most {limit}")]
    TooManyLegs {
        /// Legs given.
        count: usize,
        /// The most a route has.
        limit: usize,
    },
    /// Fewer than two legs.
    #[error("{count} leg(s); a route needs at least two")]
    TooFewLegs {
        /// Legs given.
        count: usize,
    },
    /// A leg that ends at or starts from a fix gives none.
    #[error("a {leg} leg needs its fix after the code: NAME:lat/lon, lat/lon or an earlier NAME")]
    MissingFix {
        /// The path terminator.
        leg: &'static str,
    },
    /// A leg lacks an attribute its path terminator needs.
    #[error("a {leg} leg needs {key} ({meaning})")]
    MissingKey {
        /// The key it needs.
        key: char,
        /// The path terminator.
        leg: &'static str,
        /// What the key gives.
        meaning: &'static str,
    },
    /// A leg carries an attribute its path terminator does not take.
    #[error("a {leg} leg takes no {key}")]
    KeyNotAllowed {
        /// The key given.
        key: char,
        /// The path terminator.
        leg: &'static str,
    },
    /// A leg gives one attribute twice.
    #[error("{key} appears twice in the leg")]
    RepeatedKey {
        /// The key given twice.
        key: char,
    },
    /// A bare fix name no earlier fix defines.
    #[error("no earlier fix is named {name:?}; write {name}:lat/lon the first time")]
    UnknownFix {
        /// The name given.
        name: String,
    },
    /// A leg where its path terminator cannot stand.
    #[error("a {leg} leg {rule}")]
    MisplacedLeg {
        /// The path terminator.
        leg: &'static str,
        /// Where it may stand.
        rule: &'static str,
    },
    /// A range whose lower end is not below its upper end.
    #[error("{text:?} is not lower..upper with lower below upper")]
    ReversedLimit {
        /// The range given.
        text: String,
    },
    /// The start gives no height and the link has no `h`.
    #[error("no height here and no default height `h` in the link")]
    MissingHeight,
    /// A date or time of day that does not exist.
    #[error("{text:?} is not a UTC time YYYYMMDDTHHMMSSZ: {why}")]
    BadTime {
        /// The text given.
        text: String,
        /// What is wrong with it.
        why: &'static str,
    },
    /// A navigation set without the inertial navigator.
    #[error("set {set:?} lacks ins")]
    SetWithoutIns {
        /// The set given.
        set: String,
    },
    /// A means named twice in one set.
    #[error("set {set:?} names {means} twice")]
    RepeatedMeans {
        /// The set given.
        set: String,
        /// The means named twice.
        means: &'static str,
    },
    /// Two sets with the same means.
    #[error("set {set:?} has the same means as an earlier set")]
    RepeatedSet {
        /// The later set.
        set: String,
    },
    /// The steering set is not one of the navigation sets.
    #[error("{set:?} is not one of the nav sets")]
    FlyNotInNav {
        /// The set given.
        set: String,
    },
    /// Two entries for one sensor.
    #[error("{sensor} has two entries; give each sensor one")]
    RepeatedSensor {
        /// The sensor named twice.
        sensor: &'static str,
    },
    /// A span that ends at or before its start.
    #[error("span {span:?} ends at or before its start")]
    ReversedSpan {
        /// The span given.
        span: String,
    },
    /// A span that starts before the one before it has ended.
    #[error("span {span:?} starts before the span before it ends")]
    OverlappingSpans {
        /// The later span.
        span: String,
    },
    /// A value whose written form does not read back as itself, such as an
    /// empty fix name.
    #[error("the value has no link form that reads back as itself")]
    NotRepresentable,
}

fn number_fault(fault: NumberFault, decimals: u32) -> String {
    match fault {
        NumberFault::NotDecimal => {
            "write plain digits with an optional decimal point, no exponent".to_owned()
        }
        NumberFault::TooManyDecimals => format!("at most {decimals} decimals"),
        NumberFault::TooManyDigits => "too many digits".to_owned(),
    }
}
