//! Strongly typed NAIF integer codes (`NaifId`, `FrameCode`) and deterministic `VesselId`.

use core::fmt;
use serde::{Deserialize, Serialize};

/// NAIF integer body or barycenter identifier (`0 = SSB`, `10 = Sun`, `399 = Earth`, `301 = Moon`, `499 = Mars`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct NaifId(pub i32);

impl NaifId {
    /// Solar System Barycenter (`0`).
    pub const SSB: Self = Self(0);
    /// Mercury Barycenter (`1`).
    pub const MERCURY_BARYCENTER: Self = Self(1);
    /// Venus Barycenter (`2`).
    pub const VENUS_BARYCENTER: Self = Self(2);
    /// Earth-Moon Barycenter (`3`).
    pub const EARTH_MOON_BARYCENTER: Self = Self(3);
    /// Mars System Barycenter (`4`).
    pub const MARS_BARYCENTER: Self = Self(4);
    /// Jupiter System Barycenter (`5`).
    pub const JUPITER_BARYCENTER: Self = Self(5);
    /// Saturn System Barycenter (`6`).
    pub const SATURN_BARYCENTER: Self = Self(6);
    /// Uranus System Barycenter (`7`).
    pub const URANUS_BARYCENTER: Self = Self(7);
    /// Neptune System Barycenter (`8`).
    pub const NEPTUNE_BARYCENTER: Self = Self(8);
    /// Pluto System Barycenter (`9`).
    pub const PLUTO_BARYCENTER: Self = Self(9);
    /// Sun body center (`10`).
    pub const SUN: Self = Self(10);
    /// Mercury body center (`199`).
    pub const MERCURY: Self = Self(199);
    /// Venus body center (`299`).
    pub const VENUS: Self = Self(299);
    /// Moon body center (`301`).
    pub const MOON: Self = Self(301);
    /// Earth body center (`399`).
    pub const EARTH: Self = Self(399);
    /// Phobos body center (`401`).
    pub const PHOBOS: Self = Self(401);
    /// Deimos body center (`402`).
    pub const DEIMOS: Self = Self(402);
    /// Mars body center (`499`).
    pub const MARS: Self = Self(499);
    /// Jupiter body center (`599`).
    pub const JUPITER: Self = Self(599);
    /// Saturn body center (`699`).
    pub const SATURN: Self = Self(699);

    /// Returns the planetary system barycenter (`1..=9`) for a planet or satellite ID `100..999`, if applicable.
    #[must_use]
    pub const fn parent_planetary_barycenter(self) -> Option<Self> {
        if self.0 >= 100 && self.0 < 1000 {
            Some(Self(self.0 / 100))
        } else {
            None
        }
    }
}

impl fmt::Display for NaifId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NAIF({})", self.0)
    }
}

/// NAIF reference frame integer code (`1 = J2000/ICRF`, `17 = ECLIPJ2000`, `3000 = ITRF93`, `31001 = MOON_ME`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct FrameCode(pub i32);

impl FrameCode {
    /// Inertial `J2000` / `ICRF` equator and equinox frame (`1`).
    pub const J2000: Self = Self(1);
    /// Mean ecliptic and equinox of J2000 (`17`).
    pub const ECLIPJ2000: Self = Self(17);
    /// IAU approximate body-fixed Earth frame (`10013`).
    pub const IAU_EARTH: Self = Self(10013);
    /// IAU approximate body-fixed Moon frame (`10020`).
    pub const IAU_MOON: Self = Self(10020);
    /// IAU approximate body-fixed Mars frame (`10014`).
    pub const IAU_MARS: Self = Self(10014);
    /// Precision terrestrial frame `ITRF93` (`3000`).
    pub const ITRF93: Self = Self(3000);
    /// Lunar Principal Axes frame (`31000`).
    pub const MOON_PA: Self = Self(31000);
    /// Lunar Mean-Earth / Polar-Axis cartographic frame (`31001`).
    pub const MOON_ME: Self = Self(31001);
    /// DE440 Lunar Principal Axes binary PCK class ID (`31008`).
    pub const MOON_PA_DE440: Self = Self(31008);
    /// DE440/ME421 Lunar Mean-Earth frame kernel ID (`31009`).
    pub const MOON_ME_DE440_ME421: Self = Self(31009);
}

/// Unique deterministic identifier of a simulated vessel or stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct VesselId(pub u64);

impl VesselId {
    /// Sentinel vessel identifier (`u64::MAX`) representing "no secondary participant".
    pub const NONE: Self = Self(u64::MAX);

    /// Returns `[min(a, b), max(a, b)]` for canonical pair ordering.
    #[must_use]
    pub const fn canonical_pair(a: Self, b: Self) -> [Self; 2] {
        if a.0 <= b.0 { [a, b] } else { [b, a] }
    }
}

impl fmt::Display for VesselId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "vessel-{}", self.0)
    }
}
