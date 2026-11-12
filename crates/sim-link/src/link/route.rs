//! The start state.

use crate::{Micro, Milli};

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
    /// first fix.
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
