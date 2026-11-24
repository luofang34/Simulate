//! The time scale an NDM's epochs are expressed in (`TIME_SYSTEM`).

use std::fmt;

macro_rules! time_systems {
    ($($(#[$meta:meta])* $variant:ident => $text:literal,)+) => {
        /// Time systems ODM 502.0-B-3 lists in 3.2.3.2.
        ///
        /// The set is closed: an epoch in an unknown scale cannot be
        /// interpreted, so reading one is an error rather than a guess.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub enum TimeSystem {
            $($(#[$meta])* $variant,)+
        }

        impl TimeSystem {
            /// Every listed time system.
            pub const ALL: &'static [TimeSystem] = &[$(TimeSystem::$variant,)+];

            /// The keyword value, as written in a message.
            pub fn as_str(self) -> &'static str {
                match self {
                    $(Self::$variant => $text,)+
                }
            }

            /// Looks up an exact keyword value.
            pub fn from_name(name: &str) -> Option<Self> {
                match name {
                    $($text => Some(Self::$variant),)+
                    _ => None,
                }
            }
        }
    };
}

time_systems! {
    /// Greenwich Mean Sidereal Time.
    Gmst => "GMST",
    /// Global Positioning System time.
    Gps => "GPS",
    /// Mission Elapsed Time.
    Met => "MET",
    /// Mission Relative Time.
    Mrt => "MRT",
    /// Spacecraft clock.
    Sclk => "SCLK",
    /// International Atomic Time.
    Tai => "TAI",
    /// Barycentric Coordinate Time.
    Tcb => "TCB",
    /// Barycentric Dynamical Time.
    Tdb => "TDB",
    /// Geocentric Coordinate Time.
    Tcg => "TCG",
    /// Terrestrial Time.
    Tt => "TT",
    /// Universal Time.
    Ut1 => "UT1",
    /// Coordinated Universal Time.
    Utc => "UTC",
}

impl fmt::Display for TimeSystem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}
