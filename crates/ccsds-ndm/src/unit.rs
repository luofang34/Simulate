//! Units as types: each keyword's value is a [`Quantity`] in the one unit the
//! standard defines for it.
//!
//! NDMs never convert units: a keyword has exactly one unit, and an explicit
//! `[unit]` (KVN) or `units="…"` (XML) may only restate it. The parser checks
//! that restatement; the type records which unit the number is in.

use std::fmt;
use std::marker::PhantomData;

/// A unit an NDM keyword is defined in.
pub trait Unit {
    /// The unit as NDMs spell it (`km`, `km**3/s**2`, `1/ER`).
    const SYMBOL: &'static str;
}

macro_rules! units {
    ($($(#[$meta:meta])* $name:ident => $symbol:literal,)+) => {
        $(
            $(#[$meta])*
            #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
            pub enum $name {}

            impl Unit for $name {
                const SYMBOL: &'static str = $symbol;
            }
        )+
    };
}

units! {
    /// Kilometres.
    Km => "km",
    /// Metres.
    Meters => "m",
    /// Kilometres per second.
    KmPerS => "km/s",
    /// Metres per second.
    MPerS => "m/s",
    /// Kilometres per second squared.
    KmPerS2 => "km/s**2",
    /// Metres per second squared.
    MPerS2 => "m/s**2",
    /// Degrees.
    Deg => "deg",
    /// Degrees per second.
    DegPerS => "deg/s",
    /// Inverse seconds (quaternion time derivatives).
    PerS => "1/s",
    /// Seconds.
    Seconds => "s",
    /// Seconds per second (clock drift).
    SPerS => "s/s",
    /// Days.
    Days => "d",
    /// Kilograms.
    Kg => "kg",
    /// Square metres.
    M2 => "m**2",
    /// Square metres per kilogram.
    M2PerKg => "m**2/kg",
    /// Kilogram-square metres (moments and products of inertia).
    KgM2 => "kg*m**2",
    /// Newton-metres (attitude maneuver torque).
    Nm => "N*m",
    /// Watts per kilogram (specific energy dissipation rate).
    WPerKg => "W/kg",
    /// Percentage.
    Percent => "%",
    /// Gravitational parameter, cubic kilometres per second squared.
    Km3PerS2 => "km**3/s**2",
    /// Revolutions per day.
    RevPerDay => "rev/day",
    /// Revolutions per day squared.
    RevPerDay2 => "rev/day**2",
    /// Revolutions per day cubed.
    RevPerDay3 => "rev/day**3",
    /// Inverse Earth radii (the SGP4 BSTAR drag term).
    PerEarthRadius => "1/ER",
    /// Hertz.
    Hz => "Hz",
    /// Hertz per second.
    HzPerS => "Hz/s",
    /// Decibel-watts (carrier power).
    DbW => "dBW",
    /// Decibel-hertz (signal-to-noise spectral density ratio).
    DbHz => "dBHz",
    /// Hectopascals (atmospheric pressure).
    Hpa => "hPa",
    /// Kelvin (temperature).
    Kelvin => "K",
    /// Total Electron Content Units (slant TEC).
    Tecu => "TECU",
}

/// A number in unit `U`.
pub struct Quantity<U> {
    value: f64,
    unit: PhantomData<fn() -> U>,
}

impl<U: Unit> Quantity<U> {
    /// Wraps a number expressed in `U`.
    pub const fn new(value: f64) -> Self {
        Self {
            value,
            unit: PhantomData,
        }
    }

    /// The number, in `U`.
    pub const fn value(self) -> f64 {
        self.value
    }

    /// The unit symbol.
    pub const fn symbol() -> &'static str {
        U::SYMBOL
    }
}

impl<U> Clone for Quantity<U> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<U> Copy for Quantity<U> {}

impl<U> PartialEq for Quantity<U> {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl<U: Unit> fmt::Debug for Quantity<U> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} [{}]", self.value, U::SYMBOL)
    }
}

/// Three components in one unit: a position, velocity, or velocity change.
pub type Vector3<U> = [Quantity<U>; 3];

/// Builds a [`Vector3`] from plain numbers.
pub fn vector3<U: Unit>(values: [f64; 3]) -> Vector3<U> {
    values.map(Quantity::new)
}
