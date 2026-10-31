use std::f64::consts::{FRAC_PI_2, TAU};

use glam::{DQuat, DVec3};

use crate::UtcTime;

#[cfg(test)]
mod tests;

/// Unix time of the J2000 epoch, 2000-01-01T12:00:00.
const J2000_UNIX_SECONDS: f64 = 946_728_000.0;
const SECONDS_PER_DAY: f64 = 86_400.0;
const DAYS_PER_CENTURY: f64 = 36_525.0;

/// Earth rotation angle at J2000, turns (IERS Conventions 2010).
const ERA_AT_J2000_TURNS: f64 = 0.779_057_273_264_0;
/// Turns per UT1 day beyond one whole turn.
const ERA_EXCESS_TURNS_PER_DAY: f64 = 0.002_737_811_911_354_48;

/// The IAU 2009 lunar periodic terms, one row per argument: phase at J2000
/// (degrees), rate (degrees/day), then the amplitudes (degrees) of its sine in
/// pole right ascension, its cosine in pole declination, and its sine in
/// prime-meridian angle.
const LUNAR_TERMS: [[f64; 5]; 13] = [
    [125.045, -0.052_992_1, -3.8787, 1.5419, 3.5610],
    [250.089, -0.105_984_2, -0.1204, 0.0239, 0.1208],
    [260.008, 13.012_000_9, 0.0700, -0.0278, -0.0642],
    [176.625, 13.340_715_4, -0.0172, 0.0068, 0.0158],
    [357.529, 0.985_600_3, 0.0, 0.0, 0.0252],
    [311.589, 26.405_708_4, 0.0072, -0.0029, -0.0066],
    [134.963, 13.064_993_0, 0.0, 0.0009, -0.0047],
    [276.617, 0.328_714_6, 0.0, 0.0, -0.0046],
    [34.226, 1.748_487_7, 0.0, 0.0, 0.0028],
    [15.134, -0.158_976_3, -0.0052, 0.0008, 0.0052],
    [119.743, 0.003_609_6, 0.0, 0.0, 0.0040],
    [239.961, 0.164_357_3, 0.0, 0.0, 0.0019],
    [25.053, 12.959_008_8, 0.0043, -0.0009, -0.0044],
];

/// A rotation between the J2000 inertial frame and a body-fixed frame at one
/// instant.
///
/// UTC stands in for both UT1 and TDB. The Earth rotation is about the J2000
/// pole alone, with no precession, nutation, or polar motion, so inertial
/// directions drift from true ECEF by about 0.014° per year from J2000.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertialTransform {
    j2000_to_fixed: DQuat,
}

impl InertialTransform {
    /// Earth's orientation at `utc`, from the Earth rotation angle.
    pub fn earth_at(utc: UtcTime) -> Self {
        Self {
            j2000_to_fixed: DQuat::from_rotation_z(-earth_rotation_angle_rad(utc)),
        }
    }

    /// The Moon's mean-Earth/polar-axis orientation at `utc`, from the IAU 2009
    /// pole and prime meridian including their periodic libration terms.
    pub fn moon_at(utc: UtcTime) -> Self {
        let [right_ascension, declination, prime_meridian] = moon_orientation_rad(utc);
        Self {
            j2000_to_fixed: DQuat::from_rotation_z(-prime_meridian)
                * DQuat::from_rotation_x(declination - FRAC_PI_2)
                * DQuat::from_rotation_z(-(right_ascension + FRAC_PI_2)),
        }
    }

    /// Expresses a J2000 vector in the body-fixed frame.
    pub fn j2000_to_fixed(&self, inertial: DVec3) -> DVec3 {
        self.j2000_to_fixed * inertial
    }

    /// Expresses a body-fixed vector in the J2000 frame.
    pub fn fixed_to_j2000(&self, fixed: DVec3) -> DVec3 {
        self.j2000_to_fixed.inverse() * fixed
    }

    /// The rotation [`Self::fixed_to_j2000`] applies, for composing with an
    /// attitude.
    pub fn fixed_to_j2000_rotation(&self) -> DQuat {
        self.j2000_to_fixed.inverse()
    }
}

fn days_since_j2000(utc: UtcTime) -> f64 {
    (utc.unix_seconds() - J2000_UNIX_SECONDS) / SECONDS_PER_DAY
}

/// Sums whole-day turns out of the product so the angle keeps full precision
/// decades from J2000.
fn earth_rotation_angle_rad(utc: UtcTime) -> f64 {
    let days = days_since_j2000(utc);
    let turns = ERA_AT_J2000_TURNS + ERA_EXCESS_TURNS_PER_DAY * days + days.rem_euclid(1.0);
    TAU * turns.rem_euclid(1.0)
}

/// Pole right ascension, pole declination, and prime-meridian angle, radians.
fn moon_orientation_rad(utc: UtcTime) -> [f64; 3] {
    let days = days_since_j2000(utc);
    let centuries = days / DAYS_PER_CENTURY;
    let mut right_ascension = 269.9949 + 0.0031 * centuries;
    let mut declination = 66.5392 + 0.0130 * centuries;
    let mut prime_meridian = 38.3213 + 13.176_358_15 * days - 1.4e-12 * days * days;
    for [phase, rate, ra_sine, dec_cosine, pm_sine] in LUNAR_TERMS {
        let (sine, cosine) = (phase + rate * days).to_radians().sin_cos();
        right_ascension += ra_sine * sine;
        declination += dec_cosine * cosine;
        prime_meridian += pm_sine * sine;
    }
    [
        right_ascension.to_radians(),
        declination.to_radians(),
        prime_meridian.to_radians(),
    ]
}
