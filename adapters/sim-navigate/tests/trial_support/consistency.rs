//! The consistency bands the inertial tests assert, shared so that the
//! trial reuses them rather than restating them.

#![allow(
    dead_code,
    reason = "each test crate that includes this support uses part of it"
)]

use std::ops::RangeInclusive;

/// Mean 3-DOF position NEES an unaided cruise must average, over the score
/// instants of an ensemble of seeds, for every IMU grade.
pub const DEAD_RECKONING_NEES: RangeInclusive<f64> = 0.5..=8.0;
/// Mean 3-DOF position and attitude NEES under manoeuvre, and with star
/// sightings, for every IMU grade.
pub const MANOEUVRE_NEES: RangeInclusive<f64> = 0.3..=10.0;
/// Attitude error star sightings must hold a tactical or consumer INS
/// under, radians, once the gyro bias is learnt.
pub const STAR_ATTITUDE_RAD: f64 = 1e-3;
