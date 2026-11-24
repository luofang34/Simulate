//! Decibel conversions.

use core::f64::consts::LN_10;

use sim_spacetime::{exp, ln};

/// $10\log_{10}(r)$ for a power ratio $r > 0$.
#[must_use]
pub fn db_from_ratio(ratio: f64) -> f64 {
    10.0 * ln(ratio) / LN_10
}

/// Power ratio $10^{x/10}$ for `x` decibels.
#[must_use]
pub fn ratio_from_db(db: f64) -> f64 {
    exp(db * LN_10 / 10.0)
}
