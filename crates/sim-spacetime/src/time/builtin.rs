//! The time-scale record built into every simulator binary, so a run can name
//! civil time without loading anything.

use super::{EarthEopTable, FutureUtcPolicy, LeapSecondTable, OutOfEopPolicy, TdbEpoch};
use super::{TimeError, TimeScaleCatalog};

/// Embedded NASA NAIF `naif0012.tls` leap-second kernel.
pub const BUILTIN_LSK_BYTES: &[u8] = include_bytes!("../../data/naif0012.tls");
/// SHA-256 hex digest of [`BUILTIN_LSK_BYTES`].
pub const BUILTIN_LSK_SHA256: &str =
    "678e32bdb5a744117a467cd9601cd6b373f0e9bc9bbde1371d5eee39600a039b";

impl TimeScaleCatalog {
    /// The catalog every built-in scenario and free flight uses: the embedded
    /// leap-second kernel, `TAI - UTC` frozen at its last published step from
    /// 2026 on (IERS has announced no leap second since 2017), and `UT1 = UTC`
    /// because no Earth-orientation series is embedded.
    pub fn builtin() -> Result<Self, TimeError> {
        let lsk = LeapSecondTable::from_naif_lsk_bytes(BUILTIN_LSK_BYTES)?;
        Ok(Self::new(
            lsk,
            FutureUtcPolicy::FreezeLastTaiOffset {
                expires_tai_j2000_s: TdbEpoch::JAN_1_2026_00Z_TDB.as_j2000_seconds_f64(),
            },
            EarthEopTable::empty(),
            OutOfEopPolicy::ZeroDeltaUt1Fallback,
        ))
    }
}
