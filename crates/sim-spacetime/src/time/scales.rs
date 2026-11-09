//! Deterministic `TDB <-> TT <-> TAI <-> UTC <-> UT1 <-> DecimalYear` conversions with explicit
//! future leap-second and Earth Orientation Parameter (`EOP`) expiration policies.

use serde::{Deserialize, Serialize};
use sim_geo::UtcTime;
#[cfg(feature = "world-mbd-bridges")]
use world_core::DecimalYear;

use crate::{
    hash::{ContentHash, DomainHasher},
    kpl::{KplAtom, KplError, KplPool},
    scalar_math::sin,
};

use super::TdbEpoch;

/// Seconds between Unix epoch (`1970-01-01T00:00:00Z`) and `J2000.0` (`2000-01-01T12:00:00Z`).
const J2000_UNIX_SECONDS: i64 = 946_728_000;

/// Why a time-scale lookup or conversion failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum TimeError {
    /// Failed to parse a NAIF `LSK` leap-second kernel.
    #[error("invalid NAIF leap-second kernel: {0}")]
    InvalidLsk(#[from] KplError),
    /// Malformed `DELTET/DELTA_AT` table inside an `LSK` file.
    #[error("malformed DELTET/DELTA_AT table: `{0}`")]
    MalformedLeapTable(String),
    /// Requested epoch precedes the start of the `TAI - UTC` step table (`1972-01-01`).
    #[error("epoch {tai_j2000_s:.3} s TAI precedes 1972-01-01 leap-second table start")]
    BeforeLeapTable {
        /// Requested TAI seconds from J2000.
        tai_j2000_s: f64,
    },
    /// Requested epoch exceeds the leap-second table expiration under `RejectAfterExpiration`.
    #[error(
        "epoch {tai_j2000_s:.3} s TAI exceeds leap-second expiration {expires_tai_j2000_s:.3} s"
    )]
    LeapSecondTableExpired {
        /// Requested TAI seconds from J2000.
        tai_j2000_s: f64,
        /// Configured expiration horizon in TAI seconds from J2000.
        expires_tai_j2000_s: f64,
    },
    /// Requested epoch lies outside the loaded `EarthEopTable` under `RejectOutOfRange`.
    #[error("UTC unix={utc_unix_s:.3} s lies outside Earth EOP coverage")]
    OutOfEopCoverage {
        /// Requested UTC Unix timestamp.
        utc_unix_s: f64,
    },
}

/// Policy governing `TAI <-> UTC` conversion beyond the known IERS leap-second horizon.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum FutureUtcPolicy {
    /// Fail closed with [`TimeError::LeapSecondTableExpired`] past `expires_tai_j2000_s`.
    RejectAfterExpiration {
        /// Expiration timestamp in TAI seconds from J2000.
        expires_tai_j2000_s: f64,
    },
    /// Hold the last published `TAI - UTC` step constant past `expires_tai_j2000_s`
    /// and record this policy in the replay manifest digest.
    FreezeLastTaiOffset {
        /// Expiration timestamp in TAI seconds from J2000 after which offset is held frozen.
        expires_tai_j2000_s: f64,
    },
}

/// Policy governing `UTC -> UT1` conversion outside [`EarthEopTable`] coverage.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OutOfEopPolicy {
    /// Fail closed with [`TimeError::OutOfEopCoverage`].
    RejectOutOfRange,
    /// Return `delta_ut1_s = 0, xp = 0, yp = 0` tagged `is_approximate_fallback = true`.
    ZeroDeltaUt1Fallback,
}

/// One step in the IERS `TAI - UTC` leap-second schedule (`DELTET/DELTA_AT`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LeapSecondEntry {
    /// `TAI - UTC` in whole seconds starting at this step.
    pub tai_minus_utc_s: f64,
    /// Effective UTC Unix timestamp (`s` from `1970-01-01T00:00:00Z`).
    pub effective_utc_unix_s: i64,
    /// Effective TAI seconds from `J2000.0` (`2000-01-01T12:00:00 TAI`).
    pub effective_tai_j2000_s: f64,
}

/// Parsed NAIF `LSK` (`naif0012.tls`) leap-second and `TDB - TT` model.
#[derive(Clone, Debug)]
pub struct LeapSecondTable {
    hash: ContentHash,
    delta_t_a_s: f64,
    k_s: f64,
    eb: f64,
    m0_rad: f64,
    m1_rad_per_s: f64,
    steps: Vec<LeapSecondEntry>,
}

impl LeapSecondTable {
    /// Parses a NAIF `LSK` text kernel (such as `naif0012.tls`).
    pub fn from_naif_lsk_bytes(bytes: &[u8]) -> Result<Self, TimeError> {
        let pool = KplPool::parse_bytes(bytes)?;
        let delta_t_a_s = pool.require_scalar("DELTET/DELTA_T_A")?;
        let k_s = pool.require_scalar("DELTET/K")?;
        let eb = pool.require_scalar("DELTET/EB")?;
        let m = pool.require_floats("DELTET/M")?;
        let m0_rad = m
            .first()
            .copied()
            .ok_or_else(|| TimeError::MalformedLeapTable("DELTET/M[0]".to_owned()))?;
        let m1_rad_per_s = m
            .get(1)
            .copied()
            .ok_or_else(|| TimeError::MalformedLeapTable("DELTET/M[1]".to_owned()))?;
        let raw_at = pool
            .atoms("DELTET/DELTA_AT")
            .ok_or_else(|| TimeError::MalformedLeapTable("missing DELTET/DELTA_AT".to_owned()))?;
        let steps = parse_leap_steps(raw_at)?;
        Ok(Self {
            hash: pool.content_hash(),
            delta_t_a_s,
            k_s,
            eb,
            m0_rad,
            m1_rad_per_s,
            steps,
        })
    }

    /// SHA-256 content hash of the source `LSK` bytes.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.hash
    }

    /// Loaded `TAI - UTC` steps in chronological order.
    #[must_use]
    pub fn steps(&self) -> &[LeapSecondEntry] {
        &self.steps
    }

    /// Periodic `TDB - TT` difference in seconds at `t_j2000_s` (`TDB` or `TT` seconds past J2000).
    #[must_use]
    pub fn tdb_minus_tt_seconds(&self, t_j2000_s: f64) -> f64 {
        let m = self.m0_rad + self.m1_rad_per_s * t_j2000_s;
        let ecc_anom = m + self.eb * sin(m);
        self.k_s * sin(ecc_anom)
    }

    /// Constant `TT - TAI` offset (`32.184 s`).
    #[must_use]
    pub const fn tt_minus_tai_seconds(&self) -> f64 {
        self.delta_t_a_s
    }
}

/// One daily or tabular Earth Orientation Parameter (`EOP`) sample.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EarthEopEntry {
    /// Sample UTC Unix timestamp (`s`).
    pub utc_unix_s: f64,
    /// `UT1 - UTC` in seconds (`[-0.9, +0.9]`).
    pub ut1_minus_utc_s: f64,
    /// Celestial-to-terrestrial polar motion $x_p$ in arcseconds.
    pub xp_arcsec: f64,
    /// Celestial-to-terrestrial polar motion $y_p$ in arcseconds.
    pub yp_arcsec: f64,
}

/// Evaluated `UT1` and polar-motion sample at one instant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ut1Sample {
    /// `UT1` expressed as Unix-like seconds (`utc_unix_s + ut1_minus_utc_s`).
    pub ut1_unix_s: f64,
    /// Interpolated `UT1 - UTC` in seconds.
    pub ut1_minus_utc_s: f64,
    /// Polar motion $x_p$ in radians.
    pub xp_rad: f64,
    /// Polar motion $y_p$ in radians.
    pub yp_rad: f64,
    /// `true` when produced by [`OutOfEopPolicy::ZeroDeltaUt1Fallback`].
    pub is_approximate_fallback: bool,
}

/// Sorted piecewise-linear `EarthEopTable` with content hash.
#[derive(Clone, Debug)]
pub struct EarthEopTable {
    hash: ContentHash,
    entries: Vec<EarthEopEntry>,
}

impl EarthEopTable {
    /// Builds a sorted `EarthEopTable` and computes its canonical SHA-256 hash.
    #[must_use]
    pub fn new(mut entries: Vec<EarthEopEntry>) -> Self {
        entries.sort_by(|a, b| a.utc_unix_s.total_cmp(&b.utc_unix_s));
        let mut h = DomainHasher::new(b"sim-orbital:eop:v1");
        for e in &entries {
            h.write_f64_canonical(e.utc_unix_s);
            h.write_f64_canonical(e.ut1_minus_utc_s);
            h.write_f64_canonical(e.xp_arcsec);
            h.write_f64_canonical(e.yp_arcsec);
        }
        Self {
            hash: h.finish(),
            entries,
        }
    }

    /// Empty table (requires [`OutOfEopPolicy::ZeroDeltaUt1Fallback`] or BPC Earth orientation).
    #[must_use]
    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    /// Canonical SHA-256 hash of the table entries.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.hash
    }
}

/// Complete time-scale catalog combining `LSK`, `EOP`, and explicit expiration policies.
#[derive(Clone, Debug)]
pub struct TimeScaleCatalog {
    lsk: LeapSecondTable,
    future_utc_policy: FutureUtcPolicy,
    eop: EarthEopTable,
    out_of_eop_policy: OutOfEopPolicy,
    digest: ContentHash,
}

impl TimeScaleCatalog {
    /// Assembles a hashed `TimeScaleCatalog`.
    #[must_use]
    pub fn new(
        lsk: LeapSecondTable,
        future_utc_policy: FutureUtcPolicy,
        eop: EarthEopTable,
        out_of_eop_policy: OutOfEopPolicy,
    ) -> Self {
        let mut h = DomainHasher::new(b"sim-orbital:timescales:v1");
        h.write_hash(lsk.content_hash());
        match future_utc_policy {
            FutureUtcPolicy::RejectAfterExpiration {
                expires_tai_j2000_s,
            } => {
                h.write_i32(0);
                h.write_f64_canonical(expires_tai_j2000_s);
            }
            FutureUtcPolicy::FreezeLastTaiOffset {
                expires_tai_j2000_s,
            } => {
                h.write_i32(1);
                h.write_f64_canonical(expires_tai_j2000_s);
            }
        }
        h.write_hash(eop.content_hash());
        h.write_i32(match out_of_eop_policy {
            OutOfEopPolicy::RejectOutOfRange => 0,
            OutOfEopPolicy::ZeroDeltaUt1Fallback => 1,
        });
        Self {
            lsk,
            future_utc_policy,
            eop,
            out_of_eop_policy,
            digest: h.finish(),
        }
    }

    /// Combined SHA-256 digest of `LSK`, `EOP`, and both policies.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.digest
    }

    /// Converts [`TdbEpoch`] to `TAI` seconds past `J2000.0 TAI` (`2000-01-01T12:00:00 TAI`).
    #[must_use]
    pub fn tdb_to_tai_j2000_seconds(&self, tdb: TdbEpoch) -> f64 {
        let (whole, sub) = tdb.split_j2000_seconds();
        let tdb_s = whole + sub;
        let d0 = self.lsk.tdb_minus_tt_seconds(tdb_s);
        let d1 = self.lsk.tdb_minus_tt_seconds(tdb_s - d0);
        (whole - self.lsk.tt_minus_tai_seconds()) + (sub - d1)
    }

    /// Converts [`TdbEpoch`] to [`UtcTime`], enforcing [`FutureUtcPolicy`].
    pub fn tdb_to_utc(&self, tdb: TdbEpoch) -> Result<UtcTime, TimeError> {
        let tai_j2000_s = self.tdb_to_tai_j2000_seconds(tdb);
        let first = self
            .lsk
            .steps
            .first()
            .ok_or_else(|| TimeError::MalformedLeapTable("empty".to_owned()))?;
        if tai_j2000_s < first.effective_tai_j2000_s {
            return Err(TimeError::BeforeLeapTable { tai_j2000_s });
        }
        if let FutureUtcPolicy::RejectAfterExpiration {
            expires_tai_j2000_s,
        } = self.future_utc_policy
            && tai_j2000_s > expires_tai_j2000_s
        {
            return Err(TimeError::LeapSecondTableExpired {
                tai_j2000_s,
                expires_tai_j2000_s,
            });
        }
        let mut dat = first.tai_minus_utc_s;
        for step in &self.lsk.steps {
            if tai_j2000_s >= step.effective_tai_j2000_s {
                dat = step.tai_minus_utc_s;
            }
        }
        let utc_unix_s = (J2000_UNIX_SECONDS as f64) + (tai_j2000_s - dat);
        Ok(UtcTime::from_unix_seconds(utc_unix_s))
    }

    /// Converts [`UtcTime`] to [`TdbEpoch`], enforcing [`FutureUtcPolicy`].
    pub fn utc_to_tdb(&self, utc: UtcTime) -> Result<TdbEpoch, TimeError> {
        let utc_unix_s = utc.unix_seconds();
        let first = self
            .lsk
            .steps
            .first()
            .ok_or_else(|| TimeError::MalformedLeapTable("empty".to_owned()))?;
        if utc_unix_s < first.effective_utc_unix_s as f64 {
            return Err(TimeError::BeforeLeapTable {
                tai_j2000_s: utc_unix_s - (J2000_UNIX_SECONDS as f64),
            });
        }
        let mut dat = first.tai_minus_utc_s;
        for step in &self.lsk.steps {
            if utc_unix_s >= step.effective_utc_unix_s as f64 {
                dat = step.tai_minus_utc_s;
            }
        }
        let tai_j2000_s = (utc_unix_s - (J2000_UNIX_SECONDS as f64)) + dat;
        if let FutureUtcPolicy::RejectAfterExpiration {
            expires_tai_j2000_s,
        } = self.future_utc_policy
            && tai_j2000_s > expires_tai_j2000_s
        {
            return Err(TimeError::LeapSecondTableExpired {
                tai_j2000_s,
                expires_tai_j2000_s,
            });
        }
        let tt_s = tai_j2000_s + self.lsk.tt_minus_tai_seconds();
        let tdb_s = tt_s + self.lsk.tdb_minus_tt_seconds(tt_s);
        TdbEpoch::from_j2000_seconds_f64(tdb_s)
            .ok_or_else(|| TimeError::MalformedLeapTable("non-finite TDB".to_owned()))
    }

    /// Evaluates `UT1` and polar motion at `utc`, enforcing [`OutOfEopPolicy`].
    pub fn utc_to_ut1(&self, utc: UtcTime) -> Result<Ut1Sample, TimeError> {
        let u = utc.unix_seconds();
        if let (Some(first), Some(last)) = (self.eop.entries.first(), self.eop.entries.last())
            && u >= first.utc_unix_s
            && u <= last.utc_unix_s
        {
            return Ok(interpolate_eop(&self.eop.entries, u));
        }
        match self.out_of_eop_policy {
            OutOfEopPolicy::RejectOutOfRange => Err(TimeError::OutOfEopCoverage { utc_unix_s: u }),
            OutOfEopPolicy::ZeroDeltaUt1Fallback => Ok(Ut1Sample {
                ut1_unix_s: u,
                ut1_minus_utc_s: 0.0,
                xp_rad: 0.0,
                yp_rad: 0.0,
                is_approximate_fallback: true,
            }),
        }
    }

    /// Converts a civil [`UtcTime`] into [`world_core::DecimalYear`].
    #[cfg(feature = "world-mbd-bridges")]
    #[must_use]
    pub fn utc_to_decimal_year(utc: UtcTime) -> DecimalYear {
        let whole_s = utc.unix_seconds().floor() as i64;
        let frac_s = utc.unix_seconds() - (whole_s as f64);
        let base = DecimalYear::from_unix_seconds(whole_s);
        DecimalYear(base.value() + frac_s / (365.2425 * 86_400.0))
    }

    /// Converts [`TdbEpoch`] into [`world_core::DecimalYear`] via `TDB -> UTC -> DecimalYear`,
    /// enforcing the catalog's [`FutureUtcPolicy`].
    #[cfg(feature = "world-mbd-bridges")]
    pub fn tdb_to_decimal_year(&self, tdb: TdbEpoch) -> Result<DecimalYear, TimeError> {
        let utc = self.tdb_to_utc(tdb)?;
        Ok(Self::utc_to_decimal_year(utc))
    }
}

fn interpolate_eop(entries: &[EarthEopEntry], utc_unix_s: f64) -> Ut1Sample {
    let arcsec_to_rad = core::f64::consts::PI / (180.0 * 3600.0);
    let idx = entries.partition_point(|e| e.utc_unix_s <= utc_unix_s);
    let hi_idx = idx.min(entries.len().saturating_sub(1));
    let lo_idx = idx.saturating_sub(1);
    let (Some(lo), Some(hi)) = (entries.get(lo_idx), entries.get(hi_idx)) else {
        return Ut1Sample {
            ut1_unix_s: utc_unix_s,
            ut1_minus_utc_s: 0.0,
            xp_rad: 0.0,
            yp_rad: 0.0,
            is_approximate_fallback: true,
        };
    };
    let span = hi.utc_unix_s - lo.utc_unix_s;
    let alpha = if span > 0.0 {
        ((utc_unix_s - lo.utc_unix_s) / span).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let dut1 = lo.ut1_minus_utc_s + alpha * (hi.ut1_minus_utc_s - lo.ut1_minus_utc_s);
    let xp = lo.xp_arcsec + alpha * (hi.xp_arcsec - lo.xp_arcsec);
    let yp = lo.yp_arcsec + alpha * (hi.yp_arcsec - lo.yp_arcsec);
    Ut1Sample {
        ut1_unix_s: utc_unix_s + dut1,
        ut1_minus_utc_s: dut1,
        xp_rad: xp * arcsec_to_rad,
        yp_rad: yp * arcsec_to_rad,
        is_approximate_fallback: false,
    }
}

fn parse_leap_steps(atoms: &[KplAtom]) -> Result<Vec<LeapSecondEntry>, TimeError> {
    if atoms.is_empty() || !atoms.len().is_multiple_of(2) {
        return Err(TimeError::MalformedLeapTable("odd atom count".to_owned()));
    }
    let mut out = Vec::with_capacity(atoms.len() / 2);
    let mut i = 0_usize;
    while i < atoms.len() {
        let Some(KplAtom::Float(dat)) = atoms.get(i) else {
            return Err(TimeError::MalformedLeapTable(format!("index {i}")));
        };
        let Some(KplAtom::CalendarEpoch(cal)) = atoms.get(i.wrapping_add(1)) else {
            return Err(TimeError::MalformedLeapTable(format!("cal {i}")));
        };
        let unix_s = parse_naif_calendar_unix(cal)?;
        out.push(LeapSecondEntry {
            tai_minus_utc_s: *dat,
            effective_utc_unix_s: unix_s,
            effective_tai_j2000_s: ((unix_s - J2000_UNIX_SECONDS) as f64) + *dat,
        });
        i = i.wrapping_add(2);
    }
    Ok(out)
}

fn parse_naif_calendar_unix(cal: &str) -> Result<i64, TimeError> {
    let mut parts = cal.split('-');
    let year: i32 = parts
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| TimeError::MalformedLeapTable(cal.to_owned()))?;
    let mon_str = parts
        .next()
        .ok_or_else(|| TimeError::MalformedLeapTable(cal.to_owned()))?;
    let day: u32 = parts
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| TimeError::MalformedLeapTable(cal.to_owned()))?;
    let month: u32 = match mon_str.to_ascii_uppercase().as_str() {
        "JAN" => 1,
        "FEB" => 2,
        "MAR" => 3,
        "APR" => 4,
        "MAY" => 5,
        "JUN" => 6,
        "JUL" => 7,
        "AUG" => 8,
        "SEP" => 9,
        "OCT" => 10,
        "NOV" => 11,
        "DEC" => 12,
        _ => return Err(TimeError::MalformedLeapTable(cal.to_owned())),
    };
    Ok(days_from_civil_1970(year, month, day) * 86_400)
}

const fn days_from_civil_1970(year: i32, month: u32, day: u32) -> i64 {
    let y = (year as i64) - if month <= 2 { 1 } else { 0 };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let m = month as i64;
    let mp = if m > 2 { m - 3 } else { m + 9 };
    let doy = (153 * mp + 2) / 5 + (day as i64) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}
