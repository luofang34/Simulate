//! `TAI - UTC` before 1972, when UTC ran at a steered rate instead of stepping
//! by whole leap seconds.
//!
//! NAIF leap-second kernels start at 1972-01-01, but the Apollo flights and
//! every other 1960s scenario need civil time to sit on the same TDB line as
//! the ephemerides. Each segment below is one line of the IERS `tai-utc.dat`
//! table: `TAI - UTC = offset + (MJD - reference_mjd) * rate`, with the MJD in
//! UTC days.

/// Unix seconds per day.
const SECONDS_PER_DAY: f64 = 86_400.0;
/// Modified Julian Date of the Unix epoch (`1970-01-01`).
const UNIX_EPOCH_MJD: f64 = 40_587.0;
/// First day the stepped leap-second table covers (`1972-01-01`).
const LEAP_TABLE_START_MJD: f64 = 41_317.0;

/// One steered-rate segment of pre-1972 UTC.
#[derive(Clone, Copy, Debug)]
struct RateSegment {
    start_mjd: f64,
    offset_s: f64,
    reference_mjd: f64,
    rate_s_per_day: f64,
}

impl RateSegment {
    const fn new(start_mjd: f64, offset_s: f64, reference_mjd: f64, rate_s_per_day: f64) -> Self {
        Self {
            start_mjd,
            offset_s,
            reference_mjd,
            rate_s_per_day,
        }
    }

    fn start_utc_unix_s(self) -> f64 {
        (self.start_mjd - UNIX_EPOCH_MJD) * SECONDS_PER_DAY
    }

    fn tai_minus_utc_s(self, utc_unix_s: f64) -> f64 {
        let mjd = utc_unix_s / SECONDS_PER_DAY + UNIX_EPOCH_MJD;
        self.offset_s + (mjd - self.reference_mjd) * self.rate_s_per_day
    }

    /// Inverts `tai = utc + tai_minus_utc(utc)`, which is linear in `utc`.
    fn utc_from_tai_unix_s(self, tai_unix_s: f64) -> f64 {
        let rate_per_s = self.rate_s_per_day / SECONDS_PER_DAY;
        let constant = self.offset_s + (UNIX_EPOCH_MJD - self.reference_mjd) * self.rate_s_per_day;
        (tai_unix_s - constant) / (1.0 + rate_per_s)
    }
}

/// IERS `tai-utc.dat`, 1961-01-01 to 1971-12-31.
const SEGMENTS: [RateSegment; 13] = [
    RateSegment::new(37_300.0, 1.422_818_0, 37_300.0, 0.001_296),
    RateSegment::new(37_512.0, 1.372_818_0, 37_300.0, 0.001_296),
    RateSegment::new(37_665.0, 1.845_858_0, 37_665.0, 0.001_123_2),
    RateSegment::new(38_334.0, 1.945_858_0, 37_665.0, 0.001_123_2),
    RateSegment::new(38_395.0, 3.240_130_0, 38_761.0, 0.001_296),
    RateSegment::new(38_486.0, 3.340_130_0, 38_761.0, 0.001_296),
    RateSegment::new(38_639.0, 3.440_130_0, 38_761.0, 0.001_296),
    RateSegment::new(38_761.0, 3.540_130_0, 38_761.0, 0.001_296),
    RateSegment::new(38_820.0, 3.640_130_0, 38_761.0, 0.001_296),
    RateSegment::new(38_942.0, 3.740_130_0, 38_761.0, 0.001_296),
    RateSegment::new(39_004.0, 3.840_130_0, 38_761.0, 0.001_296),
    RateSegment::new(39_126.0, 4.313_170_0, 39_126.0, 0.002_592),
    RateSegment::new(39_887.0, 4.213_170_0, 39_126.0, 0.002_592),
];

fn table_end_utc_unix_s() -> f64 {
    (LEAP_TABLE_START_MJD - UNIX_EPOCH_MJD) * SECONDS_PER_DAY
}

/// `TAI - UTC` at `utc_unix_s`, or `None` outside 1961-01-01..1972-01-01.
pub(super) fn tai_minus_utc_s(utc_unix_s: f64) -> Option<f64> {
    if utc_unix_s.is_nan() || utc_unix_s >= table_end_utc_unix_s() {
        return None;
    }
    SEGMENTS
        .iter()
        .rev()
        .find(|segment| utc_unix_s >= segment.start_utc_unix_s())
        .map(|segment| segment.tai_minus_utc_s(utc_unix_s))
}

/// UTC Unix seconds for TAI expressed as Unix-like seconds, or `None` before
/// 1961. Callers use it only for TAI earlier than the first stepped leap
/// second, so the last segment runs until UTC stepped back 0.107758 s at
/// `1972-01-01T00:00:10 TAI`. Where UTC stepped backwards at a segment
/// boundary and repeated a fraction of a second, the later segment wins.
pub(super) fn utc_from_tai_unix_s(tai_unix_s: f64) -> Option<f64> {
    SEGMENTS.iter().rev().find_map(|segment| {
        let utc = segment.utc_from_tai_unix_s(tai_unix_s);
        (utc >= segment.start_utc_unix_s()).then_some(utc)
    })
}

#[cfg(test)]
mod tests;
