//! CCSDS `Epoch` <-> `sim-spacetime` (`TdbEpoch`, `UtcTime`, `SimEpoch`) time conversions.

use ccsds_ndm::{DateForm, Epoch, TimeSystem};
use sim_geo::UtcTime;
use sim_spacetime::{SimEpoch, SimInstant, TdbEpoch, TimeScaleCatalog};

use super::CcsdsError;

/// Exact seconds between `1970-01-01T00:00:00` and `2000-01-01T12:00:00` (`TDB/TAI` reference).
const SECS_1970_TO_J2000_I64: i64 = 946_728_000;
const TT_MINUS_TAI_S: f64 = 32.184;
const NANOS_PER_SEC_F64: f64 = 1.0e9;
const NANOS_PER_SEC_I64: i64 = 1_000_000_000;

/// Converts a CCSDS [`Epoch`] in `time_system` into a [`TdbEpoch`].
pub fn epoch_to_tdb(
    epoch: &Epoch,
    time_system: TimeSystem,
    scales: &TimeScaleCatalog,
) -> Result<TdbEpoch, CcsdsError> {
    let (unix_s, nanos) = epoch_to_unix_secs_nanos(epoch)?;
    let frac_s = (nanos as f64) / NANOS_PER_SEC_F64;
    match time_system {
        TimeSystem::Utc => {
            let utc = UtcTime::from_unix_seconds((unix_s as f64) + frac_s);
            Ok(scales.utc_to_tdb(utc)?)
        }
        TimeSystem::Tdb => {
            let j2000_s = unix_s.saturating_sub(SECS_1970_TO_J2000_I64);
            let total_ns = i128::from(j2000_s)
                .saturating_mul(i128::from(NANOS_PER_SEC_I64))
                .saturating_add(i128::from(nanos));
            Ok(TdbEpoch::from_j2000_nanos(total_ns))
        }
        TimeSystem::Tai => {
            let tai_j2000_s = (unix_s.saturating_sub(SECS_1970_TO_J2000_I64) as f64) + frac_s;
            tai_j2000_to_tdb(tai_j2000_s, scales)
        }
        TimeSystem::Tt => {
            let tt_j2000_s = (unix_s.saturating_sub(SECS_1970_TO_J2000_I64) as f64) + frac_s;
            tai_j2000_to_tdb(tt_j2000_s - TT_MINUS_TAI_S, scales)
        }
        other => Err(CcsdsError::UnsupportedTimeSystem(other)),
    }
}

/// Converts a [`TdbEpoch`] into a CCSDS [`Epoch`] expressed in `time_system`.
pub fn tdb_to_epoch(
    tdb: TdbEpoch,
    time_system: TimeSystem,
    scales: &TimeScaleCatalog,
) -> Result<Epoch, CcsdsError> {
    match time_system {
        TimeSystem::Utc => {
            let utc = scales.tdb_to_utc(tdb)?;
            let (secs, nanos) = split_unix_seconds_f64(utc.unix_seconds())?;
            unix_secs_nanos_to_epoch(secs, nanos)
        }
        TimeSystem::Tdb => {
            let (w, sub) = tdb.split_j2000_seconds();
            let (secs, nanos) = split_j2000_to_unix(w, sub)?;
            unix_secs_nanos_to_epoch(secs, nanos)
        }
        TimeSystem::Tai => {
            let tai_j2000 = scales.tdb_to_tai_j2000_seconds(tdb);
            let (secs, nanos) =
                split_unix_seconds_f64((SECS_1970_TO_J2000_I64 as f64) + tai_j2000)?;
            unix_secs_nanos_to_epoch(secs, nanos)
        }
        TimeSystem::Tt => {
            let tt_j2000 = scales.tdb_to_tai_j2000_seconds(tdb) + TT_MINUS_TAI_S;
            let (secs, nanos) = split_unix_seconds_f64((SECS_1970_TO_J2000_I64 as f64) + tt_j2000)?;
            unix_secs_nanos_to_epoch(secs, nanos)
        }
        other => Err(CcsdsError::UnsupportedTimeSystem(other)),
    }
}

/// Converts a [`UtcTime`] directly into a CCSDS [`Epoch`].
pub fn utc_time_to_epoch(utc: UtcTime) -> Result<Epoch, CcsdsError> {
    let (secs, nanos) = split_unix_seconds_f64(utc.unix_seconds())?;
    unix_secs_nanos_to_epoch(secs, nanos)
}

/// Converts a CCSDS UTC [`Epoch`] directly into [`UtcTime`].
pub fn epoch_to_utc_time(epoch: &Epoch) -> Result<UtcTime, CcsdsError> {
    let (secs, nanos) = epoch_to_unix_secs_nanos(epoch)?;
    Ok(UtcTime::from_unix_seconds(
        (secs as f64) + (nanos as f64) / NANOS_PER_SEC_F64,
    ))
}

/// Exports the [`SimEpoch`] anchor (`SimInstant::START`) as a CCSDS [`Epoch`].
pub fn sim_epoch_start_to_ccsds(
    sim_epoch: &SimEpoch,
    time_system: TimeSystem,
) -> Result<Epoch, CcsdsError> {
    tdb_to_epoch(
        sim_epoch.tdb_at(SimInstant::START),
        time_system,
        sim_epoch.scales(),
    )
}

fn tai_j2000_to_tdb(tai_j2000_s: f64, scales: &TimeScaleCatalog) -> Result<TdbEpoch, CcsdsError> {
    let mut guess_s = tai_j2000_s + TT_MINUS_TAI_S;
    for _ in 0..3 {
        let guess = TdbEpoch::from_j2000_seconds_f64(guess_s).ok_or(CcsdsError::NonFinite)?;
        let err = tai_j2000_s - scales.tdb_to_tai_j2000_seconds(guess);
        guess_s += err;
    }
    TdbEpoch::from_j2000_seconds_f64(guess_s).ok_or(CcsdsError::NonFinite)
}

fn split_j2000_to_unix(w: f64, sub: f64) -> Result<(i64, u32), CcsdsError> {
    if !w.is_finite() || !sub.is_finite() {
        return Err(CcsdsError::NonFinite);
    }
    let whole_j2000 = libm::floor(w) as i64;
    let rem_s = (w - libm::floor(w)) + sub;
    let (extra_s, nanos) = split_unix_seconds_f64(rem_s)?;
    Ok((
        SECS_1970_TO_J2000_I64
            .saturating_add(whole_j2000)
            .saturating_add(extra_s),
        nanos,
    ))
}

pub(crate) fn split_unix_seconds_f64(unix_seconds: f64) -> Result<(i64, u32), CcsdsError> {
    if !unix_seconds.is_finite() {
        return Err(CcsdsError::NonFinite);
    }
    let whole = libm::floor(unix_seconds);
    let mut nanos = libm::round((unix_seconds - whole) * NANOS_PER_SEC_F64) as i64;
    let mut secs = whole as i64;
    if nanos >= NANOS_PER_SEC_I64 {
        secs = secs.saturating_add(1);
        nanos -= NANOS_PER_SEC_I64;
    } else if nanos < 0 {
        secs = secs.saturating_sub(1);
        nanos += NANOS_PER_SEC_I64;
    }
    Ok((secs, nanos as u32))
}

pub(crate) fn unix_secs_nanos_to_epoch(unix_secs: i64, nanos: u32) -> Result<Epoch, CcsdsError> {
    let days = unix_secs.div_euclid(86_400);
    let sod = unix_secs.rem_euclid(86_400);
    let (year, month, day) = civil_from_days(days);
    let hour = (sod / 3_600) as u8;
    let minute = ((sod % 3_600) / 60) as u8;
    let second = (sod % 60) as u8;
    let year_u16 = u16::try_from(year)
        .map_err(|_| CcsdsError::Epoch(format!("year `{year}` out of range")))?;
    let frac = format_nanos_trimmed(nanos);
    Epoch::new(
        year_u16,
        DateForm::Calendar { month, day },
        hour,
        minute,
        second,
        &frac,
        false,
    )
    .map_err(|e| CcsdsError::Epoch(e.to_string()))
}

pub(crate) fn epoch_to_unix_secs_nanos(epoch: &Epoch) -> Result<(i64, u32), CcsdsError> {
    let year = i32::from(epoch.year());
    let days = match epoch.date() {
        DateForm::Calendar { month, day } => days_from_civil(year, month, day),
        DateForm::Ordinal { day_of_year } => {
            days_from_civil(year, 1, 1) + i64::from(day_of_year.saturating_sub(1))
        }
    };
    let sod = i64::from(epoch.hour()) * 3_600
        + i64::from(epoch.minute()) * 60
        + i64::from(epoch.second());
    let nanos = parse_fraction_nanos(epoch.fraction_digits());
    Ok((days * 86_400 + sod, nanos))
}

fn format_nanos_trimmed(nanos: u32) -> String {
    if nanos == 0 {
        return String::new();
    }
    let s = format!("{nanos:09}");
    s.trim_end_matches('0').to_owned()
}

fn parse_fraction_nanos(digits: &str) -> u32 {
    let mut val = 0u32;
    let mut scale = 100_000_000u32;
    for b in digits.bytes().take(9) {
        if b.is_ascii_digit() {
            val = val.saturating_add(u32::from(b - b'0').saturating_mul(scale));
        }
        scale /= 10;
    }
    val
}

fn days_from_civil(year: i32, month: u8, day: u8) -> i64 {
    let y = i64::from(year) - if month <= 2 { 1 } else { 0 };
    let era = y.div_euclid(400);
    let yoe = y.rem_euclid(400);
    let m = i64::from(month);
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + i64::from(day) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

fn civil_from_days(days_since_1970: i64) -> (i32, u8, u8) {
    let z = days_since_1970 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = mp + if mp < 10 { 3 } else { -9 };
    let year = (y + if m <= 2 { 1 } else { 0 }) as i32;
    (year, m as u8, d as u8)
}
