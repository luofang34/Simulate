//! Local `ccsds_ndm::Epoch` <-> `sim_spacetime::TdbEpoch` conversions for `sim-devices`.

use ccsds_ndm::{DateForm, Epoch, TimeSystem};
use sim_geo::UtcTime;
use sim_spacetime::{TdbEpoch, TimeScaleCatalog};

use super::GroundTrackerTdmError;

const SECS_1970_TO_J2000_I64: i64 = 946_728_000;
const TT_MINUS_TAI_S: f64 = 32.184;
const NANOS_PER_SEC_F64: f64 = 1.0e9;
const NANOS_PER_SEC_I64: i64 = 1_000_000_000;

pub(super) fn epoch_to_tdb_impl(
    epoch: &Epoch,
    time_system: TimeSystem,
    scales: &TimeScaleCatalog,
) -> Result<TdbEpoch, GroundTrackerTdmError> {
    let (unix_s, nanos) = epoch_to_unix_secs_nanos(epoch);
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
        other => Err(GroundTrackerTdmError::UnsupportedTimeSystem(other)),
    }
}

pub(super) fn tdb_to_epoch_impl(
    tdb: TdbEpoch,
    time_system: TimeSystem,
    scales: &TimeScaleCatalog,
) -> Result<Epoch, GroundTrackerTdmError> {
    let (secs, nanos) = match time_system {
        TimeSystem::Utc => split_unix_seconds_f64(scales.tdb_to_utc(tdb)?.unix_seconds())?,
        TimeSystem::Tdb => {
            let (w, sub) = tdb.split_j2000_seconds();
            split_j2000_to_unix(w, sub)?
        }
        TimeSystem::Tai => {
            let tai_j2000 = scales.tdb_to_tai_j2000_seconds(tdb);
            split_unix_seconds_f64((SECS_1970_TO_J2000_I64 as f64) + tai_j2000)?
        }
        TimeSystem::Tt => {
            let tt_j2000 = scales.tdb_to_tai_j2000_seconds(tdb) + TT_MINUS_TAI_S;
            split_unix_seconds_f64((SECS_1970_TO_J2000_I64 as f64) + tt_j2000)?
        }
        other => return Err(GroundTrackerTdmError::UnsupportedTimeSystem(other)),
    };
    unix_secs_nanos_to_epoch(secs, nanos)
}

fn tai_j2000_to_tdb(
    tai_j2000_s: f64,
    scales: &TimeScaleCatalog,
) -> Result<TdbEpoch, GroundTrackerTdmError> {
    let mut guess_s = tai_j2000_s + TT_MINUS_TAI_S;
    for _ in 0..3 {
        let guess =
            TdbEpoch::from_j2000_seconds_f64(guess_s).ok_or(GroundTrackerTdmError::NonFinite)?;
        guess_s += tai_j2000_s - scales.tdb_to_tai_j2000_seconds(guess);
    }
    TdbEpoch::from_j2000_seconds_f64(guess_s).ok_or(GroundTrackerTdmError::NonFinite)
}

fn split_j2000_to_unix(w: f64, sub: f64) -> Result<(i64, u32), GroundTrackerTdmError> {
    if !w.is_finite() || !sub.is_finite() {
        return Err(GroundTrackerTdmError::NonFinite);
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

fn split_unix_seconds_f64(unix_seconds: f64) -> Result<(i64, u32), GroundTrackerTdmError> {
    if !unix_seconds.is_finite() {
        return Err(GroundTrackerTdmError::NonFinite);
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

fn unix_secs_nanos_to_epoch(unix_secs: i64, nanos: u32) -> Result<Epoch, GroundTrackerTdmError> {
    let days = unix_secs.div_euclid(86_400);
    let sod = unix_secs.rem_euclid(86_400);
    let (year, month, day) = civil_from_days(days);
    let hour = (sod / 3_600) as u8;
    let minute = ((sod % 3_600) / 60) as u8;
    let second = (sod % 60) as u8;
    let year_u16 = u16::try_from(year)
        .map_err(|_| GroundTrackerTdmError::Epoch(format!("year `{year}` out of range")))?;
    let frac = if nanos == 0 {
        String::new()
    } else {
        format!("{nanos:09}").trim_end_matches('0').to_owned()
    };
    Epoch::new(
        year_u16,
        DateForm::Calendar { month, day },
        hour,
        minute,
        second,
        &frac,
        false,
    )
    .map_err(|e| GroundTrackerTdmError::Epoch(e.to_string()))
}

fn epoch_to_unix_secs_nanos(epoch: &Epoch) -> (i64, u32) {
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
    let mut nanos = 0u32;
    let mut scale = 100_000_000u32;
    for b in epoch.fraction_digits().bytes().take(9) {
        if b.is_ascii_digit() {
            nanos = nanos.saturating_add(u32::from(b - b'0').saturating_mul(scale));
        }
        scale /= 10;
    }
    (days * 86_400 + sod, nanos)
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
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = mp + if mp < 10 { 3 } else { -9 };
    let year = y + if m <= 2 { 1 } else { 0 };
    (year as i32, m as u8, d as u8)
}
