use sim_geo::UtcTime;

use super::TrialError;

/// Parses `YYYY-MM-DDTHH:MM:SS[.fff]Z` as UTC. Leap seconds are not
/// accepted: a trial epoch is an input instant, not a timestamp to replay.
pub(super) fn parse_utc(text: &str) -> Result<UtcTime, TrialError> {
    let invalid = || TrialError::InvalidRequest {
        field: "epoch_utc",
        detail: format!("`{text}` is not YYYY-MM-DDTHH:MM:SSZ"),
    };
    let body = text.strip_suffix('Z').ok_or_else(invalid)?;
    let (date, time) = body.split_once('T').ok_or_else(invalid)?;
    let mut date_parts = date.splitn(3, '-').map(str::parse::<i64>);
    let mut time_parts = time.splitn(3, ':');
    let next_int = |part: Option<&str>| part.and_then(|p| p.parse::<i64>().ok());
    let (Some(Ok(year)), Some(Ok(month)), Some(Ok(day))) =
        (date_parts.next(), date_parts.next(), date_parts.next())
    else {
        return Err(invalid());
    };
    let (Some(hour), Some(minute)) = (next_int(time_parts.next()), next_int(time_parts.next()))
    else {
        return Err(invalid());
    };
    let second: f64 = time_parts
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(invalid)?;
    let in_range = (1..=12).contains(&month)
        && (1..=31).contains(&day)
        && (0..24).contains(&hour)
        && (0..60).contains(&minute)
        && (0.0..60.0).contains(&second);
    if !in_range {
        return Err(invalid());
    }
    let days = days_from_civil(year, month, day);
    let whole = days * 86_400 + hour * 3_600 + minute * 60;
    Ok(UtcTime::from_unix_seconds(whole as f64 + second))
}

/// Days from 1970-01-01 to a proleptic Gregorian date (Hinnant's
/// `days_from_civil`).
fn days_from_civil(year: i64, month: i64, day: i64) -> i64 {
    let year = if month <= 2 { year - 1 } else { year };
    let era = year.div_euclid(400);
    let year_of_era = year - era * 400;
    let shifted_month = (month + 9) % 12;
    let day_of_year = (153 * shifted_month + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}
