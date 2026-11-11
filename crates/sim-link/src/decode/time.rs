//! `t`: the UTC start time.

use super::token::{Site, Token};
use crate::{Parameter, Reason, Refusal, StartTime};

/// The time of `t`, `YYYYMMDDTHHMMSSZ`.
pub(super) fn start_time(value: Token<'_>) -> Result<StartTime, Refusal> {
    let refuse = |why: &'static str| {
        Site::of(Parameter::T).refuse(
            value.offset,
            Reason::BadTime {
                text: value.text.to_owned(),
                why,
            },
        )
    };
    let shape = "write 8 digits, T, 6 digits, Z";
    let bytes = value.text.as_bytes();
    let digits_at = |from: usize, count: usize| -> Option<u16> {
        let digits = bytes.get(from..from.checked_add(count)?)?;
        digits.iter().try_fold(0_u16, |sum, byte| {
            byte.is_ascii_digit().then(|| {
                sum.saturating_mul(10)
                    .saturating_add(u16::from(byte.saturating_sub(b'0')))
            })
        })
    };
    let field = |from: usize, count: usize| digits_at(from, count).ok_or_else(|| refuse(shape));
    if bytes.len() != 16 || bytes.get(8) != Some(&b'T') || bytes.get(15) != Some(&b'Z') {
        return Err(refuse(shape));
    }
    let narrow = |value: u16| u8::try_from(value).map_err(|_| refuse(shape));
    let time = StartTime {
        year: field(0, 4)?,
        month: narrow(field(4, 2)?)?,
        day: narrow(field(6, 2)?)?,
        hour: narrow(field(9, 2)?)?,
        minute: narrow(field(11, 2)?)?,
        second: narrow(field(13, 2)?)?,
    };
    if time.year == 0 {
        return Err(refuse("the year starts at 0001"));
    }
    if !(1..=12).contains(&time.month) {
        return Err(refuse("the month is 01 to 12"));
    }
    if time.day == 0 || time.day > StartTime::days_in_month(time.year, time.month) {
        return Err(refuse("the month has no such day"));
    }
    if time.hour > 23 || time.minute > 59 {
        return Err(refuse("the hour is 00 to 23 and the minute 00 to 59"));
    }
    if time.second == 60 {
        return Err(refuse("a leap second cannot be a start time"));
    }
    if time.second > 59 {
        return Err(refuse("the second is 00 to 59"));
    }
    Ok(time)
}
