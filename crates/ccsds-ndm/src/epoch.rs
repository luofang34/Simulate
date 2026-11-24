//! Epochs as written: calendar fields plus the exact fraction digits.
//!
//! An [`Epoch`] keeps which date form the message used (`YYYY-MM-DD` or
//! `YYYY-DDD`), every digit of the fractional second, and whether a `Z`
//! followed, so it writes back exactly as read. It does not convert between
//! time scales: the scale is the message's `TIME_SYSTEM`, which travels with
//! the metadata.

use std::fmt;

/// The date part of an epoch, in the form the message wrote it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DateForm {
    /// `YYYY-MM-DD`.
    Calendar {
        /// Month, 1 to 12.
        month: u8,
        /// Day of month, 1 to 31.
        day: u8,
    },
    /// `YYYY-DDD`.
    Ordinal {
        /// Day of year, 1 to 366.
        day_of_year: u16,
    },
}

/// Why a text is not a CCSDS epoch.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum EpochError {
    /// The text does not have the `date T time` shape.
    #[error("expected YYYY-MM-DDThh:mm:ss[.d][Z] or YYYY-DDDThh:mm:ss[.d][Z]")]
    Malformed,
    /// A field is outside its range.
    #[error("{field} out of range")]
    OutOfRange {
        /// The field name.
        field: &'static str,
    },
}

/// A CCSDS ASCII time code: calendar fields, fraction digits, and `Z` flag.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Epoch {
    year: u16,
    date: DateForm,
    hour: u8,
    minute: u8,
    second: u8,
    fraction: String,
    zulu: bool,
}

impl Epoch {
    /// Builds an epoch from its fields, checking each range.
    ///
    /// `fraction` holds the digits after the decimal point and may be empty.
    /// `second` may be 60 to carry a leap second.
    pub fn new(
        year: u16,
        date: DateForm,
        hour: u8,
        minute: u8,
        second: u8,
        fraction: &str,
        zulu: bool,
    ) -> Result<Self, EpochError> {
        let out_of_range = |field| EpochError::OutOfRange { field };
        if year > 9999 {
            return Err(out_of_range("year"));
        }
        match date {
            DateForm::Calendar { month, day } => {
                let last = days_in_month(year, month).ok_or(out_of_range("month"))?;
                if day == 0 || day > last {
                    return Err(out_of_range("day"));
                }
            }
            DateForm::Ordinal { day_of_year } => {
                let last = if is_leap(year) { 366 } else { 365 };
                if day_of_year == 0 || day_of_year > last {
                    return Err(out_of_range("day of year"));
                }
            }
        }
        if hour > 23 {
            return Err(out_of_range("hour"));
        }
        if minute > 59 {
            return Err(out_of_range("minute"));
        }
        if second > 60 {
            return Err(out_of_range("second"));
        }
        if !fraction.bytes().all(|b| b.is_ascii_digit()) {
            return Err(EpochError::Malformed);
        }
        Ok(Self {
            year,
            date,
            hour,
            minute,
            second,
            fraction: fraction.to_owned(),
            zulu,
        })
    }

    /// Parses `YYYY-MM-DDThh:mm:ss[.d…][Z]` or `YYYY-DDDThh:mm:ss[.d…][Z]`.
    pub fn parse(text: &str) -> Result<Self, EpochError> {
        let (date, time) = text.split_once('T').ok_or(EpochError::Malformed)?;
        let (time, zulu) = match time.strip_suffix('Z') {
            Some(rest) => (rest, true),
            None => (time, false),
        };
        let (year, rest) = date.split_once('-').ok_or(EpochError::Malformed)?;
        let year = fixed_digits(year, 4)?;
        let date = match rest.split_once('-') {
            Some((month, day)) => DateForm::Calendar {
                month: narrow(fixed_digits(month, 2)?)?,
                day: narrow(fixed_digits(day, 2)?)?,
            },
            None => DateForm::Ordinal {
                day_of_year: fixed_digits(rest, 3)?,
            },
        };
        let (clock, fraction) = match time.split_once('.') {
            Some((clock, fraction)) if !fraction.is_empty() => (clock, fraction),
            Some(_) => return Err(EpochError::Malformed),
            None => (time, ""),
        };
        let mut parts = clock.split(':');
        let mut next = || -> Result<u8, EpochError> {
            narrow(fixed_digits(parts.next().ok_or(EpochError::Malformed)?, 2)?)
        };
        let (hour, minute, second) = (next()?, next()?, next()?);
        if parts.next().is_some() {
            return Err(EpochError::Malformed);
        }
        Self::new(year, date, hour, minute, second, fraction, zulu)
    }

    /// The four-digit year.
    pub fn year(&self) -> u16 {
        self.year
    }

    /// The date fields in the form the message used.
    pub fn date(&self) -> DateForm {
        self.date
    }

    /// Hour, 0 to 23.
    pub fn hour(&self) -> u8 {
        self.hour
    }

    /// Minute, 0 to 59.
    pub fn minute(&self) -> u8 {
        self.minute
    }

    /// Whole seconds, 0 to 60.
    pub fn second(&self) -> u8 {
        self.second
    }

    /// The digits after the decimal point, exactly as written; may be empty.
    pub fn fraction_digits(&self) -> &str {
        &self.fraction
    }

    /// Whether the time ended in `Z`.
    pub fn is_zulu(&self) -> bool {
        self.zulu
    }
}

impl std::str::FromStr for Epoch {
    type Err = EpochError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        Self::parse(text)
    }
}

impl fmt::Display for Epoch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.date {
            DateForm::Calendar { month, day } => write!(f, "{:04}-{month:02}-{day:02}", self.year)?,
            DateForm::Ordinal { day_of_year } => write!(f, "{:04}-{day_of_year:03}", self.year)?,
        }
        write!(f, "T{:02}:{:02}:{:02}", self.hour, self.minute, self.second)?;
        if !self.fraction.is_empty() {
            write!(f, ".{}", self.fraction)?;
        }
        if self.zulu {
            f.write_str("Z")?;
        }
        Ok(())
    }
}

fn fixed_digits(text: &str, width: usize) -> Result<u16, EpochError> {
    if text.len() != width || !text.bytes().all(|b| b.is_ascii_digit()) {
        return Err(EpochError::Malformed);
    }
    text.parse().map_err(|_| EpochError::Malformed)
}

fn narrow(value: u16) -> Result<u8, EpochError> {
    u8::try_from(value).map_err(|_| EpochError::Malformed)
}

fn is_leap(year: u16) -> bool {
    (year.is_multiple_of(4) && !year.is_multiple_of(100)) || year.is_multiple_of(400)
}

fn days_in_month(year: u16, month: u8) -> Option<u8> {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => Some(31),
        4 | 6 | 9 | 11 => Some(30),
        2 if is_leap(year) => Some(29),
        2 => Some(28),
        _ => None,
    }
}

#[cfg(test)]
mod tests;
