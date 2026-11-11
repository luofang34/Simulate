//! The link's numbers: lexed as plain decimals, then held to their ranges.

use super::token::{Site, Token};
use crate::{Fixed, Milli, Parameter, Reason, Refusal};

/// A decimal with at most `DECIMALS` fractional digits.
pub(super) fn fixed<const DECIMALS: u32>(
    site: Site,
    token: Token<'_>,
    signed: bool,
) -> Result<Fixed<DECIMALS>, Refusal> {
    Fixed::parse(token.text, signed).map_err(|fault| {
        site.refuse(
            token.offset,
            Reason::BadNumber {
                text: token.text.to_owned(),
                decimals: DECIMALS,
                fault,
            },
        )
    })
}

/// The inclusive bounds of a quantity, in whole units, and how to say them.
pub(super) struct Bounds {
    pub(super) what: &'static str,
    pub(super) min: i64,
    pub(super) max: i64,
    pub(super) range: &'static str,
}

/// A decimal inside `bounds`.
pub(super) fn ranged<const DECIMALS: u32>(
    site: Site,
    token: Token<'_>,
    bounds: &Bounds,
) -> Result<Fixed<DECIMALS>, Refusal> {
    let value = fixed::<DECIMALS>(site, token, bounds.min < 0)?;
    let inside =
        Fixed::<DECIMALS>::from_whole(bounds.min)..=Fixed::<DECIMALS>::from_whole(bounds.max);
    if !inside.contains(&value) {
        return Err(out_of_range(site, token, bounds.what, bounds.range));
    }
    Ok(value)
}

pub(super) fn out_of_range(
    site: Site,
    token: Token<'_>,
    what: &'static str,
    range: &'static str,
) -> Refusal {
    site.refuse(
        token.offset,
        Reason::OutOfRange {
            what,
            value: token.text.to_owned(),
            range,
        },
    )
}

/// Digits only, as a whole number; `None` past `u64`.
pub(super) fn whole(site: Site, token: Token<'_>) -> Result<Option<u64>, Refusal> {
    if token.text.is_empty() || !token.text.bytes().all(|byte| byte.is_ascii_digit()) {
        return Err(site.refuse(
            token.offset,
            Reason::Malformed {
                text: token.text.to_owned(),
                expected: "a whole number in plain digits",
            },
        ));
    }
    Ok(token.text.parse().ok())
}

pub(super) fn seed(value: Token<'_>) -> Result<u64, Refusal> {
    let site = Site::of(Parameter::Seed);
    whole(site, value)?.ok_or_else(|| out_of_range(site, value, "seed", "0 to 2^64-1"))
}

pub(super) fn budget_mb(value: Token<'_>) -> Result<u32, Refusal> {
    let site = Site::of(Parameter::Mb);
    whole(site, value)?
        .and_then(|megabytes| u32::try_from(megabytes).ok())
        .filter(|megabytes| (1..=65_536).contains(megabytes))
        .ok_or_else(|| out_of_range(site, value, "budget", "1 to 65536 MB"))
}

pub(super) fn cruise_speed(value: Token<'_>) -> Result<Milli, Refusal> {
    let speed = ranged::<3>(
        Site::of(Parameter::Spd),
        value,
        &Bounds {
            what: "speed",
            min: 0,
            max: 350,
            range: "1 to 350 m/s",
        },
    )?;
    if speed < Milli::from_whole(1) {
        return Err(out_of_range(
            Site::of(Parameter::Spd),
            value,
            "speed",
            "1 to 350 m/s",
        ));
    }
    Ok(speed)
}
