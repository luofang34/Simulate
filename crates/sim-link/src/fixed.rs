//! Decimal quantities as scaled integers, so a link's numbers survive
//! decoding and encoding digit for digit.

#[cfg(test)]
mod tests;

/// Most integer digits a link number carries: with six decimals the scaled
/// value still fits an `i64`.
const MAX_INTEGER_DIGITS: usize = 12;

/// A decimal number with `DECIMALS` fractional digits, held as a whole count
/// of its last digit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Fixed<const DECIMALS: u32>(i64);

/// Millionths: degrees of latitude and longitude.
pub type Micro = Fixed<6>;
/// Thousandths: metres, seconds, degrees of heading and view, m/s, hertz.
pub type Milli = Fixed<3>;

/// Why text is not a number a link may carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NumberFault {
    /// Not digits with an optional `.` and digits: a sign where none
    /// belongs, an exponent, a bare `.`, or other characters.
    NotDecimal,
    /// More fractional digits than the quantity keeps.
    TooManyDecimals,
    /// More integer digits than any link quantity needs.
    TooManyDigits,
}

impl<const DECIMALS: u32> Fixed<DECIMALS> {
    /// Units in one whole.
    const SCALE: i64 = 10_i64.pow(DECIMALS);

    /// The number that is `scaled` counts of its last decimal.
    pub const fn from_scaled(scaled: i64) -> Self {
        Self(scaled)
    }

    /// The whole number `whole`.
    pub const fn from_whole(whole: i64) -> Self {
        Self(whole.saturating_mul(Self::SCALE))
    }

    /// The count of the last decimal.
    pub const fn scaled(self) -> i64 {
        self.0
    }

    /// The nearest `f64`, the same one the decimal text parses to.
    pub fn as_f64(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    /// Reads `-?digits[.digits]` (`digits[.digits]` when not `signed`) with
    /// at most `DECIMALS` fractional digits. Nothing is rounded.
    pub fn parse(text: &str, signed: bool) -> Result<Self, NumberFault> {
        let (negative, magnitude) = match text.strip_prefix('-') {
            Some(rest) if signed => (true, rest),
            Some(_) => return Err(NumberFault::NotDecimal),
            None => (false, text),
        };
        let (whole, fraction) = match magnitude.split_once('.') {
            Some((whole, fraction)) if !fraction.is_empty() => (whole, fraction),
            Some(_) => return Err(NumberFault::NotDecimal),
            None => (magnitude, ""),
        };
        let digits = |part: &str| part.bytes().all(|byte| byte.is_ascii_digit());
        if whole.is_empty() || !digits(whole) || !digits(fraction) {
            return Err(NumberFault::NotDecimal);
        }
        if fraction.len() > DECIMALS as usize {
            return Err(NumberFault::TooManyDecimals);
        }
        if whole.len() > MAX_INTEGER_DIGITS {
            return Err(NumberFault::TooManyDigits);
        }
        let padding = (DECIMALS as usize).saturating_sub(fraction.len());
        let scaled = whole
            .bytes()
            .chain(fraction.bytes())
            .map(|byte| i64::from(byte.saturating_sub(b'0')))
            .chain(std::iter::repeat_n(0, padding))
            .fold(0_i64, |sum, digit| {
                sum.saturating_mul(10).saturating_add(digit)
            });
        Ok(Self(if negative { -scaled } else { scaled }))
    }

    /// The decimal text with at least `min_decimals` fractional digits and
    /// no trailing zeros beyond them.
    pub fn text(self, min_decimals: u32) -> String {
        let magnitude = self.0.unsigned_abs();
        let scale = Self::SCALE.unsigned_abs();
        let mut fraction = format!("{:0width$}", magnitude % scale, width = DECIMALS as usize);
        while fraction.len() > min_decimals as usize && fraction.ends_with('0') {
            fraction.pop();
        }
        let sign = if self.0 < 0 { "-" } else { "" };
        let whole = magnitude / scale;
        if fraction.is_empty() {
            format!("{sign}{whole}")
        } else {
            format!("{sign}{whole}.{fraction}")
        }
    }
}
