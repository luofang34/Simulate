//! CCSDS real and integer values: lexical checks, parsing, and writing.
//!
//! Parsing goes through `str::parse::<f64>`, which rounds correctly, after a
//! lexical check that rejects what the standard does not allow (`inf`, `nan`,
//! hexadecimal) and what overflows to infinity. Writing uses Rust's shortest
//! round-trip formatting, so every finite `f64` reads back bit for bit.

use crate::ParseMode;

/// Parses a CCSDS real number.
///
/// Lenient mode also accepts a Fortran `D` exponent marker (`1.5D-03`).
pub(crate) fn parse_f64(text: &str, mode: ParseMode) -> Option<f64> {
    let owned;
    let text = if mode == ParseMode::Lenient && text.contains(['d', 'D']) {
        owned = text.replace(['d', 'D'], "e");
        owned.as_str()
    } else {
        text
    };
    if !is_real(text) {
        return None;
    }
    text.parse::<f64>().ok().filter(|value| value.is_finite())
}

/// Parses an optionally signed decimal integer.
pub(crate) fn parse_int<T: std::str::FromStr>(text: &str) -> Option<T> {
    let digits = text.strip_prefix(['+', '-']).unwrap_or(text);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    text.parse::<T>().ok()
}

/// Writes a finite `f64` in its shortest round-trip form.
///
/// Magnitudes far from one use exponent notation so that small covariance
/// terms do not turn into long runs of zeros. Returns `None` for NaN and
/// infinities, which no NDM can carry.
pub(crate) fn format_f64(value: f64) -> Option<String> {
    if !value.is_finite() {
        return None;
    }
    let magnitude = value.abs();
    if magnitude != 0.0 && !(1e-4..1e15).contains(&magnitude) {
        Some(format!("{value:e}"))
    } else {
        Some(format!("{value}"))
    }
}

/// Whether `text` matches `[+-]? (d+ [. d*] | . d+) ([eE] [+-]? d+)?`.
fn is_real(text: &str) -> bool {
    let body = text.strip_prefix(['+', '-']).unwrap_or(text);
    let (mantissa, exponent) = match body.find(['e', 'E']) {
        Some(at) => (body.get(..at), body.get(at.saturating_add(1)..)),
        None => (Some(body), None),
    };
    let Some(mantissa) = mantissa else {
        return false;
    };
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    let all_digits = |s: &str| s.bytes().all(|b| b.is_ascii_digit());
    let mantissa_ok =
        all_digits(whole) && all_digits(fraction) && !(whole.is_empty() && fraction.is_empty());
    let exponent_ok = exponent.is_none_or(|exp| {
        let digits = exp.strip_prefix(['+', '-']).unwrap_or(exp);
        !digits.is_empty() && all_digits(digits)
    });
    mantissa_ok && exponent_ok
}

#[cfg(test)]
mod tests;
