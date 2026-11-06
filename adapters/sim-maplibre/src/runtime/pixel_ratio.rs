//! Display pixel-ratio filtering shared by every map that forwards it to `HeadlessMap`.

/// Smallest pixel ratio forwarded to a map.
pub(crate) const MIN_PIXEL_RATIO: f64 = 0.5;
/// Largest pixel ratio forwarded to a map.
pub(crate) const MAX_PIXEL_RATIO: f64 = 4.0;
/// Changes smaller than this are window-system rounding noise and would only rebuild
/// layout state for no visible difference.
const PIXEL_RATIO_EPSILON: f64 = 1e-3;

/// The clamped ratio to apply when `requested` differs meaningfully from `last`, or `None`
/// when the request is non-finite or would not change anything.
#[must_use]
pub(crate) fn pixel_ratio_update(last: Option<f64>, requested: f64) -> Option<f64> {
    if !requested.is_finite() {
        return None;
    }
    let clamped = requested.clamp(MIN_PIXEL_RATIO, MAX_PIXEL_RATIO);
    match last {
        Some(last) if (clamped - last).abs() <= PIXEL_RATIO_EPSILON => None,
        _ => Some(clamped),
    }
}

#[cfg(test)]
mod tests;
