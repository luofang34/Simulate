//! A runway's longitudinal profile: the terrain's smoothed centreline, bent so both ends
//! sit at their published elevations, kept within plausible runway gradients and never
//! above the published field elevation, the highest point of any runway.

use sim_geo::{Geodetic, ecef_to_geodetic, geodetic_to_ecef};

use crate::AirportRecord;

/// Spacing of profile stations along the centreline.
const STATION_M: f64 = 15.0;
/// Half-width of the moving average that removes terrain noise from the centreline.
const SMOOTHING_HALF_WIDTH_M: f64 = 100.0;
/// Steepest longitudinal gradient a runway is graded to.
const MAX_GRADE: f64 = 0.02;
/// A crown smaller than this is DEM noise, not a reason to raise the runway to the field.
const MIN_CROWN_M: f64 = 0.3;
/// Most a measured crown is scaled up to reach the published field elevation.
const MAX_CROWN_GAIN: f64 = 3.0;

/// Heights at evenly spaced stations from a runway's first end to its second.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Profile {
    heights_m: Vec<f64>,
}

impl Profile {
    /// The height `fraction` of the way from the first end, `0..=1`.
    pub(crate) fn at(&self, fraction: f64) -> Option<f64> {
        let last = self.heights_m.len().checked_sub(1)?;
        let position = fraction.clamp(0.0, 1.0) * last as f64;
        let below = (position.floor() as usize).min(last);
        let above = (below + 1).min(last);
        let t = position - below as f64;
        Some(self.heights_m.get(below)? * (1.0 - t) + self.heights_m.get(above)? * t)
    }

    fn peak(&self) -> f64 {
        self.heights_m
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
    }
}

/// One runway's profile before the airport-wide field elevation is applied.
struct Draft {
    /// Published end elevations, straight between them, per station.
    straight: Vec<f64>,
    /// Smoothed terrain above or below that line, per station.
    crown: Vec<f64>,
    station_m: f64,
}

impl Draft {
    fn heights(&self, gain: f64) -> Vec<f64> {
        let raised: Vec<f64> = self
            .straight
            .iter()
            .zip(&self.crown)
            .map(|(line, crown)| line + crown * gain)
            .collect();
        limit_grade(&raised, self.station_m)
    }
}

/// The profile of every runway of `record`, by runway index, from `terrain_msl` sampled
/// along each centreline; `None` for a runway whose terrain is not all known, which
/// stays straight between its ends.
pub(crate) fn airport_profiles(
    record: &AirportRecord,
    terrain_msl: &dyn Fn(&Geodetic) -> Option<f64>,
) -> Vec<Option<Profile>> {
    let drafts: Vec<Option<Draft>> = record
        .runways
        .iter()
        .map(|runway| {
            let [a, b] = &runway.ends;
            draft(
                (
                    Geodetic::from_degrees(a.lat_deg, a.lon_deg, 0.0),
                    f64::from(a.elevation_msl_m),
                ),
                (
                    Geodetic::from_degrees(b.lat_deg, b.lon_deg, 0.0),
                    f64::from(b.elevation_msl_m),
                ),
                terrain_msl,
            )
        })
        .collect();
    let field = f64::from(record.elevation_msl_m);
    let gains = field_gains(&drafts, field);
    drafts
        .iter()
        .zip(gains)
        .map(|(draft, gain)| {
            let draft = draft.as_ref()?;
            let heights = draft
                .heights(gain)
                .into_iter()
                .zip(&draft.straight)
                .map(|(height, line)| {
                    if field.is_finite() {
                        height.min(field.max(*line))
                    } else {
                        height
                    }
                })
                .collect();
            Some(Profile { heights_m: heights })
        })
        .collect()
}

fn draft(
    (a, a_msl): (Geodetic, f64),
    (b, b_msl): (Geodetic, f64),
    terrain_msl: &dyn Fn(&Geodetic) -> Option<f64>,
) -> Option<Draft> {
    let (a_ecef, b_ecef) = (geodetic_to_ecef(&a), geodetic_to_ecef(&b));
    let length_m = (b_ecef - a_ecef).length();
    if !(length_m.is_finite() && length_m > STATION_M && a_msl.is_finite() && b_msl.is_finite()) {
        return None;
    }
    let intervals = (length_m / STATION_M).ceil().max(2.0) as usize;
    let station_m = length_m / intervals as f64;
    // The terrain is read past both ends, over the overruns, so the runway's own ends are
    // smoothed as fully as its middle.
    let margin = (SMOOTHING_HALF_WIDTH_M / station_m).round() as usize;
    let terrain = (0..=intervals + 2 * margin)
        .map(|i| {
            let t = (i as f64 - margin as f64) / intervals as f64;
            terrain_msl(&ecef_to_geodetic(a_ecef + (b_ecef - a_ecef) * t))
        })
        .collect::<Option<Vec<f64>>>()?;
    let smooth = moving_average(&terrain, margin);
    let smooth = smooth.get(margin..=margin + intervals)?;
    let (&first, &last) = (smooth.first()?, smooth.last()?);
    let straight: Vec<f64> = (0..=intervals)
        .map(|i| a_msl + (b_msl - a_msl) * (i as f64 / intervals as f64))
        .collect();
    let crown = smooth
        .iter()
        .enumerate()
        .map(|(i, height)| height - (first + (last - first) * (i as f64 / intervals as f64)))
        .collect();
    Some(Draft {
        straight,
        crown,
        station_m,
    })
}

/// How much each runway's crown is scaled: down so no runway rises above the field
/// elevation, and up, within reason, so the highest runway reaches it when the terrain
/// shows a real crown there.
fn field_gains(drafts: &[Option<Draft>], field: f64) -> Vec<f64> {
    let mut gains = vec![1.0; drafts.len()];
    if !field.is_finite() {
        return gains;
    }
    let peaks: Vec<Option<(f64, f64)>> = drafts
        .iter()
        .map(|draft| {
            let draft = draft.as_ref()?;
            let peak = Profile {
                heights_m: draft.heights(1.0),
            }
            .peak();
            let ends = draft
                .straight
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);
            Some((peak, ends))
        })
        .collect();
    let airport_peak = peaks
        .iter()
        .flatten()
        .map(|p| p.0)
        .fold(f64::NEG_INFINITY, f64::max);
    let ends_peak = peaks
        .iter()
        .flatten()
        .map(|p| p.1)
        .fold(f64::NEG_INFINITY, f64::max);
    if airport_peak >= field || ends_peak >= field {
        return gains;
    }
    for ((gain, peak), draft) in gains.iter_mut().zip(&peaks).zip(drafts) {
        let (Some((peak, _)), Some(draft)) = (peak, draft) else {
            continue;
        };
        if *peak < airport_peak {
            continue;
        }
        let rise = draft
            .crown
            .iter()
            .zip(&draft.straight)
            .filter(|(crown, _)| **crown >= MIN_CROWN_M)
            .map(|(crown, line)| (field - line) / crown)
            .fold(f64::INFINITY, f64::min);
        if rise.is_finite() && rise > 1.0 {
            *gain = rise.min(MAX_CROWN_GAIN);
        }
    }
    gains
}

/// `values` averaged over up to `half_width` neighbours on each side, as many on both
/// sides so a straight run stays straight; the ends keep their own value.
fn moving_average(values: &[f64], half_width: usize) -> Vec<f64> {
    let last = values.len().saturating_sub(1);
    (0..values.len())
        .map(|i| {
            let half = half_width.min(i).min(last - i);
            let window = values.get(i - half..=i + half).unwrap_or(&[]);
            window.iter().sum::<f64>() / window.len().max(1) as f64
        })
        .collect()
}

/// `heights` with every station-to-station gradient held to [`MAX_GRADE`] and both ends
/// kept: first within the band the two ends allow, then step by step from each end.
fn limit_grade(heights: &[f64], station_m: f64) -> Vec<f64> {
    let (Some(&first), Some(&last)) = (heights.first(), heights.last()) else {
        return heights.to_vec();
    };
    let span = station_m * heights.len().saturating_sub(1) as f64;
    let step = MAX_GRADE * station_m;
    let mut out: Vec<f64> = heights
        .iter()
        .enumerate()
        .map(|(i, &h)| {
            let x = station_m * i as f64;
            let low = (first - MAX_GRADE * x).max(last - MAX_GRADE * (span - x));
            let high = (first + MAX_GRADE * x).min(last + MAX_GRADE * (span - x));
            if low <= high { h.clamp(low, high) } else { h }
        })
        .collect();
    for i in 1..out.len() {
        if let (Some(&prev), Some(current)) = (out.get(i - 1), out.get(i)) {
            let limited = current.clamp(prev - step, prev + step);
            if let Some(slot) = out.get_mut(i) {
                *slot = limited;
            }
        }
    }
    for i in (0..out.len().saturating_sub(1)).rev() {
        if let (Some(&next), Some(current)) = (out.get(i + 1), out.get(i)) {
            let limited = current.clamp(next - step, next + step);
            if let Some(slot) = out.get_mut(i) {
                *slot = limited;
            }
        }
    }
    out
}
