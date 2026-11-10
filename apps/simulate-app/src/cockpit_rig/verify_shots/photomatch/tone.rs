//! How light and dark the airframe reads in a photograph and in the model rendered from
//! its camera: luminance percentiles over the airframe's silhouette, so lighting and tone
//! mapping are judged by numbers measured in the matched view and not by eye.

use image::{Rgb, RgbImage};
use serde_json::{Value, json};

/// Pixels shaved off the silhouette's edge, where photo and model outlines disagree and
/// background leaks in.
const EDGE_PX: usize = 4;

/// Display luminance (`0..=255`, Rec. 601) percentiles of an airframe.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Tone {
    /// 5th percentile: the deep shade.
    pub(super) p05: u8,
    /// 25th percentile.
    pub(super) p25: u8,
    /// Median.
    pub(super) p50: u8,
    /// 75th percentile.
    pub(super) p75: u8,
    /// 95th percentile: sunlit paint and highlights.
    pub(super) p95: u8,
}

impl Tone {
    /// The percentiles of `image` under `mask` (row-major, as wide as the image).
    pub(super) fn measure(image: &RgbImage, mask: &[bool]) -> Option<Self> {
        let mut values: Vec<u8> = image
            .pixels()
            .zip(mask)
            .filter(|(_, inside)| **inside)
            .map(|(p, _)| luma(*p))
            .collect();
        values.sort_unstable();
        let at = |q: f64| {
            let index = ((values.len().saturating_sub(1)) as f64 * q).round() as usize;
            values.get(index).copied()
        };
        Some(Self {
            p05: at(0.05)?,
            p25: at(0.25)?,
            p50: at(0.50)?,
            p75: at(0.75)?,
            p95: at(0.95)?,
        })
    }

    fn json(self) -> Value {
        json!([self.p05, self.p25, self.p50, self.p75, self.p95])
    }
}

/// Photo and model tone inside the silhouette `mask` (`w` wide), as the summary records
/// them: percentiles `[p05, p25, p50, p75, p95]` of each.
pub(super) fn compare(photo: &RgbImage, model: &RgbImage, mask: &[bool], w: u32) -> Value {
    let core = erode(mask, w as usize);
    json!({
        "photo": Tone::measure(photo, &core).map(Tone::json),
        "model": Tone::measure(model, &core).map(Tone::json),
    })
}

/// `mask` without the pixels within [`EDGE_PX`] of its outside.
fn erode(mask: &[bool], w: usize) -> Vec<bool> {
    let h = mask.len() / w.max(1);
    let get = |x: usize, y: usize| mask.get(y * w + x).copied().unwrap_or(false);
    (0..mask.len())
        .map(|i| {
            let (x, y) = (i % w.max(1), i / w.max(1));
            if x < EDGE_PX || y < EDGE_PX || x + EDGE_PX >= w || y + EDGE_PX >= h {
                return false;
            }
            (y - EDGE_PX..=y + EDGE_PX).all(|yy| (x - EDGE_PX..=x + EDGE_PX).all(|xx| get(xx, yy)))
        })
        .collect()
}

/// Rec. 601 luma of a display pixel.
pub(super) fn luma(p: Rgb<u8>) -> u8 {
    let [r, g, b] = p.0.map(f64::from);
    (0.299 * r + 0.587 * g + 0.114 * b)
        .round()
        .clamp(0.0, 255.0) as u8
}
