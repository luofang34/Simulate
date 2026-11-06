//! Anti-aliased strokes, fills and rings on an RGBA raster, blended source-over, for
//! drawing chart symbology without a vector graphics dependency.
//!
//! Coordinates are pixels with the origin at the top-left corner; a pixel's centre is at
//! `+0.5`. Everything outside the raster is clipped.

use image::{Rgba, RgbaImage};

/// A point in raster pixels.
pub(super) type Px = [f64; 2];

/// An RGBA raster being drawn on.
pub(super) struct Canvas {
    pub(super) image: RgbaImage,
}

impl Canvas {
    /// A transparent `width × height` raster.
    pub(super) fn new(width: u32, height: u32) -> Self {
        Self {
            image: RgbaImage::new(width, height),
        }
    }

    /// Blends `color` over pixel `(x, y)` with `coverage` in `[0, 1]`.
    pub(super) fn blend(&mut self, x: i64, y: i64, color: [u8; 4], coverage: f64) {
        let (Ok(x), Ok(y)) = (u32::try_from(x), u32::try_from(y)) else {
            return;
        };
        let Some(Rgba(dst)) = self.image.get_pixel_mut_checked(x, y) else {
            return;
        };
        let [r, g, b, a] = color.map(|c| f64::from(c) / 255.0);
        let alpha = a * coverage.clamp(0.0, 1.0);
        let [dr, dg, db, da] = dst.map(|c| f64::from(c) / 255.0);
        let out_a = alpha + da * (1.0 - alpha);
        let mix = |s: f64, d: f64| {
            if out_a <= 0.0 {
                0.0
            } else {
                (s * alpha + d * da * (1.0 - alpha)) / out_a
            }
        };
        let to_u8 = |v: f64| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
        *dst = [
            to_u8(mix(r, dr)),
            to_u8(mix(g, dg)),
            to_u8(mix(b, db)),
            to_u8(out_a),
        ];
    }

    /// Fills the axis-aligned rectangle `[x0, x1) × [y0, y1)`.
    pub(super) fn fill_rect(&mut self, [x0, y0]: Px, [x1, y1]: Px, color: [u8; 4]) {
        for y in (y0.floor() as i64)..(y1.ceil() as i64) {
            for x in (x0.floor() as i64)..(x1.ceil() as i64) {
                self.blend(x, y, color, 1.0);
            }
        }
    }

    /// Strokes the segment `a`–`b` `width` pixels wide.
    pub(super) fn line(&mut self, a: Px, b: Px, width: f64, color: [u8; 4]) {
        let half = width * 0.5;
        self.cover_bounds(bounds(&[a, b], half + 1.0), color, |p| {
            half + 0.5 - distance_to_segment(p, a, b)
        });
    }

    /// Strokes `a`–`b` as dashes `dash` long separated by `gap`.
    pub(super) fn dashed_line(
        &mut self,
        a: Px,
        b: Px,
        width: f64,
        [dash, gap]: [f64; 2],
        color: [u8; 4],
    ) {
        let length = distance(a, b);
        if length <= 0.0 || dash <= 0.0 {
            return;
        }
        let at = |t: f64| [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        let mut start = 0.0;
        while start < length {
            let end = (start + dash).min(length);
            self.line(at(start / length), at(end / length), width, color);
            start += dash + gap.max(0.0);
        }
    }

    /// Strokes the polyline through `points`.
    pub(super) fn polyline(&mut self, points: &[Px], width: f64, color: [u8; 4]) {
        for pair in points.windows(2) {
            if let [a, b] = pair {
                self.line(*a, *b, width, color);
            }
        }
    }

    /// Strokes a circle of `radius` around `center`.
    pub(super) fn ring(&mut self, center: Px, radius: f64, width: f64, color: [u8; 4]) {
        let half = width * 0.5;
        let reach = radius + half + 1.0;
        let corners = [
            [center[0] - reach, center[1] - reach],
            [center[0] + reach, center[1] + reach],
        ];
        self.cover_bounds(bounds(&corners, 0.0), color, |p| {
            half + 0.5 - (distance(p, center) - radius).abs()
        });
    }

    /// Fills the triangle `a`, `b`, `c`.
    pub(super) fn triangle(&mut self, [a, b, c]: [Px; 3], color: [u8; 4]) {
        self.cover_bounds(bounds(&[a, b, c], 1.0), color, |p| {
            let edges = [(a, b), (b, c), (c, a)];
            let inside = edges.iter().all(|(s, e)| cross(*s, *e, p) >= 0.0)
                || edges.iter().all(|(s, e)| cross(*s, *e, p) <= 0.0);
            if inside {
                1.0
            } else {
                0.5 - edges
                    .iter()
                    .map(|(s, e)| distance_to_segment(p, *s, *e))
                    .fold(f64::INFINITY, f64::min)
            }
        });
    }

    /// Blends `color` over every pixel in `[x0, x1) × [y0, y1)` by the coverage `cover`
    /// gives at the pixel's centre.
    fn cover_bounds(
        &mut self,
        [x0, y0, x1, y1]: [i64; 4],
        color: [u8; 4],
        cover: impl Fn(Px) -> f64,
    ) {
        let (width, height) = self.image.dimensions();
        for y in y0.max(0)..y1.min(i64::from(height)) {
            for x in x0.max(0)..x1.min(i64::from(width)) {
                let coverage = cover([x as f64 + 0.5, y as f64 + 0.5]);
                if coverage > 0.0 {
                    self.blend(x, y, color, coverage);
                }
            }
        }
    }
}

fn bounds(points: &[Px], pad: f64) -> [i64; 4] {
    let (mut min, mut max) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for [x, y] in points {
        min = [min[0].min(*x), min[1].min(*y)];
        max = [max[0].max(*x), max[1].max(*y)];
    }
    [
        (min[0] - pad).floor() as i64,
        (min[1] - pad).floor() as i64,
        (max[0] + pad).ceil() as i64,
        (max[1] + pad).ceil() as i64,
    ]
}

fn distance(a: Px, b: Px) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn cross(a: Px, b: Px, p: Px) -> f64 {
    (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
}

fn distance_to_segment(p: Px, a: Px, b: Px) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let length2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if length2 > 0.0 {
        (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / length2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    distance(p, [a[0] + ab[0] * t, a[1] + ab[1] * t])
}
