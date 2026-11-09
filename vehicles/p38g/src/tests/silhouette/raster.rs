//! Inch-grid rasterisation, boundary extraction and the distance transform behind the
//! silhouette-overlap metrics.

use glam::Vec2;
use image::{Rgba, RgbaImage};

/// A raster over a rectangle of the view plane (`right`, `up`), in inches.
pub(super) struct Grid {
    origin: Vec2,
    res_in: f32,
    pub(super) width: usize,
    pub(super) height: usize,
}

impl Grid {
    /// Covers `min..max` with square cells `res_in` wide plus a margin of `margin_in`.
    pub(super) fn covering(min: Vec2, max: Vec2, res_in: f32, margin_in: f32) -> Self {
        let origin = min - Vec2::splat(margin_in);
        let span = max - min + Vec2::splat(2.0 * margin_in);
        Self {
            origin,
            res_in,
            width: (span.x / res_in).ceil() as usize + 1,
            height: (span.y / res_in).ceil() as usize + 1,
        }
    }

    /// Continuous pixel coordinates of view point `p` (row 0 at the top).
    fn to_px(&self, p: Vec2) -> Vec2 {
        Vec2::new(
            (p.x - self.origin.x) / self.res_in,
            self.height as f32 - (p.y - self.origin.y) / self.res_in,
        )
    }

    fn cells(&self) -> usize {
        self.width * self.height
    }
}

/// Fills the even-odd interior of `rings` (closed polylines in view inches).
pub(super) fn fill_rings(grid: &Grid, rings: &[Vec<Vec2>]) -> Vec<bool> {
    let mut mask = vec![false; grid.cells()];
    let edges: Vec<(Vec2, Vec2)> = rings
        .iter()
        .flat_map(|ring| {
            let px: Vec<Vec2> = ring.iter().map(|&p| grid.to_px(p)).collect();
            let n = px.len();
            (0..n).map(move |i| (px[i], px[(i + 1) % n]))
        })
        .collect();
    let mut crossings = Vec::new();
    for row in 0..grid.height {
        let yc = row as f32 + 0.5;
        crossings.clear();
        for &(a, b) in &edges {
            if (a.y <= yc) != (b.y <= yc) {
                crossings.push(a.x + (yc - a.y) / (b.y - a.y) * (b.x - a.x));
            }
        }
        crossings.sort_by(f32::total_cmp);
        for pair in crossings.as_chunks::<2>().0 {
            let x0 = (pair[0] - 0.5).ceil().max(0.0) as usize;
            let x1 = (pair[1] - 0.5).floor();
            if x1 < 0.0 {
                continue;
            }
            for col in x0..=(x1 as usize).min(grid.width - 1) {
                mask[row * grid.width + col] = true;
            }
        }
    }
    mask
}

/// Marks every cell a triangle covers, plus the cells along its edges so triangles seen
/// edge-on still leave a one-cell trace.
pub(super) fn fill_triangles(grid: &Grid, tris: &[[Vec2; 3]]) -> Vec<bool> {
    let mut mask = vec![false; grid.cells()];
    for tri in tris {
        let [a, b, c] = tri.map(|p| grid.to_px(p));
        fill_one_triangle(grid, &mut mask, a, b, c);
        for (p, q) in [(a, b), (b, c), (c, a)] {
            mark_segment(grid, &mut mask, p, q);
        }
    }
    mask
}

fn fill_one_triangle(grid: &Grid, mask: &mut [bool], a: Vec2, b: Vec2, c: Vec2) {
    let area = (b - a).perp_dot(c - a);
    if area.abs() < 1e-9 {
        return;
    }
    let lo = a.min(b).min(c).floor().max(Vec2::ZERO);
    let hi = a.max(b).max(c).ceil();
    let x1 = (hi.x as usize).min(grid.width - 1);
    let y1 = (hi.y as usize).min(grid.height - 1);
    for row in lo.y as usize..=y1 {
        for col in lo.x as usize..=x1 {
            let p = Vec2::new(col as f32 + 0.5, row as f32 + 0.5);
            let w0 = (b - p).perp_dot(c - p) / area;
            let w1 = (c - p).perp_dot(a - p) / area;
            if w0 >= 0.0 && w1 >= 0.0 && w0 + w1 <= 1.0 {
                mask[row * grid.width + col] = true;
            }
        }
    }
}

fn mark_segment(grid: &Grid, mask: &mut [bool], p: Vec2, q: Vec2) {
    let steps = ((q - p).length() * 2.0).ceil().max(1.0) as usize;
    for i in 0..=steps {
        let s = p.lerp(q, i as f32 / steps as f32);
        if s.x >= 0.0 && s.y >= 0.0 && (s.x as usize) < grid.width && (s.y as usize) < grid.height {
            mask[s.y as usize * grid.width + s.x as usize] = true;
        }
    }
}

/// Cells of `mask` with a 4-neighbour outside the mask (or on the grid edge).
fn boundary(grid: &Grid, mask: &[bool]) -> Vec<bool> {
    let (w, h) = (grid.width, grid.height);
    let mut out = vec![false; mask.len()];
    for row in 0..h {
        for col in 0..w {
            let i = row * w + col;
            if mask[i]
                && (col == 0
                    || row == 0
                    || col + 1 == w
                    || row + 1 == h
                    || !mask[i - 1]
                    || !mask[i + 1]
                    || !mask[i - w]
                    || !mask[i + w])
            {
                out[i] = true;
            }
        }
    }
    out
}

/// Exact squared Euclidean distance (in cells²) from every cell to the nearest seed
/// (Felzenszwalb and Huttenlocher's separable lower-envelope transform).
fn distance_transform(grid: &Grid, seeds: &[bool]) -> Vec<f64> {
    const FAR: f64 = 1e20;
    let (w, h) = (grid.width, grid.height);
    let mut d: Vec<f64> = seeds.iter().map(|&s| if s { 0.0 } else { FAR }).collect();
    for col in 0..w {
        let column: Vec<f64> = (0..h).map(|row| d[row * w + col]).collect();
        for (row, v) in lower_envelope(&column).into_iter().enumerate() {
            d[row * w + col] = v;
        }
    }
    for row in 0..h {
        let line = lower_envelope(&d[row * w..(row + 1) * w]);
        d[row * w..(row + 1) * w].copy_from_slice(&line);
    }
    d
}

fn lower_envelope(f: &[f64]) -> Vec<f64> {
    let n = f.len();
    let (mut v, mut z) = (vec![0_usize; n], vec![0.0_f64; n + 1]);
    let mut k = 0;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    let cut = |q: usize, p: usize| {
        let (qf, pf) = (q as f64, p as f64);
        ((f[q] + qf * qf) - (f[p] + pf * pf)) / (2.0 * (qf - pf))
    };
    for q in 1..n {
        let mut s = cut(q, v[k]);
        while s <= z[k] {
            k -= 1;
            s = cut(q, v[k]);
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        z[k + 1] = f64::INFINITY;
    }
    k = 0;
    (0..n)
        .map(|q| {
            while z[k + 1] < q as f64 {
                k += 1;
            }
            let dq = q as f64 - v[k] as f64;
            dq * dq + f[v[k]]
        })
        .collect()
}

/// Overlap between a reference silhouette and a model silhouette.
#[derive(Debug, Clone, Copy)]
pub(super) struct Metrics {
    /// Intersection over union of the two areas.
    pub(super) iou: f32,
    /// Largest distance from a boundary cell of either silhouette to the other boundary.
    pub(super) max_dev_in: f32,
    /// 95th percentile of the same boundary-to-boundary distances.
    pub(super) p95_dev_in: f32,
}

/// Compares two masks on the same grid.
pub(super) fn compare(grid: &Grid, reference: &[bool], model: &[bool]) -> Metrics {
    let (mut inter, mut union) = (0_usize, 0_usize);
    for (&r, &m) in reference.iter().zip(model) {
        inter += usize::from(r && m);
        union += usize::from(r || m);
    }
    let (b_ref, b_mod) = (boundary(grid, reference), boundary(grid, model));
    let (d_ref, d_mod) = (
        distance_transform(grid, &b_ref),
        distance_transform(grid, &b_mod),
    );
    let mut devs: Vec<f32> = b_ref
        .iter()
        .zip(&d_mod)
        .filter(|(b, _)| **b)
        .chain(b_mod.iter().zip(&d_ref).filter(|(b, _)| **b))
        .map(|(_, d)| d.sqrt() as f32 * grid.res_in)
        .collect();
    devs.sort_by(f32::total_cmp);
    let pick = |q: f32| {
        devs.get(((devs.len() as f32 - 1.0) * q).round() as usize)
            .copied()
            .unwrap_or(f32::INFINITY)
    };
    Metrics {
        iou: inter as f32 / union.max(1) as f32,
        max_dev_in: pick(1.0),
        p95_dev_in: pick(0.95),
    }
}

/// Reference-only cells red, model-only blue, shared grey, reference outline black.
pub(super) fn diff_image(grid: &Grid, reference: &[bool], model: &[bool]) -> RgbaImage {
    let edge = boundary(grid, reference);
    let mut img = RgbaImage::from_pixel(
        grid.width as u32,
        grid.height as u32,
        Rgba([255, 255, 255, 255]),
    );
    for (i, px) in img.pixels_mut().enumerate() {
        *px = match (reference[i], model[i], edge[i]) {
            (_, _, true) => Rgba([0, 0, 0, 255]),
            (true, true, _) => Rgba([170, 170, 170, 255]),
            (true, false, _) => Rgba([230, 40, 40, 255]),
            (false, true, _) => Rgba([40, 90, 230, 255]),
            _ => continue,
        };
    }
    img
}
