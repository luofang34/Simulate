//! Panel breaks as line sets in drawing units, with a grid for finding the
//! nearest break to a point of the skin quickly.

use std::collections::BTreeMap;

use glam::Vec2;

/// The nearest break to a point.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Hit {
    /// Distance to the break, inches.
    pub(crate) distance_in: f32,
    /// Distance along the break's polyline from its start, inches; rivets
    /// are spaced on it.
    pub(crate) along_in: f32,
    /// Unit direction in the table's axes (scaled to inches) from the break
    /// toward the point.
    pub(crate) away: Vec2,
    /// Which polyline of the table the break is.
    pub(crate) line: u32,
}

#[derive(Clone, Copy, Debug)]
struct Segment {
    line: u32,
    a: Vec2,
    b: Vec2,
    /// Inches of the polyline before `a`, in unscaled table units.
    start: f32,
}

/// The breaks of one table, bucketed by grid cell.
#[derive(Clone, Debug)]
pub(crate) struct LineSet {
    segments: Vec<Segment>,
    cell: Vec2,
    grid: BTreeMap<(i32, i32), Vec<u32>>,
}

impl LineSet {
    /// The breaks of `table`, bucketed in cells of `cell` table units (each
    /// at least the reach of a break's seam and rivet rows).
    pub(crate) fn new(table: &[&[[f32; 2]]], cell: Vec2) -> Self {
        let mut segments = Vec::new();
        for (index, line) in table.iter().enumerate() {
            let id = u32::try_from(index).unwrap_or(u32::MAX);
            let mut start = 0.0;
            for pair in line.windows(2) {
                let (Some(&a), Some(&b)) = (pair.first(), pair.get(1)) else {
                    continue;
                };
                let (a, b) = (Vec2::from(a), Vec2::from(b));
                segments.push(Segment {
                    line: id,
                    a,
                    b,
                    start,
                });
                start += a.distance(b);
            }
        }
        let mut grid: BTreeMap<(i32, i32), Vec<u32>> = BTreeMap::new();
        for (index, s) in segments.iter().enumerate() {
            let lo = (s.a.min(s.b) / cell).floor();
            let hi = (s.a.max(s.b) / cell).floor();
            for x in lo.x as i32..=hi.x as i32 {
                for y in lo.y as i32..=hi.y as i32 {
                    grid.entry((x, y))
                        .or_default()
                        .push(u32::try_from(index).unwrap_or(0));
                }
            }
        }
        Self {
            segments,
            cell,
            grid,
        }
    }

    /// The nearest break to `p` within one cell, measuring the table's
    /// second axis `v_scale` times larger (inches per unit) than its first.
    pub(crate) fn nearest(&self, p: Vec2, v_scale: f32) -> Option<Hit> {
        let metric = Vec2::new(1.0, v_scale);
        let c = (p / self.cell).floor();
        let mut best: Option<Hit> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                let key = (c.x as i32 + dx, c.y as i32 + dy);
                for &index in self.grid.get(&key).into_iter().flatten() {
                    let Some(s) = self.segments.get(index as usize) else {
                        continue;
                    };
                    let (a, b, q) = (s.a * metric, s.b * metric, p * metric);
                    let ab = b - a;
                    let t = ((q - a).dot(ab) / ab.length_squared().max(1e-9)).clamp(0.0, 1.0);
                    let closest = a + ab * t;
                    let distance = q.distance(closest);
                    if best.is_none_or(|h| distance < h.distance_in) {
                        best = Some(Hit {
                            distance_in: distance,
                            along_in: s.start + t * s.a.distance(s.b),
                            line: s.line,
                            away: (q - closest)
                                .try_normalize()
                                .unwrap_or(ab.perp().normalize_or_zero()),
                        });
                    }
                }
            }
        }
        best
    }
}
