//! Packing islands into one square atlas at a single texel density.
//!
//! Islands go onto shelves, tallest first, each with a gutter of texels
//! around it that later takes its edge colours, so mips do not bleed one
//! island into the next. The density is the largest, in steps of one
//! percent, at which every island fits; ties keep the islands' given order,
//! so the same skin always packs the same way.

use glam::Vec2;

/// Steps by which the density shrinks until everything fits.
const SHRINK: f32 = 0.99;
/// Attempts before giving up on a hopeless layout.
const MAX_TRIES: u32 = 2_000;

/// Where one island landed: its corner in texels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Placement {
    /// Texel of the island's `uv_m` origin.
    pub(crate) origin: Vec2,
}

/// A packed atlas: the density and each island's placement, in the order
/// the sizes were given.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Packing {
    /// Texels per metre of skin.
    pub(crate) texels_per_m: f32,
    pub(crate) placements: Vec<Placement>,
}

/// Packs islands of `sizes_m` (width, height in metres) into a `side`-texel
/// square with `gutter` texels around each; `None` when even a tiny density
/// does not fit (too many islands for the gutters).
pub(crate) fn pack(sizes_m: &[Vec2], side: u32, gutter: u32) -> Option<Packing> {
    let area: f32 = sizes_m.iter().map(|s| s.x * s.y).sum();
    let side_f = side as f32;
    let longest = sizes_m.iter().fold(0.0_f32, |m, s| m.max(s.x).max(s.y));
    let mut density = (side_f * side_f / area.max(1e-6))
        .sqrt()
        .min((side_f - 2.0 * gutter as f32) / longest.max(1e-6));
    let mut order: Vec<usize> = (0..sizes_m.len()).collect();
    order.sort_by(|&a, &b| {
        let (sa, sb) = (sizes_m.get(a), sizes_m.get(b));
        let height = |s: Option<&Vec2>| s.map_or(0.0, |s| s.y);
        height(sb).total_cmp(&height(sa)).then(a.cmp(&b))
    });
    for _ in 0..MAX_TRIES {
        if let Some(placements) = shelves(sizes_m, &order, density, side_f, gutter as f32) {
            return Some(Packing {
                texels_per_m: density,
                placements,
            });
        }
        density *= SHRINK;
    }
    None
}

/// Shelf placement at `density`, or `None` if it overflows.
fn shelves(
    sizes_m: &[Vec2],
    order: &[usize],
    density: f32,
    side: f32,
    gutter: f32,
) -> Option<Vec<Placement>> {
    let mut placements = vec![Placement { origin: Vec2::ZERO }; sizes_m.len()];
    let (mut x, mut y, mut shelf) = (0.0_f32, 0.0_f32, 0.0_f32);
    for &index in order {
        let size = sizes_m.get(index).copied().unwrap_or(Vec2::ZERO) * density;
        let (w, h) = (size.x.ceil() + 2.0 * gutter, size.y.ceil() + 2.0 * gutter);
        if x + w > side {
            (x, y, shelf) = (0.0, y + shelf, 0.0);
        }
        if x + w > side || y + h > side {
            return None;
        }
        if let Some(slot) = placements.get_mut(index) {
            slot.origin = Vec2::new(x + gutter, y + gutter);
        }
        x += w;
        shelf = shelf.max(h);
    }
    Some(placements)
}
