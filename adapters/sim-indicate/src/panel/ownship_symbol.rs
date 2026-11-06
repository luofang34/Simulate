//! Miniature top-down aircraft stamped at the HSI compass centre.
//!
//! The deviation bar reads as displacement relative to ownship only with an
//! ownship reference at the rose centre, so the symbol is stamped into the
//! rasterized frame, anti-aliased by distance to its strokes.

use indicate_instrument_raster::FramebufferDims;

/// Compass-rose centre of Indicate's HSI, panel pixels.
pub(super) const HSI_CENTRE_PX: [f32; 2] = [240.0, 190.0];

/// Aircraft reference yellow.
const REFERENCE_YELLOW: [u8; 4] = [255, 210, 0, 255];
/// Dark contrast outline.
const OUTLINE_DARK: [u8; 4] = [12, 16, 22, 240];
/// Outline width added around each stroke, pixels.
const OUTLINE_PX: f32 = 1.5;

/// One stroke of the silhouette: a capsule from `a` to `b` with half-width `half_px`.
struct Stroke {
    a: [f32; 2],
    b: [f32; 2],
    half_px: f32,
}

/// Nose-up silhouette relative to the rose centre: fuselage, wings, tailplane.
const STROKES: [Stroke; 3] = [
    Stroke {
        a: [0.0, -10.0],
        b: [0.0, 11.0],
        half_px: 1.7,
    },
    Stroke {
        a: [-11.0, -1.0],
        b: [11.0, -1.0],
        half_px: 1.6,
    },
    Stroke {
        a: [-5.0, 9.0],
        b: [5.0, 9.0],
        half_px: 1.4,
    },
];

/// Stamps the ownship symbol centred on `centre` into a tightly or loosely strided RGBA8 frame.
pub(super) fn stamp_ownship(pixels: &mut [u8], dims: FramebufferDims, centre: [f32; 2]) {
    let [cx, cy] = centre;
    let reach = 13.0 + OUTLINE_PX + 1.0;
    let x0 = (cx - reach).floor().max(0.0) as u32;
    let y0 = (cy - reach).floor().max(0.0) as u32;
    let x1 = ((cx + reach).ceil() as u32).min(dims.width);
    let y1 = ((cy + reach).ceil() as u32).min(dims.height);
    for y in y0..y1 {
        for x in x0..x1 {
            let p = [x as f32 + 0.5 - cx, y as f32 + 0.5 - cy];
            let (outline, fill) = coverage(p);
            if outline <= 0.0 {
                continue;
            }
            let at = (y as usize)
                .saturating_mul(dims.stride_bytes as usize)
                .saturating_add((x as usize).saturating_mul(4));
            let Some(px) = pixels.get_mut(at..at.saturating_add(4)) else {
                continue;
            };
            blend(px, OUTLINE_DARK, outline);
            blend(px, REFERENCE_YELLOW, fill);
        }
    }
}

/// Outline and fill coverage in `[0, 1]` at offset `p` from the centre.
fn coverage(p: [f32; 2]) -> (f32, f32) {
    let mut outline = 0.0_f32;
    let mut fill = 0.0_f32;
    for stroke in &STROKES {
        let d = distance_to_segment(p, stroke.a, stroke.b);
        outline = outline.max((stroke.half_px + OUTLINE_PX + 0.5 - d).clamp(0.0, 1.0));
        fill = fill.max((stroke.half_px + 0.5 - d).clamp(0.0, 1.0));
    }
    (outline, fill)
}

fn distance_to_segment(p: [f32; 2], a: [f32; 2], b: [f32; 2]) -> f32 {
    let [px, py] = p;
    let [ax, ay] = a;
    let [bx, by] = b;
    let (dx, dy) = (bx - ax, by - ay);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((px - ax) * dx + (py - ay) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (px - (ax + t * dx)).hypot(py - (ay + t * dy))
}

fn blend(dst: &mut [u8], src: [u8; 4], coverage: f32) {
    let [.., src_alpha] = src;
    let alpha = coverage * f32::from(src_alpha) / 255.0;
    if alpha <= 0.0 {
        return;
    }
    for (d, s) in dst.iter_mut().zip(src).take(3) {
        let mixed = f32::from(s) * alpha + f32::from(*d) * (1.0 - alpha);
        *d = mixed.round().clamp(0.0, 255.0) as u8;
    }
    if let Some(a) = dst.get_mut(3) {
        *a = (*a).max((alpha * 255.0).round() as u8);
    }
}
