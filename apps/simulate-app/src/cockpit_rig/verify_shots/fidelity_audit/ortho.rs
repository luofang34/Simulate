//! Orthographic silhouettes of the built-in airframes, rasterised on the CPU at the
//! pixel scale and anchor of a published general-arrangement drawing, so the PNG can be
//! composited straight over that drawing to measure planform and profile errors.
//!
//! The drawings themselves are copyrighted or archival scans and stay outside the
//! repository; only their sheet size, scale and one anchor point are encoded here.

use std::path::Path;

use glam::{DVec3, Vec3};
use image::{Rgba, RgbaImage};
use sim_maplibre::{PartRole, UsdAircraftAsset};

use crate::error::AppSimError;

/// One orthographic view onto a drawing sheet.
pub(super) struct SheetView {
    /// Output file stem.
    pub name: &'static str,
    /// Drawing sheet size in pixels.
    pub sheet_px: [u32; 2],
    /// Model axis drawn horizontally (`0` = x starboard, `1` = y forward, `2` = z up)
    /// and its sign (`+1` when increasing model coordinate moves right on the sheet).
    pub h_axis: (usize, f64),
    /// Model axis drawn vertically and its sign (`+1` when increasing coordinate moves up
    /// the sheet; plan views drawn nose-down use `-1` on `y`).
    pub v_axis: (usize, f64),
    /// Pixels per metre along the sheet's horizontal and vertical axes; published
    /// sketches are not always isotropic.
    pub px_per_m: [f64; 2],
    /// Model-frame point and the sheet pixel it lands on.
    pub anchor: (DVec3, [f64; 2]),
    /// Sign of the remaining model axis that points towards the viewer.
    pub toward_viewer: f64,
}

impl SheetView {
    fn project(&self, p: Vec3) -> (f32, f32, f32) {
        let p = p.as_dvec3();
        let (a, [ax, ay]) = self.anchor;
        let (h, hs) = self.h_axis;
        let (v, vs) = self.v_axis;
        let depth_axis = 3 - h - v;
        let px = ax + hs * (p[h] - a[h]) * self.px_per_m[0];
        let py = ay - vs * (p[v] - a[v]) * self.px_per_m[1];
        let depth = self.toward_viewer * p[depth_axis];
        (px as f32, py as f32, depth as f32)
    }
}

/// Writes `<name>.png` (transparent sheet with the shaded model at ~60 % opacity and a
/// solid magenta silhouette outline) into `dir`.
pub(super) fn write_sheet_overlay(
    asset: &UsdAircraftAsset,
    view: &SheetView,
    dir: &Path,
) -> Result<(), AppSimError> {
    let [w, h] = view.sheet_px;
    let mut img = RgbaImage::from_pixel(w, h, Rgba([0, 0, 0, 0]));
    let mut zbuf = vec![f32::NEG_INFINITY; (w as usize) * (h as usize)];
    for part in asset.parts() {
        if part.role == PartRole::PropellerDisc {
            continue;
        }
        let verts: Vec<(Vec3, Vec3)> = part
            .vertices
            .iter()
            .map(|v| {
                let p = part
                    .rest_to_aircraft
                    .transform_point3(Vec3::from(v.position).as_dvec3());
                let n = part
                    .rest_to_aircraft
                    .transform_vector3(Vec3::from(v.normal).as_dvec3());
                (p.as_vec3(), n.as_vec3().normalize_or(Vec3::Z))
            })
            .collect();
        let [r, g, b] = part.material.diffuse_rgb;
        for &[i0, i1, i2] in part.indices.as_chunks::<3>().0 {
            let corner = |i: u32| {
                verts
                    .get(i as usize)
                    .copied()
                    .unwrap_or((Vec3::ZERO, Vec3::Z))
            };
            let (c0, c1, c2) = (corner(i0), corner(i1), corner(i2));
            let n = (c0.1 + c1.1 + c2.1).normalize_or(Vec3::Z);
            let shade = 0.45 + 0.55 * n.dot(Vec3::new(0.3, 0.4, 0.86)).abs();
            let col = Rgba([gamma(r * shade), gamma(g * shade), gamma(b * shade), 120]);
            let tri_px = [view.project(c0.0), view.project(c1.0), view.project(c2.0)];
            raster_tri(&mut img, &mut zbuf, tri_px, col);
        }
    }
    outline_silhouette(&mut img);
    img.save(dir.join(format!("{}.png", view.name)))
        .map_err(|e| AppSimError::Screenshot {
            detail: format!("{}: {e}", view.name),
        })
}

fn gamma(c: f32) -> u8 {
    (c.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0) as u8
}

fn raster_tri(img: &mut RgbaImage, zbuf: &mut [f32], tri: [(f32, f32, f32); 3], col: Rgba<u8>) {
    let [(x0, y0, z0), (x1, y1, z1), (x2, y2, z2)] = tri;
    let area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if area.abs() < 1e-6 {
        return;
    }
    let (w, h) = (img.width() as f32, img.height() as f32);
    let min_x = x0.min(x1).min(x2).floor().max(0.0) as u32;
    let max_x = x0.max(x1).max(x2).ceil().min(w - 1.0).max(0.0) as u32;
    let min_y = y0.min(y1).min(y2).floor().max(0.0) as u32;
    let max_y = y0.max(y1).max(y2).ceil().min(h - 1.0).max(0.0) as u32;
    for py in min_y..=max_y {
        for px in min_x..=max_x {
            let (fx, fy) = (px as f32 + 0.5, py as f32 + 0.5);
            let w0 = ((x1 - fx) * (y2 - fy) - (x2 - fx) * (y1 - fy)) / area;
            let w1 = ((x2 - fx) * (y0 - fy) - (x0 - fx) * (y2 - fy)) / area;
            let w2 = 1.0 - w0 - w1;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            let z = w0 * z0 + w1 * z1 + w2 * z2;
            let idx = (py as usize) * (img.width() as usize) + px as usize;
            if let Some(slot) = zbuf.get_mut(idx)
                && z > *slot
            {
                *slot = z;
                img.put_pixel(px, py, col);
            }
        }
    }
}

/// Paints every covered pixel that touches an uncovered one solid magenta.
fn outline_silhouette(img: &mut RgbaImage) {
    let (w, h) = img.dimensions();
    let covered = |img: &RgbaImage, x: i64, y: i64| {
        x >= 0
            && y >= 0
            && x < i64::from(w)
            && y < i64::from(h)
            && img.get_pixel(x as u32, y as u32).0[3] > 0
    };
    let mut edge = Vec::new();
    for y in 0..i64::from(h) {
        for x in 0..i64::from(w) {
            if covered(img, x, y)
                && [(1, 0), (-1, 0), (0, 1), (0, -1)]
                    .iter()
                    .any(|(dx, dy)| !covered(img, x + dx, y + dy))
            {
                edge.push((x as u32, y as u32));
            }
        }
    }
    for (x, y) in edge {
        for (ex, ey) in [(x, y), (x.saturating_add(1), y), (x, y.saturating_add(1))] {
            if ex < w && ey < h {
                img.put_pixel(ex, ey, Rgba([255, 0, 200, 255]));
            }
        }
    }
}

/// Inches to metres.
const IN: f64 = 0.0254;

/// AN 01-75F-2 p.2 general arrangement (P-38 through P-38G-15), `1967 x 2447` scan.
/// Anchors: gondola nose tip and fuselage reference line (side), ship centreline and
/// nose tip (plan), ship centreline and fuselage reference line (front). The model's
/// thrust line is `z = +0.04 m` and the FRL lies `5.078 in` below it.
pub(super) fn p38g_sheet_views() -> [SheetView; 3] {
    const SHEET: [u32; 2] = [1967, 2447];
    let frl_z = 0.04 - 5.078 * IN;
    [
        SheetView {
            name: "p38g_ortho_side_on_AN01-75F-2",
            sheet_px: SHEET,
            h_axis: (1, -1.0),
            v_axis: (2, 1.0),
            px_per_m: [2.4365 / IN; 2],
            anchor: (DVec3::new(0.0, 3.55, frl_z), [324.5, 337.0]),
            toward_viewer: -1.0,
        },
        SheetView {
            name: "p38g_ortho_plan_on_AN01-75F-2",
            sheet_px: SHEET,
            h_axis: (0, -1.0),
            v_axis: (1, -1.0),
            px_per_m: [2.439 / IN; 2],
            anchor: (DVec3::new(0.0, 3.55, 0.0), [975.0, 1810.0]),
            toward_viewer: 1.0,
        },
        SheetView {
            name: "p38g_ortho_front_on_AN01-75F-2",
            sheet_px: SHEET,
            h_axis: (0, -1.0),
            v_axis: (2, 1.0),
            px_per_m: [2.425 / IN; 2],
            anchor: (DVec3::new(0.0, 0.0, frl_z), [975.0, 2162.0]),
            toward_viewer: 1.0,
        },
    ]
}

/// Cessna 172S Information Manual Figure 1-1 at 300 dpi. The figure is a sketch whose
/// axes are scaled differently, so each axis is fitted to its own dimension callout:
/// sheet 1 plan (span 433 in across 1095 px; nose y 1308 to tail 508 for 326 in),
/// sheet 1 front (span 433 in across 1093 px; ground to beacon 107 in across 256 px),
/// sheet 2 side (length 326 in across 1101 px; height 107 in across 316 px).
/// `nose_y`, `tail_y` and `ground_z` are the model's spinner tip, rudder trailing edge
/// and tyre bottom, so the model is registered to the sketch's own extents.
pub(super) fn c172_sheet_views(nose_y: f64, ground_z: f64) -> [SheetView; 3] {
    [
        SheetView {
            name: "c172_ortho_side_on_POH_fig1-1_sheet2",
            sheet_px: [1654, 2553],
            h_axis: (1, -1.0),
            v_axis: (2, 1.0),
            px_per_m: [1101.0 / (326.0 * IN), 316.0 / (107.0 * IN)],
            anchor: (DVec3::new(0.0, nose_y, ground_z), [285.0, 807.0]),
            toward_viewer: -1.0,
        },
        SheetView {
            name: "c172_ortho_plan_on_POH_fig1-1_sheet1",
            sheet_px: [1650, 2550],
            h_axis: (0, -1.0),
            v_axis: (1, -1.0),
            px_per_m: [1095.0 / (433.0 * IN), 800.0 / (326.0 * IN)],
            anchor: (DVec3::new(0.0, nose_y, 0.0), [727.0, 1308.0]),
            toward_viewer: 1.0,
        },
        SheetView {
            name: "c172_ortho_front_on_POH_fig1-1_sheet1",
            sheet_px: [1650, 2550],
            h_axis: (0, -1.0),
            v_axis: (2, 1.0),
            px_per_m: [1093.0 / (433.0 * IN), 256.0 / (107.0 * IN)],
            anchor: (DVec3::new(0.0, 0.0, ground_z), [740.0, 1843.0]),
            toward_viewer: 1.0,
        },
    ]
}
