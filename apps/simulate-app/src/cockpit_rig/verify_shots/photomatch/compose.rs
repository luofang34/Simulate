//! Side-by-side, overlay and difference images of a photograph and the model rendered
//! from its resected camera.

use std::path::Path;

use image::{Rgb, RgbImage, imageops};

use super::Match;
use super::render::{out_path, shot_error};
use super::resection::PinholePose;
use super::tone::{self, luma};
use crate::error::AppSimError;

/// A channel difference above this between the frames with and without the airframe
/// marks a pixel as airframe (or its shadow).
const SILHOUETTE_THRESHOLD: u8 = 6;
/// Radius, pixels, of the closing that fills the silhouette.
const CLOSE_RADIUS: usize = 3;
const MEASURED: Rgb<u8> = Rgb([40, 220, 60]);
const REPROJECTED: Rgb<u8> = Rgb([235, 40, 40]);
const OUTLINE: Rgb<u8> = Rgb([255, 210, 0]);

/// How the photo's frame sits inside the render: the render's vertical field of view and
/// the render rectangle the photo, scaled by `scale`, covers.
pub(super) struct Framing {
    pub(super) render_fov_y_rad: f64,
    scale: f64,
    origin_px: [u32; 2],
    size_px: [u32; 2],
}

impl Framing {
    /// Fits a `photo_px` frame with `pose`'s centred focal length inside a `render_px`
    /// render.
    pub(super) fn new(pose: &PinholePose, [w, h]: [f64; 2], [rw, rh]: [u32; 2]) -> Self {
        let (rw, rh) = (f64::from(rw), f64::from(rh));
        let scale = (rw / w).min(rh / h);
        let render_focal = pose.focal_px * scale;
        let size = [(w * scale).floor(), (h * scale).floor()];
        let px = |v: f64| v.max(0.0) as u32;
        Self {
            render_fov_y_rad: 2.0 * (0.5 * rh / render_focal).atan(),
            scale,
            origin_px: [px(0.5 * (rw - size[0])), px(0.5 * (rh - size[1]))],
            size_px: [px(size[0]), px(size[1])],
        }
    }
}

/// Writes `<id>_side.png`, `<id>_overlay.png`, `<id>_diff.png` and the unmarked
/// `<id>_model.png` for `m` into `dir`, and returns the airframe's tone in photo and model
/// (see [`tone::compare`]).
pub(super) fn write_comparisons(
    dir: &Path,
    m: &Match,
    framing: &Framing,
    photo: &RgbImage,
    [shown, hidden]: [&[u8]; 2],
) -> Result<serde_json::Value, AppSimError> {
    let [w, h] = framing.size_px;
    let photo = imageops::resize(photo, w, h, imageops::FilterType::Triangle);
    let model = crop(shown, framing);
    let empty = crop(hidden, framing);
    let raw: Vec<bool> = model
        .pixels()
        .zip(empty.pixels())
        .map(|(a, b)| {
            a.0.iter()
                .zip(b.0)
                .any(|(x, y)| x.abs_diff(y) > SILHOUETTE_THRESHOLD)
        })
        .collect();
    // Closing fills the specks where the airframe happens to match what is behind it.
    let mask = morph(&morph(&raw, w, true), w, false);
    let marks = landmark_marks(m, framing.scale);
    save(&model, &out_path(dir, &m.photo, "model"))?;
    let tone = tone::compare(&photo, [&model, &empty], &mask, w);

    let mut side = RgbImage::new(w.saturating_mul(2), h);
    imageops::replace(&mut side, &photo, 0, 0);
    imageops::replace(&mut side, &model, i64::from(w), 0);
    for (measured, reprojected) in &marks {
        ring(&mut side, *measured, MEASURED);
        cross(&mut side, *reprojected, REPROJECTED);
        cross(
            &mut side,
            [reprojected[0] + f64::from(w), reprojected[1]],
            REPROJECTED,
        );
    }
    save(&side, &out_path(dir, &m.photo, "side"))?;

    let mut overlay = photo.clone();
    for (i, (px, model_px)) in overlay.pixels_mut().zip(model.pixels()).enumerate() {
        if mask.get(i).copied().unwrap_or(false) {
            *px = blend(*px, *model_px, 0.45);
        }
    }
    outline(&mut overlay, &mask, w);
    for (measured, reprojected) in &marks {
        ring(&mut overlay, *measured, MEASURED);
        cross(&mut overlay, *reprojected, REPROJECTED);
    }
    save(&overlay, &out_path(dir, &m.photo, "overlay"))?;

    let mut diff = RgbImage::new(w, h);
    for (i, ((d, p), r)) in diff
        .pixels_mut()
        .zip(photo.pixels())
        .zip(model.pixels())
        .enumerate()
    {
        let delta = luma(*p).abs_diff(luma(*r));
        *d = if mask.get(i).copied().unwrap_or(false) {
            Rgb([delta, delta, delta])
        } else {
            Rgb([luma(*p) / 6; 3])
        };
    }
    outline(&mut diff, &mask, w);
    save(&diff, &out_path(dir, &m.photo, "diff"))?;
    Ok(tone)
}

/// `(measured, reprojected)` pixel of every landmark, scaled into the comparison frame.
fn landmark_marks(m: &Match, scale: f64) -> Vec<([f64; 2], [f64; 2])> {
    m.used
        .iter()
        .filter_map(|(_, point, [u, v])| {
            let [pu, pv] = m.solved.pose.project(*point)?;
            Some(([u * scale, v * scale], [pu * scale, pv * scale]))
        })
        .collect()
}

fn crop(rgba: &[u8], framing: &Framing) -> RgbImage {
    let [x0, y0] = framing.origin_px;
    let [w, h] = framing.size_px;
    let stride = super::super::SHOT_SIZE[0] as usize;
    RgbImage::from_fn(w, h, |x, y| {
        let at = ((y + y0) as usize * stride + (x + x0) as usize) * 4;
        let px = |k: usize| rgba.get(at + k).copied().unwrap_or(0);
        Rgb([px(0), px(1), px(2)])
    })
}

/// Dilates (`grow`) or erodes `mask` (row-major, `w` wide) by [`CLOSE_RADIUS`].
fn morph(mask: &[bool], w: u32, grow: bool) -> Vec<bool> {
    let w = w as usize;
    let h = mask.len() / w.max(1);
    let get = |x: usize, y: usize| mask.get(y * w + x).copied().unwrap_or(false);
    (0..mask.len())
        .map(|i| {
            let (x, y) = (i % w, i / w);
            let xs = x.saturating_sub(CLOSE_RADIUS)..=(x + CLOSE_RADIUS).min(w - 1);
            let ys = y.saturating_sub(CLOSE_RADIUS)..=(y + CLOSE_RADIUS).min(h.saturating_sub(1));
            let mut window = ys.flat_map(|yy| xs.clone().map(move |xx| (xx, yy)));
            if grow {
                window.any(|(xx, yy)| get(xx, yy))
            } else {
                window.all(|(xx, yy)| get(xx, yy))
            }
        })
        .collect()
}

fn blend(a: Rgb<u8>, b: Rgb<u8>, t: f64) -> Rgb<u8> {
    let mix = |x: u8, y: u8| (f64::from(x) * (1.0 - t) + f64::from(y) * t).round() as u8;
    Rgb([mix(a[0], b[0]), mix(a[1], b[1]), mix(a[2], b[2])])
}

/// Paints the edge of `mask` (row-major, `w` wide).
fn outline(img: &mut RgbImage, mask: &[bool], w: u32) {
    let w = w as usize;
    let at = |i: Option<usize>| i.and_then(|i| mask.get(i).copied()).unwrap_or(false);
    for i in 0..mask.len() {
        let inside = at(Some(i));
        let edge = inside
            && !(at(i.checked_sub(1))
                && at(i.checked_add(1))
                && at(i.checked_sub(w))
                && at(i.checked_add(w)));
        if edge {
            img.put_pixel((i % w) as u32, (i / w) as u32, OUTLINE);
        }
    }
}

fn plot(img: &mut RgbImage, x: f64, y: f64, c: Rgb<u8>) {
    if x >= 0.0 && y >= 0.0 && x < f64::from(img.width()) && y < f64::from(img.height()) {
        img.put_pixel(x as u32, y as u32, c);
    }
}

fn ring(img: &mut RgbImage, [x, y]: [f64; 2], c: Rgb<u8>) {
    for k in 0..48 {
        let a = f64::from(k) * std::f64::consts::TAU / 48.0;
        for r in [6.0, 7.0] {
            plot(img, x + r * a.cos(), y + r * a.sin(), c);
        }
    }
}

fn cross(img: &mut RgbImage, [x, y]: [f64; 2], c: Rgb<u8>) {
    for d in -6..=6 {
        let d = f64::from(d);
        plot(img, x + d, y + d, c);
        plot(img, x + d, y - d, c);
    }
}

fn save(img: &RgbImage, path: &Path) -> Result<(), AppSimError> {
    img.save(path).map_err(|e| shot_error(&e))
}
