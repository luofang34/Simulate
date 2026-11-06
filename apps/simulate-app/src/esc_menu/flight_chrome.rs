//! In-flight vector chrome drawn over the cockpit while the hub is closed: the scenario
//! takeover banner and the EFB's bezel, header, and buttons.
//!
//! The EFB map itself is a compositor quad under this overlay, so nothing here fills the
//! map's rectangle: the expanded tablet's bezel is four strips around it.

use indicate_instrument_scene::{Anchor, HAlign, LayerId, PaintMode, SceneWriter, VAlign};
use sim_maplibre::EfbDisplayMode;
use sim_render::VectorVertex;

use super::scene::{SceneResult, centre, paint_banner, tessellate};
use super::view::{Palette, glyph_safe};
use crate::cockpit_rig::efb_chrome::{EfbButton, EfbChrome};
use crate::error::AppSimError;

const CHROME_BYTES: usize = 32 * 1024;
const BUTTON_TEXT: f32 = 9.5;
const TITLE_TEXT: f32 = 10.5;

/// Draws the optional takeover `banner` and EFB `chrome` for a `surface_px` window.
pub fn tessellate_flight_chrome(
    banner: Option<&str>,
    chrome: Option<&EfbChrome>,
    surface_px: [u32; 2],
) -> Result<Vec<VectorVertex>, AppSimError> {
    let [sw, sh] = surface_px.map(|v| v.max(1) as f32);
    let design = chrome.map_or([720.0 * sw / sh, 720.0], |c| c.design);
    let mut buf = vec![0_u8; CHROME_BYTES];
    let mut w = SceneWriter::new(&mut buf)?;
    w.begin_layer(LayerId::Background)?;
    if let Some(chrome) = chrome {
        paint_efb(&mut w, chrome)?;
    }
    if let Some(text) = banner {
        paint_banner(&mut w, text, design[0])?;
    }
    w.end_layer(LayerId::Background)?;
    let len = w.finish();
    tessellate(buf.get(..len).unwrap_or_default(), design, surface_px)
}

fn paint_efb(w: &mut SceneWriter<'_>, chrome: &EfbChrome) -> SceneResult {
    let [mx, my, mw, mh] = chrome.map;
    let (title, [tx, ty]) = &chrome.title;
    let title = glyph_safe(title, 96);
    match chrome.mode {
        EfbDisplayMode::CompactCorner => {
            if !title.is_empty() {
                let pill_w = title.chars().count() as f32 * TITLE_TEXT * 6.0 / 7.0 + 12.0;
                w.fill_color(Palette::PANEL)?;
                w.rect(PaintMode::Fill, tx - 6.0, ty - 9.0, pill_w, 18.0)?;
            }
        }
        EfbDisplayMode::ExpandedEfb => {
            let [bx, by, bw, bh] = chrome.bezel;
            w.fill_color(Palette::CARD)?;
            w.rect(PaintMode::Fill, bx, by, bw, my - by)?;
            w.rect(PaintMode::Fill, bx, my + mh, bw, by + bh - my - mh)?;
            w.rect(PaintMode::Fill, bx, my, mx - bx, mh)?;
            w.rect(PaintMode::Fill, mx + mw, my, bx + bw - mx - mw, mh)?;
            w.stroke(Palette::EDGE, 1.5)?;
            w.rect(PaintMode::Stroke, bx, by, bw, bh)?;
        }
    }
    if mw > 0.0 && mh > 0.0 {
        w.stroke(Palette::EDGE, 1.5)?;
        w.rect(PaintMode::Stroke, mx, my, mw, mh)?;
    }
    if !title.is_empty() {
        w.fill_color(Palette::TITLE)?;
        let left = Anchor {
            h: HAlign::Left,
            v: VAlign::Middle,
        };
        w.text(*tx, *ty, TITLE_TEXT, left, &title)?;
    }
    for button in &chrome.buttons {
        paint_button(w, button)?;
    }
    Ok(())
}

fn paint_button(w: &mut SceneWriter<'_>, button: &EfbButton) -> SceneResult {
    let [x, y, bw, bh] = button.rect;
    w.fill_color(if button.active {
        Palette::ACTIVE
    } else {
        Palette::BUTTON
    })?;
    w.stroke(Palette::EDGE, 1.0)?;
    w.rect(PaintMode::FillStroke, x, y, bw, bh)?;
    let (cx, cy, r) = (x + bw * 0.5, y + bh * 0.5, (bh * 0.22).min(4.5));
    match button.label.as_str() {
        "-" => {
            w.stroke(Palette::TITLE, 1.6)?;
            w.line(cx - r, cy, cx + r, cy)
        }
        "+" => {
            w.stroke(Palette::TITLE, 1.6)?;
            w.line(cx - r, cy, cx + r, cy)?;
            w.line(cx, cy - r, cx, cy + r)
        }
        _ => {
            w.fill_color(Palette::TITLE)?;
            let label = glyph_safe(&button.label, 16);
            w.text(cx, cy, BUTTON_TEXT, centre(), &label)
        }
    }
}
