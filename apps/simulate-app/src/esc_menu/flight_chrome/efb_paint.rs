//! Vector painting of [`EfbChrome`] (compact corner map, expanded EFB bezel, and the
//! `Pilotage`-inspired Mission Planner & Ad-Hoc Runway dock).

use indicate_instrument_scene::{Anchor, HAlign, PaintMode, SceneWriter, VAlign};
use sim_maplibre::EfbDisplayMode;

use crate::cockpit_rig::efb_chrome::{EfbButton, EfbButtonTone, EfbChrome};
use crate::esc_menu::scene::{SceneResult, centre};
use crate::esc_menu::view::{ADVANCE_PER_SIZE, Palette, glyph_safe};

const BUTTON_TEXT: f32 = 9.0;
const TITLE_TEXT: f32 = 10.5;

pub(super) fn paint_efb(w: &mut SceneWriter<'_>, chrome: &EfbChrome) -> SceneResult {
    let [mx, my, mw, mh] = chrome.map;
    let (title, [tx, ty]) = &chrome.title;
    let title = glyph_safe(title, 96);
    match chrome.mode {
        EfbDisplayMode::CompactCorner => {
            if !title.is_empty() {
                let pill_w = title.chars().count() as f32 * TITLE_TEXT * ADVANCE_PER_SIZE + 12.0;
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
    let left = Anchor {
        h: HAlign::Left,
        v: VAlign::Middle,
    };
    if !title.is_empty() {
        w.fill_color(Palette::TITLE)?;
        w.text(*tx, *ty, TITLE_TEXT, left, &title)?;
    }
    for panel in &chrome.panels {
        let [px, py, pw, ph] = panel.rect;
        w.fill_color(panel.fill)?;
        if let Some(edge) = panel.edge {
            w.stroke(edge, 1.2)?;
            w.rect(PaintMode::FillStroke, px, py, pw, ph)?;
        } else {
            w.rect(PaintMode::Fill, px, py, pw, ph)?;
        }
    }
    for label in &chrome.labels {
        let [lx, ly] = label.at;
        let text = glyph_safe(&label.text, label.max_chars);
        if !text.is_empty() {
            w.fill_color(label.color)?;
            w.text(lx, ly, label.size, left, &text)?;
        }
    }
    for button in &chrome.buttons {
        paint_button(w, button)?;
    }
    Ok(())
}

fn paint_button(w: &mut SceneWriter<'_>, button: &EfbButton) -> SceneResult {
    let [x, y, bw, bh] = button.rect;
    let fill = match (button.tone, button.active) {
        (EfbButtonTone::Field, _) => Palette::STUDIO,
        (_, true) => Palette::ACTIVE,
        (EfbButtonTone::Emerald, false) => Palette::PRIMARY,
        (EfbButtonTone::Amber, false) => Palette::ATTENTION,
        (EfbButtonTone::Standard, false) => Palette::BUTTON,
    };
    let edge = if button.active {
        Palette::FOCUS
    } else {
        Palette::EDGE
    };
    w.fill_color(fill)?;
    w.stroke(edge, if button.active { 1.4 } else { 1.0 })?;
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
            let font_sz = if bw > 140.0 { 8.4 } else { BUTTON_TEXT };
            let max_chars = ((bw - 6.0).max(8.0) / (font_sz * ADVANCE_PER_SIZE)).floor() as usize;
            let label = glyph_safe(&button.label, max_chars.max(2));
            w.fill_color(Palette::TITLE)?;
            if button.tone == EfbButtonTone::Field {
                let left = Anchor {
                    h: HAlign::Left,
                    v: VAlign::Middle,
                };
                w.text(x + 7.0, cy, font_sz, left, &label)
            } else {
                w.text(cx, cy, font_sz, centre(), &label)
            }
        }
    }
}
