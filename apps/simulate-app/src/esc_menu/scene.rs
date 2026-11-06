//! Paints a [`HubView`] (and the scenario takeover banner) into an Indicate scene and
//! tessellates it at the window's physical resolution.

use indicate_instrument_scene::{Anchor, HAlign, LayerId, PaintMode, SceneWriter, VAlign};
use sim_render::VectorVertex;

use super::view::{
    DecorKind, HubView, Palette, Widget, WidgetIcon, WidgetId, WidgetStyle, glyph_safe, label_size,
};
use crate::error::AppSimError;

const SCENE_BYTES: usize = 160 * 1024;
/// Rows taller than this carry a subtitle line under their title.
const SUBTITLED_ROW_MIN_H: f32 = 36.0;
/// Title centre below the top of a subtitled row.
const ROW_TITLE_DY: f32 = 15.0;
/// Space between a leading icon and its label.
const ICON_GAP: f32 = 5.0;
const BANNER_BYTES: usize = 8 * 1024;

pub(super) type SceneResult = Result<(), indicate_instrument_scene::SceneError>;

/// Interaction highlights painted over the view.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Highlights {
    /// Keyboard / gamepad focus.
    pub focus: Option<WidgetId>,
    /// Mouse hover.
    pub hovered: Option<WidgetId>,
}

/// Draws and tessellates `view` for a `surface_px` window.
pub fn tessellate_hub(
    view: &HubView,
    highlights: Highlights,
    surface_px: [u32; 2],
) -> Result<Vec<VectorVertex>, AppSimError> {
    let mut buf = vec![0_u8; SCENE_BYTES];
    let mut w = SceneWriter::new(&mut buf)?;
    w.begin_layer(LayerId::Background)?;
    w.save()?;
    paint_shapes(&mut w, view, highlights)?;
    w.restore()?;
    w.end_layer(LayerId::Background)?;
    w.begin_layer(LayerId::Tapes)?;
    w.save()?;
    paint_text(&mut w, view)?;
    w.restore()?;
    w.end_layer(LayerId::Tapes)?;
    let len = w.finish();
    tessellate(
        buf.get(..len).unwrap_or_default(),
        view.design(),
        surface_px,
    )
}

/// Draws a one-line banner across the top of a `surface_px` window.
pub fn tessellate_banner(
    text: &str,
    surface_px: [u32; 2],
) -> Result<Vec<VectorVertex>, AppSimError> {
    let [sw, sh] = surface_px.map(|v| v.max(1) as f32);
    let design = [720.0 * sw / sh, 720.0];
    let mut buf = vec![0_u8; BANNER_BYTES];
    let mut w = SceneWriter::new(&mut buf)?;
    w.begin_layer(LayerId::Background)?;
    paint_banner(&mut w, text, design[0])?;
    w.end_layer(LayerId::Background)?;
    let len = w.finish();
    tessellate(buf.get(..len).unwrap_or_default(), design, surface_px)
}

/// Paints the banner centred across a design frame `design_w` wide, `720` high.
pub(super) fn paint_banner(w: &mut SceneWriter<'_>, text: &str, design_w: f32) -> SceneResult {
    let text = glyph_safe(text, 96);
    let width = text.chars().count() as f32 * 13.0 * super::view::ADVANCE_PER_SIZE + 40.0;
    let rect = [(design_w - width) * 0.5, 58.0, width, 34.0];
    w.fill_color(Palette::ATTENTION)?;
    w.stroke(Palette::TITLE, 1.5)?;
    w.rect(PaintMode::FillStroke, rect[0], rect[1], rect[2], rect[3])?;
    w.fill_color(Palette::TITLE)?;
    w.text(design_w * 0.5, rect[1] + 17.0, 13.0, centre(), &text)
}

pub(super) fn tessellate(
    scene: &[u8],
    [dw, dh]: [f32; 2],
    [tw, th]: [u32; 2],
) -> Result<Vec<VectorVertex>, AppSimError> {
    Ok(sim_indicate::tessellate_scene_cmds(
        scene,
        dw,
        dh,
        tw.max(1),
        th.max(1),
    )?)
}

fn paint_shapes(w: &mut SceneWriter<'_>, view: &HubView, highlights: Highlights) -> SceneResult {
    let [dw, dh] = view.design();
    let hole = view.preview_rect();
    w.fill_color(Palette::BACKDROP)?;
    fill_rect_cutout(w, [0.0, 0.0, dw, dh], hole)?;
    for decor in view.decor() {
        let [x, y, rw, rh] = decor.rect;
        match decor.kind {
            DecorKind::Panel { fill, edge } => {
                w.fill_color(fill)?;
                fill_rect_cutout(w, [x, y, rw, rh], hole)?;
                if let Some(edge) = edge {
                    w.stroke(edge, 1.5)?;
                    w.rect(PaintMode::Stroke, x, y, rw, rh)?;
                }
            }
            DecorKind::Bar {
                value,
                centered,
                ink,
            } => {
                w.fill_color(Palette::TRACK)?;
                w.rect(PaintMode::Fill, x, y, rw, rh)?;
                w.fill_color(ink)?;
                let v = value as f32;
                if centered {
                    let half = rw * 0.5 * v.clamp(-1.0, 1.0);
                    let (bx, bw) = if half >= 0.0 {
                        (x + rw * 0.5, half)
                    } else {
                        (x + rw * 0.5 + half, -half)
                    };
                    w.rect(PaintMode::Fill, bx, y, bw.max(1.0), rh)?;
                } else {
                    w.rect(PaintMode::Fill, x, y, (rw * v.clamp(0.0, 1.0)).max(1.0), rh)?;
                }
            }
        }
    }
    for widget in view.widgets() {
        paint_widget(w, widget, highlights)?;
    }
    Ok(())
}

/// Fills `outer` while leaving `hole` completely transparent (`alpha = 0`) whenever `outer`
/// encloses `hole`, so the live 3D globe or hangar texture drawn behind `MenuOverlay` shines
/// through while smaller badges and widgets placed *inside* the map corners paint normally.
fn fill_rect_cutout(
    w: &mut SceneWriter<'_>,
    [ox, oy, ow, oh]: [f32; 4],
    hole: Option<[f32; 4]>,
) -> SceneResult {
    let (ox1, oy1) = (ox + ow, oy + oh);
    let Some([hx, hy, hw, hh]) = hole else {
        return w.rect(PaintMode::Fill, ox, oy, ow, oh);
    };
    let (hx1, hy1) = (hx + hw, hy + hh);
    if !(ox <= hx && oy <= hy && ox1 >= hx1 && oy1 >= hy1 && hw > 0.0 && hh > 0.0) {
        return w.rect(PaintMode::Fill, ox, oy, ow, oh);
    }
    if hy > oy {
        w.rect(PaintMode::Fill, ox, oy, ow, hy - oy)?;
    }
    if oy1 > hy1 {
        w.rect(PaintMode::Fill, ox, hy1, ow, oy1 - hy1)?;
    }
    if hx > ox {
        w.rect(PaintMode::Fill, ox, hy, hx - ox, hh)?;
    }
    if ox1 > hx1 {
        w.rect(PaintMode::Fill, hx1, hy, ox1 - hx1, hh)?;
    }
    Ok(())
}

fn paint_widget(w: &mut SceneWriter<'_>, widget: &Widget, highlights: Highlights) -> SceneResult {
    let [x, y, rw, rh] = widget.rect;
    let fill = match (widget.style, widget.active) {
        (WidgetStyle::Primary, _) => Some(Palette::PRIMARY),
        (WidgetStyle::Attention, _) => Some(Palette::ATTENTION),
        (WidgetStyle::Row, false) | (WidgetStyle::Track, _) => None,
        (WidgetStyle::Muted, _) => Some(Palette::TRACK),
        (WidgetStyle::Field, _) => Some(Palette::PANEL),
        (_, true) => Some(Palette::ACTIVE),
        (_, false) => Some(Palette::BUTTON),
    };
    if let Some(fill) = fill {
        w.fill_color(fill)?;
        w.rect(PaintMode::Fill, x, y, rw, rh)?;
    }
    if widget.style == WidgetStyle::Tab && widget.active {
        w.fill_color(Palette::FOCUS)?;
        w.rect(PaintMode::Fill, x, y + rh - 3.0, rw, 3.0)?;
    }
    if highlights.hovered == Some(widget.id) {
        w.fill_color(Palette::HOVER)?;
        w.rect(PaintMode::Fill, x, y, rw, rh)?;
    }
    let ring = if highlights.focus == Some(widget.id) {
        Some((Palette::FOCUS, 2.0))
    } else if widget.style == WidgetStyle::Field {
        Some((Palette::EDGE, 1.5))
    } else {
        None
    };
    if let Some((ink, width)) = ring {
        w.stroke(ink, width)?;
        w.rect(PaintMode::Stroke, x, y, rw, rh)?;
    }
    paint_icon(w, widget)
}

/// Strokes the widget's icon, if any.
fn paint_icon(w: &mut SceneWriter<'_>, widget: &Widget) -> SceneResult {
    let LabelPlacement {
        icon_at: [cx, cy],
        icon_r: r,
        ..
    } = label_placement(widget);
    w.stroke(Palette::TITLE, 1.8)?;
    match widget.icon {
        WidgetIcon::None => {}
        WidgetIcon::ChevronLeft => {
            let tip = cx - r * 0.5;
            w.polyline(&[[tip + r, cy - r], [tip, cy], [tip + r, cy + r]])?;
        }
        WidgetIcon::ChevronRight => {
            let tip = cx + r * 0.5;
            w.polyline(&[[tip - r, cy - r], [tip, cy], [tip - r, cy + r]])?;
        }
        WidgetIcon::Minus => w.line(cx - r, cy, cx + r, cy)?,
        WidgetIcon::Plus => {
            w.line(cx - r, cy, cx + r, cy)?;
            w.line(cx, cy - r, cx, cy + r)?;
        }
    }
    Ok(())
}

fn paint_text(w: &mut SceneWriter<'_>, view: &HubView) -> SceneResult {
    for label in view.labels() {
        if label.text.is_empty() {
            continue;
        }
        w.fill_color(label.ink)?;
        let anchor = Anchor {
            h: label.align,
            v: VAlign::Middle,
        };
        w.text(label.at[0], label.at[1], label.size, anchor, &label.text)?;
    }
    for widget in view.widgets() {
        if widget.label.is_empty() {
            continue;
        }
        let ink = match (widget.style, widget.active) {
            (WidgetStyle::Row, false) => Palette::LABEL,
            (WidgetStyle::Muted, _) => Palette::HINT,
            _ => Palette::TITLE,
        };
        w.fill_color(ink)?;
        let place = label_placement(widget);
        let anchor = Anchor {
            h: place.align,
            v: VAlign::Middle,
        };
        w.text(place.at[0], place.at[1], place.size, anchor, &widget.label)?;
    }
    Ok(())
}

/// Where a widget's label and icon are drawn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LabelPlacement {
    /// Label anchor, vertically centred.
    pub at: [f32; 2],
    /// Label cell height.
    pub size: f32,
    /// Label alignment about `at`.
    pub align: HAlign,
    /// Icon centre.
    pub icon_at: [f32; 2],
    /// Icon half-extent.
    pub icon_r: f32,
}

/// Lays out `widget`'s label and icon: list rows and fields read from the left (rows tall
/// enough for a subtitle put the title in their upper part), everything else is centred,
/// with an icon leading its label.
#[must_use]
pub fn label_placement(widget: &Widget) -> LabelPlacement {
    let [x, y, rw, rh] = widget.rect;
    let size = label_size(widget.style);
    let (cx, cy) = (x + rw * 0.5, y + rh * 0.5);
    let icon_r = rw.min(rh) * 0.2;
    if matches!(widget.style, WidgetStyle::Row | WidgetStyle::Field) {
        let row_y = if widget.style == WidgetStyle::Row && rh > SUBTITLED_ROW_MIN_H {
            y + ROW_TITLE_DY
        } else {
            cy
        };
        return LabelPlacement {
            at: [x + 10.0, row_y],
            size,
            align: HAlign::Left,
            icon_at: [cx, cy],
            icon_r,
        };
    }
    let text_w = widget.label.chars().count() as f32 * size * super::view::ADVANCE_PER_SIZE;
    if widget.icon == WidgetIcon::None || text_w == 0.0 {
        return LabelPlacement {
            at: [cx, cy],
            size,
            align: HAlign::Center,
            icon_at: [cx, cy],
            icon_r,
        };
    }
    let start = cx - (2.0 * icon_r + ICON_GAP + text_w) * 0.5;
    LabelPlacement {
        at: [start + 2.0 * icon_r + ICON_GAP, cy],
        size,
        align: HAlign::Left,
        icon_at: [start + icon_r, cy],
        icon_r,
    }
}

pub(super) const fn centre() -> Anchor {
    Anchor {
        h: HAlign::Center,
        v: VAlign::Middle,
    }
}
