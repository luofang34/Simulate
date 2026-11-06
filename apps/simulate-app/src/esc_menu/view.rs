//! The hub laid out as hit-testable widgets and passive decorations in a `1120 × 680`
//! logical card, scaled to the window so it stays crisp on Retina.
//!
//! One [`HubView`] drives drawing, mouse hit testing, and focus navigation, so what the
//! operator sees is exactly what a click or a key acts on.

use indicate_instrument_scene::{HAlign, Rgba8};
use sim_render::ViewportRect;

use super::catalog::HubCatalog;
use super::{EscMenu, HubTab};

mod controls_tab;
mod flight_tab;
mod hangar_tab;
mod palette;
mod scenario_tab;
mod status;
#[cfg(test)]
mod tests;
mod widget_id;
mod world_tab;

pub use palette::Palette;
pub use status::{ControlsStatus, MenuStatus};
pub use widget_id::{MenuCommand, TimelineScrub, WidgetIcon, WidgetId};

/// Logical height of the design frame on landscape windows.
const DESIGN_HEIGHT: f32 = 760.0;
/// Narrowest logical width; narrower windows grow the design height instead.
const MIN_DESIGN_WIDTH: f32 = 1160.0;
/// Card width and height, logical units.
pub const CARD: [f32; 2] = [1120.0, 680.0];
const HEADER_H: f32 = 58.0;
const TABS_H: f32 = 40.0;
const FOOTER_H: f32 = 28.0;
const PAD: f32 = 20.0;
/// Registry rows shown at once.
pub const VEHICLE_ROWS: usize = 8;
/// Aerodrome rows shown at once (full left-column height now that search sits in the map).
pub const AERODROME_ROWS: usize = 12;
/// Scenario rows shown at once.
pub const SCENARIO_ROWS: usize = 10;
/// Longest aerodrome search text.
pub const SEARCH_MAX: usize = 18;
/// Pen advance per character over cell height in the panel glyph pack.
pub const ADVANCE_PER_SIZE: f32 = 6.0 / 7.0;

/// How a widget is painted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WidgetStyle {
    /// A tab in the bar.
    Tab,
    /// An ordinary button.
    Button,
    /// The tab's main action.
    Primary,
    /// A call to action that needs attention (manual takeover during a scenario).
    Attention,
    /// A list row.
    Row,
    /// A small option chip.
    Chip,
    /// A chip whose action has nowhere to go now (a timeline step past the history's end).
    Muted,
    /// A clickable track drawn by its decoration (the rewind timeline bar).
    Track,
    /// A text field.
    Field,
}

/// One interactive element.
#[derive(Debug, Clone, PartialEq)]
pub struct Widget {
    /// What it is.
    pub id: WidgetId,
    /// `[x, y, w, h]`, logical units.
    pub rect: [f32; 4],
    /// Text, glyph-safe.
    pub label: String,
    /// Vector glyph drawn instead of text.
    pub icon: WidgetIcon,
    /// Whether `label` lost characters to the widget width or to the glyph pack.
    pub clipped: bool,
    /// Paint style.
    pub style: WidgetStyle,
    /// Whether it shows a chosen / active state.
    pub active: bool,
}

/// One passive drawn element.
#[derive(Debug, Clone, Copy)]
pub enum DecorKind {
    /// A filled panel with an optional edge.
    Panel {
        /// Fill colour.
        fill: Rgba8,
        /// Edge colour.
        edge: Option<Rgba8>,
    },
    /// A telemetry bar: `value` in `-1..1` from the centre, or `0..1` from the left.
    Bar {
        /// Value.
        value: f64,
        /// Whether `value` grows from the centre.
        centered: bool,
        /// Bar colour.
        ink: Rgba8,
    },
}

/// A passive panel or bar.
#[derive(Debug, Clone, Copy)]
pub struct Decor {
    /// `[x, y, w, h]`, logical units.
    pub rect: [f32; 4],
    /// What to draw.
    pub kind: DecorKind,
}

/// A passive text run.
#[derive(Debug, Clone)]
pub struct Label {
    /// Anchor point, logical units (vertically centred).
    pub at: [f32; 2],
    /// Cell height.
    pub size: f32,
    /// Horizontal alignment.
    pub align: HAlign,
    /// Colour.
    pub ink: Rgba8,
    /// Glyph-safe text.
    pub text: String,
}

/// The hub as laid out for one window, menu state, and status.
#[derive(Debug, Clone)]
pub struct HubView {
    design: [f32; 2],
    surface_px: [u32; 2],
    card: [f32; 4],
    widgets: Vec<Widget>,
    decor: Vec<Decor>,
    labels: Vec<Label>,
    preview: Option<[f32; 4]>,
}

impl HubView {
    /// Lays the hub out for a `surface_px` window.
    #[must_use]
    pub fn build(
        menu: &EscMenu,
        catalog: &HubCatalog,
        status: &MenuStatus,
        surface_px: [u32; 2],
    ) -> Self {
        let [w, h] = surface_px.map(|v| v.max(1) as f32);
        let aspect = w / h;
        let mut design = [DESIGN_HEIGHT * aspect, DESIGN_HEIGHT];
        if design[0] < MIN_DESIGN_WIDTH {
            design = [MIN_DESIGN_WIDTH, MIN_DESIGN_WIDTH / aspect];
        }
        let [cw, ch] = CARD;
        let card = [
            (design[0] - cw) * 0.5,
            ((design[1] - ch) * 0.5).max(0.0),
            cw,
            ch,
        ];
        let mut view = Self {
            design,
            surface_px,
            card,
            widgets: Vec::new(),
            decor: Vec::new(),
            labels: Vec::new(),
            preview: None,
        };
        view.frame(menu, status);
        let content = view.content();
        match menu.tab() {
            HubTab::Flight => flight_tab::build(&mut view, status, content),
            HubTab::Hangar => hangar_tab::build(&mut view, menu, catalog, status, content),
            HubTab::World => world_tab::build(&mut view, menu, catalog, content),
            HubTab::Scenarios => scenario_tab::build(&mut view, menu, catalog, status, content),
            HubTab::Controls => controls_tab::build(&mut view, status, content),
        }
        view
    }

    fn frame(&mut self, menu: &EscMenu, status: &MenuStatus) {
        let [cx, cy, cw, ch] = self.card;
        self.panel(self.card, Palette::CARD, Some(Palette::EDGE));
        let title = if status.paused {
            "SIMULATE FLIGHT HUB   PAUSED"
        } else {
            "SIMULATE FLIGHT HUB"
        };
        self.text(
            [cx + PAD, cy + 22.0],
            18.0,
            HAlign::Left,
            Palette::TITLE,
            title,
        );
        let readout = readout_line(status);
        self.text(
            [cx + cw - PAD, cy + 22.0],
            10.5,
            HAlign::Right,
            Palette::READOUT,
            &readout,
        );
        let context = format!(
            "{}   {}   {}",
            status.scenario, status.aircraft, status.airport
        );
        self.text(
            [cx + PAD, cy + 44.0],
            10.5,
            HAlign::Left,
            Palette::HINT,
            &context,
        );
        let tab_w = (cw - 2.0 * PAD) / HubTab::ALL.len() as f32;
        for (i, tab) in HubTab::ALL.into_iter().enumerate() {
            let rect = [
                cx + PAD + tab_w * i as f32,
                cy + HEADER_H,
                tab_w - 6.0,
                TABS_H - 6.0,
            ];
            self.widget(
                WidgetId::Tab(tab),
                rect,
                tab.title(),
                WidgetStyle::Tab,
                menu.tab() == tab,
            );
        }
        self.text(
            [cx + cw * 0.5, cy + ch - FOOTER_H * 0.5],
            10.0,
            HAlign::Center,
            Palette::HINT,
            "ESC RESUME   TAB/F1-F5 TABS   ARROWS MOVE   ENTER SELECT   CMD-Q QUIT",
        );
    }

    /// The content area below the tab bar, `[x, y, w, h]`.
    fn content(&self) -> [f32; 4] {
        let [cx, cy, cw, ch] = self.card;
        let top = cy + HEADER_H + TABS_H + 10.0;
        [
            cx + PAD,
            top,
            cw - 2.0 * PAD,
            cy + ch - FOOTER_H - 6.0 - top,
        ]
    }

    pub(crate) fn widget(
        &mut self,
        id: WidgetId,
        rect: [f32; 4],
        label: &str,
        style: WidgetStyle,
        active: bool,
    ) {
        self.icon_label_widget(id, rect, WidgetIcon::None, label, style, active);
    }

    /// A widget showing the vector `icon` instead of a label.
    pub(crate) fn icon_widget(
        &mut self,
        id: WidgetId,
        rect: [f32; 4],
        icon: WidgetIcon,
        active: bool,
    ) {
        self.icon_label_widget(id, rect, icon, "", WidgetStyle::Chip, active);
    }

    /// A widget whose `label` is led by the vector `icon` (`WidgetIcon::None` for none).
    pub(crate) fn icon_label_widget(
        &mut self,
        id: WidgetId,
        rect: [f32; 4],
        icon: WidgetIcon,
        label: &str,
        style: WidgetStyle,
        active: bool,
    ) {
        let shown = glyph_safe(label, fit_chars(rect[2] - 16.0, label_size(style)));
        let undrawable = label
            .chars()
            .any(|c| !c.is_whitespace() && palette::glyph_char(c) == ' ');
        self.widgets.push(Widget {
            id,
            rect,
            clipped: undrawable || shown != glyph_safe(label, usize::MAX),
            label: shown,
            icon,
            style,
            active,
        });
    }

    pub(crate) fn panel(&mut self, rect: [f32; 4], fill: Rgba8, edge: Option<Rgba8>) {
        self.decor.push(Decor {
            rect,
            kind: DecorKind::Panel { fill, edge },
        });
    }

    pub(crate) fn bar(&mut self, rect: [f32; 4], value: f64, centered: bool, ink: Rgba8) {
        let value = if value.is_finite() { value } else { 0.0 };
        self.decor.push(Decor {
            rect,
            kind: DecorKind::Bar {
                value,
                centered,
                ink,
            },
        });
    }

    pub(crate) fn text(&mut self, at: [f32; 2], size: f32, align: HAlign, ink: Rgba8, text: &str) {
        self.labels.push(Label {
            at,
            size,
            align,
            ink,
            text: glyph_safe(text, 160),
        });
    }

    /// Text clipped to `width` logical units.
    pub(crate) fn text_fit(&mut self, at: [f32; 2], size: f32, ink: Rgba8, width: f32, text: &str) {
        self.labels.push(Label {
            at,
            size,
            align: HAlign::Left,
            ink,
            text: glyph_safe(text, fit_chars(width, size)),
        });
    }

    pub(crate) fn set_preview(&mut self, rect: [f32; 4]) {
        self.preview = Some(rect);
    }

    /// Logical `[x, y, w, h]` of the live 3D preview cutout, when the tab has one.
    #[must_use]
    pub const fn preview_rect(&self) -> Option<[f32; 4]> {
        self.preview
    }

    /// Design frame `[width, height]` in logical units.
    #[must_use]
    pub const fn design(&self) -> [f32; 2] {
        self.design
    }

    /// Interactive widgets, in focus order.
    #[must_use]
    pub fn widgets(&self) -> &[Widget] {
        &self.widgets
    }

    /// Passive panels and bars, painted first.
    #[must_use]
    pub fn decor(&self) -> &[Decor] {
        &self.decor
    }

    /// Passive text.
    #[must_use]
    pub fn labels(&self) -> &[Label] {
        &self.labels
    }

    /// The widget `id`, when laid out.
    #[must_use]
    pub fn find(&self, id: WidgetId) -> Option<&Widget> {
        self.widgets.iter().find(|w| w.id == id)
    }

    /// The rectangle of widget `id`, when laid out.
    #[must_use]
    pub fn widget_rect(&self, id: WidgetId) -> Option<[f32; 4]> {
        self.find(id).map(|w| w.rect)
    }

    /// The first widget below the tab bar, else the first tab.
    #[must_use]
    pub fn first_focusable(&self) -> Option<WidgetId> {
        self.widgets
            .iter()
            .find(|w| !matches!(w.id, WidgetId::Tab(_)))
            .or_else(|| self.widgets.first())
            .map(|w| w.id)
    }

    /// The widget under `cursor_px` (physical window pixels).
    #[must_use]
    pub fn hit(&self, cursor_px: [f64; 2]) -> Option<WidgetId> {
        let [x, y] = self.to_design(cursor_px);
        self.widgets
            .iter()
            .rev()
            .find(|w| {
                let [rx, ry, rw, rh] = w.rect;
                x >= rx && x < rx + rw && y >= ry && y < ry + rh
            })
            .map(|w| w.id)
    }

    /// `cursor_px` (physical window pixels) in logical units.
    #[must_use]
    pub fn to_design(&self, cursor_px: [f64; 2]) -> [f32; 2] {
        let [sw, sh] = self.surface_px.map(|v| f64::from(v.max(1)));
        let [dw, dh] = self.design;
        [
            (cursor_px[0] / sw) as f32 * dw,
            (cursor_px[1] / sh) as f32 * dh,
        ]
    }

    /// The hangar's live 3D preview viewport as a fraction of the window.
    #[must_use]
    pub fn preview_viewport(&self) -> Option<ViewportRect> {
        let [x, y, w, h] = self.preview?;
        let [dw, dh] = self.design;
        Some(ViewportRect {
            x: x / dw,
            y: y / dh,
            width: w / dw,
            height: h / dh,
        })
    }

    /// The hangar preview viewport in physical pixels `[width, height]`.
    #[must_use]
    pub fn preview_px(&self) -> Option<[u32; 2]> {
        let rect = self.preview_viewport()?;
        let [sw, sh] = self.surface_px.map(|v| v as f32);
        Some([
            (rect.width * sw).round().max(16.0) as u32,
            (rect.height * sh).round().max(16.0) as u32,
        ])
    }
}

/// Header readout, e.g. `RT 1.00X  60 FPS  16.6 MS  HIST 60S  EPOCH 1  SENSOR 90`.
#[must_use]
pub fn readout_line(status: &MenuStatus) -> String {
    let p = &status.pacing;
    glyph_safe(
        &format!(
            "RT {:.2}X  {:.0} FPS  {:.1} MS  HIST {:.0}S  EPOCH {}  SENSOR {}",
            p.real_time_ratio,
            p.fps,
            p.mean_frame_ms,
            status.history_s,
            status.epoch,
            status.sensor_frames
        ),
        120,
    )
}

/// Label cell height of a widget painted in `style`.
#[must_use]
pub const fn label_size(style: WidgetStyle) -> f32 {
    match style {
        WidgetStyle::Tab | WidgetStyle::Chip | WidgetStyle::Muted | WidgetStyle::Track => 11.0,
        WidgetStyle::Row | WidgetStyle::Field | WidgetStyle::Button => 12.5,
        WidgetStyle::Primary | WidgetStyle::Attention => 13.5,
    }
}

/// Characters of cell height `size` that fit `width` logical units.
#[must_use]
pub fn fit_chars(width: f32, size: f32) -> usize {
    (width.max(0.0) / (size * ADVANCE_PER_SIZE)).floor() as usize
}

/// Uppercases `text`, replaces anything outside the panel glyph pack (`&` and `—` become
/// `/` and `-`, the rest spaces), trims, and truncates to `max_chars`, so the fail-closed
/// glyph check never refuses a hub frame.
#[must_use]
pub fn glyph_safe(text: &str, max_chars: usize) -> String {
    let mapped: String = text.chars().map(palette::glyph_char).collect();
    mapped.trim().chars().take(max_chars).collect()
}
