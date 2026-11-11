//! Where the EFB sits and the vector chrome around it: the compact `4:3` moving map in
//! the top-right corner (clear of the HUD and the PFD/HSI dock) with a zoom pill and three
//! corner buttons, or the expanded EFB tablet with its header and toolbar.
//!
//! The chrome is laid out once per frame in window fractions; drawing and mouse hit
//! testing read the same rectangles, so a click acts on exactly what is drawn.

use sim_maplibre::{EfbBasemap, EfbDisplayMode};
use sim_render::ViewportRect;

use crate::map_rig::{EfbCommand, EfbStatus};

/// Map width over height.
const MAP_ASPECT: f32 = 4.0 / 3.0;
/// Compact map top edge, fraction of the window height.
const COMPACT_TOP: f32 = 0.018;
/// Gap right of the compact map, fraction of the window width.
const COMPACT_RIGHT_GAP: f32 = 0.015;
/// Compact map height, fraction of the window height.
const COMPACT_HEIGHT: f32 = 0.235;
/// Widest compact map, fraction of the window width, so a portrait window keeps the
/// corner map in its corner.
const COMPACT_MAX_WIDTH: f32 = 0.28;
/// Expanded map top edge, below the header and toolbar.
const EXPANDED_TOP: f32 = 0.08;
/// Expanded map height, fraction of the window height.
const EXPANDED_HEIGHT: f32 = 0.80;
/// Widest expanded map, fraction of the window width.
const EXPANDED_MAX_WIDTH: f32 = 0.96;
/// Logical height of the chrome's design frame.
pub const CHROME_DESIGN_HEIGHT: f32 = 720.0;
/// Narrowest toolbar chip before the toolbar wraps, design units.
const MIN_CHIP_W: f32 = 80.0;
/// Zoom step of the `-` and `+` buttons and of one mouse-wheel row.
pub const ZOOM_STEP: f64 = 0.5;

/// Where the EFB map of `mode` sits in a `surface` window.
#[must_use]
pub fn efb_map_rect([sw, sh]: [u32; 2], mode: EfbDisplayMode) -> ViewportRect {
    let px_aspect = sh.max(1) as f32 / sw.max(1) as f32;
    let (height, max_width) = match mode {
        EfbDisplayMode::CompactCorner => (COMPACT_HEIGHT, COMPACT_MAX_WIDTH),
        EfbDisplayMode::ExpandedEfb => (EXPANDED_HEIGHT, EXPANDED_MAX_WIDTH),
    };
    let mut width = height * MAP_ASPECT * px_aspect;
    let mut height = height;
    if width > max_width {
        width = max_width;
        height = width / (MAP_ASPECT * px_aspect);
    }
    match mode {
        EfbDisplayMode::CompactCorner => ViewportRect {
            x: 1.0 - COMPACT_RIGHT_GAP - width,
            y: COMPACT_TOP,
            width,
            height,
        },
        EfbDisplayMode::ExpandedEfb => ViewportRect {
            x: (1.0 - width) * 0.5,
            y: EXPANDED_TOP,
            width,
            height,
        },
    }
}

/// The physical pixel size of `rect` in a `surface` window.
#[must_use]
pub fn rect_px(rect: ViewportRect, [sw, sh]: [u32; 2]) -> [u32; 2] {
    [
        (rect.width * sw as f32).round().max(1.0) as u32,
        (rect.height * sh as f32).round().max(1.0) as u32,
    ]
}

/// One clickable chrome button.
#[derive(Debug, Clone, PartialEq)]
pub struct EfbButton {
    /// What a click does.
    pub command: EfbCommand,
    /// Where it is, design units `[x, y, w, h]`.
    pub rect: [f32; 4],
    /// Glyph-safe label.
    pub label: String,
    /// Whether it shows an engaged state.
    pub active: bool,
}

/// What a click on the EFB lands on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EfbHit {
    /// A chrome button.
    Button(EfbCommand),
    /// The map itself.
    Map,
    /// The expanded tablet's bezel around the map.
    Bezel,
}

/// The EFB's vector chrome for one frame, in a `CHROME_DESIGN_HEIGHT`-high design frame
/// matching the window's aspect.
#[derive(Debug, Clone, PartialEq)]
pub struct EfbChrome {
    /// Design frame `[width, height]`.
    pub design: [f32; 2],
    /// The shown view.
    pub mode: EfbDisplayMode,
    /// The map, design units.
    pub map: [f32; 4],
    /// The expanded tablet around the map (the map itself when compact).
    pub bezel: [f32; 4],
    /// Header pill (compact) or title (expanded) text and its left-centre anchor.
    pub title: (String, [f32; 2]),
    /// Clickable buttons.
    pub buttons: Vec<EfbButton>,
}

impl EfbChrome {
    /// Lays the chrome out for `status` in a `surface` window.
    #[must_use]
    pub fn new(surface: [u32; 2], status: &EfbStatus) -> Self {
        let [sw, sh] = surface.map(|v| v.max(1) as f32);
        let design = [CHROME_DESIGN_HEIGHT * sw / sh, CHROME_DESIGN_HEIGHT];
        let rect = efb_map_rect(surface, status.mode);
        let [dw, dh] = design;
        let map = [rect.x * dw, rect.y * dh, rect.width * dw, rect.height * dh];
        match status.mode {
            EfbDisplayMode::CompactCorner => compact(design, map, status),
            EfbDisplayMode::ExpandedEfb => expanded(design, map, status),
        }
    }

    /// The map's window rectangle.
    #[must_use]
    pub fn map_viewport(&self) -> ViewportRect {
        let [dw, dh] = self.design;
        let [x, y, w, h] = self.map;
        ViewportRect {
            x: x / dw,
            y: y / dh,
            width: w / dw,
            height: h / dh,
        }
    }

    /// What `cursor_px` in a `surface` window lands on.
    #[must_use]
    pub fn hit(&self, cursor_px: [f64; 2], surface: [u32; 2]) -> Option<EfbHit> {
        let [dw, dh] = self.design;
        let [sw, sh] = surface.map(|v| f64::from(v.max(1)));
        let at = [
            (cursor_px[0] / sw) as f32 * dw,
            (cursor_px[1] / sh) as f32 * dh,
        ];
        if let Some(button) = self.buttons.iter().find(|b| inside(b.rect, at)) {
            return Some(EfbHit::Button(button.command));
        }
        if inside(self.map, at) {
            return Some(EfbHit::Map);
        }
        inside(self.bezel, at).then_some(EfbHit::Bezel)
    }
}

fn inside([x, y, w, h]: [f32; 4], [px, py]: [f32; 2]) -> bool {
    px >= x && px < x + w && py >= y && py < y + h
}

fn compact(design: [f32; 2], map: [f32; 4], status: &EfbStatus) -> EfbChrome {
    let [mx, my, mw, mh] = map;
    let fold_w = 38.0;
    let fold_rect = [mx + mw - 6.0 - fold_w, my + 6.0, fold_w, 18.0];
    if status.collapsed {
        return EfbChrome {
            design,
            mode: EfbDisplayMode::CompactCorner,
            map: [0.0; 4],
            bezel: [0.0; 4],
            title: (String::new(), [0.0, 0.0]),
            buttons: vec![EfbButton {
                command: EfbCommand::SetCollapsed(false),
                rect: fold_rect,
                label: "MAP".to_owned(),
                active: false,
            }],
        };
    }
    let button_y = my + mh - 6.0 - 18.0;
    let mut right = mx + mw - 6.0;
    let mut buttons = Vec::new();
    for (command, label, width) in [
        (
            EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb),
            "EXP",
            34.0,
        ),
        (EfbCommand::ZoomBy(ZOOM_STEP), "+", 22.0),
        (EfbCommand::ZoomBy(-ZOOM_STEP), "-", 22.0),
    ] {
        right -= width;
        buttons.push(EfbButton {
            command,
            rect: [right, button_y, width, 18.0],
            label: label.to_owned(),
            active: false,
        });
        right -= 4.0;
    }
    buttons.reverse();
    buttons.push(EfbButton {
        command: EfbCommand::SetCollapsed(true),
        rect: fold_rect,
        label: "HIDE".to_owned(),
        active: false,
    });
    EfbChrome {
        design,
        mode: EfbDisplayMode::CompactCorner,
        map,
        bezel: map,
        title: (String::new(), [mx, my]),
        buttons,
    }
}

/// The tablet toolbar's chips: command, label, and whether it reads as on.
fn tablet_chips(status: &EfbStatus) -> [(EfbCommand, String, bool); 9] {
    let basemap = if status.satellite {
        EfbBasemap::Chart
    } else {
        EfbBasemap::Satellite
    };
    [
        (EfbCommand::ZoomBy(-ZOOM_STEP), "-".to_owned(), false),
        (EfbCommand::ZoomBy(ZOOM_STEP), "+".to_owned(), false),
        (
            EfbCommand::ToggleNorthUp,
            orientation_label(status),
            status.north_up || status.custom_bearing,
        ),
        (
            EfbCommand::TogglePlate,
            on_off("PLATE", status.plate_shown),
            status.plate_shown,
        ),
        (
            EfbCommand::CycleOpacity,
            format!("OPAC {}%", status.plate_opacity_pct),
            false,
        ),
        (
            EfbCommand::ToggleDiagram,
            on_off("DIAG", status.diagram_shown),
            status.diagram_shown,
        ),
        (
            EfbCommand::SetBasemap(basemap),
            on_off("SAT", status.satellite),
            status.satellite,
        ),
        (EfbCommand::Recenter, "CENTER".to_owned(), !status.free_pan),
        (
            EfbCommand::SetMode(EfbDisplayMode::CompactCorner),
            "MINIMIZE".to_owned(),
            false,
        ),
    ]
}

fn expanded(design: [f32; 2], map: [f32; 4], status: &EfbStatus) -> EfbChrome {
    let [mx, my, mw, mh] = map;
    let toolbar_y = my - 26.0;
    let chips = tablet_chips(status);
    // A narrow (portrait) tablet wraps the toolbar into two rows, the second under the map.
    let gap = 4.0;
    let width_of = |per_row: usize| (mw - gap * (per_row as f32 - 1.0)) / per_row as f32;
    let per_row = if width_of(chips.len()) >= MIN_CHIP_W {
        chips.len()
    } else {
        chips.len().div_ceil(2)
    };
    let chip_w = width_of(per_row);
    let buttons = chips
        .into_iter()
        .enumerate()
        .map(|(i, (command, label, active))| {
            let column = (i % per_row) as f32;
            let y = if i < per_row {
                toolbar_y
            } else {
                my + mh + 5.0
            };
            EfbButton {
                command,
                rect: [mx + column * (chip_w + gap), y, chip_w, 21.0],
                label,
                active,
            }
        })
        .collect();
    let below = if per_row < 8 { 26.0 } else { 0.0 };
    let plate = status.plate_title.as_deref().unwrap_or("MOVING MAP");
    EfbChrome {
        design,
        mode: EfbDisplayMode::ExpandedEfb,
        map,
        bezel: [mx - 8.0, my - 52.0, mw + 16.0, mh + 60.0 + below],
        title: (
            format!("{}   {plate}   Z{:.1}", status.airport, status.zoom),
            [mx, my - 39.0],
        ),
        buttons,
    }
}

fn orientation_label(status: &EfbStatus) -> String {
    if status.north_up {
        "NTH UP"
    } else if status.custom_bearing {
        "BRG UP"
    } else {
        "TRK UP"
    }
    .to_owned()
}

fn on_off(name: &str, on: bool) -> String {
    format!("{name} {}", if on { "ON" } else { "OFF" })
}

#[cfg(test)]
mod tests;
