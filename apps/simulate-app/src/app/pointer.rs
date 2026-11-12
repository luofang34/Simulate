//! Mouse routing: with the hub open, its widgets and its live 3D preview; with the hub
//! closed, the cockpit's EFB.

use winit::event::MouseScrollDelta;
use winit::event_loop::ActiveEventLoop;

use super::FlightApp;
use crate::esc_menu::{HubTab, MenuInput};

/// Logical pixels per mouse-wheel line.
const WHEEL_PIXELS_PER_ROW: f64 = 40.0;

/// What a held left button is dragging, with the cursor's last physical position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PointerDrag {
    /// The hangar preview's orbit camera.
    Hangar {
        /// Cursor at the last move.
        last: [f64; 2],
    },
    /// The World tab's airport globe; a release near `origin` is a click.
    Globe {
        /// Cursor at the last move.
        last: [f64; 2],
        /// Cursor at the press.
        origin: [f64; 2],
    },
    /// The expanded EFB's map; a release near `origin` picks a fix or inserts a waypoint.
    Efb {
        /// Cursor at the last move.
        last: [f64; 2],
        /// Cursor at the press.
        origin: [f64; 2],
    },
}

impl FlightApp {
    pub(super) fn mouse_moved(&mut self, position: [f64; 2]) {
        match self.hub.drag {
            Some(PointerDrag::Hangar { last }) => {
                if let Some(hangar) = self.hangar.as_mut() {
                    hangar.drag([position[0] - last[0], position[1] - last[1]]);
                }
                self.hub.drag = Some(PointerDrag::Hangar { last: position });
            }
            Some(PointerDrag::Globe { last, origin }) => {
                self.drag_globe(last, position);
                self.hub.drag = Some(PointerDrag::Globe {
                    last: position,
                    origin,
                });
            }
            Some(PointerDrag::Efb { last, origin }) => {
                self.pan_efb(last, position);
                self.hub.drag = Some(PointerDrag::Efb {
                    last: position,
                    origin,
                });
            }
            None => {}
        }
        self.cursor_px = position;
        if self.esc_menu.is_open() {
            let view = self.hub_view();
            self.esc_menu.hover_at(position, &view);
        }
    }

    pub(super) fn mouse_pressed(&mut self, event_loop: &ActiveEventLoop) {
        if !self.esc_menu.is_open() {
            if !self.efb_pressed() && self.efb_workspace.ad_hoc.pick_mode.is_world_3d() {
                self.world_3d_pick_pressed();
            }
            return;
        }
        let view = self.hub_view();
        if view.hit(self.cursor_px).is_some() {
            let outcome = self.esc_menu.click_at(self.cursor_px, &view, &self.catalog);
            self.apply_menu_outcome(event_loop, outcome);
            return;
        }
        if self.cursor_in_preview() {
            let at = self.cursor_px;
            self.hub.drag = Some(if self.esc_menu.tab() == HubTab::World {
                PointerDrag::Globe {
                    last: at,
                    origin: at,
                }
            } else {
                PointerDrag::Hangar { last: at }
            });
        }
    }

    pub(super) fn mouse_released(&mut self) {
        match self.hub.drag.take() {
            Some(PointerDrag::Globe { origin, .. }) if self.esc_menu.is_open() => {
                self.release_globe(origin, self.cursor_px);
            }
            Some(PointerDrag::Efb { origin, .. }) if !self.esc_menu.is_open() => {
                self.release_efb_map(origin, self.cursor_px);
            }
            _ => {}
        }
    }

    pub(super) fn mouse_wheel(&mut self, event_loop: &ActiveEventLoop, delta: MouseScrollDelta) {
        let rows = match delta {
            MouseScrollDelta::LineDelta(_, y) => -f64::from(y),
            MouseScrollDelta::PixelDelta(p) => -p.y / WHEEL_PIXELS_PER_ROW,
        };
        if !self.esc_menu.is_open() {
            let _taken = self.efb_wheel(rows);
            return;
        }
        if self.cursor_in_preview() {
            if self.esc_menu.tab() == HubTab::World {
                self.zoom_globe(rows);
            } else if let Some(hangar) = self.hangar.as_mut() {
                hangar.zoom(rows);
            }
            return;
        }
        let whole = rows.round() as i32;
        if whole != 0 {
            self.handle_menu_input(event_loop, MenuInput::Scroll(whole));
        }
    }

    /// Two-finger trackpad pinch (`delta > 0` opens fingers to zoom in, `delta < 0` closes
    /// fingers to zoom out).
    pub(super) fn pinch_gesture(&mut self, delta: f64) {
        if !delta.is_finite() || delta == 0.0 {
            return;
        }
        if !self.esc_menu.is_open() {
            let _taken = self.efb_pinch(delta);
            return;
        }
        if self.cursor_in_preview()
            && let Some(zoom_delta) = super::efb_input::pinch_zoom_delta(delta)
        {
            let rows = -zoom_delta / crate::cockpit_rig::efb_chrome::ZOOM_STEP;
            if self.esc_menu.tab() == HubTab::World {
                self.zoom_globe(rows);
            } else if let Some(hangar) = self.hangar.as_mut() {
                hangar.zoom(rows);
            }
        }
    }

    /// Two-finger trackpad twist (`delta` in degrees, `> 0` counter-clockwise).
    pub(super) fn rotation_gesture(&mut self, delta: f32) {
        let delta_deg = f64::from(delta);
        if !delta_deg.is_finite() || delta_deg == 0.0 {
            return;
        }
        if !self.esc_menu.is_open() {
            let _taken = self.efb_rotate(delta_deg);
        }
    }

    pub(super) fn cursor_in_preview(&self) -> bool {
        let [w, h] = self.surface_size().map(|v| f64::from(v.max(1)));
        self.hub_preview.is_some_and(|r| {
            let (x, y) = (self.cursor_px[0] / w, self.cursor_px[1] / h);
            let (rx, ry) = (f64::from(r.x), f64::from(r.y));
            x >= rx && x < rx + f64::from(r.width) && y >= ry && y < ry + f64::from(r.height)
        })
    }
}
