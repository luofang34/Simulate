//! The cockpit EFB under the pointer and the keyboard while the hub is closed: chrome
//! buttons, click-to-expand on the corner map, drag to pan the tablet, wheel to zoom, and
//! `Esc` to minimize the tablet before it opens the hub.

use sim_maplibre::EfbDisplayMode;
use tracing::warn;
use winit::event::{ElementState, KeyEvent};
use winit::keyboard::{KeyCode, PhysicalKey};

use super::FlightApp;
use super::pointer::PointerDrag;
use crate::cockpit_rig::efb_chrome::{EfbChrome, EfbHit, ZOOM_STEP};
use crate::map_rig::EfbCommand;

impl FlightApp {
    /// The EFB's chrome while the hub is closed and the EFB is shown, unless a page's map
    /// takes the EFB's commands.
    pub(super) fn efb_chrome(&self) -> Option<EfbChrome> {
        if self.esc_menu.is_open() {
            return None;
        }
        let rig = self
            .cockpit_rig
            .as_ref()
            .filter(|rig| !rig.page_commands_efb())?;
        rig.efb_chrome(self.flight.world())
    }

    /// Applies an EFB command; a failure is logged and leaves the flight running.
    pub(super) fn efb_command(&mut self, command: EfbCommand) {
        if let Some(rig) = self.cockpit_rig.as_mut()
            && let Err(err) = rig.efb_command(command)
        {
            warn!(error = %err, ?command, "EFB command failed");
        }
    }

    /// `Esc` with the EFB tablet up minimizes it instead of opening the hub; returns
    /// whether it did.
    pub(super) fn efb_escape(&mut self, event: &KeyEvent) -> bool {
        let escape = event.state == ElementState::Pressed
            && !event.repeat
            && event.physical_key == PhysicalKey::Code(KeyCode::Escape);
        if !escape
            || self
                .efb_chrome()
                .is_none_or(|c| c.mode != EfbDisplayMode::ExpandedEfb)
        {
            return false;
        }
        self.efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner));
        true
    }

    /// A left press over the EFB: a button acts, the corner map expands, the tablet's map
    /// starts a pan. Returns whether the EFB took the press.
    pub(super) fn efb_pressed(&mut self) -> bool {
        let Some(chrome) = self.efb_chrome() else {
            return false;
        };
        match chrome.hit(self.cursor_px, self.surface_size()) {
            Some(EfbHit::Button(command)) => self.efb_command(command),
            Some(EfbHit::Map) if chrome.mode == EfbDisplayMode::CompactCorner => {
                self.efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb));
            }
            Some(EfbHit::Map) => {
                let rect = chrome.map_viewport();
                let [sw, sh] = self.surface_size();
                let [cx, cy] = self.cursor_px;
                let local_px = [
                    cx as f32 - rect.x * (sw as f32),
                    cy as f32 - rect.y * (sh as f32),
                ];
                let hud_hit = self
                    .cockpit_rig
                    .as_mut()
                    .is_some_and(|rig| rig.activate_efb_hud(local_px));
                if !hud_hit {
                    self.hub.drag = Some(PointerDrag::Efb {
                        last: self.cursor_px,
                    });
                }
            }
            Some(EfbHit::Bezel) => {}
            None => return false,
        }
        true
    }

    /// Pans the tablet's map so the ground under the cursor follows it from `last` to
    /// `position` (physical pixels, which are the tablet texture's pixels).
    pub(super) fn pan_efb(&mut self, last: [f64; 2], position: [f64; 2]) {
        let delta = [
            (last[0] - position[0]) as f32,
            (last[1] - position[1]) as f32,
        ];
        if delta != [0.0, 0.0] {
            self.efb_command(EfbCommand::PanPixels(delta));
        }
    }

    /// The wheel over the EFB zooms it by `rows` (positive scrolls down, zooming out);
    /// returns whether the EFB took it.
    pub(super) fn efb_wheel(&mut self, rows: f64) -> bool {
        let over = self
            .efb_chrome()
            .and_then(|c| c.hit(self.cursor_px, self.surface_size()))
            .is_some();
        if over && rows != 0.0 {
            self.efb_command(EfbCommand::ZoomBy(-rows * ZOOM_STEP));
        }
        over
    }

    /// Two-finger open/close pinch over the compact or expanded EFB zooms 1:1 in `2^zoom`
    /// magnification (`delta > 0` zooms in, `delta < 0` zooms out).
    pub(super) fn efb_pinch(&mut self, delta: f64) -> bool {
        let over = self
            .efb_chrome()
            .and_then(|c| c.hit(self.cursor_px, self.surface_size()))
            .is_some();
        if over && let Some(zoom_delta) = pinch_zoom_delta(delta) {
            self.efb_command(EfbCommand::ZoomBy(zoom_delta));
        }
        over
    }

    /// Two-finger twist over the compact or expanded EFB rotates its bearing (`delta_deg > 0`
    /// counter-clockwise, `< 0` clockwise).
    pub(super) fn efb_rotate(&mut self, delta_deg: f64) -> bool {
        let over = self
            .efb_chrome()
            .and_then(|c| c.hit(self.cursor_px, self.surface_size()))
            .is_some();
        if over && delta_deg.is_finite() && delta_deg != 0.0 {
            self.efb_command(EfbCommand::RotateBy(delta_deg));
        }
        over
    }
}

/// Converts a trackpad magnification delta (`delta > 0` open, `< 0` close) into a `2^zoom`
/// level step, rejecting zero or non-finite values.
pub(super) fn pinch_zoom_delta(delta: f64) -> Option<f64> {
    (delta.is_finite() && delta != 0.0).then(|| (1.0 + delta).max(0.25).log2())
}

#[cfg(test)]
mod tests {
    use super::pinch_zoom_delta;

    #[test]
    fn trackpad_pinch_maps_magnification_to_log2_zoom_levels() {
        assert_eq!(pinch_zoom_delta(0.0), None);
        assert_eq!(pinch_zoom_delta(f64::NAN), None);
        assert!((pinch_zoom_delta(1.0).unwrap() - 1.0).abs() < 1e-12);
        assert!((pinch_zoom_delta(-0.5).unwrap() + 1.0).abs() < 1e-12);
        assert_eq!(pinch_zoom_delta(-5.0), Some(-2.0));
    }
}
