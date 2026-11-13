//! The cockpit EFB under the pointer and the keyboard while the hub is closed: chrome
//! buttons, Mission Planner omni-search typing, click-to-expand on the corner map, drag to
//! pan the tablet, click to pick/insert waypoints or ad-hoc thresholds, wheel/pinch/rotate,
//! and `Esc` to cancel pickers / unfocus search / minimize the tablet before opening the hub.

mod effects;

use sim_maplibre::EfbDisplayMode;
use tracing::warn;
use winit::event::{ElementState, KeyEvent};
use winit::keyboard::{KeyCode, PhysicalKey};

use super::FlightApp;
use super::pointer::PointerDrag;
use crate::cockpit_rig::efb_chrome::{EfbChrome, EfbHit, ZOOM_STEP};
use crate::efb_plan::{AdHocPickMode, EfbTab};
use crate::map_rig::EfbCommand;

/// Maximum cursor motion between press and release that counts as a map click rather than a
/// pan drag, physical pixels.
const MAP_CLICK_SLOP_PX: f64 = 6.0;

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
        let mut chrome = rig.efb_chrome(self.flight.world())?;
        let (own_deg, gs_kt) = self.ownship_deg_and_gs_kt();
        chrome.attach_workspace(
            &self.efb_workspace,
            own_deg,
            gs_kt,
            &self.catalog,
            self.flight.world_surface(),
        );
        Some(chrome)
    }

    /// Applies an EFB command; a failure is logged and leaves the flight running.
    pub(super) fn efb_command(&mut self, command: EfbCommand) {
        if let EfbCommand::Plan(action) = command {
            self.apply_efb_plan_action(action);
            return;
        }
        if let Some(rig) = self.cockpit_rig.as_mut()
            && let Err(err) = rig.efb_command(command)
        {
            warn!(error = %err, ?command, "EFB command failed");
        }
    }

    /// `Esc` cancels an active ad-hoc runway picker, unfocuses the omni-search box, or
    /// minimizes the Expanded EFB tablet before opening the hub; returns whether it did.
    pub(super) fn efb_escape(&mut self, event: &KeyEvent) -> bool {
        let escape = event.state == ElementState::Pressed
            && !event.repeat
            && event.physical_key == PhysicalKey::Code(KeyCode::Escape);
        if !escape {
            return false;
        }
        if self.efb_workspace.ad_hoc.pick_mode != AdHocPickMode::None {
            let was_world = self.efb_workspace.ad_hoc.pick_mode.is_world_3d();
            self.efb_workspace.ad_hoc.cancel_pick();
            self.efb_workspace.status_banner = "CANCELLED RUNWAY THRESHOLD PICK".to_owned();
            if was_world {
                self.efb_workspace.tab = EfbTab::AdHocRwy;
                self.efb_command(EfbCommand::SetMode(EfbDisplayMode::ExpandedEfb));
            }
            return true;
        }
        let expanded = self
            .efb_chrome()
            .is_some_and(|c| c.mode == EfbDisplayMode::ExpandedEfb);
        if !expanded {
            return false;
        }
        if self.efb_workspace.omni_focused {
            self.efb_workspace.omni_focused = false;
            return true;
        }
        self.efb_command(EfbCommand::SetMode(EfbDisplayMode::CompactCorner));
        true
    }

    /// Captures typing into the EFB Mission Planner omni-search box when focused on the
    /// Expanded EFB, preventing flight control hotkeys (`W`/`A`/`S`/`D`) from firing.
    pub(super) fn efb_key_input(&mut self, event: &KeyEvent) -> bool {
        if !self.efb_workspace.omni_focused
            || self
                .efb_chrome()
                .is_none_or(|c| c.mode != EfbDisplayMode::ExpandedEfb)
        {
            return false;
        }
        if event.state != ElementState::Pressed {
            return true;
        }
        match event.physical_key {
            PhysicalKey::Code(KeyCode::Enter | KeyCode::NumpadEnter) => {
                if self.efb_workspace.submit_omni_tokens(&self.catalog) > 0 {
                    self.sync_efb_route_to_rig();
                }
                return true;
            }
            PhysicalKey::Code(KeyCode::Backspace) => {
                let _ = self.efb_workspace.backspace_omni();
                return true;
            }
            _ => {}
        }
        if let Some(text) = event.text.as_deref() {
            for ch in text.chars() {
                self.efb_workspace.push_omni_char(ch);
            }
        }
        true
    }

    /// A left press over the EFB: a button acts, the corner map expands, the tablet's map
    /// starts a pan (or click on release). Returns whether the EFB took the press.
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
                self.efb_workspace.omni_focused = false;
                let hud_hit = chrome
                    .map_local_px(self.cursor_px, self.surface_size())
                    .and_then(|local_px| {
                        self.cockpit_rig
                            .as_mut()
                            .map(|rig| rig.activate_efb_hud(local_px))
                    })
                    .unwrap_or(false);
                if !hud_hit {
                    self.hub.drag = Some(PointerDrag::Efb {
                        last: self.cursor_px,
                        origin: self.cursor_px,
                    });
                }
            }
            Some(EfbHit::Bezel) => {
                self.efb_workspace.omni_focused = false;
            }
            None => return false,
        }
        true
    }

    /// Handles a mouse release on the Expanded EFB map: when the cursor moved by at most
    /// [`MAP_CLICK_SLOP_PX`], treats it as a click to pick an ad-hoc threshold, select a
    /// route fix, or insert a `MapPin` waypoint.
    pub(super) fn release_efb_map(&mut self, origin: [f64; 2], at: [f64; 2]) {
        let dx = at.first().copied().unwrap_or(0.0) - origin.first().copied().unwrap_or(0.0);
        let dy = at.get(1).copied().unwrap_or(0.0) - origin.get(1).copied().unwrap_or(0.0);
        if dx.hypot(dy) > MAP_CLICK_SLOP_PX {
            return;
        }
        let Some(local_px) = self
            .efb_chrome()
            .and_then(|c| c.map_local_px(at, self.surface_size()))
        else {
            return;
        };
        let (picked_fix, geo_deg) = match self.cockpit_rig.as_ref() {
            Some(rig) => (
                rig.efb_pick_fix(local_px, 18.0).map(|f| f.id),
                rig.efb_screen_to_geodetic(local_px),
            ),
            None => (None, None),
        };
        if self.efb_workspace.handle_efb_map_click(picked_fix, geo_deg) {
            self.sync_efb_route_to_rig();
        }
    }

    /// Pans the tablet's map so the ground under the cursor follows it from `last` to
    /// `position` (physical pixels, which are the tablet texture's pixels).
    pub(super) fn pan_efb(&mut self, last: [f64; 2], position: [f64; 2]) {
        let delta = [
            (last.first().copied().unwrap_or(0.0) - position.first().copied().unwrap_or(0.0))
                as f32,
            (last.get(1).copied().unwrap_or(0.0) - position.get(1).copied().unwrap_or(0.0)) as f32,
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
