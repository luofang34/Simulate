//! Touch-cockpit frame synchronization, action dispatch, and web `localStorage` view persistence.

use winit::event_loop::ActiveEventLoop;

use super::FlightApp;
use crate::cockpit_rig::{CockpitViewMode, InstrumentDockStyle};
use crate::controls::touch::{TouchAction, TouchUiSnapshot};
use crate::host_guidance::DialedAvionicsReadout;
use crate::keyboard::CockpitAction;
use crate::map_rig::EfbCommand;

#[cfg(target_arch = "wasm32")]
const VIEW_MODE_STORAGE_KEY: &str = "simulate.view_mode";

/// Reads any persisted [`CockpitViewMode`] preference from browser `localStorage`.
#[cfg(target_arch = "wasm32")]
pub(super) fn load_saved_view_mode() -> Option<CockpitViewMode> {
    let storage = web_sys::window()?.local_storage().ok()??;
    let raw = storage.get_item(VIEW_MODE_STORAGE_KEY).ok()??;
    CockpitViewMode::from_code(&raw)
}

#[cfg(target_arch = "wasm32")]
fn save_view_mode(mode: CockpitViewMode) {
    if let Some(storage) = web_sys::window()
        .and_then(|w| w.local_storage().ok())
        .flatten()
    {
        storage.set_item(VIEW_MODE_STORAGE_KEY, mode.code()).ok();
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn save_view_mode(_mode: CockpitViewMode) {}

impl FlightApp {
    /// Restores any saved mobile view preference and mounts `#touch-cockpit` on handheld web.
    #[cfg(target_arch = "wasm32")]
    pub(super) fn mount_mobile_touch_cockpit(&mut self, page_view: bool) {
        if !self.handheld || page_view {
            return;
        }
        if let (Some(mode), Some(rig)) = (load_saved_view_mode(), self.cockpit_rig.as_mut()) {
            rig.set_view_mode(mode);
            self.sync_hub_layout();
        }
        self.touch_cockpit = crate::platform::TouchCockpit::mount();
    }
    /// Builds the live UI snapshot pushed to `#touch-cockpit` (callable in native tests too).
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "called by wasm32 touch overlay loop")
    )]
    #[must_use]
    pub(super) fn touch_ui_snapshot(&self, now_s: f64) -> TouchUiSnapshot {
        let hub_open = self.esc_menu.is_open();
        let throttle_norm = self.mapper.controls().throttle;
        let Some(rig) = self.cockpit_rig.as_ref() else {
            return TouchUiSnapshot {
                hub_open,
                throttle_norm,
                view_mode: CockpitViewMode::HudAndEfis,
                dock_style: InstrumentDockStyle::LeftRail,
                hides_yoke: false,
                efb_open: false,
                efb_label: "MAP",
                efis_left_fraction: 0.0,
                dial_readout: DialedAvionicsReadout {
                    heading_bug_deg: None,
                    altitude_bug_ft: None,
                    baro_hpa: 1_013.25,
                    baro_inhg: 29.92,
                },
                now_s,
            };
        };
        let surface_size = self
            .renderer
            .as_ref()
            .map_or([1, 1], sim_render::Renderer::surface_size);
        let status = rig.maps().and_then(|m| m.efb_status());
        let efb_open = status.as_ref().is_some_and(|s| !s.collapsed);
        let efb_label = match status.as_ref() {
            Some(s) if !s.collapsed => match s.mode {
                sim_maplibre::EfbDisplayMode::CompactCorner => "EFB",
                sim_maplibre::EfbDisplayMode::ExpandedEfb => "HIDE MAP",
            },
            _ => "MAP",
        };
        TouchUiSnapshot {
            hub_open,
            throttle_norm,
            view_mode: rig.view_mode(),
            dock_style: rig.dock_style(),
            hides_yoke: rig.hides_cockpit_controls(),
            efb_open,
            efb_label,
            efis_left_fraction: rig.efis_left_clear_fraction(surface_size),
            dial_readout: rig.dialed_readout(&self.flight),
            now_s,
        }
    }

    /// Dispatches a discrete action originating from the mobile `#touch-cockpit` overlay.
    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "dispatched by wasm32 touch overlay loop")
    )]
    pub(super) fn handle_touch_action(
        &mut self,
        event_loop: &ActiveEventLoop,
        action: TouchAction,
    ) {
        match action {
            TouchAction::ToggleHub => {
                self.handle_menu_input(event_loop, crate::esc_menu::MenuInput::Toggle);
            }
            TouchAction::CycleViewport => {
                self.handle_cockpit_action(event_loop, CockpitAction::CycleViewport);
            }
            TouchAction::TogglePause => {
                self.handle_cockpit_action(event_loop, CockpitAction::TogglePause);
            }
            TouchAction::FlapsDown => {
                self.handle_cockpit_action(event_loop, CockpitAction::FlapsDown);
            }
            TouchAction::FlapsUp => {
                self.handle_cockpit_action(event_loop, CockpitAction::FlapsUp);
            }
            TouchAction::ToggleGear => {
                self.handle_cockpit_action(event_loop, CockpitAction::ToggleGear);
            }
            TouchAction::CycleEfb => self.efb_command(EfbCommand::CycleMobileState),
            TouchAction::ResetLook => {
                self.look.reset();
                self.touch.clear_manual_look();
            }
            other => self.handle_rig_touch_action(other),
        }
    }

    #[cfg_attr(
        not(target_arch = "wasm32"),
        allow(dead_code, reason = "dispatched by wasm32 touch overlay loop")
    )]
    fn handle_rig_touch_action(&mut self, action: TouchAction) {
        let Some(rig) = self.cockpit_rig.as_mut() else {
            return;
        };
        match action {
            TouchAction::SetViewMode(mode) => {
                rig.set_view_mode(mode);
                self.sync_hub_layout();
                save_view_mode(mode);
            }
            TouchAction::ToggleDockStyle => {
                rig.toggle_dock_style();
            }
            TouchAction::ToggleYoke => {
                let next = !rig.hides_cockpit_controls();
                rig.set_hide_cockpit_controls(next);
            }
            TouchAction::AdjustHeadingBug(deg) => {
                rig.adjust_heading_bug(&self.flight, f64::from(deg));
            }
            TouchAction::SyncHeadingBug => {
                rig.sync_heading_bug(&self.flight);
            }
            TouchAction::AdjustAltitudeBug(ft) => {
                rig.adjust_altitude_bug(&self.flight, f64::from(ft));
            }
            TouchAction::SyncAltitudeBug => {
                rig.sync_altitude_bug(&self.flight);
            }
            TouchAction::AdjustBaroTenthsHpa(tenths) => {
                rig.adjust_baro_hpa(&mut self.flight, f64::from(tenths) * 0.1);
            }
            TouchAction::SetBaroStd => {
                rig.set_baro_std(&mut self.flight);
            }
            _ => {}
        }
    }

    #[cfg(target_arch = "wasm32")]
    pub(super) fn sync_touch_cockpit(&mut self, event_loop: &ActiveEventLoop) {
        let now_s = web_sys::window()
            .and_then(|w| w.performance())
            .map_or(0.0, |p| p.now() / 1000.0);
        let initial_snap = self.touch_ui_snapshot(now_s);
        let actions = {
            let Some(cockpit) = self.touch_cockpit.as_ref() else {
                return;
            };
            cockpit.sync_ui(&initial_snap);
            cockpit.apply_to(
                &mut self.mapper,
                &mut self.controls,
                &mut self.flight,
                !initial_snap.hub_open,
            )
        };
        if !actions.is_empty() {
            for action in actions {
                self.handle_touch_action(event_loop, action);
            }
            let updated_snap = self.touch_ui_snapshot(now_s);
            if let Some(cockpit) = self.touch_cockpit.as_ref() {
                cockpit.sync_ui(&updated_snap);
            }
        }
    }
}
