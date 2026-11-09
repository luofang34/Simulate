//! The `Esc` Flight Hub in the app: its status, layout, drawing with the live hangar
//! preview, and the commands only the hub issues.

use sim_input::Control;
use sim_maplibre::HostGpu;
use sim_render::{OverlayQuad, Renderer, ViewportRect};
use sim_scenario::SimTimeScale;
use tracing::{info, warn};
use winit::event_loop::ActiveEventLoop;

use super::FlightApp;
use crate::controls::{
    CONTROLS, CaptureTarget, ControlScheme, KeyAction, VehicleClass, control_value, key_label,
};
use crate::esc_menu::view::TimelineScrub;
use crate::esc_menu::{
    AvionicsLayout, ControlsStatus, HubTab, HubView, MenuCommand, MenuOverlay, MenuStatus,
    OverlayContent, RateChoice, SceneryMode, Step,
};
use crate::hangar::HangarRig;
use crate::map_rig::{AvionicsPreset, ScenerySource};
use crate::vehicles::FreeFlightVehicle;

mod globe;
mod spawn;
#[cfg(test)]
mod tests;

/// Seconds the scenario takeover hint stays up after the pilot touches the controls.
const HINT_HOLD_S: f64 = 4.0;
/// Stick deflection that counts as the pilot trying to fly a scenario.
const HINT_DEFLECTION: f64 = 0.3;
/// The takeover hint.
pub const TAKEOVER_HINT: &str = "AUTOPILOT SCENARIO - PRESS M OR ESC TO TAKE MANUAL CONTROL";

/// Hub selections that live outside the menu state machine.
#[derive(Debug, Clone, Copy)]
pub struct HubSettings {
    /// Instrument fit.
    pub layout: AvionicsLayout,
    /// Tile source.
    pub scenery: SceneryMode,
    /// Whether `Shift` is held (for `Shift+Tab`).
    pub shift: bool,
    /// Whether `Cmd` / `Super` is held (for `Cmd+Q`).
    pub command: bool,
    /// Seconds left on the takeover hint.
    pub hint_s: f64,
    /// What a held left button is dragging.
    pub drag: Option<super::pointer::PointerDrag>,
}

impl HubSettings {
    /// Starting selections for `scenery`.
    #[must_use]
    pub fn new(scenery: &ScenerySource) -> Self {
        Self {
            layout: AvionicsLayout::FullGlass,
            scenery: match scenery {
                ScenerySource::Catalog => SceneryMode::Offline,
                ScenerySource::RealWorld { .. } => SceneryMode::Online,
            },
            shift: false,
            command: false,
            hint_s: 0.0,
            drag: None,
        }
    }
}

/// The hub's instrument fit for what the cockpit shows: the dock hidden is HUD only,
/// the map-less preset is the basic six, and any map preset is full glass. Without
/// maps the preset is unknown, so a shown dock keeps the selected panel fit.
#[must_use]
pub fn shown_layout(
    instruments: bool,
    preset: Option<AvionicsPreset>,
    selected: AvionicsLayout,
) -> AvionicsLayout {
    match (instruments, preset) {
        (false, _) => AvionicsLayout::HudOnly,
        (true, Some(AvionicsPreset::NoMapGauges)) => AvionicsLayout::BasicSix,
        (true, Some(_)) => AvionicsLayout::FullGlass,
        (true, None) if selected == AvionicsLayout::HudOnly => AvionicsLayout::FullGlass,
        (true, None) => selected,
    }
}

/// The control class of a free-flight plant.
#[must_use]
pub const fn vehicle_class(vehicle: FreeFlightVehicle) -> VehicleClass {
    match vehicle {
        FreeFlightVehicle::AviateX500Direct => VehicleClass::Multirotor,
        FreeFlightVehicle::JsbsimC172
        | FreeFlightVehicle::JsbsimP38g
        | FreeFlightVehicle::AviateFixedWingDirect
        | FreeFlightVehicle::Kinematic => VehicleClass::FixedWing,
    }
}

/// Throttle a fresh spawn of `vehicle` trims to: centred hold for the X500 (whose ground
/// spawn is a low hover), cruise power for the Aviate fixed-wing (whose ground spawn starts
/// airborne), idle for aeroplanes on the ground, cruise power otherwise.
#[must_use]
pub const fn trim_throttle(vehicle: FreeFlightVehicle, on_ground: bool) -> f64 {
    match (vehicle, on_ground) {
        (FreeFlightVehicle::AviateX500Direct, _) => 0.5,
        (FreeFlightVehicle::AviateFixedWingDirect, _) | (_, false) => 0.65,
        (_, true) => 0.0,
    }
}

fn controls_status(scheme: &ControlScheme) -> ControlsStatus {
    ControlsStatus {
        preset: scheme.preset(),
        automatic: scheme.is_automatic(),
        rows: scheme.axis_rows(),
        keys: KeyAction::ALL
            .into_iter()
            .map(|action| (action, key_label(scheme.keys().key_for(action))))
            .collect(),
        capture: scheme.capture(),
        live: scheme.live(),
        source: scheme.source(),
        controller: scheme.controller().map(str::to_owned),
    }
}

impl FlightApp {
    /// Trims the throttle for the free-flight spawn just made and follows its control class.
    pub(super) fn after_spawn(&mut self) {
        if let Some(vehicle) = self.flight.free_vehicle() {
            let throttle = trim_throttle(vehicle, self.flight.spawned_on_ground());
            self.mapper.set(Control::Throttle, throttle);
            self.controls.set_vehicle_class(vehicle_class(vehicle));
            self.esc_menu.sync_vehicle(&self.catalog, vehicle);
        }
        self.controls.sync_live(self.mapper.controls());
    }

    /// What the hub shows about the current flight.
    pub(super) fn menu_status(&self) -> MenuStatus {
        let profile = self.flight.profile();
        let rig = self.cockpit_rig.as_ref();
        let sup = rig.map(|r| r.supervisor());
        MenuStatus {
            pacing: self.flight.pacing_stats(),
            history_s: self.flight.history_span_s(),
            rewind_offset_s: self.flight.rewind_offset_s(),
            epoch: rig.map_or(0, |r| r.guidance_epoch()),
            sensor_frames: rig
                .and_then(|r| r.maps())
                .map_or(0, |m| m.sensor_frame_count()),
            sensor_last_s: rig
                .and_then(|r| r.maps())
                .and_then(|m| m.last_sensor_stamp())
                .map(|stamp| {
                    let since = stamp.instant.duration_since(sim_core::SimInstant::START);
                    since.as_seconds()
                }),
            scenario: profile
                .map_or("FREE FLIGHT", sim_scenario::ScenarioProfile::title)
                .to_owned(),
            scenario_active: profile.is_some(),
            airport: sup
                .map_or("", |s| s.effective_airport_code(profile))
                .to_owned(),
            aircraft: self.flight.preferred_vehicle().title().to_owned(),
            active_vehicle: self.flight.free_vehicle(),
            sky: self.sky.mode().label().to_owned(),
            paused: true,
            rate: sup.map_or(RateChoice::Real, |s| RateChoice::of(s.time_scale())),
            layout: self.hub.layout,
            scenery: self.hub.scenery,
            auto_rotate: self.hangar.as_ref().is_none_or(|h| h.auto_rotate()),
            controls: controls_status(&self.controls),
        }
    }

    /// The hub laid out for the current window and state.
    pub(super) fn hub_view(&self) -> HubView {
        HubView::build(
            &self.esc_menu,
            &self.catalog,
            &self.menu_status(),
            self.surface_size(),
        )
    }

    /// Counts down the takeover hint, restarting it while the pilot moves the controls of a
    /// scripted scenario.
    pub(super) fn update_takeover_hint(&mut self, elapsed_s: f64) {
        let live = self.mapper.controls();
        let deflected = [Control::Roll, Control::Pitch, Control::Yaw]
            .into_iter()
            .any(|c| control_value(&live, c).abs() > HINT_DEFLECTION);
        let trying = deflected || self.keyboard.any_held();
        if self.flight.profile().is_some() && !self.esc_menu.is_open() && trying {
            self.hub.hint_s = HINT_HOLD_S;
        } else if self.flight.profile().is_none() {
            self.hub.hint_s = 0.0;
        } else {
            self.hub.hint_s = (self.hub.hint_s - elapsed_s.max(0.0)).max(0.0);
        }
    }

    /// Draws the open hub (with the hangar preview or the airport globe on their tabs), or
    /// the in-flight chrome: the EFB's buttons and the takeover hint.
    pub(super) fn render_menu(&mut self, event_loop: &ActiveEventLoop, elapsed_s: f64) {
        let view = self.esc_menu.is_open().then(|| self.hub_view());
        let hangar_model = view
            .as_ref()
            .filter(|_| self.esc_menu.tab() == HubTab::Hangar)
            .and_then(|v| {
                Some((
                    v.preview_px()?,
                    self.catalog.vehicles.get(self.esc_menu.vehicle())?.model,
                ))
            });
        let globe_size = view
            .as_ref()
            .filter(|_| self.esc_menu.tab() == HubTab::World)
            .and_then(HubView::preview_px);
        let live = self.controls.live();
        let chrome = self.efb_chrome();
        let hint = (self.hub.hint_s > 0.0).then_some(TAKEOVER_HINT);
        let (Some(renderer), Some(overlay)) = (&self.renderer, self.menu_overlay.as_mut()) else {
            return;
        };
        let content = match (&view, hint, &chrome) {
            (Some(view), _, _) => OverlayContent::Hub {
                view,
                highlights: self.esc_menu.highlights(),
            },
            (None, Some(hint), None) => OverlayContent::Banner(hint),
            (None, banner, Some(efb)) => OverlayContent::FlightChrome {
                banner,
                efb: Some(efb),
            },
            (None, None, None) => OverlayContent::Hidden,
        };
        if let Err(err) = overlay.render(
            renderer.device(),
            renderer.queue(),
            content,
            renderer.surface_size(),
        ) {
            return self.fail(event_loop, "drawing the flight hub failed", &err);
        }
        if let Some(hangar) = self.hangar.as_mut() {
            hangar.hide();
            if let Some((size, model)) = hangar_model
                && let Err(err) = hangar.render(
                    renderer.device(),
                    renderer.queue(),
                    model,
                    size,
                    live,
                    elapsed_s,
                )
            {
                warn!(error = %err, "hangar preview unavailable");
            }
        }
        self.hub_preview = view.and_then(|v| v.preview_viewport());
        if let Some(size) = globe_size {
            self.render_globe(size);
        }
    }

    /// Applies a command only the hub issues; returns whether the hub should close.
    pub(super) fn apply_hub_command(
        &mut self,
        event_loop: &ActiveEventLoop,
        command: MenuCommand,
    ) -> bool {
        match command {
            MenuCommand::Rate(rate) => self.set_rate(rate.time_scale()),
            MenuCommand::Layout(layout) => self.set_layout(event_loop, layout),
            MenuCommand::Scenery(mode) => self.set_scenery(mode),
            MenuCommand::PreviewVehicle(_) => {}
            MenuCommand::Scrub(scrub) => self.scrub_timeline(event_loop, scrub),
            MenuCommand::ResetHangarView => {
                if let Some(h) = self.hangar.as_mut() {
                    h.reset_view();
                }
            }
            MenuCommand::ToggleAutoRotate => {
                if let Some(h) = self.hangar.as_mut() {
                    h.toggle_auto_rotate();
                }
            }
            MenuCommand::ApplyVehicle { vehicle, mode } => {
                return self.apply_vehicle(event_loop, vehicle, mode);
            }
            MenuCommand::Spawn {
                aerodrome,
                runway,
                mode,
            } => {
                return self.spawn(event_loop, aerodrome, runway, mode);
            }
            MenuCommand::LaunchScenario(index) => {
                let profile = self.catalog.scenarios.get(index).and_then(|s| s.profile);
                self.select(event_loop, profile);
                return true;
            }
            other => self.apply_control_command(other),
        }
        false
    }

    /// Moves the held flight along its rewind history.
    fn scrub_timeline(&mut self, event_loop: &ActiveEventLoop, scrub: TimelineScrub) {
        let moved = match scrub {
            TimelineScrub::Forward(seconds) => self.flight.scrub_by_seconds(f64::from(seconds)),
            TimelineScrub::Live => self.flight.scrub_to_live(),
            TimelineScrub::Permille(p) => self.flight.scrub_to_fraction(f64::from(p) / 1000.0),
        };
        match moved {
            Ok(true) => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    rig.invalidate_chart();
                }
                info!(
                    ?scrub,
                    offset_s = self.flight.rewind_offset_s(),
                    "scrubbed timeline"
                );
            }
            Ok(false) => {}
            Err(err) => self.fail(event_loop, "scrubbing the rewind timeline failed", &err),
        }
    }

    fn apply_control_command(&mut self, command: MenuCommand) {
        let forward = |step: Step| step == Step::Next;
        match command {
            MenuCommand::Preset(step) => {
                let preset = self.controls.step_preset(forward(step));
                info!(preset = preset.title(), "selected control preset");
            }
            MenuCommand::PresetAuto => self.controls.reset_automatic(),
            MenuCommand::RebindAxis(control) => {
                self.controls.begin_capture(CaptureTarget::Axis(control))
            }
            MenuCommand::InvertAxis(control) => self.controls.toggle_invert(control),
            MenuCommand::Deadzone(control, step) => {
                self.controls.step_deadzone(control, forward(step))
            }
            MenuCommand::Expo(control, step) => self.controls.step_expo(control, forward(step)),
            MenuCommand::RebindKey(action) => {
                self.controls.begin_capture(CaptureTarget::Key(action))
            }
            _ => {}
        }
        debug_assert!(CONTROLS.len() == 4);
    }

    fn set_rate(&mut self, scale: SimTimeScale) {
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.supervisor_mut().set_time_scale(scale);
            info!(?scale, "set simulation rate");
        }
    }

    fn set_layout(&mut self, event_loop: &ActiveEventLoop, layout: AvionicsLayout) {
        let Some(rig) = self.cockpit_rig.as_mut() else {
            return;
        };
        let (preset, instruments) = match layout {
            AvionicsLayout::FullGlass => (Some(AvionicsPreset::GlassCockpit), true),
            AvionicsLayout::BasicSix => (Some(AvionicsPreset::NoMapGauges), true),
            AvionicsLayout::HudOnly => (None, false),
        };
        rig.set_show_instruments(instruments);
        rig.set_show_conformal_hud(true);
        if let Some(preset) = preset
            && let Err(err) = rig.set_avionics_preset(preset)
        {
            return self.fail(event_loop, "switching avionics layout failed", &err);
        }
        self.hub.layout = layout;
    }

    fn set_scenery(&mut self, mode: SceneryMode) {
        let (Some(renderer), Some(rig)) = (self.renderer.as_ref(), self.cockpit_rig.as_mut())
        else {
            return;
        };
        let source = match mode {
            SceneryMode::Online => ScenerySource::real_world_from_environment(),
            SceneryMode::Offline => ScenerySource::Catalog,
        };
        let enable = renderer.backend() != wgpu::Backend::Gl;
        rig.rebuild_maps(host_gpu(renderer), enable.then_some(&source));
        self.hub.scenery = mode;
        info!(scenery = mode.label(), "switched scenery source");
    }
}

/// The window renderer's GPU as a map host.
pub fn host_gpu(renderer: &Renderer) -> HostGpu {
    HostGpu {
        instance: renderer.instance().clone(),
        adapter: renderer.adapter().clone(),
        device: renderer.device().clone(),
        queue: renderer.queue().clone(),
    }
}

/// Appends the tab's live 3D preview (the hangar's sRGB panel or the globe's linear map
/// texture) underneath the hub vector overlay's transparent preview cutout, so crisp UI
/// controls placed inside the map's corners draw cleanly over the live globe.
pub fn hub_quads<'a>(
    overlay: Option<&'a MenuOverlay>,
    preview: Option<ViewportRect>,
    preview_view: Option<HubPreviewView<'a>>,
    quads: &mut Vec<OverlayQuad<'a>>,
) {
    match (preview, preview_view) {
        (Some(rect), Some(HubPreviewView::Hangar(view))) => {
            quads.push(OverlayQuad::panel(rect, view, true, false));
        }
        (Some(rect), Some(HubPreviewView::Globe(view))) => {
            quads.push(OverlayQuad::inset_map(rect, view, false));
        }
        _ => {}
    }
    if let Some(menu) = overlay.and_then(MenuOverlay::view) {
        quads.push(OverlayQuad::vector_overlay(ViewportRect::FULL, menu));
    }
}

/// The live 3D view drawn over the hub's preview card.
#[derive(Debug, Clone, Copy)]
pub enum HubPreviewView<'a> {
    /// The hangar's turntable (an `Indicate`-style sRGB panel).
    Hangar(&'a wgpu::TextureView),
    /// The airport globe (a `maplibre` map texture).
    Globe(&'a wgpu::TextureView),
}

/// The preview view of `tab`: the hangar's or the globe's last frame.
#[must_use]
pub fn hub_preview_view<'a>(
    tab: HubTab,
    hangar: Option<&'a HangarRig>,
    maps: Option<&'a crate::map_rig::MapLibreRig>,
) -> Option<HubPreviewView<'a>> {
    match tab {
        HubTab::World => maps
            .and_then(crate::map_rig::MapLibreRig::globe_view)
            .map(HubPreviewView::Globe),
        _ => hangar.and_then(HangarRig::view).map(HubPreviewView::Hangar),
    }
}
