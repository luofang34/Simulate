use std::collections::HashMap;
use std::sync::Arc;

use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::SimInstant;
use sim_input::{Control, InputMapper, ProfileLibrary};
use sim_render::{RenderError, Renderer, VectorScenePass};
use sim_scenario::{ScenarioWorld, WorldScene};
use tracing::{error, info};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoopProxy};
use winit::window::{Window, WindowId};

use crate::camera_view::LookOffset;
use crate::cockpit_rig::CockpitRig;
use crate::controls::ControlScheme;
use crate::error::AppSimError;
use crate::esc_menu::{
    EscMenu, HubCatalog, HubTab, MenuOverlay, menu_input_for_code, menu_input_for_key,
};
use crate::flight::Flight;
use crate::gamepad::Gamepads;
use crate::hangar::HangarRig;
use crate::keyboard::{Keyboard, cockpit_action, scenario_key};
use crate::map_rig::ScenerySource;
use crate::platform;
use crate::sky_rig::SkyRig;
use crate::telemetry;

mod actions;
mod efb_input;
mod frame;
mod hub;
mod pointer;

#[cfg(test)]
pub use hub::TAKEOVER_HINT;
#[cfg(test)]
pub use hub::trim_throttle as trim_throttle_for_tests;
use hub::{HubSettings, trim_throttle, vehicle_class};

/// Seconds between window title updates: often enough to read, rarely enough
/// not to spend a browser's time rewriting the page title.
const TITLE_INTERVAL_S: f64 = 0.25;

/// The window's renderer and the vector passes compiled on its device.
///
/// The passes are built here, asynchronously, because pipeline validation awaits a
/// browser promise on `WebGPU` and cannot block.
pub struct GpuReady {
    /// The window renderer.
    pub renderer: Renderer,
    /// Vector pass for the conformal HUD.
    pub hud_pass: VectorScenePass,
    /// Vector pass for the `Esc` menu.
    pub menu_pass: VectorScenePass,
}

impl GpuReady {
    /// Creates the renderer for `window` and compiles both vector passes on its device.
    pub async fn new(window: Arc<Window>, width: u32, height: u32) -> Result<Self, RenderError> {
        let renderer = Renderer::new(window, width, height).await?;
        let hud_pass = VectorScenePass::new(renderer.device()).await?;
        let menu_pass = VectorScenePass::new(renderer.device()).await?;
        Ok(Self {
            renderer,
            hud_pass,
            menu_pass,
        })
    }
}

/// Events posted back to the event loop from asynchronous work.
pub enum AppEvent {
    /// GPU setup finished (or failed).
    RendererReady(Box<Result<GpuReady, RenderError>>),
}

/// The application: window, shared-GPU renderer, composite cockpit rig, `Esc` menu, sky,
/// input, and simulated flight.
pub struct FlightApp {
    proxy: EventLoopProxy<AppEvent>,
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
    cockpit_rig: Option<CockpitRig>,
    menu_overlay: Option<MenuOverlay>,
    esc_menu: EscMenu,
    sky: SkyRig,
    flight: Flight,
    mapper: InputMapper,
    keyboard: Keyboard,
    gamepads: Gamepads,
    scenes: HashMap<ScenarioWorld, WorldScene>,
    drawn_world: Option<ScenarioWorld>,
    cockpit: CockpitInstrumentSnapshot,
    cursor_px: [f64; 2],
    title_age_s: f64,
    failed: bool,
    controls: ControlScheme,
    catalog: HubCatalog,
    hangar: Option<HangarRig>,
    hub_preview: Option<sim_render::ViewportRect>,
    hub: HubSettings,
    look: LookOffset,
    occluded: bool,
    #[cfg(not(target_arch = "wasm32"))]
    next_occluded_tick: Option<std::time::Instant>,
}

impl FlightApp {
    /// Initialises the flight simulation and input bindings before the GPU surface is created.
    pub fn new(proxy: EventLoopProxy<AppEvent>) -> Result<Self, AppSimError> {
        let flight = Flight::new()?;
        let mut mapper = InputMapper::new(ProfileLibrary::default());
        let vehicle = flight.preferred_vehicle();
        mapper.set(Control::Throttle, trim_throttle(vehicle, false));
        let mut controls = ControlScheme::new(vehicle_class(vehicle));
        controls.sync_live(mapper.controls());
        let catalog = HubCatalog::with_global_airports();
        let mut esc_menu = EscMenu::default();
        if let Some((kdov, _)) = sim_scenery::AirportDatabase::builtin().by_code("KDOV")
            && let Some(idx) = catalog.index_of_airport(kdov)
        {
            esc_menu.select_aerodrome(idx);
        }
        Ok(Self {
            proxy,
            window: None,
            renderer: None,
            cockpit_rig: None,
            menu_overlay: None,
            esc_menu,
            sky: SkyRig::new(),
            flight,
            mapper,
            keyboard: Keyboard::default(),
            gamepads: Gamepads::new(),
            scenes: HashMap::new(),
            drawn_world: None,
            cockpit: CockpitInstrumentSnapshot::blank(SimInstant::START),
            cursor_px: [0.0, 0.0],
            title_age_s: TITLE_INTERVAL_S,
            failed: false,
            controls,
            catalog,
            hangar: None,
            hub_preview: None,
            hub: HubSettings::new(&ScenerySource::from_environment()),
            look: LookOffset::default(),
            occluded: false,
            #[cfg(not(target_arch = "wasm32"))]
            next_occluded_tick: None,
        })
    }

    /// Whether the app stopped because of an error.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn failed(&self) -> bool {
        self.failed
    }

    fn fail(&mut self, event_loop: &ActiveEventLoop, what: &str, err: &dyn std::error::Error) {
        error!(error = %err, source = ?err.source(), "{what}");
        self.failed = true;
        event_loop.exit();
    }

    fn surface_size(&self) -> [u32; 2] {
        self.renderer
            .as_ref()
            .map_or([1, 1], Renderer::surface_size)
    }

    fn gpu_ready(&mut self, event_loop: &ActiveEventLoop, gpu: GpuReady) {
        let GpuReady {
            mut renderer,
            hud_pass,
            menu_pass,
        } = gpu;
        let scale_factor = self.window.as_ref().map_or(1.0, |w| w.scale_factor());
        if let Some(window) = &self.window {
            let size = window.inner_size();
            renderer.resize(size.width, size.height);
            window.request_redraw();
        }
        match CockpitRig::from_renderer(&renderer, hud_pass, self.flight.world_surface().clone()) {
            Ok(mut rig) => {
                rig.set_pixel_ratio(scale_factor);
                let code = rig
                    .supervisor()
                    .effective_aerodrome(self.flight.profile())
                    .code();
                if let Some(index) = self.catalog.aerodromes.iter().position(|a| a.code == code) {
                    self.esc_menu.select_aerodrome(index);
                }
                self.hangar = HangarRig::new(renderer.device())
                    .inspect_err(|err| tracing::warn!(error = %err, "hangar preview unavailable"))
                    .ok();
                info!(backend = ?renderer.backend(), scale_factor, "composite cockpit ready");
                self.cockpit_rig = Some(rig);
                self.menu_overlay = Some(MenuOverlay::new(menu_pass));
                self.renderer = Some(renderer);
            }
            Err(err) => self.fail(event_loop, "cockpit initialization failed", &err),
        }
    }

    fn keyboard_input(&mut self, event_loop: &ActiveEventLoop, event: &winit::event::KeyEvent) {
        if self.hub.command
            && event.state == ElementState::Pressed
            && event.physical_key
                == winit::keyboard::PhysicalKey::Code(winit::keyboard::KeyCode::KeyQ)
        {
            event_loop.exit();
            return;
        }
        let open = self.esc_menu.is_open();
        if open
            && event.state == ElementState::Pressed
            && !event.repeat
            && let winit::keyboard::PhysicalKey::Code(code) = event.physical_key
            && self.controls.capture_key(code)
        {
            return;
        }
        if open && self.preview_flight_key(event) {
            return;
        }
        if !open && self.efb_escape(event) {
            return;
        }
        if let Some(input) = menu_input_for_key(event, open, self.hub.shift) {
            self.handle_menu_input(event_loop, input);
            return;
        }
        if open {
            return;
        }
        if self.keyboard.handle(
            event,
            self.controls.keys(),
            &mut self.mapper,
            &mut self.flight,
        ) {
            self.controls.note_keyboard();
        } else if let Some(profile) = scenario_key(event) {
            self.select(event_loop, profile);
        } else if let Some(action) = cockpit_action(event) {
            self.handle_cockpit_action(event_loop, action);
        }
    }

    /// On the hub's Hangar and Controls tabs, a non-navigation flight key moves the live
    /// telemetry and the hangar preview instead of the hub; key releases always do.
    fn preview_flight_key(&mut self, event: &winit::event::KeyEvent) -> bool {
        let winit::keyboard::PhysicalKey::Code(code) = event.physical_key else {
            return false;
        };
        let Some(action) = self.controls.keys().action_for(code) else {
            return false;
        };
        let pressed = event.state == ElementState::Pressed;
        let tab_previews = matches!(self.esc_menu.tab(), HubTab::Hangar | HubTab::Controls);
        let navigation =
            code == winit::keyboard::KeyCode::Tab || menu_input_for_code(code, false).is_some();
        if pressed && (navigation || !tab_previews) {
            return false;
        }
        self.controls.preview_key(action, pressed);
        pressed
    }
}

impl ApplicationHandler<AppEvent> for FlightApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        match event_loop.create_window(platform::window_attributes()) {
            Ok(window) => {
                let window = Arc::new(window);
                self.window = Some(Arc::clone(&window));
                platform::spawn_renderer(window, self.proxy.clone());
            }
            Err(err) => self.fail(event_loop, "creating the window failed", &err),
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: AppEvent) {
        match event {
            AppEvent::RendererReady(result) => match *result {
                Ok(gpu) => self.gpu_ready(event_loop, gpu),
                Err(err) => self.fail(event_loop, "GPU setup failed", &err),
            },
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let Some(renderer) = self.renderer.as_mut() {
                    renderer.resize(size.width, size.height);
                }
            }
            WindowEvent::ScaleFactorChanged { scale_factor, .. } => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    rig.set_pixel_ratio(scale_factor);
                }
                info!(scale_factor, "display scale factor changed");
            }
            WindowEvent::KeyboardInput { event, .. } => self.keyboard_input(event_loop, &event),
            WindowEvent::ModifiersChanged(modifiers) => {
                self.hub.shift = modifiers.state().shift_key();
                self.hub.command = modifiers.state().super_key();
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.mouse_moved([position.x, position.y]);
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                if state == ElementState::Pressed {
                    self.mouse_pressed(event_loop);
                } else {
                    self.mouse_released();
                }
            }
            WindowEvent::MouseWheel { delta, .. } => self.mouse_wheel(event_loop, delta),
            WindowEvent::PinchGesture { delta, .. } => self.pinch_gesture(delta),
            WindowEvent::RotationGesture { delta, .. } => self.rotation_gesture(delta),
            WindowEvent::Occluded(occluded) => self.set_occluded(occluded),
            WindowEvent::RedrawRequested => self.redraw(event_loop),
            _ => {}
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        let menu_inputs = self.gamepads.poll(
            &mut self.controls,
            &mut self.mapper,
            self.esc_menu.is_open(),
        );
        for input in menu_inputs {
            self.handle_menu_input(event_loop, input);
        }
        self.pace_after_events(event_loop);
    }

    fn exiting(&mut self, _event_loop: &ActiveEventLoop) {
        telemetry::log_pacing_summary(&self.flight);
    }
}

impl FlightApp {
    fn set_occluded(&mut self, occluded: bool) {
        if self.occluded == occluded {
            return;
        }
        self.occluded = occluded;
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.next_occluded_tick = None;
        }
        info!(occluded, "window occlusion changed");
        if !occluded {
            self.flight.note_wall_break();
            if let Some(window) = &self.window {
                window.request_redraw();
            }
        }
    }

    /// Drives the window's frame loop: full display-link VSync while visible, or a gentle
    /// offscreen-only pulse while occluded so unpaused flight and in-flight map tiles stay
    /// resident in VRAM without touching `surface.get_current_texture()`.
    fn pace_after_events(&mut self, event_loop: &ActiveEventLoop) {
        if !self.occluded {
            event_loop.set_control_flow(ControlFlow::Wait);
            if let Some(window) = &self.window {
                window.request_redraw();
            }
            return;
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let sim_running = self.view_and_time_scale().1 != sim_scenario::SimTimeScale::Paused;
            let maps_streaming = self
                .cockpit_rig
                .as_ref()
                .and_then(CockpitRig::maps)
                .is_some_and(crate::map_rig::MapLibreRig::needs_redraw);
            let active = sim_running || maps_streaming;
            let interval = std::time::Duration::from_millis(if active { 50 } else { 250 });
            let now = std::time::Instant::now();
            if active && self.next_occluded_tick.is_none_or(|due| now >= due) {
                self.redraw(event_loop);
            }
            let next = std::time::Instant::now() + interval;
            self.next_occluded_tick = Some(next);
            event_loop.set_control_flow(ControlFlow::WaitUntil(next));
        }
        #[cfg(target_arch = "wasm32")]
        {
            event_loop.set_control_flow(ControlFlow::Wait);
        }
    }
}
