use std::collections::HashMap;
use std::collections::hash_map::Entry;
use std::sync::Arc;

use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::SimInstant;
use sim_input::{Control, InputMapper, ProfileLibrary};
use sim_render::{FrameOutcome, RenderError, Renderer, SceneLighting};
use sim_scenario::{
    OperatorViewportMode, ScenarioProfile, ScenarioWorld, SimTimeScale, WorldScene,
};
use tracing::{error, info};
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoopProxy};
use winit::window::{Window, WindowId};

use crate::camera_view::camera_from_virtual_spec;
use crate::cockpit_rig::CockpitRig;
use crate::error::AppSimError;
use crate::flight::Flight;
use crate::gamepad::Gamepads;
use crate::keyboard::{
    CockpitAction, Keyboard, apply_aircraft_system_action, cockpit_action, scenario_key,
};
use crate::platform;
use crate::telemetry;

/// Seconds between window title updates: often enough to read, rarely enough
/// not to spend a browser's time rewriting the page title.
const TITLE_INTERVAL_S: f64 = 0.25;
const INITIAL_CRUISE_THROTTLE: f64 = 0.72;

/// Events posted back to the event loop from asynchronous work.
pub enum AppEvent {
    /// GPU setup finished (or failed).
    RendererReady(Result<Renderer, RenderError>),
}

/// The application: window, shared-GPU renderer, composite cockpit rig, input, and simulated flight.
pub struct FlightApp {
    proxy: EventLoopProxy<AppEvent>,
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
    cockpit_rig: Option<CockpitRig>,
    flight: Flight,
    mapper: InputMapper,
    keyboard: Keyboard,
    gamepads: Gamepads,
    scenes: HashMap<ScenarioWorld, WorldScene>,
    drawn_world: Option<ScenarioWorld>,
    cockpit: CockpitInstrumentSnapshot,
    title_age_s: f64,
    failed: bool,
}

impl FlightApp {
    /// Initialises the flight simulation and input bindings before the GPU surface is created.
    pub fn new(proxy: EventLoopProxy<AppEvent>) -> Result<Self, AppSimError> {
        let mut mapper = InputMapper::new(ProfileLibrary::default());
        mapper.set(Control::Throttle, INITIAL_CRUISE_THROTTLE);
        Ok(Self {
            proxy,
            window: None,
            renderer: None,
            cockpit_rig: None,
            flight: Flight::new()?,
            mapper,
            keyboard: Keyboard::default(),
            gamepads: Gamepads::new(),
            scenes: HashMap::new(),
            drawn_world: None,
            cockpit: CockpitInstrumentSnapshot::blank(SimInstant::START),
            title_age_s: TITLE_INTERVAL_S,
            failed: false,
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

    fn select(&mut self, event_loop: &ActiveEventLoop, profile: Option<ScenarioProfile>) {
        if profile.is_none() {
            self.mapper.set(Control::Throttle, INITIAL_CRUISE_THROTTLE);
        }
        let name = profile.map_or("free flight", ScenarioProfile::title);
        info!(scenario = name, "selected");
        if let Err(err) = self.flight.select(profile) {
            self.fail(event_loop, "the scenario could not start", &err);
        }
    }

    fn handle_cockpit_action(&mut self, event_loop: &ActiveEventLoop, action: CockpitAction) {
        match action {
            CockpitAction::CycleViewport => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    let mode = rig.supervisor_mut().cycle_viewport_mode();
                    info!(?mode, "switched operator viewport mode");
                }
            }
            CockpitAction::CycleVirtualCamera => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    let spec = rig
                        .supervisor_mut()
                        .cycle_virtual_camera(self.flight.profile());
                    info!(camera = spec.title, id = spec.id, "selected virtual camera");
                }
            }
            CockpitAction::CycleAirport => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    let id = rig.supervisor_mut().cycle_aerodrome();
                    if let Some(diag) = rig.supervisor().diagram(id).cloned() {
                        self.mapper.set(Control::Throttle, INITIAL_CRUISE_THROTTLE);
                        if let Err(err) = self.flight.respawn_at_diagram(&diag) {
                            self.fail(event_loop, "respawning at airport failed", &err);
                            return;
                        }
                        info!(
                            airport = diag.id.code(),
                            name = diag.name,
                            "switched active airport"
                        );
                    }
                }
            }
            CockpitAction::CycleVehicle => {
                self.mapper.set(Control::Throttle, INITIAL_CRUISE_THROTTLE);
                match self.flight.cycle_vehicle() {
                    Ok(vehicle) => info!(vehicle = vehicle.title(), "selected free-flight vehicle"),
                    Err(err) => self.fail(event_loop, "switching free-flight vehicle failed", &err),
                }
            }
            CockpitAction::ToggleInstruments => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    let visible = rig.toggle_instruments();
                    info!(visible, "toggled glass-cockpit instrument dock");
                }
            }
            CockpitAction::ToggleHud => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    let visible = rig.toggle_conformal_hud();
                    info!(visible, "toggled conformal HUD overlay");
                }
            }
            CockpitAction::TogglePause => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    let scale = rig.supervisor_mut().toggle_pause();
                    info!(?scale, "toggled simulation pause");
                }
            }
            CockpitAction::CycleWarp => {
                if let Some(rig) = self.cockpit_rig.as_mut() {
                    let scale = rig.supervisor_mut().cycle_warp();
                    info!(?scale, "updated simulation time warp");
                }
            }
            CockpitAction::FlapsDown
            | CockpitAction::FlapsUp
            | CockpitAction::CycleMagnetos
            | CockpitAction::MixtureRich
            | CockpitAction::MixtureLean => {
                apply_aircraft_system_action(action, &mut self.flight);
            }
        }
    }

    /// Uploads the fallback `GlobeScene` mesh for `world` when needed (e.g. `LunarTranquility`
    /// or `WebGL2` fallback).
    fn ensure_globe_world(&mut self, event_loop: &ActiveEventLoop, world: ScenarioWorld) {
        let Some(renderer) = self.renderer.as_mut() else {
            return;
        };
        if self.drawn_world == Some(world) {
            return;
        }
        let scene = match self.scenes.entry(world) {
            Entry::Occupied(scene) => scene.into_mut(),
            Entry::Vacant(slot) => match world.scene() {
                Ok(scene) => slot.insert(scene),
                Err(err) => return self.fail(event_loop, "building the terrain failed", &err),
            },
        };
        if let Err(err) = renderer.set_scene(&scene.mesh, scene.atlas.as_ref()) {
            return self.fail(event_loop, "uploading the terrain failed", &err);
        }
        self.drawn_world = Some(world);
    }

    /// Logs cockpit state transitions and refreshes the window title periodically.
    fn observe_cockpit(&mut self, elapsed_s: f64) {
        let cockpit = self.flight.cockpit();
        telemetry::log_changes(&self.cockpit, &cockpit);
        self.cockpit = cockpit;
        self.title_age_s += elapsed_s;
        if self.title_age_s >= TITLE_INTERVAL_S
            && let Some(window) = &self.window
        {
            self.title_age_s = 0.0;
            let profile = self.flight.profile();
            let (viewport_mode, cam_title, aero_code, time_scale) =
                self.cockpit_rig.as_ref().map_or(
                    (
                        OperatorViewportMode::Cockpit,
                        None,
                        None,
                        SimTimeScale::RealTime,
                    ),
                    |rig| {
                        let sup = rig.supervisor();
                        let earth = self.flight.world().body() == sim_geo::BodyEllipsoid::WGS84;
                        (
                            sup.viewport_mode(),
                            Some(sup.active_virtual_camera(profile).title),
                            earth.then(|| sup.effective_aerodrome(profile).code()),
                            sup.time_scale(),
                        )
                    },
                );
            platform::set_title(
                window,
                &telemetry::title(
                    &self.flight,
                    viewport_mode,
                    cam_title,
                    aero_code,
                    time_scale,
                    &cockpit,
                ),
            );
        }
    }

    fn redraw(&mut self, event_loop: &ActiveEventLoop) {
        let (viewport_mode, time_scale) = self.cockpit_rig.as_ref().map_or(
            (OperatorViewportMode::Cockpit, SimTimeScale::RealTime),
            |rig| {
                (
                    rig.supervisor().viewport_mode(),
                    rig.supervisor().time_scale(),
                )
            },
        );
        let controls = self.mapper.controls();
        match self.flight.advance_to_now(controls, time_scale) {
            Ok(elapsed_s) => {
                self.keyboard.update(elapsed_s, &mut self.mapper);
                self.observe_cockpit(elapsed_s);
            }
            Err(err) => return self.fail(event_loop, "the flight model stopped", &err),
        }

        let mut camera = match self.flight.camera(viewport_mode) {
            Ok(camera) => camera,
            Err(err) => return self.fail(event_loop, "the view could not be placed", &err),
        };
        if viewport_mode == OperatorViewportMode::SensorAperture
            && let Some(rig) = self.cockpit_rig.as_ref()
            && let Some(vcam) = camera_from_virtual_spec(
                &rig.supervisor()
                    .active_virtual_camera(self.flight.profile()),
                &self.flight.vehicle_state(),
            )
        {
            camera = vcam;
        }
        let world = self.flight.world();
        let lighting = SceneLighting {
            sun_direction_ecef: world.sun_direction_ecef(),
        };

        let need_globe = match (&self.renderer, self.cockpit_rig.as_mut()) {
            (Some(renderer), Some(rig)) => match rig.prepare_frame(
                renderer.queue(),
                renderer.surface_size(),
                &self.flight,
                &camera,
                lighting.sun_direction_ecef,
            ) {
                Ok(need_globe) => need_globe,
                Err(err) => return self.fail(event_loop, "cockpit avionics update failed", &err),
            },
            (Some(_), None) => true,
            (None, _) => return,
        };
        if need_globe {
            self.ensure_globe_world(event_loop, world);
            if self.failed {
                return;
            }
        }

        let (Some(renderer), Some(rig)) = (self.renderer.as_mut(), self.cockpit_rig.as_ref())
        else {
            return;
        };
        let quads = rig.overlay_quads(world);
        let globe_arg = need_globe.then_some((&camera, &lighting));
        match renderer.render_composite(globe_arg, &quads) {
            Ok(FrameOutcome::Presented | FrameOutcome::Skipped) => {}
            Err(err) => self.fail(event_loop, "rendering failed", &err),
        }
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
            AppEvent::RendererReady(Ok(mut renderer)) => {
                if let Some(window) = &self.window {
                    let size = window.inner_size();
                    renderer.resize(size.width, size.height);
                    window.request_redraw();
                }
                match CockpitRig::from_renderer(&renderer) {
                    Ok(rig) => {
                        info!(backend = ?renderer.backend(), "composite cockpit ready");
                        self.cockpit_rig = Some(rig);
                        self.renderer = Some(renderer);
                    }
                    Err(err) => self.fail(event_loop, "cockpit initialization failed", &err),
                }
            }
            AppEvent::RendererReady(Err(err)) => self.fail(event_loop, "GPU setup failed", &err),
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
            WindowEvent::KeyboardInput { event, .. } => {
                if let Some(profile) = scenario_key(&event) {
                    self.select(event_loop, profile);
                } else if let Some(action) = cockpit_action(&event) {
                    self.handle_cockpit_action(event_loop, action);
                }
                self.keyboard
                    .handle(&event, &mut self.mapper, &mut self.flight);
            }
            WindowEvent::RedrawRequested => self.redraw(event_loop),
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        self.gamepads.poll(&mut self.mapper);
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
