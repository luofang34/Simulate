use std::collections::HashMap;
use std::collections::hash_map::Entry;
use std::sync::Arc;

use sim_apollo::CockpitInstrumentSnapshot;
use sim_core::SimInstant;
use sim_input::{InputMapper, ProfileLibrary};
use sim_render::{FrameOutcome, RenderError, Renderer, SceneLighting};
use sim_scenario::{ScenarioProfile, ScenarioWorld, WorldScene};
use tracing::{error, info};
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoopProxy};
use winit::window::{Window, WindowId};

use crate::flight::Flight;
use crate::gamepad::Gamepads;
use crate::keyboard::{Keyboard, scenario_key};
use crate::platform;
use crate::telemetry;

/// Seconds between window title updates: often enough to read, rarely enough
/// not to spend a browser's time rewriting the page title.
const TITLE_INTERVAL_S: f64 = 0.25;

/// Events posted back to the event loop from asynchronous work.
pub enum AppEvent {
    /// GPU setup finished (or failed).
    RendererReady(Result<Renderer, RenderError>),
}

/// The application: window, renderer, input, and the simulated flight.
pub struct FlightApp {
    proxy: EventLoopProxy<AppEvent>,
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
    flight: Flight,
    mapper: InputMapper,
    keyboard: Keyboard,
    gamepads: Gamepads,
    scenes: HashMap<ScenarioWorld, WorldScene>,
    drawn_world: ScenarioWorld,
    cockpit: CockpitInstrumentSnapshot,
    title_age_s: f64,
    failed: bool,
}

impl FlightApp {
    pub fn new(proxy: EventLoopProxy<AppEvent>) -> Self {
        Self {
            proxy,
            window: None,
            renderer: None,
            flight: Flight::new(),
            mapper: InputMapper::new(ProfileLibrary::default()),
            keyboard: Keyboard::default(),
            gamepads: Gamepads::new(),
            scenes: HashMap::new(),
            drawn_world: ScenarioWorld::EarthGlobe,
            cockpit: CockpitInstrumentSnapshot::blank(SimInstant::START),
            title_age_s: TITLE_INTERVAL_S,
            failed: false,
        }
    }

    /// Whether the app stopped because of an error.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn failed(&self) -> bool {
        self.failed
    }

    fn fail(&mut self, event_loop: &ActiveEventLoop, what: &str, err: &dyn std::error::Error) {
        error!(error = %err, source = ?err.source(), "{what}");
        self.failed = true;
        event_loop.exit();
    }

    fn select(&mut self, event_loop: &ActiveEventLoop, profile: Option<ScenarioProfile>) {
        let name = profile.map_or("free flight", ScenarioProfile::title);
        info!(scenario = name, "selected");
        if let Err(err) = self.flight.select(profile) {
            self.fail(event_loop, "the scenario could not start", &err);
        }
    }

    /// Puts the current world's terrain on the renderer if it is not already
    /// there. Scenes are kept once built, so going back and forth is instant.
    fn show_world(&mut self, event_loop: &ActiveEventLoop) {
        let world = self.flight.world();
        let Some(renderer) = self.renderer.as_mut() else {
            return;
        };
        if world == self.drawn_world {
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
        self.drawn_world = world;
    }

    /// Reports the panel, and refreshes the window title now and then.
    fn observe_cockpit(&mut self, elapsed_s: f64) {
        let cockpit = self.flight.cockpit();
        telemetry::log_changes(&self.cockpit, &cockpit);
        self.cockpit = cockpit;
        self.title_age_s += elapsed_s;
        if self.title_age_s >= TITLE_INTERVAL_S
            && let Some(window) = &self.window
        {
            self.title_age_s = 0.0;
            platform::set_title(window, &telemetry::title(self.flight.profile(), &cockpit));
        }
    }

    fn redraw(&mut self, event_loop: &ActiveEventLoop) {
        let controls = self.mapper.controls();
        match self.flight.advance_to_now(controls) {
            Ok(elapsed_s) => {
                self.keyboard.update(elapsed_s, &mut self.mapper);
                self.observe_cockpit(elapsed_s);
            }
            Err(err) => return self.fail(event_loop, "the flight model stopped", &err),
        }
        self.show_world(event_loop);
        let camera = match self.flight.camera() {
            Ok(camera) => camera,
            Err(err) => return self.fail(event_loop, "the view could not be placed", &err),
        };
        let Some(renderer) = self.renderer.as_mut() else {
            return;
        };
        let lighting = SceneLighting {
            sun_direction_ecef: self.drawn_world.sun_direction_ecef(),
        };
        match renderer.render(&camera, &lighting) {
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
                info!("renderer ready");
                self.renderer = Some(renderer);
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
                }
                self.keyboard.handle(&event, &mut self.mapper);
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
