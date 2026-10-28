use std::sync::Arc;

use fsg_input::{InputMapper, ProfileLibrary};
use fsg_render::{FrameOutcome, RenderError, Renderer, SceneLighting};
use glam::DVec3;
use tracing::{error, info};
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoopProxy};
use winit::window::{Window, WindowId};

use crate::camera_view::cockpit_camera;
use crate::flight::Flight;
use crate::gamepad::Gamepads;
use crate::keyboard::Keyboard;
use crate::platform;

/// A fixed sun direction until the ephemeris is wired in: roughly mid-morning
/// over the Pacific coast of North America.
const SUN_DIRECTION_ECEF: DVec3 = DVec3::new(-0.2, -0.9, 0.4);

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

    fn redraw(&mut self, event_loop: &ActiveEventLoop) {
        let controls = self.mapper.controls();
        match self.flight.advance_to_now(controls) {
            Ok(elapsed_s) => self.keyboard.update(elapsed_s, &mut self.mapper),
            Err(err) => return self.fail(event_loop, "the flight model stopped", &err),
        }
        let Some(renderer) = self.renderer.as_mut() else {
            return;
        };
        let camera = cockpit_camera(&self.flight.state());
        let lighting = SceneLighting {
            sun_direction_ecef: SUN_DIRECTION_ECEF,
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
                self.keyboard.handle(&event, &mut self.mapper)
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
