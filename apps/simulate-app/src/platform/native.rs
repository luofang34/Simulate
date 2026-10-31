use std::process::ExitCode;
use std::sync::Arc;

use sim_render::Renderer;
use tracing::error;
use winit::event_loop::{EventLoop, EventLoopProxy};
use winit::window::{Window, WindowAttributes};

use crate::app::{AppEvent, FlightApp};

/// Runs the app until its window closes.
pub fn run() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
                tracing_subscriber::EnvFilter::new("info,wgpu_core=warn,wgpu_hal=warn")
            }),
        )
        .init();
    let event_loop = match EventLoop::<AppEvent>::with_user_event().build() {
        Ok(event_loop) => event_loop,
        Err(err) => {
            error!(%err, "creating the event loop failed");
            return ExitCode::FAILURE;
        }
    };
    let mut app = FlightApp::new(event_loop.create_proxy());
    match event_loop.run_app(&mut app) {
        Ok(()) if app.failed() => ExitCode::FAILURE,
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            error!(%err, "the event loop stopped with an error");
            ExitCode::FAILURE
        }
    }
}

/// Shows `title` on the window.
pub fn set_title(window: &Window, title: &str) {
    window.set_title(title);
}

/// Window settings for desktop.
pub fn window_attributes() -> WindowAttributes {
    Window::default_attributes()
        .with_title("Simulate")
        .with_inner_size(winit::dpi::LogicalSize::new(1280.0, 720.0))
}

/// Creates the renderer for `window` and posts it back to the event loop.
pub fn spawn_renderer(window: Arc<Window>, proxy: EventLoopProxy<AppEvent>) {
    let size = window.inner_size();
    let renderer = pollster::block_on(Renderer::new(window, size.width, size.height));
    if proxy.send_event(AppEvent::RendererReady(renderer)).is_err() {
        error!("event loop closed before the renderer was ready");
    }
}
