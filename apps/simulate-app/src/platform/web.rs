use std::sync::Arc;

use sim_render::Renderer;
use tracing::{Level, error};
use tracing_wasm::WASMLayerConfigBuilder;
use winit::event_loop::{EventLoop, EventLoopProxy};
use winit::platform::web::{EventLoopExtWebSys, WindowAttributesExtWebSys};
use winit::window::{Window, WindowAttributes};

use crate::app::{AppEvent, FlightApp};

/// Starts the app; the browser drives the event loop from here on.
pub fn run() {
    console_error_panic_hook::set_once();
    // Every event reaches the console, so the default of all levels would
    // write out each of JSBSim's per-step trace events.
    tracing_wasm::set_as_global_default_with_config(
        WASMLayerConfigBuilder::new()
            .set_max_level(Level::INFO)
            .build(),
    );
    let event_loop = match EventLoop::<AppEvent>::with_user_event().build() {
        Ok(event_loop) => event_loop,
        Err(err) => {
            error!(%err, "creating the event loop failed");
            return;
        }
    };
    let app = FlightApp::new(event_loop.create_proxy());
    event_loop.spawn_app(app);
}

/// Shows `title` on the page's tab: winit's own title only labels the canvas.
pub fn set_title(_window: &Window, title: &str) {
    if let Some(document) = web_sys::window().and_then(|window| window.document()) {
        document.set_title(title);
    }
}

/// Window settings for the browser: winit creates the canvas and appends it
/// to the page body; CSS in `index.html` sizes it to the viewport.
pub fn window_attributes() -> WindowAttributes {
    Window::default_attributes()
        .with_title("Simulate")
        .with_append(true)
}

/// Starts creating the renderer for `window`; it is posted back to the event
/// loop when the browser has granted a GPU device.
pub fn spawn_renderer(window: Arc<Window>, proxy: EventLoopProxy<AppEvent>) {
    let size = window.inner_size();
    wasm_bindgen_futures::spawn_local(async move {
        let renderer = Renderer::new(window, size.width, size.height).await;
        if proxy.send_event(AppEvent::RendererReady(renderer)).is_err() {
            error!("event loop closed before the renderer was ready");
        }
    });
}
