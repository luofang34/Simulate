use std::sync::Arc;

use tracing::{Level, error};
use tracing_wasm::WASMLayerConfigBuilder;
use winit::event_loop::{EventLoop, EventLoopProxy};
use winit::platform::web::{EventLoopExtWebSys, WindowAttributesExtWebSys};
use winit::window::{Window, WindowAttributes};

use crate::app::{AppEvent, FlightApp, GpuReady};

mod link_start;
mod package_run;
mod progress_surface;
mod section;

use progress_surface::ProgressSurface;

/// Starts the app; the browser drives the event loop from here on.
pub fn run() {
    // A panic ends the wasm instance; the console alone would leave a
    // visitor looking at a frozen page.
    std::panic::set_hook(Box::new(|info| {
        console_error_panic_hook::hook(info);
        ProgressSurface::of_page().fail("Simulate stopped: internal error", &info.to_string());
    }));
    // Every event reaches the console, so the default of all levels would
    // write out each of JSBSim's per-step trace events.
    tracing_wasm::set_as_global_default_with_config(
        WASMLayerConfigBuilder::new()
            .set_max_level(Level::INFO)
            .build(),
    );
    let Err(surface) = link_start::show_link(ProgressSurface::of_page()) else {
        return;
    };
    surface.phase("Starting the simulator");
    let event_loop = match EventLoop::<AppEvent>::with_user_event().build() {
        Ok(event_loop) => event_loop,
        Err(err) => {
            error!(%err, "creating the event loop failed");
            report_failure("creating the event loop failed", &err);
            return;
        }
    };
    let app = match FlightApp::new(event_loop.create_proxy()) {
        Ok(app) => app,
        Err(err) => {
            error!(%err, "starting the flight simulation failed");
            report_failure("starting the flight simulation failed", &err);
            return;
        }
    };
    event_loop.spawn_app(app);
}

/// Removes the page's progress surface: the app has its first frame to draw.
pub fn startup_done() {
    ProgressSurface::of_page().dismiss();
}

/// Puts a failure on the page, where a visitor without the console sees it.
pub fn report_failure(what: &str, err: &dyn std::error::Error) {
    let mut detail = err.to_string();
    let mut source = err.source();
    while let Some(cause) = source {
        detail.push('\n');
        detail.push_str(&cause.to_string());
        source = cause.source();
    }
    ProgressSurface::of_page().fail(&format!("Simulate stopped: {what}"), &detail);
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
        let gpu = GpuReady::new(window, size.width, size.height).await;
        if proxy
            .send_event(AppEvent::RendererReady(Box::new(gpu)))
            .is_err()
        {
            error!("event loop closed before the renderer was ready");
        }
    });
}
