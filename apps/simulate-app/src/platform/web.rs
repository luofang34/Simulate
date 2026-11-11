use std::sync::Arc;

use tracing::{Level, error};
use tracing_wasm::WASMLayerConfigBuilder;
use winit::event_loop::{EventLoop, EventLoopProxy};
use winit::platform::web::EventLoopExtWebSys;
use winit::window::Window;

use crate::app::{AppEvent, FlightApp, GpuReady};
use crate::vehicles::PageFeed;

mod chain_start;
mod flight_run;
mod host;
mod link_start;
mod monitor_view;
mod package_run;
mod progress_surface;
mod section;

pub use host::{Host, PageView};
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
            Host::Page.report_failure("creating the event loop failed", &err);
            return;
        }
    };
    let app = match FlightApp::new(event_loop.create_proxy(), Host::Page) {
        Ok(app) => app,
        Err(err) => {
            error!(%err, "starting the flight simulation failed");
            Host::Page.report_failure("starting the flight simulation failed", &err);
            return;
        }
    };
    event_loop.spawn_app(app);
}

/// Starts the app in `view` inside a link's page, drawing the flight the page
/// steps and publishes on `feed`. A failure to start stays on the view's line.
pub fn show_world(view: PageView, feed: PageFeed) {
    let host = Host::View(view);
    let event_loop = match EventLoop::<AppEvent>::with_user_event().build() {
        Ok(event_loop) => event_loop,
        Err(err) => {
            error!(%err, "creating the world view's event loop failed");
            host.report_failure("creating the event loop failed", &err);
            return;
        }
    };
    let app = FlightApp::new(event_loop.create_proxy(), host.clone()).and_then(|mut app| {
        app.show_page_mission(feed)?;
        Ok(app)
    });
    match app {
        Ok(app) => event_loop.spawn_app(app),
        Err(err) => {
            error!(%err, "starting the world view failed");
            host.report_failure("starting the simulation failed", &err);
        }
    }
}

/// Starts creating the renderer for `window`, and the page map's surface when `host`
/// has a map canvas; they are posted back to the event loop when the browser has
/// granted a GPU device.
pub fn spawn_renderer(window: Arc<Window>, proxy: EventLoopProxy<AppEvent>, host: &Host) {
    let size = window.inner_size();
    let map_canvas = host.map_canvas();
    wasm_bindgen_futures::spawn_local(async move {
        let mut gpu = GpuReady::new(window, size.width, size.height).await;
        if let (Ok(ready), Some(canvas)) = (gpu.as_mut(), map_canvas) {
            ready.page_map = page_map_presenter(&ready.renderer, canvas).await;
        }
        if proxy
            .send_event(AppEvent::RendererReady(Box::new(gpu)))
            .is_err()
        {
            error!("event loop closed before the renderer was ready");
        }
    });
}

/// The page map's surface on `canvas`, on the renderer's device. Without one the page
/// keeps its own drawing of the map.
async fn page_map_presenter(
    renderer: &sim_render::Renderer,
    canvas: web_sys::HtmlCanvasElement,
) -> Option<sim_render::CanvasPresenter> {
    let (width, height) = (canvas.width().max(1), canvas.height().max(1));
    let target = wgpu::SurfaceTarget::Canvas(canvas);
    sim_render::CanvasPresenter::new(renderer, target, width, height)
        .await
        .inspect_err(|err| tracing::warn!(error = %err, "the page map has no surface"))
        .ok()
}
