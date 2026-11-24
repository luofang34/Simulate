use std::sync::Arc;

use tracing::{Level, error};
use tracing_subscriber::Layer;
use tracing_subscriber::filter::filter_fn;
use tracing_subscriber::layer::SubscriberExt;
use tracing_wasm::{WASMLayer, WASMLayerConfigBuilder};
use wasm_bindgen::{JsCast, JsValue, closure::Closure};
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
mod route_editor;
mod section;
mod touch_controls;

pub use host::{Host, PageView};
use progress_surface::ProgressSurface;
pub use touch_controls::TouchCockpit;

/// Starts the app; the browser drives the event loop from here on.
pub fn run() {
    // A panic ends the wasm instance; the console alone would leave a
    // visitor looking at a frozen page.
    std::panic::set_hook(Box::new(|info| {
        console_error_panic_hook::hook(info);
        ProgressSurface::of_page().fail("Simulate stopped: internal error", &info.to_string());
    }));
    // Every event reaches the console, so the default of all levels would
    // write out each of JSBSim's per-step trace events. Nothing goes to the
    // page's performance timeline, which keeps every entry: spans stay out,
    // since the layer marks each one entered and the maps enter thousands a
    // second, and events are not marked.
    let console = WASMLayer::new(
        WASMLayerConfigBuilder::new()
            .set_max_level(Level::INFO)
            .set_report_logs_in_timings(false)
            .build(),
    )
    .with_filter(filter_fn(|meta| {
        crate::telemetry::console_shows(meta.is_event(), *meta.level())
    }));
    // Only a page that already set one is refused, and it then logs through that one.
    tracing::subscriber::set_global_default(tracing_subscriber::registry().with(console)).ok();
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
    let app = FlightApp::new_page_mission(event_loop.create_proxy(), host.clone(), feed);
    match app {
        Ok(app) => event_loop.spawn_app(app),
        Err(err) => {
            error!(%err, "starting the world view failed");
            host.report_failure("starting the simulation failed", &err);
        }
    }
}

/// Whether the current browser should use handheld budgets and touch controls.
#[must_use]
pub fn is_handheld_browser() -> bool {
    crate::map_rig::device_tier::browser_is_handheld()
}

/// Maximum browser boot budget spent waiting for OpenFreeMap TileJSON alongside GPU init.
const TILE_JSON_PREFETCH_TIMEOUT_MS: i32 = 2_500;

fn prefetch_openfreemap_tile_json_promise() -> js_sys::Promise {
    let fetch = wasm_bindgen_futures::future_to_promise(async {
        let _ok =
            sim_maplibre::prefetch_tile_json(sim_maplibre::OPENFREEMAP_PLANET_TILEJSON_URL).await;
        Ok(JsValue::UNDEFINED)
    });
    let timeout = js_sys::Promise::new(&mut |resolve, _reject| {
        if let Some(window) = web_sys::window() {
            let resolve_timeout = resolve.clone();
            let cb = Closure::once_into_js(move || {
                resolve_timeout.call0(&JsValue::UNDEFINED).ok();
            });
            if window
                .set_timeout_with_callback_and_timeout_and_arguments_0(
                    cb.unchecked_ref(),
                    TILE_JSON_PREFETCH_TIMEOUT_MS,
                )
                .is_err()
            {
                resolve.call0(&JsValue::UNDEFINED).ok();
            }
        } else {
            resolve.call0(&JsValue::UNDEFINED).ok();
        }
    });
    js_sys::Promise::race(&js_sys::Array::of2(&fetch, &timeout))
}

/// Starts creating the renderer for `window`; it is posted back to the event
/// loop when the browser has granted a GPU device, after `host` has said what follows.
pub fn spawn_renderer(window: Arc<Window>, proxy: EventLoopProxy<AppEvent>, host: &Host) {
    let size = window.inner_size();
    let ([width, height], _) = crate::app::surface_scale::capped_surface_size(
        [size.width, size.height],
        window.scale_factor(),
        is_handheld_browser(),
    );
    let host = host.clone();
    wasm_bindgen_futures::spawn_local(async move {
        let tile_json_race = prefetch_openfreemap_tile_json_promise();
        let gpu = GpuReady::new(window, width, height).await;
        wasm_bindgen_futures::JsFuture::from(tile_json_race)
            .await
            .ok();
        // Building the cockpit holds the page's only thread until the first frame, so
        // the words for it have to be on screen before the renderer is handed over.
        if gpu.is_ok() && host.startup_phase("Loading the aircraft and maps") {
            progress_surface::painted().await;
        }
        if proxy
            .send_event(AppEvent::RendererReady(Box::new(gpu)))
            .is_err()
        {
            error!("event loop closed before the renderer was ready");
        }
    });
}
