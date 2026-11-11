use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;

use tracing::{error, info};
use winit::event_loop::{EventLoop, EventLoopProxy};
use winit::window::{Window, WindowAttributes};

use crate::app::{AppEvent, FlightApp, GpuReady};
use crate::gallery;

/// Runs the app until its window closes (or renders a headless gallery when `--render-gallery <dir>` is passed).
pub fn run() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
                tracing_subscriber::EnvFilter::new("info,wgpu_core=warn,wgpu_hal=warn")
            }),
        )
        .init();
    if let Some(dir) = parse_gallery_arg() {
        return match gallery::render_gallery(&dir) {
            Ok(count) => {
                info!(count, dir = %dir.display(), "rendered headless screenshot gallery");
                ExitCode::SUCCESS
            }
            Err(err) => {
                error!(error = %err, source = ?std::error::Error::source(&err), "gallery render failed");
                ExitCode::FAILURE
            }
        };
    }
    let event_loop = match EventLoop::<AppEvent>::with_user_event().build() {
        Ok(event_loop) => event_loop,
        Err(err) => {
            error!(%err, "creating the event loop failed");
            return ExitCode::FAILURE;
        }
    };
    let mut app = match FlightApp::new(event_loop.create_proxy(), Host) {
        Ok(app) => app,
        Err(err) => {
            error!(%err, "starting the flight simulation failed");
            return ExitCode::FAILURE;
        }
    };
    match event_loop.run_app(&mut app) {
        Ok(()) if app.failed() => ExitCode::FAILURE,
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            error!(%err, "the event loop stopped with an error");
            ExitCode::FAILURE
        }
    }
}

/// Where the app is shown: on desktop, always its own window.
#[derive(Debug, Clone, Copy, Default)]
pub struct Host;

impl Host {
    /// The app has its first frame to draw; the desktop window needs no word of it.
    pub fn startup_done(&self) {}

    /// A failure the log already carries; the desktop has no other place for it.
    pub fn report_failure(&self, _what: &str, _err: &dyn std::error::Error) {}

    /// Shows `title` on the window.
    pub fn set_title(&self, window: &Window, title: &str) {
        window.set_title(title);
    }

    /// Window settings for desktop.
    pub fn window_attributes(&self) -> WindowAttributes {
        Window::default_attributes()
            .with_title("Simulate")
            .with_inner_size(winit::dpi::LogicalSize::new(1280.0, 720.0))
    }
}

/// Creates the renderer for `window` and posts it back to the event loop.
pub fn spawn_renderer(window: Arc<Window>, proxy: EventLoopProxy<AppEvent>) {
    let size = window.inner_size();
    let gpu = pollster::block_on(GpuReady::new(window, size.width, size.height));
    if proxy
        .send_event(AppEvent::RendererReady(Box::new(gpu)))
        .is_err()
    {
        error!("event loop closed before the renderer was ready");
    }
}

fn parse_gallery_arg() -> Option<PathBuf> {
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--render-gallery" {
            return args.next().map(PathBuf::from);
        }
    }
    None
}
