//! Where the app is shown in the browser: the whole page, or a view inside
//! a link's page beside its monitor. A view inside the page must leave the
//! page alone: its start, its title and its failures stay in the view, so
//! the flight and the monitor go on whatever becomes of the renderer.

use sim_render::{PanelSurface, Renderer};
use tracing::warn;
use web_sys::{Element, HtmlCanvasElement};
use winit::platform::web::WindowAttributesExtWebSys;
use winit::window::{Window, WindowAttributes};

use super::progress_surface::ProgressSurface;
use super::section::write_live;
use crate::app::page_panels::{PagePanel, PagePanels};

/// A view of the app inside a page: the canvas it draws on and the line
/// under it that says what it shows, or why it shows nothing.
#[derive(Debug, Clone)]
pub struct PageView {
    /// The canvas the app draws on.
    pub canvas: HtmlCanvasElement,
    /// The status lines under the canvas.
    pub line: Element,
    /// Where the view states how it runs: the maps' device tier and why.
    pub diagnostics: Element,
    /// The canvases the PFD and the HSI are drawn on beside the view, when the page has
    /// room for them.
    pub panels: Option<[HtmlCanvasElement; 2]>,
    /// The page's own map canvas, which shows the app's EFB map expanded.
    pub map_canvas: Option<HtmlCanvasElement>,
}

/// Where the app is shown.
#[derive(Debug, Clone)]
pub enum Host {
    /// The whole page: winit appends a canvas the page's CSS sizes to the
    /// viewport, and the progress surface steps aside once it draws.
    Page,
    /// A view inside a link's page.
    View(PageView),
}

impl Host {
    /// The app has its first frame to draw.
    pub fn startup_done(&self) {
        match self {
            Self::Page => ProgressSurface::of_page().dismiss(),
            Self::View(_) => {}
        }
    }

    /// Puts a failure where a visitor without the console sees it: on the
    /// page, or on the view's own line.
    pub fn report_failure(&self, what: &str, err: &dyn std::error::Error) {
        let mut detail = err.to_string();
        let mut source = err.source();
        while let Some(cause) = source {
            detail.push_str("; ");
            detail.push_str(&cause.to_string());
            source = cause.source();
        }
        match self {
            Self::Page => {
                ProgressSurface::of_page().fail(&format!("Simulate stopped: {what}"), &detail);
            }
            Self::View(view) => {
                if let Some(document) = view.line.owner_document() {
                    let lines = ["No world view".to_owned(), what.to_owned(), detail];
                    write_live(&document, &view.line, &lines);
                }
                view.canvas.set_attribute("hidden", "").ok();
            }
        }
    }

    /// States how the app runs in the view's diagnostics: `line`, with `detail` as its
    /// tooltip; the whole page has the log.
    pub fn diagnostics(&self, line: &str, detail: &str) {
        if let Self::View(view) = self {
            view.diagnostics.set_text_content(Some(line));
            view.diagnostics.set_attribute("title", detail).ok();
        }
    }

    /// The PFD and the HSI on the view's panel canvases, sharing `renderer`'s device; `None`
    /// docks them over the view, as on the whole page.
    pub fn page_panels(&self, renderer: &Renderer) -> Option<PagePanels> {
        let Self::View(view) = self else {
            return None;
        };
        let [pfd, hsi] = view.panels.clone()?;
        let panel = |canvas: HtmlCanvasElement| {
            let [width, height] = laid_out_px(&canvas);
            let surface = renderer
                .panel_surface(wgpu::SurfaceTarget::Canvas(canvas.clone()), width, height)
                .inspect_err(|err| warn!(error = %err, "instrument panel unavailable; docking"))
                .ok()?;
            Some(PagePanel::new(
                surface,
                Box::new(move || laid_out_px(&canvas)),
            ))
        };
        match (panel(pfd.clone()), panel(hsi.clone())) {
            (Some(pfd), Some(hsi)) => Some(PagePanels::new(pfd, hsi)),
            _ => {
                for canvas in [pfd, hsi] {
                    canvas.set_attribute("hidden", "").ok();
                }
                None
            }
        }
    }

    /// Shows `title` on the page's tab; a view leaves the page's title alone and shows
    /// its status lines instead.
    pub fn set_title(&self, _window: &Window, title: &str) {
        if let Self::Page = self
            && let Some(document) = web_sys::window().and_then(|window| window.document())
        {
            document.set_title(title);
        }
    }

    /// Whether the app is a view with status lines under it.
    pub const fn shows_status(&self) -> bool {
        matches!(self, Self::View(_))
    }

    /// Shows `lines` under the view, each line in its own place.
    pub fn set_status(&self, lines: &[String]) {
        if let Self::View(view) = self
            && let Some(document) = view.line.owner_document()
        {
            write_live(&document, &view.line, lines);
        }
    }

    /// The surface of the view's page map canvas, sharing `renderer`'s device, sized to
    /// the canvas's drawing buffer, which the page keeps to its width on screen.
    pub fn page_map_surface(&self, renderer: &Renderer) -> Option<PanelSurface> {
        let Self::View(view) = self else {
            return None;
        };
        let canvas = view.map_canvas.clone()?;
        let (width, height) = (canvas.width().max(1), canvas.height().max(1));
        renderer
            .panel_surface(wgpu::SurfaceTarget::Canvas(canvas), width, height)
            .inspect_err(|err| warn!(error = %err, "the page map has no surface"))
            .ok()
    }

    /// Window settings: a canvas winit appends to the page, or the view's.
    pub fn window_attributes(&self) -> WindowAttributes {
        let attributes = Window::default_attributes().with_title("Simulate");
        match self {
            Self::Page => attributes.with_append(true),
            Self::View(view) => attributes
                .with_canvas(Some(view.canvas.clone()))
                .with_append(false),
        }
    }
}

/// `canvas`'s laid-out size in physical pixels, with its drawing buffer set to match.
fn laid_out_px(canvas: &HtmlCanvasElement) -> [u32; 2] {
    let ratio = web_sys::window().map_or(1.0, |window| window.device_pixel_ratio());
    let px = |css: i32| ((f64::from(css) * ratio).round() as u32).max(1);
    let (width, height) = (px(canvas.client_width()), px(canvas.client_height()));
    if canvas.width() != width {
        canvas.set_width(width);
    }
    if canvas.height() != height {
        canvas.set_height(height);
    }
    [width, height]
}
