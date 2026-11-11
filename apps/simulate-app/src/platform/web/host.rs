//! Where the app is shown in the browser: the whole page, or a view inside
//! a link's page beside its monitor. A view inside the page must leave the
//! page alone: its start, its title and its failures stay in the view, so
//! the flight and the monitor go on whatever becomes of the renderer.

use web_sys::{Element, HtmlCanvasElement};
use winit::platform::web::WindowAttributesExtWebSys;
use winit::window::{Window, WindowAttributes};

use super::progress_surface::ProgressSurface;

/// A view of the app inside a page: the canvas it draws on and the line
/// under it that says what it shows, or why it shows nothing.
#[derive(Debug, Clone)]
pub struct PageView {
    /// The canvas the app draws on.
    pub canvas: HtmlCanvasElement,
    /// The line under the canvas.
    pub line: Element,
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
                view.line
                    .set_text_content(Some(&format!("No world view: {what}: {detail}")));
                view.canvas.set_attribute("hidden", "").ok();
            }
        }
    }

    /// Shows `title`: on the page's tab, or on the view's line.
    pub fn set_title(&self, _window: &Window, title: &str) {
        match self {
            Self::Page => {
                if let Some(document) = web_sys::window().and_then(|window| window.document()) {
                    document.set_title(title);
                }
            }
            Self::View(view) => view.line.set_text_content(Some(title)),
        }
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
