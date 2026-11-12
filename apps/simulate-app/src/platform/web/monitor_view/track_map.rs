//! The track map: the route, the simulator's true track and each set's
//! estimated track from above, over the sealed package's imagery in grey.
//! Each set's latest estimate carries a ring of its stated one-sigma, and
//! each terrain-camera fix the filter judged is marked where the camera
//! truly was when the frame was exposed.
//!
//! The progress stream states one horizontal sigma, `sqrt(σN² + σE²)`, and
//! no correlation, so the ring is a circle of that radius, not an ellipse.

use std::collections::BTreeSet;

use sim_link_host::{CameraWorld, MapView, Monitor};
use wasm_bindgen::{Clamped, JsCast, JsValue};
use web_sys::{CanvasRenderingContext2d, Document, Element, HtmlCanvasElement, ImageData};

use super::super::section::append;
use super::series_colour;

/// Shortest and tallest the map's shape may be, height over width.
const ASPECT: (f64, f64) = (0.4, 0.75);
/// Room around everything drawn, of the larger side.
const MARGIN: f64 = 0.08;
/// Extra room taken when the map must grow, so it does not grow every
/// frame while an estimate drifts away.
const GROWTH: f64 = 0.25;
/// Smallest ring drawn, CSS pixels: a sigma of a few metres is under a
/// pixel on a map of the whole route.
const MIN_RING_PX: f64 = 4.0;

/// The true track and position: pale yellow, as the app's map draws them.
const TRUTH: &str = "#ffe066";
const LABEL: &str = "#f2f4f8";
const ROUTE: &str = "#8f98ad";
const FONT: &str = "12px system-ui, sans-serif";

/// What a drawing showed, so an unchanged map is not drawn again.
#[derive(Debug, Clone, PartialEq)]
struct Drawn {
    samples: usize,
    marks: usize,
    hidden: BTreeSet<String>,
    width: f64,
}

/// The map's canvas and the picture under it.
pub struct TrackMap {
    canvas: HtmlCanvasElement,
    key: Option<Element>,
    context: CanvasRenderingContext2d,
    world: Option<CameraWorld>,
    picture: Option<HtmlCanvasElement>,
    view: Option<MapView>,
    aspect: Option<f64>,
    drawn: Option<Drawn>,
}

impl TrackMap {
    /// A map at the end of `parent`, over `world` when there is one.
    pub fn under(
        document: &Document,
        parent: &Element,
        world: Option<CameraWorld>,
    ) -> Option<Self> {
        let canvas: HtmlCanvasElement = append(document, parent, "canvas", "track-map")?
            .dyn_into()
            .ok()?;
        canvas
            .set_attribute(
                "aria-label",
                "The route, the simulator groundtruth's track and each set's estimated track \
                 from above",
            )
            .ok();
        let context = canvas.get_context("2d").ok()??.dyn_into().ok()?;
        let key = append(document, parent, "p", "detail");
        if let Some(key) = &key {
            key.set_text_content(Some(&key_words(world.is_some())));
        }
        Some(Self {
            canvas,
            key,
            context,
            world,
            picture: None,
            view: None,
            aspect: None,
            drawn: None,
        })
    }

    /// Takes the map and its key off the page, for good: the app's map shows the flight.
    pub fn retire(&self) {
        self.canvas.set_attribute("hidden", "").ok();
        if let Some(key) = &self.key {
            key.set_attribute("hidden", "").ok();
        }
    }

    /// Draws `monitor` with the sets `hidden` names left out, when anything
    /// it shows changed.
    pub fn draw(&mut self, monitor: &Monitor, hidden: &BTreeSet<String>) {
        let width = f64::from(self.canvas.client_width()).max(200.0);
        let drawn = Drawn {
            samples: monitor.traces().iter().map(|t| t.points().len()).sum(),
            marks: monitor.camera().marks().len(),
            hidden: hidden.clone(),
            width,
        };
        if self.drawn.as_ref() == Some(&drawn) {
            return;
        }
        // A set shown or hidden refits the map to what is shown, so hiding a
        // set that drifted far brings the route back to size.
        if self.drawn.as_ref().is_some_and(|d| d.hidden != *hidden) {
            self.view = None;
        }
        let Some(view) = self.fit(monitor, hidden, width) else {
            return;
        };
        let height = width * self.aspect.unwrap_or(ASPECT.0);
        let Some(ratio) = web_sys::window().map(|w| w.device_pixel_ratio().max(1.0)) else {
            return;
        };
        self.canvas.set_width((width * ratio) as u32);
        self.canvas.set_height((height * ratio) as u32);
        let context = &self.context;
        context.set_transform(ratio, 0.0, 0.0, ratio, 0.0, 0.0).ok();
        context.set_fill_style_str("#0b0e17");
        context.fill_rect(0.0, 0.0, width, height);
        if let Some(picture) = &self.picture {
            context
                .draw_image_with_html_canvas_element_and_dw_and_dh(picture, 0.0, 0.0, width, height)
                .ok();
        }
        let painter = Painter {
            context,
            view,
            size: (width, height),
        };
        painter.route(monitor);
        painter.tracks(monitor, hidden);
        painter.scale_bar();
        self.drawn = Some(drawn);
    }

    /// The view that holds everything shown, grown and its picture made
    /// again only when something left the current one.
    fn fit(&mut self, monitor: &Monitor, hidden: &BTreeSet<String>, width: f64) -> Option<MapView> {
        // The flight may start off the route; a view fitted before its
        // first sample would have to grow at once.
        if monitor.truth().is_empty() {
            return None;
        }
        let points = monitor
            .waypoints()
            .iter()
            .map(|(_, at)| *at)
            .chain(monitor.truth().iter().map(|(_, at)| *at))
            .chain(
                monitor
                    .traces()
                    .iter()
                    .filter(|t| !hidden.contains(&t.name))
                    .flat_map(|t| t.points().iter().map(|p| p.estimate_deg)),
            );
        // The view must hold the points, not their margin: a track along the
        // route's edge passes it by metres, which must not grow the map.
        let bare = MapView::around(points, 0.0)?;
        let needed = grow(bare, MARGIN);
        let aspect = *self.aspect.get_or_insert_with(|| {
            ((needed.y1 - needed.y0) / (needed.x1 - needed.x0)).clamp(ASPECT.0, ASPECT.1)
        });
        let width_changed = self.drawn.as_ref().is_some_and(|d| d.width != width);
        let view = match self.view {
            Some(view) if view.holds(&bare) => {
                if !width_changed {
                    return Some(view);
                }
                view
            }
            Some(_) => grow(needed, GROWTH).with_aspect(aspect),
            None => needed.with_aspect(aspect),
        };
        self.view = Some(view);
        self.picture = self.picture_of(&view, width, width * aspect);
        Some(view)
    }

    /// The world's imagery under `view` on a canvas of its own, one pixel
    /// per CSS pixel of the map.
    fn picture_of(&self, view: &MapView, width: f64, height: f64) -> Option<HtmlCanvasElement> {
        let world = self.world.as_ref()?;
        let (w, h) = (width as u32, height as u32);
        let rgba = world.overview_rgba(view, w, h);
        let image = ImageData::new_with_u8_clamped_array_and_sh(Clamped(&rgba), w, h).ok()?;
        let document = self.canvas.owner_document()?;
        let picture: HtmlCanvasElement = document.create_element("canvas").ok()?.dyn_into().ok()?;
        picture.set_width(w);
        picture.set_height(h);
        let context: CanvasRenderingContext2d = picture.get_context("2d").ok()??.dyn_into().ok()?;
        context.put_image_data(&image, 0.0, 0.0).ok()?;
        Some(picture)
    }
}

/// `view` grown by `fraction` of its larger side on each edge.
fn grow(view: MapView, fraction: f64) -> MapView {
    let pad = fraction * (view.x1 - view.x0).max(view.y1 - view.y0);
    MapView {
        x0: view.x0 - pad,
        y0: view.y0 - pad,
        x1: view.x1 + pad,
        y1: view.y1 + pad,
    }
}

/// What the map's marks mean.
fn key_words(pictured: bool) -> String {
    let under = if pictured {
        "Over the sealed package's imagery in grey. "
    } else {
        ""
    };
    format!(
        "{under}Yellow: the simulator groundtruth, its track and its position as a crossed \
         circle. \
         Coloured: each set's estimate with its one-sigma ring (at least {MIN_RING_PX} px). \
         Dots and rings: camera frames accepted and rejected. Dashed: the planned route."
    )
}

/// Draws on the map's canvas in its view.
struct Painter<'a> {
    context: &'a CanvasRenderingContext2d,
    view: MapView,
    size: (f64, f64),
}

impl Painter<'_> {
    fn at(&self, point: (f64, f64)) -> (f64, f64) {
        self.view.pixel(point, self.size.0, self.size.1)
    }

    fn polyline(&self, points: impl Iterator<Item = (f64, f64)>, colour: &str, width: f64) {
        let context = self.context;
        context.set_stroke_style_str(colour);
        context.set_line_width(width);
        context.set_line_join("round");
        context.begin_path();
        for (index, point) in points.enumerate() {
            let (x, y) = self.at(point);
            if index == 0 {
                context.move_to(x, y);
            } else {
                context.line_to(x, y);
            }
        }
        context.stroke();
    }

    fn route(&self, monitor: &Monitor) {
        let context = self.context;
        let dash = js_sys::Array::of2(&JsValue::from(6.0), &JsValue::from(4.0));
        context.set_line_dash(&dash).ok();
        self.polyline(monitor.waypoints().iter().map(|(_, at)| *at), ROUTE, 1.0);
        context.set_line_dash(&js_sys::Array::new()).ok();
        context.set_font(FONT);
        context.set_text_align("left");
        context.set_text_baseline("bottom");
        for (ident, at) in monitor.waypoints() {
            let (x, y) = self.at(*at);
            context.set_fill_style_str(ROUTE);
            context.begin_path();
            context.arc(x, y, 3.0, 0.0, std::f64::consts::TAU).ok();
            context.fill();
            context.set_fill_style_str(LABEL);
            context.fill_text(ident, x + 5.0, y - 3.0).ok();
        }
    }

    fn tracks(&self, monitor: &Monitor, hidden: &BTreeSet<String>) {
        self.polyline(monitor.truth().iter().map(|(_, at)| *at), TRUTH, 2.0);
        let metres_per_px = self.view.metres_per_pixel(self.size.0);
        let shown = |index: usize| {
            monitor
                .traces()
                .get(index)
                .is_some_and(|t| !hidden.contains(&t.name))
        };
        for (index, trace) in monitor.traces().iter().enumerate() {
            if !shown(index) {
                continue;
            }
            let colour = series_colour(index);
            self.polyline(trace.points().iter().map(|p| p.estimate_deg), colour, 1.5);
            if let Some(latest) = trace.latest() {
                let (x, y) = self.at(latest.estimate_deg);
                self.dot((x, y), 3.5, colour);
                if let Some(sigma) = latest.sigma_horizontal_m {
                    self.ring((x, y), (sigma / metres_per_px).max(MIN_RING_PX), colour);
                }
            }
        }
        for mark in monitor.camera().marks() {
            if !shown(mark.set) {
                continue;
            }
            let (x, y) = self.at(mark.at_deg);
            let colour = series_colour(mark.set);
            if mark.accepted {
                self.dot((x, y), 2.5, colour);
            } else {
                self.ring((x, y), 3.0, colour);
            }
        }
        if let Some((_, at)) = monitor.truth().last() {
            let (x, y) = self.at(*at);
            self.truth_mark((x, y));
            let context = self.context;
            context.set_font(FONT);
            context.set_text_align("left");
            context.set_text_baseline("top");
            context.set_fill_style_str(TRUTH);
            context
                .fill_text(super::GROUNDTRUTH, x + 12.0, y + 8.0)
                .ok();
        }
    }

    /// The simulator groundtruth: a circle crossed by hairs, dark-cased, as the app's
    /// map marks it.
    fn truth_mark(&self, (x, y): (f64, f64)) {
        let context = self.context;
        for (width, colour) in [(4.0, "#000000"), (2.0, TRUTH)] {
            context.set_stroke_style_str(colour);
            context.set_line_width(width);
            context.begin_path();
            context.arc(x, y, 7.0, 0.0, std::f64::consts::TAU).ok();
            for (dx, dy) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                context.move_to(x + dx * 2.5, y + dy * 2.5);
                context.line_to(x + dx * 12.0, y + dy * 12.0);
            }
            context.stroke();
        }
    }

    fn dot(&self, (x, y): (f64, f64), radius: f64, colour: &str) {
        let context = self.context;
        context.set_fill_style_str(colour);
        context.begin_path();
        context.arc(x, y, radius, 0.0, std::f64::consts::TAU).ok();
        context.fill();
    }

    fn ring(&self, (x, y): (f64, f64), radius: f64, colour: &str) {
        let context = self.context;
        context.set_stroke_style_str(colour);
        context.set_line_width(1.5);
        context.begin_path();
        context.arc(x, y, radius, 0.0, std::f64::consts::TAU).ok();
        context.stroke();
    }

    /// A bar of a round length at the bottom left.
    fn scale_bar(&self) {
        let metres_per_px = self.view.metres_per_pixel(self.size.0);
        let target_m = metres_per_px * self.size.0 / 5.0;
        let Some(metres) = [100.0, 200.0, 500.0, 1e3, 2e3, 5e3, 1e4, 2e4, 5e4, 1e5]
            .into_iter()
            .rev()
            .find(|m| *m <= target_m)
        else {
            return;
        };
        let length = metres / metres_per_px;
        let (x, y) = (12.0, self.size.1 - 12.0);
        let context = self.context;
        context.set_stroke_style_str(LABEL);
        context.set_line_width(2.0);
        context.begin_path();
        context.move_to(x, y);
        context.line_to(x + length, y);
        context.stroke();
        context.set_fill_style_str(LABEL);
        context.set_text_align("left");
        context.set_text_baseline("bottom");
        let words = if metres >= 1000.0 {
            format!("{} km", metres / 1000.0)
        } else {
            format!("{metres} m")
        };
        context.fill_text(&words, x, y - 4.0).ok();
    }
}
