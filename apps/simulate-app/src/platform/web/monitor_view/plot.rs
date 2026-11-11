//! The error plot: each shown set's horizontal error against the truth over
//! route time, on a logarithmic metre scale, with the waypoints sequenced
//! so far marked and each line labelled at its end.

use sim_link_host::{Monitor, SetTrace, clock_words};
use sim_navigate::trial::EventKind;
use wasm_bindgen::{JsCast, JsValue};
use web_sys::{CanvasRenderingContext2d, Document, Element, HtmlCanvasElement};

use super::super::section::append;
use super::series_colour;

/// Height of the plot, CSS pixels.
const HEIGHT_PX: f64 = 260.0;
/// Room for the metre labels at the left, CSS pixels.
const LEFT_PX: f64 = 52.0;
/// Room for the line labels at the right, CSS pixels.
const RIGHT_PX: f64 = 128.0;
/// Room for the waypoint labels at the top, CSS pixels.
const TOP_PX: f64 = 22.0;
/// Room for the time labels at the bottom, CSS pixels.
const BOTTOM_PX: f64 = 24.0;
/// Closest two line labels may sit, CSS pixels.
const LABEL_GAP_PX: f64 = 14.0;

const GRID: &str = "#1b2030";
const AXIS_TEXT: &str = "#8f98ad";
const TEXT: &str = "#d7dbe6";
const FONT: &str = "12px system-ui, sans-serif";

/// The plot's canvas.
pub struct Plot {
    canvas: HtmlCanvasElement,
    context: CanvasRenderingContext2d,
}

/// Where values land on the canvas.
struct Frame {
    width: f64,
    t1_s: f64,
    log_lo: f64,
    log_hi: f64,
}

impl Frame {
    fn x(&self, t_s: f64) -> f64 {
        LEFT_PX + (t_s / self.t1_s).clamp(0.0, 1.0) * (self.width - LEFT_PX - RIGHT_PX)
    }

    fn y(&self, metres: f64) -> f64 {
        let at = (metres.max(1e-3).log10() - self.log_lo) / (self.log_hi - self.log_lo);
        TOP_PX + (1.0 - at.clamp(0.0, 1.0)) * (HEIGHT_PX - TOP_PX - BOTTOM_PX)
    }

    fn columns(&self) -> usize {
        (self.width - LEFT_PX - RIGHT_PX).max(1.0) as usize
    }
}

impl Plot {
    /// A plot at the end of `parent`.
    pub fn under(document: &Document, parent: &Element) -> Option<Self> {
        let canvas: HtmlCanvasElement = append(document, parent, "canvas", "plot")?
            .dyn_into()
            .ok()?;
        canvas
            .set_attribute(
                "aria-label",
                "Horizontal error of each set against the truth over route time",
            )
            .ok();
        let context = canvas.get_context("2d").ok()??.dyn_into().ok()?;
        Some(Self { canvas, context })
    }

    /// Draws `monitor`, with the sets `shown` says yes to.
    pub fn draw(&self, monitor: &Monitor, shown: &dyn Fn(&str) -> bool) {
        let Some(frame) = self.fit(monitor, shown) else {
            return;
        };
        let context = &self.context;
        context.clear_rect(0.0, 0.0, frame.width, HEIGHT_PX);
        context.set_font(FONT);
        self.grid(&frame);
        self.waypoints(&frame, monitor);
        let mut labels = Vec::new();
        for (index, trace) in monitor.traces().iter().enumerate() {
            if shown(&trace.name) {
                self.line(&frame, trace, series_colour(index));
                if let Some(latest) = trace.latest() {
                    labels.push((frame.y(latest.horizontal_error_m), trace.name.as_str()));
                }
            }
        }
        self.labels(&frame, labels);
    }

    /// Sizes the canvas to its box and the scales to what is shown.
    fn fit(&self, monitor: &Monitor, shown: &dyn Fn(&str) -> bool) -> Option<Frame> {
        let window = web_sys::window()?;
        let ratio = window.device_pixel_ratio().max(1.0);
        let width = f64::from(self.canvas.client_width()).max(LEFT_PX + RIGHT_PX + 50.0);
        self.canvas.set_width((width * ratio) as u32);
        self.canvas.set_height((HEIGHT_PX * ratio) as u32);
        self.context
            .set_transform(ratio, 0.0, 0.0, ratio, 0.0, 0.0)
            .ok()?;
        let errors = || {
            monitor
                .traces()
                .iter()
                .filter(|trace| shown(&trace.name))
                .flat_map(|trace| trace.points().iter().map(|p| p.horizontal_error_m))
                .filter(|metres| metres.is_finite() && *metres > 0.0)
        };
        let largest = errors().fold(10.0_f64, f64::max);
        let smallest = errors().fold(1.0_f64, f64::min);
        let t1_s = monitor
            .planned_duration_s()
            .unwrap_or(0.0)
            .max(monitor.sim_time_s())
            .max(60.0);
        Some(Frame {
            width,
            t1_s,
            log_lo: smallest.log10().floor().max(-1.0),
            log_hi: (largest * 1.2).log10().ceil(),
        })
    }

    fn grid(&self, frame: &Frame) {
        let context = &self.context;
        context.set_line_width(1.0);
        context.set_stroke_style_str(GRID);
        context.set_fill_style_str(AXIS_TEXT);
        context.set_text_align("right");
        context.set_text_baseline("middle");
        let mut decade = frame.log_lo as i32;
        while f64::from(decade) <= frame.log_hi {
            let metres = 10_f64.powi(decade);
            let y = frame.y(metres);
            context.begin_path();
            context.move_to(LEFT_PX, y);
            context.line_to(frame.width - RIGHT_PX, y);
            context.stroke();
            let label = if metres >= 1000.0 {
                format!("{} km", metres / 1000.0)
            } else {
                format!("{metres} m")
            };
            context.fill_text(&label, LEFT_PX - 6.0, y).ok();
            decade = decade.wrapping_add(1);
        }
        context.set_text_align("center");
        context.set_text_baseline("top");
        let step = time_step(frame.t1_s);
        let mut t_s = 0.0;
        while t_s <= frame.t1_s {
            let label = clock_words(t_s);
            context
                .fill_text(&label, frame.x(t_s), HEIGHT_PX - BOTTOM_PX + 6.0)
                .ok();
            t_s += step;
        }
    }

    fn waypoints(&self, frame: &Frame, monitor: &Monitor) {
        let context = &self.context;
        context.set_stroke_style_str(GRID);
        context.set_fill_style_str(AXIS_TEXT);
        context.set_text_align("left");
        context.set_text_baseline("bottom");
        let dash = js_sys::Array::of2(&JsValue::from(3.0), &JsValue::from(3.0));
        context.set_line_dash(&dash).ok();
        for event in monitor.events() {
            if let EventKind::WaypointSequenced { ident, .. } = &event.kind {
                let x = frame.x(event.t_s);
                context.begin_path();
                context.move_to(x, TOP_PX);
                context.line_to(x, HEIGHT_PX - BOTTOM_PX);
                context.stroke();
                context.fill_text(ident, x + 3.0, TOP_PX - 3.0).ok();
            }
        }
        context.set_line_dash(&js_sys::Array::new()).ok();
    }

    /// One set's line through the largest error of each column, broken
    /// where a column has no sample.
    fn line(&self, frame: &Frame, trace: &SetTrace, colour: &str) {
        let context = &self.context;
        context.set_stroke_style_str(colour);
        context.set_line_width(2.0);
        context.set_line_join("round");
        context.begin_path();
        let count = frame.columns();
        let mut drawing = false;
        for (index, column) in trace.columns(0.0, frame.t1_s, count).iter().enumerate() {
            let Some(column) = column else {
                drawing = false;
                continue;
            };
            let t_s = (index as f64 + 0.5) / count as f64 * frame.t1_s;
            let (x, y) = (frame.x(t_s), frame.y(column.max_m));
            if drawing {
                context.line_to(x, y);
            } else {
                context.move_to(x, y);
                drawing = true;
            }
        }
        context.stroke();
    }

    /// Each shown line's name at the right edge, at its latest error,
    /// pushed apart where they would overlap.
    fn labels(&self, frame: &Frame, mut labels: Vec<(f64, &str)>) {
        labels.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut floor = f64::NEG_INFINITY;
        let context = &self.context;
        context.set_fill_style_str(TEXT);
        context.set_text_align("left");
        context.set_text_baseline("middle");
        for (y, name) in labels {
            let y = y.max(floor);
            floor = y + LABEL_GAP_PX;
            context
                .fill_text(name, frame.width - RIGHT_PX + 8.0, y)
                .ok();
        }
    }
}

/// A round time between labels: about six labels across the span.
fn time_step(span_s: f64) -> f64 {
    const STEPS: [f64; 8] = [30.0, 60.0, 120.0, 300.0, 600.0, 900.0, 1800.0, 3600.0];
    STEPS
        .into_iter()
        .find(|step| span_s / step <= 7.0)
        .unwrap_or(7200.0)
}
