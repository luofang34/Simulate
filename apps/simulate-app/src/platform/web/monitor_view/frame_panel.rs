//! The terrain camera's latest frame as the flight received it, captioned
//! with what each visual set made of it. The page renders the frame itself,
//! so showing it costs a copy of bytes already in hand.

use sim_link_host::{Monitor, PlanLine, clock_words, outcome_words};
use sim_navigate::NavMeans;
use wasm_bindgen::{Clamped, JsCast};
use web_sys::{CanvasRenderingContext2d, Document, Element, HtmlCanvasElement, ImageData};

use super::super::section::{append, write_lines};

/// What the panel says the frame is.
const ABOUT: &str = "What the terrain camera saw: rendered by this page from the sealed \
    package and handed to the navigators as the camera's grey image.";

/// The frame panel.
pub struct FramePanel {
    document: Document,
    canvas: HtmlCanvasElement,
    context: CanvasRenderingContext2d,
    caption: Element,
    shown: Option<(u64, Vec<PlanLine>)>,
}

impl FramePanel {
    /// A panel at the end of `parent`.
    pub fn under(document: &Document, parent: &Element) -> Option<Self> {
        let panel = append(document, parent, "div", "frame-panel")?;
        let canvas: HtmlCanvasElement = append(document, &panel, "canvas", "frame")?
            .dyn_into()
            .ok()?;
        canvas
            .set_attribute("aria-label", "The terrain camera's latest frame")
            .ok();
        let context = canvas.get_context("2d").ok()??.dyn_into().ok()?;
        let side = append(document, &panel, "div", "")?;
        if let Some(about) = append(document, &side, "p", "detail") {
            about.set_text_content(Some(ABOUT));
        }
        let caption = append(document, &side, "dl", "lines live-rows")?;
        panel.set_attribute("hidden", "").ok();
        Some(Self {
            document: document.clone(),
            canvas,
            context,
            caption,
            shown: None,
        })
    }

    /// Shows the monitor's latest frame and its outcomes when either
    /// changed since the last call.
    pub fn draw(&mut self, monitor: &Monitor) {
        let camera = monitor.camera();
        let Some(shot) = camera.latest() else {
            return;
        };
        let mut lines = vec![PlanLine {
            label: format!("Frame {}", shot.index),
            text: format!("exposed at {}", clock_words(shot.mid_s)),
        }];
        lines.extend(
            monitor
                .traces()
                .iter()
                .filter(|trace| trace.means.contains(&NavMeans::Visual))
                .map(|trace| PlanLine {
                    label: trace.name.clone(),
                    text: camera.outcomes(&trace.name).map_or_else(
                        || "no outcome yet".to_owned(),
                        |keys| {
                            keys.iter()
                                .map(|key| outcome_words(key))
                                .collect::<Vec<_>>()
                                .join(" · ")
                        },
                    ),
                }),
        );
        let new_frame = self.shown.as_ref().is_none_or(|(i, _)| *i != shot.index);
        if !new_frame && self.shown.as_ref().is_some_and(|(_, l)| *l == lines) {
            return;
        }
        if new_frame {
            self.paint(shot.size, &shot.luma);
        }
        write_lines(&self.document, &self.caption, &lines);
        self.shown = Some((shot.index, lines));
    }

    /// Paints `luma`, `size` grey pixels, onto the canvas at its own size.
    fn paint(&self, (width, height): (u32, u32), luma: &[u8]) {
        let rgba: Vec<u8> = luma
            .iter()
            .flat_map(|&grey| [grey, grey, grey, 255])
            .collect();
        let Ok(image) = ImageData::new_with_u8_clamped_array_and_sh(Clamped(&rgba), width, height)
        else {
            return;
        };
        self.canvas.set_width(width);
        self.canvas.set_height(height);
        self.context.put_image_data(&image, 0.0, 0.0).ok();
        if let Some(panel) = self.canvas.parent_element() {
            panel.remove_attribute("hidden").ok();
        }
    }
}
