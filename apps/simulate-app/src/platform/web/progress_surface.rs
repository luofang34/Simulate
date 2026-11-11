//! The page's progress surface, as the app drives it. `index.html` shows it
//! and `boot.mjs` fills it while the wasm downloads; the app takes the same
//! elements over for its own start-up and removes them when it draws.

use sim_link_host::PlanLine;
use web_sys::{Document, Element};

use crate::platform::surface_ids::{BAR, DETAIL, FILL, LINES, PHASE, ROOT};

/// The surface's elements. A page without them gives a surface that does
/// nothing, so the app also runs in a bare page.
pub struct ProgressSurface {
    elements: Option<Elements>,
}

struct Elements {
    document: Document,
    root: Element,
    phase: Element,
    bar: Element,
    fill: Element,
    detail: Element,
    lines: Element,
}

impl ProgressSurface {
    /// The surface of the current page.
    pub fn of_page() -> Self {
        let elements = (|| {
            let document = web_sys::window()?.document()?;
            let find = |id: &str| document.get_element_by_id(id);
            Some(Elements {
                root: find(ROOT)?,
                phase: find(PHASE)?,
                bar: find(BAR)?,
                fill: find(FILL)?,
                detail: find(DETAIL)?,
                lines: find(LINES)?,
                document,
            })
        })();
        Self { elements }
    }

    /// Says what is happening, with work that has no measure.
    pub fn phase(&self, text: &str) {
        let Some(elements) = &self.elements else {
            return;
        };
        elements.phase.set_text_content(Some(text));
        elements.detail.set_text_content(None);
        attribute(&elements.fill, "style", None);
        attribute(&elements.bar, "data-unknown", Some(""));
    }

    /// Replaces the surface with a failure and its text.
    pub fn fail(&self, what: &str, detail: &str) {
        let Some(elements) = &self.elements else {
            return;
        };
        attribute(&elements.root, "data-state", Some("failed"));
        elements.phase.set_text_content(Some(what));
        elements.detail.set_text_content(Some(detail));
    }

    /// Replaces the surface's content with `heading` and labelled `lines`,
    /// in `state`: `plan` for a link's plan, `refused` for a refusal.
    pub fn lines(&self, state: &str, heading: &str, lines: &[PlanLine]) {
        let Some(elements) = &self.elements else {
            return;
        };
        attribute(&elements.root, "data-state", Some(state));
        elements.phase.set_text_content(Some(heading));
        elements.detail.set_text_content(None);
        elements.lines.set_text_content(None);
        for line in lines {
            for (tag, text) in [("dt", &line.label), ("dd", &line.text)] {
                // Creating an element fails only for a malformed tag name.
                if let Ok(cell) = elements.document.create_element(tag) {
                    cell.set_text_content(Some(text));
                    elements.lines.append_child(&cell).ok();
                }
            }
        }
    }

    /// Removes the surface: the app has something to draw.
    pub fn dismiss(&self) {
        if let Some(elements) = &self.elements {
            attribute(&elements.root, "data-state", Some("done"));
        }
    }
}

/// Sets or removes an attribute. The DOM refuses only a malformed attribute
/// name, and the names here are literals.
fn attribute(element: &Element, name: &str, value: Option<&str>) {
    match value {
        Some(value) => element.set_attribute(name, value).ok(),
        None => element.remove_attribute(name).ok(),
    };
}
