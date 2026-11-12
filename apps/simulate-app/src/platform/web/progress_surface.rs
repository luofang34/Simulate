//! The page's progress surface, as the app drives it. `index.html` shows it
//! and `boot.mjs` fills it while the wasm downloads; the app takes the same
//! elements over for its own start-up and removes them when it draws.

use sim_link_host::PlanLine;
use web_sys::{Document, Element};

use super::section::{Section, write_lines};
use crate::platform::surface_ids::{BAR, CARD, DETAIL, FILL, LINES, PHASE, ROOT};

/// The surface's elements. A page without them gives a surface that does
/// nothing, so the app also runs in a bare page.
pub struct ProgressSurface {
    elements: Option<Elements>,
}

struct Elements {
    document: Document,
    root: Element,
    card: Element,
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
                card: find(CARD)?,
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
        write_lines(&elements.document, &elements.lines, lines);
    }

    /// Adds a section under what the surface shows, headed `heading`, with
    /// `rows` lines of status. `None` in a page without the surface.
    pub fn section(&self, heading: &str, rows: usize) -> Option<Section> {
        let elements = self.elements.as_ref()?;
        Section::under(&elements.document, &elements.card, heading, rows)
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
pub(super) fn attribute(element: &Element, name: &str, value: Option<&str>) {
    match value {
        Some(value) => element.set_attribute(name, value).ok(),
        None => element.remove_attribute(name).ok(),
    };
}
