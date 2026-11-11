//! The page's progress surface, as the app drives it. `index.html` shows it
//! and `boot.mjs` fills it while the wasm downloads; the app takes the same
//! elements over for its own start-up and removes them when it draws.

use web_sys::Element;

use crate::platform::surface_ids::{BAR, DETAIL, FILL, PHASE, ROOT};

/// The surface's elements. A page without them gives a surface that does
/// nothing, so the app also runs in a bare page.
pub struct ProgressSurface {
    elements: Option<Elements>,
}

struct Elements {
    root: Element,
    phase: Element,
    bar: Element,
    fill: Element,
    detail: Element,
}

impl ProgressSurface {
    /// The surface of the current page.
    pub fn of_page() -> Self {
        let document = web_sys::window().and_then(|window| window.document());
        let find = |id: &str| document.as_ref()?.get_element_by_id(id);
        let elements = (|| {
            Some(Elements {
                root: find(ROOT)?,
                phase: find(PHASE)?,
                bar: find(BAR)?,
                fill: find(FILL)?,
                detail: find(DETAIL)?,
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
