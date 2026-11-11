//! A section the app adds to the page's surface: a heading, a line of
//! status, a progress bar, labelled lines, and a button to wait on.

use sim_link_host::PlanLine;
use wasm_bindgen_futures::JsFuture;
use web_sys::{Document, Element};

use super::progress_surface::attribute;

/// One section of the surface.
pub struct Section {
    document: Document,
    root: Element,
    heading: Element,
    bar: Element,
    fill: Element,
    detail: Element,
    lines: Element,
}

/// Replaces the content of the list `lines` with `content`, a term and a
/// definition each.
pub(super) fn write_lines(document: &Document, lines: &Element, content: &[PlanLine]) {
    lines.set_text_content(None);
    for line in content {
        for (tag, text) in [("dt", &line.label), ("dd", &line.text)] {
            // Creating an element fails only for a malformed tag name.
            if let Ok(cell) = document.create_element(tag) {
                cell.set_text_content(Some(text));
                lines.append_child(&cell).ok();
            }
        }
    }
}

/// A new `tag` element of `class` (none when empty) at the end of `parent`.
pub(super) fn append(
    document: &Document,
    parent: &Element,
    tag: &str,
    class: &str,
) -> Option<Element> {
    let element = document.create_element(tag).ok()?;
    if !class.is_empty() {
        attribute(&element, "class", Some(class));
    }
    parent.append_child(&element).ok()?;
    Some(element)
}

impl Section {
    /// A new section at the end of `parent`.
    pub(super) fn under(document: &Document, parent: &Element, heading: &str) -> Option<Self> {
        let child = |parent: &Element, tag: &str, class: &str| append(document, parent, tag, class);
        let root = child(parent, "section", "section")?;
        let title = child(&root, "h2", "")?;
        title.set_text_content(Some(heading));
        let bar = child(&root, "div", "bar")?;
        attribute(&bar, "hidden", Some(""));
        let section = Self {
            fill: child(&bar, "div", "fill")?,
            detail: child(&root, "p", "detail")?,
            lines: child(&root, "dl", "lines")?,
            document: document.clone(),
            heading: title,
            root,
            bar,
        };
        Some(section)
    }

    /// The section's element, for content of its own after its lines.
    #[cfg(feature = "monitor-replay")]
    pub(super) const fn root(&self) -> &Element {
        &self.root
    }

    /// The page.
    #[cfg(feature = "monitor-replay")]
    pub(super) const fn document(&self) -> &Document {
        &self.document
    }

    /// Says what the section is doing, as work with no measure.
    pub fn working(&self, text: &str) {
        self.detail.set_text_content(Some(text));
        attribute(&self.fill, "style", None);
        attribute(&self.bar, "data-unknown", Some(""));
        attribute(&self.bar, "hidden", None);
    }

    /// Shows measured progress, `fraction` of the whole, with a line of
    /// numbers.
    pub fn progress(&self, fraction: f64, text: &str) {
        let percent = (fraction.clamp(0.0, 1.0) * 100.0).round();
        attribute(&self.bar, "data-unknown", None);
        attribute(&self.fill, "style", Some(&format!("width: {percent}%")));
        attribute(&self.bar, "hidden", None);
        self.detail.set_text_content(Some(text));
    }

    /// Ends the work: no bar, and `text` as the standing status.
    pub fn settled(&self, text: &str) {
        attribute(&self.bar, "hidden", Some(""));
        self.detail.set_text_content(Some(text));
    }

    /// Replaces the section's labelled lines.
    pub fn lines(&self, lines: &[PlanLine]) {
        write_lines(&self.document, &self.lines, lines);
    }

    /// Turns the section into a refusal: `heading` in its place, no bar,
    /// and the reason in `lines`.
    pub fn refuse(&self, heading: &str, lines: &[PlanLine]) {
        attribute(&self.root, "data-state", Some("refused"));
        self.heading.set_text_content(Some(heading));
        attribute(&self.bar, "hidden", Some(""));
        self.detail.set_text_content(None);
        self.lines(lines);
    }

    /// Shows a button and waits for its click; the button then goes.
    pub async fn clicked(&self, label: &str) {
        let Ok(button) = self.document.create_element("button") else {
            return;
        };
        button.set_text_content(Some(label));
        if self.root.append_child(&button).is_err() {
            return;
        }
        // The promise's own resolver is the listener: one click settles it.
        let click = js_sys::Promise::new(&mut |resolve, _reject| {
            button
                .add_event_listener_with_callback("click", &resolve)
                .ok();
        });
        JsFuture::from(click).await.ok();
        button.remove();
    }
}
