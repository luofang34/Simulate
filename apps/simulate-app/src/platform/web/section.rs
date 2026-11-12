//! A section the app adds to the page's surface: a heading, a block of
//! status lines whose count never changes, a progress bar, labelled lines,
//! and a button to wait on.

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
    rows: usize,
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

/// Writes `lines` into `block`, one line element each, keeping the elements
/// already there and changing only text that changed: a block written every
/// frame does not churn the page. A line's full text is also its tooltip,
/// for a line a narrow page cuts short.
pub(super) fn write_live(document: &Document, block: &Element, lines: &[String]) {
    let children = block.children();
    if block.child_nodes().length() != children.length() {
        block.set_text_content(None);
    }
    while children.length() as usize > lines.len() {
        match block.last_element_child() {
            Some(last) => last.remove(),
            None => break,
        }
    }
    for (index, text) in lines.iter().enumerate() {
        let existing = u32::try_from(index)
            .ok()
            .and_then(|index| children.item(index));
        let Some(line) = existing.or_else(|| append(document, block, "div", "live-line")) else {
            return;
        };
        if line.text_content().as_deref() != Some(text.as_str()) {
            line.set_text_content(Some(text));
            attribute(&line, "title", Some(text));
        }
    }
}

/// `lines` made exactly `rows` long: a dash for each line missing.
fn exactly(lines: &[String], rows: usize) -> Vec<String> {
    (0..rows)
        .map(|index| lines.get(index).cloned().unwrap_or_else(|| "—".to_owned()))
        .collect()
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
    /// A new section at the end of `parent`, whose status block always
    /// shows `rows` lines.
    pub(super) fn under(
        document: &Document,
        parent: &Element,
        heading: &str,
        rows: usize,
    ) -> Option<Self> {
        let child = |parent: &Element, tag: &str, class: &str| append(document, parent, tag, class);
        let root = child(parent, "section", "section")?;
        let title = child(&root, "h2", "")?;
        title.set_text_content(Some(heading));
        let bar = child(&root, "div", "bar")?;
        attribute(&bar, "hidden", Some(""));
        let section = Self {
            fill: child(&bar, "div", "fill")?,
            detail: child(&root, "div", "live")?,
            rows,
            lines: child(&root, "dl", "lines")?,
            document: document.clone(),
            heading: title,
            root,
            bar,
        };
        section.status(&[]);
        Some(section)
    }

    /// The section's element, for content of its own after its lines.
    pub(super) const fn root(&self) -> &Element {
        &self.root
    }

    /// The page.
    pub(super) const fn document(&self) -> &Document {
        &self.document
    }

    /// Shows `lines` as the status, exactly as many as the section's rows.
    fn status(&self, lines: &[String]) {
        write_live(&self.document, &self.detail, &exactly(lines, self.rows));
    }

    /// Says what the section is doing, as work with no measure.
    pub fn working(&self, lines: &[String]) {
        self.status(lines);
        attribute(&self.fill, "style", None);
        attribute(&self.bar, "data-unknown", Some(""));
        attribute(&self.bar, "hidden", None);
    }

    /// Shows measured progress, `fraction` of the whole, with its numbers.
    pub fn progress(&self, fraction: f64, lines: &[String]) {
        let percent = (fraction.clamp(0.0, 1.0) * 100.0).round();
        attribute(&self.bar, "data-unknown", None);
        attribute(&self.fill, "style", Some(&format!("width: {percent}%")));
        attribute(&self.bar, "hidden", None);
        self.status(lines);
    }

    /// Ends the work: no bar, and `lines` as the standing status.
    pub fn settled(&self, lines: &[String]) {
        attribute(&self.bar, "hidden", Some(""));
        self.status(lines);
    }

    /// Replaces the section's labelled lines.
    pub fn lines(&self, lines: &[PlanLine]) {
        write_lines(&self.document, &self.lines, lines);
    }

    /// Adds `line` after the section's labelled lines.
    pub fn add_line(&self, line: &PlanLine) {
        for (tag, text) in [("dt", &line.label), ("dd", &line.text)] {
            if let Some(cell) = append(&self.document, &self.lines, tag, "") {
                cell.set_text_content(Some(text));
            }
        }
    }

    /// Turns the section into a refusal: `heading` in its place, no bar,
    /// `status` as the status and the reason in `lines`.
    pub fn refuse(&self, heading: &str, status: &[String], lines: &[PlanLine]) {
        attribute(&self.root, "data-state", Some("refused"));
        self.heading.set_text_content(Some(heading));
        attribute(&self.bar, "hidden", Some(""));
        self.status(status);
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
