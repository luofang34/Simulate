//! Writes indented XML from blocks.

use quick_xml::escape::escape;

use crate::block::Block;

/// Accumulates XML text, two spaces per level.
#[derive(Debug, Default)]
pub(crate) struct XmlWriter {
    out: String,
    depth: usize,
}

impl XmlWriter {
    /// Starts a document with its XML declaration.
    pub fn document() -> Self {
        Self {
            out: "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n".to_owned(),
            depth: 0,
        }
    }

    /// The text written so far.
    pub fn finish(self) -> String {
        self.out
    }

    fn indent(&mut self) {
        for _ in 0..self.depth {
            self.out.push_str("  ");
        }
    }

    fn tag(&mut self, name: &str, attributes: &[(&str, &str)]) {
        self.out.push('<');
        self.out.push_str(name);
        for (key, value) in attributes {
            self.out.push(' ');
            self.out.push_str(key);
            self.out.push_str("=\"");
            self.out.push_str(&escape(*value));
            self.out.push('"');
        }
    }

    /// Opens an element on its own line.
    pub fn open(&mut self, name: &str, attributes: &[(&str, &str)]) {
        self.indent();
        self.tag(name, attributes);
        self.out.push_str(">\n");
        self.depth = self.depth.saturating_add(1);
    }

    /// Closes the innermost element.
    pub fn close(&mut self, name: &str) {
        self.depth = self.depth.saturating_sub(1);
        self.indent();
        self.out.push_str("</");
        self.out.push_str(name);
        self.out.push_str(">\n");
    }

    /// An element holding only text.
    pub fn leaf(&mut self, name: &str, attributes: &[(&str, &str)], text: &str) {
        self.indent();
        self.tag(name, attributes);
        if text.is_empty() {
            self.out.push_str("/>\n");
            return;
        }
        self.out.push('>');
        self.out.push_str(&escape(text));
        self.out.push_str("</");
        self.out.push_str(name);
        self.out.push_str(">\n");
    }

    /// `COMMENT` elements.
    pub fn comments(&mut self, comments: &[String]) {
        for comment in comments {
            self.leaf("COMMENT", &[], comment);
        }
    }

    /// A block's comments and entries as leaf elements. Entries whose keyword
    /// starts with `user_prefix` become `<USER_DEFINED parameter="…">`.
    pub fn entries(&mut self, block: &Block, user_prefix: Option<&str>) {
        self.comments(&block.comments);
        for entry in &block.entries {
            let units: Vec<(&str, &str)> = entry
                .unit
                .as_deref()
                .map(|unit| ("units", unit))
                .into_iter()
                .collect();
            match user_prefix.and_then(|p| entry.keyword.strip_prefix(p)) {
                Some(parameter) => {
                    self.leaf("USER_DEFINED", &[("parameter", parameter)], &entry.value);
                }
                None => self.leaf(&entry.keyword, &units, &entry.value),
            }
        }
    }

    /// A block wrapped in its own element.
    pub fn block(&mut self, name: &str, block: &Block, user_prefix: Option<&str>) {
        self.open(name, &[]);
        self.entries(block, user_prefix);
        self.close(name);
    }
}
