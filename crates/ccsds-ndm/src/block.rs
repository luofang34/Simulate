//! The syntax-neutral middle layer: a logical block of comments and
//! keyword/value entries.
//!
//! KVN sections, XML elements and JSON members all become [`Block`]s, and the
//! typed message builders read and write only blocks. Each keyword's rules
//! (unit, range, absence) therefore live in one place for every notation.

mod read;
mod write;

pub(crate) use read::Fields;
pub(crate) use write::BlockWriter;

use crate::error::Location;

/// How the notation delivered a value, which matters only for JSON.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ValueKind {
    /// KVN or XML text.
    Text,
    /// A JSON string.
    JsonString,
    /// A JSON number.
    JsonNumber,
    /// JSON `null`.
    JsonNull,
}

/// One keyword and its value as the notation gave it.
#[derive(Debug, Clone)]
pub(crate) struct Entry {
    /// The keyword (KVN key, XML element name, JSON member name).
    pub keyword: String,
    /// The value text, trimmed; a KVN `[unit]` suffix is still attached.
    pub value: String,
    /// A unit given out of band (XML `units` attribute).
    pub unit: Option<String>,
    /// Where the value starts, when the notation has positions.
    pub at: Option<Location>,
    /// How the value was delivered.
    pub kind: ValueKind,
}

impl Entry {
    /// An entry from KVN or XML text.
    pub fn text(keyword: impl Into<String>, value: impl Into<String>, at: Location) -> Self {
        Self {
            keyword: keyword.into(),
            value: value.into(),
            unit: None,
            at: Some(at),
            kind: ValueKind::Text,
        }
    }

    /// Whether the value is empty or JSON `null`.
    pub fn is_empty(&self) -> bool {
        self.kind == ValueKind::JsonNull || self.value.is_empty()
    }
}

/// A logical block: its leading comments and its entries in order.
#[derive(Debug, Clone, Default)]
pub(crate) struct Block {
    /// Comment lines, without the `COMMENT` keyword.
    pub comments: Vec<String>,
    /// Entries in input order.
    pub entries: Vec<Entry>,
    /// Where the block starts, for errors about what it lacks.
    pub at: Option<Location>,
}

impl Block {
    /// An empty block starting at `at`.
    pub fn starting_at(at: Option<Location>) -> Self {
        Self {
            at,
            ..Self::default()
        }
    }

    /// Whether the block has any entry with this keyword.
    pub fn has(&self, keyword: &str) -> bool {
        self.entries.iter().any(|entry| entry.keyword == keyword)
    }
}
