//! Typed parse and write errors with the position and keyword they concern.

use std::fmt;

use crate::epoch::EpochError;

/// A one-based position in the input text.
///
/// Columns count characters, not bytes, so they match what an editor shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Location {
    /// Line number, starting at 1.
    pub line: usize,
    /// Column number, starting at 1.
    pub column: usize,
}

impl Location {
    /// Builds a location from a one-based line and column.
    pub const fn new(line: usize, column: usize) -> Self {
        Self { line, column }
    }

    /// Finds the location of a byte offset in `text`.
    ///
    /// An offset past the end, or inside a multi-byte character, resolves to
    /// the nearest preceding character boundary.
    pub fn of_offset(text: &str, offset: usize) -> Self {
        let mut end = offset.min(text.len());
        while !text.is_char_boundary(end) {
            end = end.saturating_sub(1);
        }
        let prefix = text.get(..end).unwrap_or_default();
        let line = prefix.matches('\n').count().saturating_add(1);
        let line_start = prefix.rfind('\n').map_or(0, |at| at.saturating_add(1));
        let column = prefix
            .get(line_start..)
            .map_or(0, |rest| rest.chars().count())
            .saturating_add(1);
        Self { line, column }
    }
}

impl fmt::Display for Location {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}, column {}", self.line, self.column)
    }
}

/// What went wrong, independent of where.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ErrorKind {
    /// A keyword the message requires is absent.
    #[error("missing mandatory keyword")]
    MissingKeyword,
    /// A keyword appears twice where it may appear once.
    #[error("duplicate keyword")]
    DuplicateKeyword,
    /// A keyword this message type does not define (strict mode only).
    #[error("keyword not defined for this message")]
    UnknownKeyword,
    /// A keyword appears in a block that has already been closed.
    #[error("keyword out of order: its block has already ended")]
    KeywordOutOfOrder,
    /// A keyword is present with no value.
    #[error("empty value")]
    EmptyValue,
    /// A value that should be a CCSDS real number is not one, or is not finite.
    #[error("invalid number `{text}`")]
    InvalidNumber {
        /// The offending text.
        text: String,
    },
    /// A value that should be an integer is not one, or is out of range.
    #[error("invalid integer `{text}`")]
    InvalidInteger {
        /// The offending text.
        text: String,
    },
    /// A value that should be an epoch is not one.
    #[error("invalid epoch `{text}`")]
    InvalidEpoch {
        /// The offending text.
        text: String,
        /// Why it is not an epoch.
        #[source]
        reason: EpochError,
    },
    /// A value outside the set the keyword allows.
    #[error("invalid value `{text}`, expected {expected}")]
    InvalidValue {
        /// The offending text.
        text: String,
        /// What the keyword allows.
        expected: &'static str,
    },
    /// A unit that does not match the unit the keyword is defined in.
    #[error("unit `{found}` does not match the defined unit `{expected}`")]
    UnitMismatch {
        /// The unit the standard defines for this keyword (empty if none).
        expected: &'static str,
        /// The unit the input gave.
        found: String,
    },
    /// A message version this crate does not read.
    #[error("unsupported message version `{text}`")]
    UnsupportedVersion {
        /// The version text.
        text: String,
    },
    /// The input holds a different message type than the one requested.
    #[error("expected a {expected} message, found `{found}`")]
    WrongMessage {
        /// The message type requested.
        expected: &'static str,
        /// What the input starts with instead.
        found: String,
    },
    /// A line that is neither a keyword, a comment, nor data in this position.
    #[error("unexpected line `{text}`")]
    UnexpectedLine {
        /// The offending line, trimmed.
        text: String,
    },
    /// A data line with the wrong number of columns.
    #[error("data line has {found} values, expected {expected}")]
    ColumnCount {
        /// The column counts that are allowed.
        expected: &'static str,
        /// The column count found.
        found: usize,
    },
    /// The input ended inside a structure.
    #[error("unexpected end of input, expected {expected}")]
    UnexpectedEnd {
        /// What should have followed.
        expected: &'static str,
    },
    /// The input contains no message at all.
    #[error("no message found")]
    Empty,
    /// The XML is not well formed.
    #[error("malformed XML")]
    Xml(#[source] quick_xml::Error),
    /// An XML element this position does not allow.
    #[error("unexpected XML element <{name}>")]
    UnexpectedElement {
        /// The element name.
        name: String,
    },
    /// An XML element this position requires is absent.
    #[error("missing XML element <{name}>")]
    MissingElement {
        /// The element name.
        name: &'static str,
    },
    /// XML nested deeper than any NDM schema allows.
    #[error("XML nested deeper than {limit} levels")]
    XmlTooDeep {
        /// The nesting limit.
        limit: usize,
    },
    /// An XML entity reference other than the five predefined ones.
    #[error("unknown XML entity `&{name};`")]
    UnknownEntity {
        /// The entity name.
        name: String,
    },
    /// The JSON is not well formed.
    #[cfg(feature = "json")]
    #[error("malformed JSON")]
    Json(#[source] serde_json::Error),
    /// A JSON value of the wrong type for its keyword.
    #[error("JSON value has the wrong type, expected {expected}")]
    JsonType {
        /// The JSON type expected.
        expected: &'static str,
    },
    /// A number that cannot be written because it is NaN or infinite.
    #[error("cannot write a non-finite number")]
    NonFiniteNumber,
}

/// A parse or write failure: what went wrong, and where.
#[derive(Debug)]
pub struct Error {
    kind: ErrorKind,
    location: Option<Location>,
    keyword: Option<String>,
    message_index: Option<usize>,
}

impl Error {
    /// Builds an error with no position or keyword attached.
    pub fn new(kind: ErrorKind) -> Self {
        Self {
            kind,
            location: None,
            keyword: None,
            message_index: None,
        }
    }

    /// Attaches the position the error was found at.
    #[must_use]
    pub fn at(mut self, location: Location) -> Self {
        self.location = Some(location);
        self
    }

    /// Attaches the keyword the error concerns.
    #[must_use]
    pub fn with_keyword(mut self, keyword: impl Into<String>) -> Self {
        self.keyword = Some(keyword.into());
        self
    }

    /// Attaches the zero-based index of the message within a multi-message
    /// input, unless one is already attached.
    #[must_use]
    pub fn in_message(mut self, index: usize) -> Self {
        self.message_index.get_or_insert(index);
        self
    }

    /// What went wrong.
    pub fn kind(&self) -> &ErrorKind {
        &self.kind
    }

    /// Where it went wrong, when the input format carries positions.
    pub fn location(&self) -> Option<Location> {
        self.location
    }

    /// The keyword concerned, when there is one.
    pub fn keyword(&self) -> Option<&str> {
        self.keyword.as_deref()
    }

    /// The zero-based message index in a multi-message input.
    pub fn message_index(&self) -> Option<usize> {
        self.message_index
    }
}

impl From<ErrorKind> for Error {
    fn from(kind: ErrorKind) -> Self {
        Self::new(kind)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(index) = self.message_index {
            write!(f, "message {index}: ")?;
        }
        if let Some(location) = self.location {
            write!(f, "{location}: ")?;
        }
        if let Some(keyword) = &self.keyword {
            write!(f, "{keyword}: ")?;
        }
        write!(f, "{}", self.kind)
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        // Display already prints the kind, so the chain continues at its cause.
        std::error::Error::source(&self.kind)
    }
}

/// Result alias for this crate.
pub type Result<T, E = Error> = std::result::Result<T, E>;
