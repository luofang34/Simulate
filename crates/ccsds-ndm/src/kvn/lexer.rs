//! Splits KVN text into classified lines with their positions.

use crate::ParseMode;
use crate::block::Entry;
use crate::error::Location;

/// One non-blank KVN line.
#[derive(Debug, Clone)]
pub(crate) enum Line {
    /// `COMMENT text`.
    Comment {
        /// The text after the keyword, trimmed.
        text: String,
        /// Where the line starts.
        at: Location,
    },
    /// `KEYWORD = value`.
    Pair(Entry),
    /// Anything else: block markers (`META_START`) and data lines.
    Bare {
        /// The line, trimmed.
        text: String,
        /// Where the text starts.
        at: Location,
    },
}

impl Line {
    /// Where the line starts.
    pub fn at(&self) -> Location {
        match self {
            Self::Comment { at, .. } | Self::Bare { at, .. } => *at,
            Self::Pair(entry) => entry.at.unwrap_or(Location::new(1, 1)),
        }
    }

    /// The keyword, if this is a `KEYWORD = value` line.
    pub fn keyword(&self) -> Option<&str> {
        match self {
            Self::Pair(entry) => Some(entry.keyword.as_str()),
            _ => None,
        }
    }
}

/// Classifies every non-blank line of `text`.
///
/// Lenient mode reads lower-case keywords as upper case.
pub(crate) fn lex(text: &str, mode: ParseMode) -> Vec<Line> {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    text.lines()
        .enumerate()
        .filter_map(|(index, raw)| classify(raw, index.saturating_add(1), mode))
        .collect()
}

fn classify(raw: &str, line: usize, mode: ParseMode) -> Option<Line> {
    let raw = raw.strip_suffix('\r').unwrap_or(raw);
    let trimmed = raw.trim_start();
    if trimmed.trim_end().is_empty() {
        return None;
    }
    let indent = raw.len().saturating_sub(trimmed.len());
    let column_of = |byte: usize| Location::new(line, column(raw, byte));
    if let Some(rest) = trimmed.strip_prefix("COMMENT")
        && (rest.is_empty() || rest.starts_with(char::is_whitespace))
    {
        return Some(Line::Comment {
            text: rest.trim().to_owned(),
            at: column_of(indent),
        });
    }
    if let Some((key, value)) = trimmed.split_once('=') {
        let key = key.trim_end();
        let valid = |c: char| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_';
        let key = match mode {
            ParseMode::Strict => key.to_owned(),
            ParseMode::Lenient => key.to_ascii_uppercase(),
        };
        if !key.is_empty() && key.chars().all(valid) {
            let value_start = raw.len().saturating_sub(value.trim_start().len());
            return Some(Line::Pair(Entry::text(
                key,
                value.trim(),
                column_of(value_start),
            )));
        }
    }
    Some(Line::Bare {
        text: trimmed.trim_end().to_owned(),
        at: column_of(indent),
    })
}

/// One-based character column of a byte offset within a line.
fn column(raw: &str, byte: usize) -> usize {
    raw.get(..byte)
        .map_or(0, |prefix| prefix.chars().count())
        .saturating_add(1)
}

/// Splits a line sequence into messages, each starting at `version_key`.
///
/// Lines before the first version line are returned as their own leading
/// group so the caller can report them.
pub(crate) fn split_messages(lines: Vec<Line>, version_key: &str) -> Vec<Vec<Line>> {
    let mut messages: Vec<Vec<Line>> = Vec::new();
    for line in lines {
        let starts = line.keyword() == Some(version_key);
        match messages.last_mut() {
            Some(current) if !starts => current.push(line),
            _ => messages.push(vec![line]),
        }
    }
    messages
}

#[cfg(test)]
mod tests;
