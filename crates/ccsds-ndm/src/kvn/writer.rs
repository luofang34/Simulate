//! Writes blocks as KVN lines.

use std::fmt::Write as _;

use crate::block::Block;
use crate::error::{Error, ErrorKind, Result};

/// KVN has no way to continue a value onto another line.
fn single_line<'a>(keyword: &str, text: &'a str) -> Result<&'a str> {
    if text.contains(['\n', '\r']) {
        return Err(Error::new(ErrorKind::InvalidValue {
            text: text.to_owned(),
            expected: "a single-line value",
        })
        .with_keyword(keyword));
    }
    Ok(text)
}

/// Accumulates KVN text.
#[derive(Debug, Default)]
pub(crate) struct KvnWriter {
    out: String,
}

impl KvnWriter {
    /// The text written so far.
    pub fn finish(self) -> String {
        self.out
    }

    /// One `KEYWORD = value` line, unaligned.
    pub fn pair(&mut self, keyword: &str, value: &str) {
        // Writing into a String cannot fail.
        writeln!(self.out, "{keyword} = {value}").ok();
    }

    /// One line written as is (block markers, data lines).
    pub fn line(&mut self, text: &str) {
        self.out.push_str(text);
        self.out.push('\n');
    }

    /// A blank separator line.
    pub fn blank(&mut self) {
        self.out.push('\n');
    }

    /// Comment lines.
    pub fn comments(&mut self, comments: &[String]) -> Result<()> {
        for comment in comments {
            let comment = single_line("COMMENT", comment)?;
            if comment.is_empty() {
                self.line("COMMENT");
            } else {
                writeln!(self.out, "COMMENT {comment}").ok();
            }
        }
        Ok(())
    }

    /// A block: its comments, then its entries with the `=` signs aligned.
    pub fn block(&mut self, block: &Block) -> Result<()> {
        self.comments(&block.comments)?;
        let width = block
            .entries
            .iter()
            .map(|entry| entry.keyword.len())
            .max()
            .unwrap_or(0);
        for entry in &block.entries {
            let keyword = &entry.keyword;
            let value = single_line(keyword, &entry.value)?;
            match &entry.unit {
                Some(unit) => writeln!(self.out, "{keyword:<width$} = {value} [{unit}]"),
                None => writeln!(self.out, "{keyword:<width$} = {value}"),
            }
            .ok();
        }
        Ok(())
    }
}
