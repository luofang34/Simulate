//! Builds blocks from typed values, formatting each value once for every
//! notation.

use super::{Block, Entry, ValueKind};
use crate::error::{Error, ErrorKind, Result};
use crate::unit::{Quantity, Unit};
use crate::{WriteOptions, number};

/// Accumulates one block's comments and entries.
pub(crate) struct BlockWriter {
    block: Block,
    options: WriteOptions,
}

impl BlockWriter {
    /// Starts a block with its comments.
    pub fn new(comments: &[String], options: WriteOptions) -> Self {
        Self {
            block: Block {
                comments: comments.to_vec(),
                ..Block::default()
            },
            options,
        }
    }

    /// The finished block.
    pub fn finish(self) -> Block {
        self.block
    }

    fn push(&mut self, keyword: &str, value: String, unit: Option<&'static str>, kind: ValueKind) {
        self.block.entries.push(Entry {
            keyword: keyword.to_owned(),
            value,
            unit: unit.filter(|_| self.options.units).map(str::to_owned),
            at: None,
            kind,
        });
    }

    /// A text value, written as is.
    pub fn text(&mut self, keyword: &str, value: impl std::fmt::Display) {
        self.push(keyword, value.to_string(), None, ValueKind::JsonString);
    }

    /// A text value, if present.
    pub fn text_opt(&mut self, keyword: &str, value: Option<impl std::fmt::Display>) {
        if let Some(value) = value {
            self.text(keyword, value);
        }
    }

    /// An integer.
    pub fn integer(&mut self, keyword: &str, value: impl std::fmt::Display) {
        self.push(keyword, value.to_string(), None, ValueKind::JsonNumber);
    }

    /// An integer, if present.
    pub fn integer_opt(&mut self, keyword: &str, value: Option<impl std::fmt::Display>) {
        if let Some(value) = value {
            self.integer(keyword, value);
        }
    }

    /// A real number in unit `symbol` (empty for none).
    pub fn real_in(&mut self, keyword: &str, value: f64, symbol: &'static str) -> Result<()> {
        let text = number::format_f64(value)
            .ok_or_else(|| Error::new(ErrorKind::NonFiniteNumber).with_keyword(keyword))?;
        let unit = Some(symbol).filter(|s| !s.is_empty());
        self.push(keyword, text, unit, ValueKind::JsonNumber);
        Ok(())
    }

    /// A dimensionless real number.
    pub fn real(&mut self, keyword: &str, value: f64) -> Result<()> {
        self.real_in(keyword, value, "")
    }

    /// A dimensionless real number, if present.
    pub fn real_opt(&mut self, keyword: &str, value: Option<f64>) -> Result<()> {
        value.map_or(Ok(()), |value| self.real(keyword, value))
    }

    /// A quantity, with its unit restated if the options ask for it.
    pub fn quantity<U: Unit>(&mut self, keyword: &str, value: Quantity<U>) -> Result<()> {
        self.real_in(keyword, value.value(), U::SYMBOL)
    }

    /// A quantity, if present.
    pub fn quantity_opt<U: Unit>(
        &mut self,
        keyword: &str,
        value: Option<Quantity<U>>,
    ) -> Result<()> {
        value.map_or(Ok(()), |value| self.quantity(keyword, value))
    }
}
