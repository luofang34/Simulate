//! A piece of the fragment and where it sits, so a refusal can point at it.

use crate::{Parameter, Reason, Refusal};

/// Text of the fragment with the count of characters before it.
#[derive(Debug, Clone, Copy)]
pub(super) struct Token<'a> {
    pub(super) text: &'a str,
    pub(super) offset: usize,
}

impl<'a> Token<'a> {
    pub(super) const fn new(text: &'a str, offset: usize) -> Self {
        Self { text, offset }
    }

    /// The pieces between each one-byte `separator`, with their offsets.
    pub(super) fn split(self, separator: char) -> impl Iterator<Item = Token<'a>> {
        self.text
            .split(separator)
            .scan(self.offset, |offset, text| {
                let token = Token::new(text, *offset);
                *offset = offset.saturating_add(text.len()).saturating_add(1);
                Some(token)
            })
    }

    /// The text before and after the first one-byte `separator`.
    pub(super) fn split_once(self, separator: char) -> Option<(Token<'a>, Token<'a>)> {
        self.text.split_once(separator).map(|(before, after)| {
            let after_offset = self.offset.saturating_add(before.len()).saturating_add(1);
            (
                Token::new(before, self.offset),
                Token::new(after, after_offset),
            )
        })
    }

    /// The token, or a refusal when it is empty.
    pub(super) fn non_empty(self, site: Site, expected: &'static str) -> Result<Self, Refusal> {
        if self.text.is_empty() {
            return Err(site.refuse(self.offset, Reason::Empty { expected }));
        }
        Ok(self)
    }
}

/// The parameter, and the entry of its list, a refusal is about.
#[derive(Debug, Clone, Copy)]
pub(super) struct Site {
    parameter: Parameter,
    item: Option<usize>,
}

impl Site {
    pub(super) const fn of(parameter: Parameter) -> Self {
        Self {
            parameter,
            item: None,
        }
    }

    /// The site of the parameter's entry at zero-based `index`.
    pub(super) const fn entry(self, index: usize) -> Self {
        Self {
            parameter: self.parameter,
            item: Some(index.saturating_add(1)),
        }
    }

    pub(super) const fn refuse(self, offset: usize, reason: Reason) -> Refusal {
        Refusal {
            parameter: self.parameter,
            item: self.item,
            offset,
            reason,
        }
    }
}
