//! Typed access to a block's entries, with the strict/lenient rules applied
//! once for every keyword.

use super::{Block, Entry, ValueKind};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::extension::{Extension, ExtensionValue};
use crate::frame::{CenterName, RefFrame, YesNo};
use crate::time_system::TimeSystem;
use crate::unit::{Quantity, Unit};
use crate::{Epoch, ParseMode, number};

/// A block being consumed keyword by keyword.
pub(crate) struct Fields {
    entries: Vec<Option<Entry>>,
    at: Option<Location>,
    mode: ParseMode,
}

impl Fields {
    /// Takes a block's entries, rejecting a keyword that appears twice.
    pub fn new(block: Block, mode: ParseMode) -> Result<(Vec<String>, Self)> {
        let mut seen = std::collections::HashSet::new();
        for entry in &block.entries {
            if !seen.insert(entry.keyword.as_str()) {
                return Err(entry.error(ErrorKind::DuplicateKeyword));
            }
        }
        let fields = Self {
            entries: block.entries.into_iter().map(Some).collect(),
            at: block.at,
            mode,
        };
        Ok((block.comments, fields))
    }

    /// Whether an entry with this keyword is still unread.
    pub fn has(&self, keyword: &str) -> bool {
        self.entries.iter().flatten().any(|e| e.keyword == keyword)
    }

    /// Whether any keyword from `keywords` is still unread.
    pub fn has_any(&self, keywords: &[&str]) -> bool {
        keywords.iter().any(|kw| self.has(kw))
    }

    fn take(&mut self, keyword: &str) -> Option<Entry> {
        self.entries
            .iter_mut()
            .find(|slot| slot.as_ref().is_some_and(|e| e.keyword == keyword))
            .and_then(Option::take)
    }

    /// An optional keyword. Empty values are an error in strict mode and
    /// absent in lenient mode.
    pub fn optional<T>(
        &mut self,
        keyword: &str,
        parse: impl FnOnce(&Entry, ParseMode) -> Result<T>,
    ) -> Result<Option<T>> {
        match self.take(keyword) {
            None => Ok(None),
            Some(entry) if entry.is_empty() => match self.mode {
                ParseMode::Strict => Err(entry.error(ErrorKind::EmptyValue)),
                ParseMode::Lenient => Ok(None),
            },
            Some(entry) => parse(&entry, self.mode).map(Some),
        }
    }

    /// A mandatory keyword, in both modes.
    pub fn required<T>(
        &mut self,
        keyword: &str,
        parse: impl FnOnce(&Entry, ParseMode) -> Result<T>,
    ) -> Result<T> {
        let entry = self.take(keyword).ok_or_else(|| self.missing(keyword))?;
        if entry.is_empty() {
            return Err(entry.error(ErrorKind::EmptyValue));
        }
        parse(&entry, self.mode)
    }

    /// A keyword the standard makes mandatory but producers omit or leave
    /// blank: required in strict mode, optional in lenient mode.
    pub fn expected<T>(
        &mut self,
        keyword: &str,
        parse: impl FnOnce(&Entry, ParseMode) -> Result<T>,
    ) -> Result<Option<T>> {
        match self.mode {
            ParseMode::Strict => self.required(keyword, parse).map(Some),
            ParseMode::Lenient => self.optional(keyword, parse),
        }
    }

    /// An error about an unread entry, located at it when it is present.
    pub fn error_for(&self, keyword: &str, kind: ErrorKind) -> Error {
        match self.entries.iter().flatten().find(|e| e.keyword == keyword) {
            Some(entry) => entry.error(kind),
            None => Error::new(kind).with_keyword(keyword),
        }
    }

    /// The error for a mandatory keyword that is absent.
    pub fn missing(&self, keyword: &str) -> Error {
        let error = Error::new(ErrorKind::MissingKeyword).with_keyword(keyword);
        match self.at {
            Some(at) => error.at(at),
            None => error,
        }
    }

    /// Ends reading: what is left is undefined here. Strict mode rejects it;
    /// lenient mode keeps it as extensions.
    pub fn finish(self, extensions: &mut Vec<Extension>) -> Result<()> {
        for entry in self.entries.into_iter().flatten() {
            if self.mode == ParseMode::Strict {
                return Err(entry.error(ErrorKind::UnknownKeyword));
            }
            let value = match entry.kind {
                ValueKind::Text | ValueKind::JsonString => ExtensionValue::Text(entry.value),
                ValueKind::JsonNumber => ExtensionValue::Number(entry.value),
                ValueKind::JsonNull => ExtensionValue::Null,
            };
            extensions.push(Extension {
                keyword: entry.keyword,
                value,
                unit: entry.unit,
            });
        }
        Ok(())
    }
}

impl Entry {
    /// An error about this entry, carrying its keyword and position.
    pub fn error(&self, kind: ErrorKind) -> Error {
        let error = Error::new(kind).with_keyword(self.keyword.clone());
        match self.at {
            Some(at) => error.at(at),
            None => error,
        }
    }

    /// The value as free text.
    pub fn string(&self, _: ParseMode) -> Result<String> {
        Ok(self.value.clone())
    }

    /// A real number in unit `U`.
    pub fn quantity<U: Unit>(&self, mode: ParseMode) -> Result<Quantity<U>> {
        self.real_in(U::SYMBOL, mode).map(Quantity::new)
    }

    /// A real number with no unit.
    pub fn real(&self, mode: ParseMode) -> Result<f64> {
        self.real_in("", mode)
    }

    /// A real number whose unit, if restated, must be `symbol`.
    pub fn real_in(&self, symbol: &'static str, mode: ParseMode) -> Result<f64> {
        if self.kind == ValueKind::JsonString && mode == ParseMode::Strict {
            return Err(self.error(ErrorKind::JsonType { expected: "number" }));
        }
        let (text, unit) = self.split_unit();
        if let Some(unit) = unit {
            let matches = match mode {
                ParseMode::Strict => unit == symbol,
                ParseMode::Lenient => unit.eq_ignore_ascii_case(symbol),
            };
            if !matches {
                return Err(self.error(ErrorKind::UnitMismatch {
                    expected: symbol,
                    found: unit.to_owned(),
                }));
            }
        }
        number::parse_f64(text, mode).ok_or_else(|| {
            self.error(ErrorKind::InvalidNumber {
                text: text.to_owned(),
            })
        })
    }

    /// An integer.
    pub fn integer<T: std::str::FromStr>(&self, mode: ParseMode) -> Result<T> {
        if self.kind == ValueKind::JsonString && mode == ParseMode::Strict {
            return Err(self.error(ErrorKind::JsonType { expected: "number" }));
        }
        number::parse_int(&self.value).ok_or_else(|| {
            self.error(ErrorKind::InvalidInteger {
                text: self.value.clone(),
            })
        })
    }

    /// An epoch.
    pub fn epoch(&self, _: ParseMode) -> Result<Epoch> {
        Epoch::parse(&self.value).map_err(|reason| {
            self.error(ErrorKind::InvalidEpoch {
                text: self.value.clone(),
                reason,
            })
        })
    }

    /// A reference frame.
    pub fn frame(&self, _: ParseMode) -> Result<RefFrame> {
        Ok(RefFrame::from_name(&self.value))
    }

    /// A frame origin.
    pub fn center(&self, _: ParseMode) -> Result<CenterName> {
        Ok(CenterName::from_name(&self.value))
    }

    /// A time system.
    pub fn time_system(&self, _: ParseMode) -> Result<TimeSystem> {
        TimeSystem::from_name(&self.value).ok_or_else(|| {
            self.error(ErrorKind::InvalidValue {
                text: self.value.clone(),
                expected: "a time system listed in ODM 502.0-B-3 3.2.3.2",
            })
        })
    }

    /// A `YES` or `NO` flag.
    pub fn yes_no(&self, mode: ParseMode) -> Result<YesNo> {
        YesNo::from_name(&self.value, mode == ParseMode::Lenient).ok_or_else(|| {
            self.error(ErrorKind::InvalidValue {
                text: self.value.clone(),
                expected: "YES or NO",
            })
        })
    }

    /// Splits a KVN `value [unit]`, or pairs the value with its XML unit.
    fn split_unit(&self) -> (&str, Option<&str>) {
        if let Some(unit) = &self.unit {
            return (self.value.as_str(), Some(unit.as_str()));
        }
        if let Some(head) = self.value.strip_suffix(']')
            && let Some((number, unit)) = head.rsplit_once('[')
        {
            return (number.trim_end(), Some(unit.trim()));
        }
        (self.value.as_str(), None)
    }
}
