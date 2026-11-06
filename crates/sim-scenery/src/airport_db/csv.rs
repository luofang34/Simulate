//! A streaming RFC 4180 CSV reader: quoted fields with embedded commas,
//! doubled quotes and line breaks, LF or CRLF records.

use std::iter::Peekable;
use std::str::Chars;

use crate::SceneryError;

/// Reads records one at a time into a reused field buffer.
pub(crate) struct CsvReader<'a> {
    chars: Peekable<Chars<'a>>,
    /// The file being read, for error messages.
    name: &'a str,
    record: u64,
}

impl<'a> CsvReader<'a> {
    /// A reader over `text`, skipping a leading byte-order mark.
    pub(crate) fn new(name: &'a str, text: &'a str) -> Self {
        Self {
            chars: text
                .strip_prefix('\u{feff}')
                .unwrap_or(text)
                .chars()
                .peekable(),
            name,
            record: 0,
        }
    }

    /// Reads the next record into `fields`; `false` at the end of the text.
    pub(crate) fn next_record(&mut self, fields: &mut Vec<String>) -> Result<bool, SceneryError> {
        fields.clear();
        if self.chars.peek().is_none() {
            return Ok(false);
        }
        self.record = self.record.wrapping_add(1);
        let mut field = String::new();
        let mut in_quotes = false;
        loop {
            let Some(c) = self.chars.next() else {
                if in_quotes {
                    return Err(SceneryError::InvalidCatalog {
                        detail: format!("{}: record {} ends inside quotes", self.name, self.record),
                    });
                }
                fields.push(field);
                return Ok(true);
            };
            if in_quotes {
                if c != '"' {
                    field.push(c);
                } else if self.chars.peek() == Some(&'"') {
                    self.chars.next();
                    field.push('"');
                } else {
                    in_quotes = false;
                }
                continue;
            }
            match c {
                '"' if field.is_empty() => in_quotes = true,
                ',' => fields.push(std::mem::take(&mut field)),
                '\r' | '\n' => {
                    if c == '\r' && self.chars.peek() == Some(&'\n') {
                        self.chars.next();
                    }
                    fields.push(field);
                    return Ok(true);
                }
                _ => field.push(c),
            }
        }
    }
}

/// Column positions read from a header record.
pub(crate) struct CsvHeader {
    names: Vec<String>,
    file: String,
}

impl CsvHeader {
    /// Reads the header record of `reader`.
    pub(crate) fn read(reader: &mut CsvReader<'_>) -> Result<Self, SceneryError> {
        let mut names = Vec::new();
        if !reader.next_record(&mut names)? {
            return Err(SceneryError::InvalidCatalog {
                detail: format!("{}: no header record", reader.name),
            });
        }
        Ok(Self {
            names,
            file: reader.name.to_owned(),
        })
    }

    /// The position of column `name`.
    pub(crate) fn column(&self, name: &str) -> Result<usize, SceneryError> {
        self.names
            .iter()
            .position(|candidate| candidate == name)
            .ok_or_else(|| SceneryError::InvalidCatalog {
                detail: format!("{}: no {name} column", self.file),
            })
    }

    /// The positions of `names`, in order.
    pub(crate) fn columns<const N: usize>(
        &self,
        names: [&str; N],
    ) -> Result<[usize; N], SceneryError> {
        let mut out = [0; N];
        for (slot, name) in out.iter_mut().zip(names) {
            *slot = self.column(name)?;
        }
        Ok(out)
    }
}

/// The trimmed field at `index`, empty when the record is short.
pub(crate) fn field(fields: &[String], index: usize) -> &str {
    fields.get(index).map_or("", |value| value.trim())
}

/// The field at `index` as a number; `None` when empty or malformed.
pub(crate) fn number(fields: &[String], index: usize) -> Option<f64> {
    field(fields, index)
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
}
