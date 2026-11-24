//! OMM in the flat JSON layout CelesTrak and Space-Track serve: one object
//! per message, one member per keyword, optionally in an array.
//!
//! ODM 502.0-B-3 defines no JSON notation; this follows the producers.
//! CelesTrak omits the version, header, centre, frame, time system and
//! theory, so its output needs [`ParseMode::Lenient`]; Space-Track sends
//! every number as a string and adds members of its own, which lenient mode
//! keeps as [`Extension`](crate::Extension)s. Only header comments travel
//! (as one `COMMENT` member, lines joined by newlines); JSON has nowhere to
//! put the others.

use std::collections::BTreeMap;

use serde_json::Value;

use crate::block::{Block, Entry, ValueKind};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::flat::{FlatMessage, HEADER};
use crate::omm::{SCHEMA, layout};
use crate::{Omm, ParseMode, WriteOptions};

impl Omm {
    /// Reads one JSON OMM: an object, or an array holding exactly one.
    pub fn from_json(text: &str, mode: ParseMode) -> Result<Self> {
        let mut all = Self::from_json_all(text, mode)?;
        match (all.pop(), all.is_empty()) {
            (Some(omm), true) => Ok(omm),
            (None, _) => Err(Error::new(ErrorKind::Empty)),
            (Some(_), false) => Err(Error::new(ErrorKind::JsonType {
                expected: "a single OMM object",
            })),
        }
    }

    /// Reads every OMM of a JSON array (or a single object).
    pub fn from_json_all(text: &str, mode: ParseMode) -> Result<Vec<Self>> {
        let value: Value = serde_json::from_str(text).map_err(|source| {
            let at = Location::new(source.line(), source.column());
            Error::new(ErrorKind::Json(source)).at(at)
        })?;
        let objects = match value {
            Value::Array(items) => items,
            object @ Value::Object(_) => vec![object],
            _ => {
                return Err(Error::new(ErrorKind::JsonType {
                    expected: "an object or an array of objects",
                }));
            }
        };
        objects
            .into_iter()
            .enumerate()
            .map(|(index, object)| read_object(object, mode).map_err(|e| e.in_message(index)))
            .collect()
    }

    /// Writes the message as one JSON object.
    pub fn to_json(&self, options: WriteOptions) -> Result<String> {
        write_object(self, options)
    }

    /// Writes messages as a JSON array.
    pub fn to_json_all(messages: &[Self], options: WriteOptions) -> Result<String> {
        let objects = messages
            .iter()
            .enumerate()
            .map(|(index, omm)| write_object(omm, options).map_err(|e| e.in_message(index)))
            .collect::<Result<Vec<_>>>()?;
        Ok(format!("[{}]", objects.join(",")))
    }
}

fn read_object(object: Value, mode: ParseMode) -> Result<Omm> {
    let Value::Object(members) = object else {
        return Err(Error::new(ErrorKind::JsonType {
            expected: "an object",
        }));
    };
    let extensions = SCHEMA.sections.len();
    let mut blocks: BTreeMap<usize, Block> = BTreeMap::new();
    let mut version = None;
    for (key, value) in members {
        if key == "COMMENT" {
            let Value::String(text) = value else {
                return Err(
                    Error::new(ErrorKind::JsonType { expected: "string" }).with_keyword(key)
                );
            };
            blocks
                .entry(HEADER)
                .or_default()
                .comments
                .extend(text.lines().map(str::to_owned));
            continue;
        }
        let entry = entry(key, value)?;
        if entry.keyword == SCHEMA.version_key {
            version = Some(entry);
            continue;
        }
        let section = SCHEMA
            .sections
            .iter()
            .position(|s| s.holds(&entry.keyword))
            .unwrap_or(extensions);
        blocks.entry(section).or_default().entries.push(entry);
    }
    let blocks = blocks.into_iter().collect();
    layout::from_flat(FlatMessage { version, blocks }, mode)
}

fn entry(keyword: String, value: Value) -> Result<Entry> {
    let (value, kind) = match value {
        Value::String(text) => (text.trim().to_owned(), ValueKind::JsonString),
        Value::Number(number) => (number.to_string(), ValueKind::JsonNumber),
        Value::Null => (String::new(), ValueKind::JsonNull),
        Value::Bool(_) | Value::Array(_) | Value::Object(_) => {
            return Err(Error::new(ErrorKind::JsonType {
                expected: "a string, a number or null",
            })
            .with_keyword(keyword));
        }
    };
    Ok(Entry {
        keyword,
        value,
        unit: None,
        at: None,
        kind,
    })
}

fn write_object(omm: &Omm, options: WriteOptions) -> Result<String> {
    let mut members: Vec<String> = Vec::new();
    let mut member = |key: &str, value: String| {
        members.push(format!("{}:{value}", Value::String(key.to_owned())));
    };
    if let Some(version) = omm.version {
        member(
            SCHEMA.version_key,
            Value::String(version.as_str().to_owned()).to_string(),
        );
    }
    if !omm.header.comments.is_empty() {
        member(
            "COMMENT",
            Value::String(omm.header.comments.join("\n")).to_string(),
        );
    }
    for (_, block) in layout::to_blocks(omm, options)? {
        for entry in block.entries {
            let value = match entry.kind {
                ValueKind::JsonNumber => entry.value,
                ValueKind::JsonNull => "null".to_owned(),
                ValueKind::Text | ValueKind::JsonString => Value::String(entry.value).to_string(),
            };
            member(&entry.keyword, value);
        }
    }
    Ok(format!("{{{}}}", members.join(",")))
}

#[cfg(test)]
mod tests;
