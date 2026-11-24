//! What a message carries beyond the standard keywords: user-defined
//! parameters, which the standard allows, and extensions, which lenient mode
//! keeps rather than discarding.

use crate::block::{Block, BlockWriter, Entry, Fields, ValueKind};
use crate::error::{ErrorKind, Result};
use crate::{ParseMode, WriteOptions};

/// The `USER_DEFINED_*` parameters of an OPM or OMM.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct UserDefined {
    /// Comments at the start of the block.
    pub comments: Vec<String>,
    /// Parameters in message order.
    pub parameters: Vec<UserParameter>,
}

/// One `USER_DEFINED_<name> = value` parameter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UserParameter {
    /// The name after `USER_DEFINED_` (the XML `parameter` attribute).
    pub name: String,
    /// The value, as text.
    pub value: String,
}

/// A keyword the message type does not define, kept by lenient mode.
///
/// Space-Track's JSON, for instance, adds `TLE_LINE1`, `GP_ID` and
/// `DECAY_DATE` to every OMM.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Extension {
    /// The keyword or JSON member name.
    pub keyword: String,
    /// The value.
    pub value: ExtensionValue,
    /// The unit an XML `units` attribute gave, if any.
    pub unit: Option<String>,
}

/// The value of an [`Extension`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExtensionValue {
    /// Text: any KVN or XML value, or a JSON string.
    Text(String),
    /// A JSON number, in its JSON spelling.
    Number(String),
    /// JSON `null`.
    Null,
}

/// The keyword prefix of user-defined parameters.
pub(crate) const PREFIX: &str = "USER_DEFINED_";

/// Reads a user-defined block; anything without the prefix is undefined.
pub(crate) fn read(
    block: Block,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<UserDefined> {
    let mut parameters: Vec<UserParameter> = Vec::new();
    let mut rest = Block::starting_at(block.at);
    for entry in block.entries {
        let name = entry.keyword.strip_prefix(PREFIX).filter(|n| !n.is_empty());
        match name {
            Some(name) if parameters.iter().any(|p| p.name == name) => {
                return Err(entry.error(ErrorKind::DuplicateKeyword));
            }
            Some(name) => parameters.push(UserParameter {
                name: name.to_owned(),
                value: entry.value.clone(),
            }),
            None => rest.entries.push(entry),
        }
    }
    let (_, fields) = Fields::new(rest, mode)?;
    fields.finish(extensions)?;
    Ok(UserDefined {
        comments: block.comments,
        parameters,
    })
}

/// Writes a user-defined block.
pub(crate) fn write(user: &UserDefined, options: WriteOptions) -> Block {
    let mut out = BlockWriter::new(&user.comments, options);
    for parameter in &user.parameters {
        out.text(&format!("{PREFIX}{}", parameter.name), &parameter.value);
    }
    out.finish()
}

/// Writes kept extensions as a trailing block of their own.
pub(crate) fn write_extensions(extensions: &[Extension]) -> Block {
    let entries = extensions
        .iter()
        .map(|extension| {
            let (value, kind) = match &extension.value {
                ExtensionValue::Text(text) => (text.clone(), ValueKind::JsonString),
                ExtensionValue::Number(number) => (number.clone(), ValueKind::JsonNumber),
                ExtensionValue::Null => (String::new(), ValueKind::JsonNull),
            };
            Entry {
                keyword: extension.keyword.clone(),
                value,
                unit: extension.unit.clone(),
                at: None,
                kind,
            }
        })
        .collect();
    Block {
        entries,
        ..Block::default()
    }
}
