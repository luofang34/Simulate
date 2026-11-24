//! The message version and the header every ODM shares.

use std::fmt;

use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::{ErrorKind, Result};
use crate::extension::Extension;
use crate::{Epoch, ParseMode, WriteOptions};

/// The `CCSDS_xxx_VERS` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Version {
    /// `1.0`, ODM 502.0-B-1.
    V1,
    /// `2.0`, ODM 502.0-B-2 (what CelesTrak serves).
    V2,
    /// `3.0`, ODM 502.0-B-3.
    V3,
}

impl Version {
    /// The version as written.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::V1 => "1.0",
            Self::V2 => "2.0",
            Self::V3 => "3.0",
        }
    }

    /// Reads an exact version value.
    pub fn from_name(text: &str) -> Option<Self> {
        match text {
            "1.0" => Some(Self::V1),
            "2.0" => Some(Self::V2),
            "3.0" => Some(Self::V3),
            _ => None,
        }
    }

    pub(crate) fn parse(entry: &Entry, _: ParseMode) -> Result<Self> {
        Self::from_name(&entry.value).ok_or_else(|| {
            entry.error(ErrorKind::UnsupportedVersion {
                text: entry.value.clone(),
            })
        })
    }
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The ODM header.
///
/// `CREATION_DATE` and `ORIGINATOR` are mandatory in the standard but are
/// `None` when a lenient read finds them absent or blank, as CelesTrak sends
/// them.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Header {
    /// Comments at the start of the header.
    pub comments: Vec<String>,
    /// `CLASSIFICATION` (version 3).
    pub classification: Option<String>,
    /// `CREATION_DATE`, in UTC.
    pub creation_date: Option<Epoch>,
    /// `ORIGINATOR`.
    pub originator: Option<String>,
    /// `MESSAGE_ID` (version 3).
    pub message_id: Option<String>,
}

/// Header keywords other than the version line.
pub(crate) const HEADER_KEYWORDS: &[&str] = &[
    "CLASSIFICATION",
    "CREATION_DATE",
    "ORIGINATOR",
    "MESSAGE_ID",
];

impl Header {
    pub(crate) fn read(
        block: Block,
        mode: ParseMode,
        extensions: &mut Vec<Extension>,
    ) -> Result<Self> {
        let (comments, mut fields) = Fields::new(block, mode)?;
        let header = Self {
            comments,
            classification: fields.optional("CLASSIFICATION", Entry::string)?,
            creation_date: fields.expected("CREATION_DATE", Entry::epoch)?,
            originator: fields.expected("ORIGINATOR", Entry::string)?,
            message_id: fields.optional("MESSAGE_ID", Entry::string)?,
        };
        fields.finish(extensions)?;
        Ok(header)
    }

    pub(crate) fn write(&self, options: WriteOptions) -> Block {
        let mut out = BlockWriter::new(&self.comments, options);
        out.text_opt("CLASSIFICATION", self.classification.as_ref());
        out.text_opt("CREATION_DATE", self.creation_date.as_ref());
        out.text_opt("ORIGINATOR", self.originator.as_ref());
        out.text_opt("MESSAGE_ID", self.message_id.as_ref());
        out.finish()
    }
}
