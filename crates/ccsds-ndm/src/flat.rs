//! The layout of single-segment messages (OPM, OMM): a version, a header, a
//! metadata block, and data blocks, with no markers between them in KVN.
//!
//! A [`FlatSchema`] names each block's keywords (for KVN) and element (for
//! XML), so both notations read into, and write from, one [`FlatMessage`].

use crate::ParseMode;
use crate::block::{Block, Entry, Fields};
use crate::error::Result;
use crate::extension::Extension;

/// One logical block of a flat message.
pub(crate) struct Section {
    /// The XML element that holds the block.
    pub element: &'static str,
    /// The keywords the block holds.
    pub keywords: &'static [&'static str],
    /// A keyword prefix the block also holds (`USER_DEFINED_`).
    pub prefix: Option<&'static str>,
    /// For a block that may repeat, the keyword that opens each instance.
    pub repeats_at: Option<&'static str>,
}

impl Section {
    /// Whether this block defines `keyword`.
    pub fn holds(&self, keyword: &str) -> bool {
        self.keywords.contains(&keyword) || self.prefix.is_some_and(|p| keyword.starts_with(p))
    }
}

/// Index of the header in every schema.
pub(crate) const HEADER: usize = 0;
/// Index of the metadata in every schema.
pub(crate) const METADATA: usize = 1;

/// The blocks of one message type, header and metadata first.
pub(crate) struct FlatSchema {
    /// Message name for errors (`OPM`).
    pub name: &'static str,
    /// The XML root element (`opm`).
    pub root: &'static str,
    /// The KVN version keyword (`CCSDS_OPM_VERS`).
    pub version_key: &'static str,
    /// Every block, in the order the standard writes them.
    pub sections: &'static [Section],
}

/// A message's version and blocks, tagged with their section index.
pub(crate) struct FlatMessage {
    /// The `CCSDS_xxx_VERS` value; JSON OMMs may omit it.
    pub version: Option<Entry>,
    /// Blocks in message order.
    pub blocks: Vec<(usize, Block)>,
}

impl FlatMessage {
    /// Removes and returns the first block of a section.
    pub fn take(&mut self, section: usize) -> Option<Block> {
        let at = self.blocks.iter().position(|(s, _)| *s == section)?;
        Some(self.blocks.remove(at).1)
    }

    /// Removes and returns every block of a section, in order.
    pub fn take_all(&mut self, section: usize) -> Vec<Block> {
        let (taken, kept) = std::mem::take(&mut self.blocks)
            .into_iter()
            .partition(|(s, _)| *s == section);
        self.blocks = kept;
        taken.into_iter().map(|(_, block)| block).collect()
    }
}

/// Reads a block that may be absent.
pub(crate) fn optional_block<T>(
    flat: &mut FlatMessage,
    section: usize,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
    read: impl FnOnce(Vec<String>, &mut Fields) -> Result<T>,
) -> Result<Option<T>> {
    let Some(block) = flat.take(section) else {
        return Ok(None);
    };
    let (comments, mut fields) = Fields::new(block, mode)?;
    let value = read(comments, &mut fields)?;
    fields.finish(extensions)?;
    Ok(Some(value))
}
