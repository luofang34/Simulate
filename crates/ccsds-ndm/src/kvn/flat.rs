//! Reads and writes messages whose KVN form has no block markers (OPM, OMM): each
//! keyword names the logical block it belongs to, and blocks follow a fixed
//! order.
//!
//! Comments attach to the block of the keyword that follows them, which is
//! where the standard places them (at the start of a block). A comment in the
//! middle of a block joins that block's leading comments.

use super::lexer::Line;
use super::writer::KvnWriter;
use crate::ParseMode;
use crate::block::{Block, Entry};
use crate::error::{Error, ErrorKind, Result};
use crate::flat::{FlatMessage, FlatSchema, Section};

/// Groups a message's lines into blocks.
///
/// `lines` must start with the version line. Unknown keywords stay in the
/// block they appear in, for the block reader to reject or keep.
pub(crate) fn read(lines: Vec<Line>, schema: &FlatSchema, mode: ParseMode) -> Result<FlatMessage> {
    let version_key = schema.version_key;
    let sections = schema.sections;
    let mut lines = lines.into_iter();
    let version = match lines.next() {
        Some(Line::Pair(entry)) if entry.keyword == version_key => entry,
        Some(other) => {
            return Err(Error::new(ErrorKind::WrongMessage {
                expected: version_key,
                found: describe(&other),
            })
            .at(other.at()));
        }
        None => return Err(Error::new(ErrorKind::Empty)),
    };
    let mut blocks: Vec<(usize, Block)> = Vec::new();
    let mut pending: Vec<String> = Vec::new();
    for line in lines {
        let entry = match line {
            Line::Comment { text, .. } => {
                pending.push(text);
                continue;
            }
            Line::Bare { text, at } => {
                return Err(Error::new(ErrorKind::UnexpectedLine { text }).at(at));
            }
            Line::Pair(entry) => entry,
        };
        place(&mut blocks, sections, entry, &mut pending, mode)?;
    }
    if let Some((_, block)) = blocks.last_mut() {
        block.comments.append(&mut pending);
    } else if !pending.is_empty() {
        blocks.push((
            0,
            Block {
                comments: pending,
                ..Block::default()
            },
        ));
    }
    Ok(FlatMessage {
        version: Some(version),
        blocks,
    })
}

/// Adds an entry to the block its keyword belongs to, opening a block when
/// the keyword starts a new one.
fn place(
    blocks: &mut Vec<(usize, Block)>,
    sections: &[Section],
    entry: Entry,
    pending: &mut Vec<String>,
    mode: ParseMode,
) -> Result<()> {
    let current = blocks.last().map(|(section, _)| *section);
    let section = sections
        .iter()
        .position(|s| s.holds(&entry.keyword))
        .or(current)
        .unwrap_or(0);
    if current.is_some_and(|c| section < c) {
        if mode == ParseMode::Strict {
            return Err(entry.error(ErrorKind::KeywordOutOfOrder));
        }
        place_out_of_order(blocks, section, entry, pending);
        return Ok(());
    }
    // A repeating block (a maneuver) starts again at its opening keyword or
    // when a keyword recurs; any other block keeps its duplicates for the
    // block reader to reject.
    let opener = sections.get(section).and_then(|s| s.repeats_at);
    let restart = opener.is_some()
        && (opener == Some(entry.keyword.as_str())
            || blocks
                .last()
                .is_some_and(|(_, block)| block.has(&entry.keyword)));
    match blocks.last_mut() {
        Some((_, block)) if current == Some(section) && !restart => {
            block.comments.append(pending);
            block.entries.push(entry);
        }
        _ => {
            let mut block = Block::starting_at(entry.at);
            block.comments = std::mem::take(pending);
            block.entries.push(entry);
            blocks.push((section, block));
        }
    }
    Ok(())
}

/// Lenient placement of a keyword whose block has already ended: it joins the
/// last block of its own section, or opens one.
fn place_out_of_order(
    blocks: &mut Vec<(usize, Block)>,
    section: usize,
    entry: Entry,
    pending: &mut Vec<String>,
) {
    let target = blocks.iter_mut().rev().find(|(s, _)| *s == section);
    match target {
        Some((_, block)) => {
            block.comments.append(pending);
            block.entries.push(entry);
        }
        None => {
            let mut block = Block::starting_at(entry.at);
            block.comments = std::mem::take(pending);
            block.entries.push(entry);
            let at = blocks
                .iter()
                .position(|(s, _)| *s > section)
                .unwrap_or(blocks.len());
            blocks.insert(at, (section, block));
        }
    }
}

/// Writes a flat message: the version line, then each block after a blank
/// line.
pub(crate) fn write(message: &FlatMessage, schema: &FlatSchema) -> Result<String> {
    let mut out = KvnWriter::default();
    let version = message
        .version
        .as_ref()
        .ok_or_else(|| Error::new(ErrorKind::MissingKeyword).with_keyword(schema.version_key))?;
    out.pair(schema.version_key, &version.value);
    for (_, block) in &message.blocks {
        out.blank();
        out.block(block)?;
    }
    Ok(out.finish())
}

/// A short description of a line for error messages.
pub(crate) fn describe(line: &Line) -> String {
    match line {
        Line::Comment { .. } => "COMMENT".to_owned(),
        Line::Pair(entry) => entry.keyword.clone(),
        Line::Bare { text, .. } => text.chars().take(40).collect(),
    }
}

#[cfg(test)]
mod tests;
