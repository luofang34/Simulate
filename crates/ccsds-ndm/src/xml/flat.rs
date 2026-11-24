//! Maps a single-segment message element (`<opm>`, `<omm>`) to and from a
//! [`FlatMessage`].

use super::tree::Element;
use super::writer::XmlWriter;
use crate::ParseMode;
use crate::block::{Block, Entry};
use crate::error::{ErrorKind, Result};
use crate::flat::{FlatMessage, FlatSchema, HEADER, METADATA};

/// Reads a message element.
///
/// Comments placed directly in `<data>` join the block that follows them, as
/// they would in KVN. A leaf element `<data>` does not define is kept for the
/// block reader to reject (strict) or keep (lenient).
pub(crate) fn read(root: &Element, schema: &FlatSchema, mode: ParseMode) -> Result<FlatMessage> {
    if root.name != schema.root {
        return Err(root.error(ErrorKind::WrongMessage {
            expected: schema.name,
            found: root.name.clone(),
        }));
    }
    let version = version_entry(root, schema.version_key, mode)?;
    let mut blocks = vec![(HEADER, root.required_child("header")?.block()?)];
    let segment = root.required_child("body")?.required_child("segment")?;
    blocks.push((METADATA, segment.required_child("metadata")?.block()?));
    let data = segment.required_child("data")?;
    let mut pending: Vec<String> = Vec::new();
    let mut extras = Block::starting_at(Some(data.at));
    for child in &data.children {
        if child.name == "COMMENT" {
            pending.push(child.text.trim().to_owned());
            continue;
        }
        let Some(section) = schema
            .sections
            .iter()
            .skip(METADATA.saturating_add(1))
            .position(|s| s.element == child.name)
            .map(|index| index.saturating_add(METADATA).saturating_add(1))
        else {
            if !child.children.is_empty() {
                return Err(child.unexpected());
            }
            extras.entries.push(child.entry(child.name.clone()));
            continue;
        };
        let prefix = schema.sections.get(section).and_then(|s| s.prefix);
        let mut block = match prefix {
            Some(prefix) => user_block(child, prefix)?,
            None => child.block()?,
        };
        if !pending.is_empty() {
            pending.append(&mut block.comments);
            block.comments = std::mem::take(&mut pending);
        }
        blocks.push((section, block));
    }
    if let Some((_, last)) = blocks.last_mut() {
        last.comments.append(&mut pending);
        last.entries.append(&mut extras.entries);
    }
    Ok(FlatMessage {
        version: Some(version),
        blocks,
    })
}

/// The `version` attribute as the version entry; strict mode also checks the
/// `id` attribute names the message.
pub(crate) fn version_entry(root: &Element, key: &'static str, mode: ParseMode) -> Result<Entry> {
    if mode == ParseMode::Strict && root.attribute("id") != Some(key) {
        return Err(root.error(ErrorKind::InvalidValue {
            text: root.attribute("id").unwrap_or_default().to_owned(),
            expected: "an id attribute naming the message version keyword",
        }));
    }
    let version = root
        .attribute("version")
        .ok_or_else(|| root.error(ErrorKind::MissingKeyword).with_keyword(key))?;
    Ok(Entry::text(key, version.trim(), root.at))
}

/// `<USER_DEFINED parameter="X">v</USER_DEFINED>` as `USER_DEFINED_X = v`.
fn user_block(element: &Element, prefix: &str) -> Result<Block> {
    let mut block = Block::starting_at(Some(element.at));
    for child in &element.children {
        match (child.name.as_str(), child.attribute("parameter")) {
            ("COMMENT", _) => block.comments.push(child.text.trim().to_owned()),
            ("USER_DEFINED", Some(parameter)) if child.children.is_empty() => {
                block
                    .entries
                    .push(child.entry(format!("{prefix}{parameter}")));
            }
            _ => return Err(child.unexpected()),
        }
    }
    Ok(block)
}

/// Writes a message element; `attributes` go on the root before `id` and
/// `version`.
pub(crate) fn write(
    out: &mut XmlWriter,
    message: &FlatMessage,
    schema: &FlatSchema,
    attributes: &[(&str, &str)],
) {
    let mut root = attributes.to_vec();
    root.push(("id", schema.version_key));
    if let Some(version) = &message.version {
        root.push(("version", &version.value));
    }
    out.open(schema.root, &root);
    let block = |index: usize| message.blocks.iter().filter(move |(s, _)| *s == index);
    for (_, header) in block(HEADER) {
        out.block("header", header, None);
    }
    out.open("body", &[]);
    out.open("segment", &[]);
    for (_, metadata) in block(METADATA) {
        out.block("metadata", metadata, None);
    }
    out.open("data", &[]);
    for (index, data) in &message.blocks {
        if *index <= METADATA {
            continue;
        }
        match schema.sections.get(*index) {
            Some(section) => out.block(section.element, data, section.prefix),
            None => out.entries(data, None),
        }
    }
    out.close("data");
    out.close("segment");
    out.close("body");
    out.close(schema.root);
}
