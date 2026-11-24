//! TDM XML reader and writer (`<tdm>`).

use super::layout::{self, VERSION_KEY};
use super::{Tdm, TdmData, TdmObservation, TdmSegment};
use crate::block::{Block, Fields};
use crate::error::{Error, ErrorKind, Result};
use crate::extension::{self, Extension, ExtensionValue};
use crate::header::Header;
use crate::xml::flat::version_entry;
use crate::xml::tree::Element;
use crate::xml::writer::XmlWriter;
use crate::{ParseMode, WriteOptions, number, xml};

pub(super) fn read(root: &Element, mode: ParseMode) -> Result<Tdm> {
    if root.name != "tdm" {
        return Err(root.error(ErrorKind::WrongMessage {
            expected: "TDM",
            found: root.name.clone(),
        }));
    }
    let version = layout::check_tdm_version(&version_entry(root, VERSION_KEY, mode)?, mode)?;
    let mut extensions = Vec::new();
    let header = Header::read(
        root.required_child("header")?.block()?,
        mode,
        &mut extensions,
    )?;
    let body = root.required_child("body")?;
    let mut segments = Vec::new();
    for child in &body.children {
        if child.name == "segment" {
            segments.push(read_segment(child, mode, &mut extensions)?);
        } else if mode == ParseMode::Lenient && child.children.is_empty() {
            let e = child.entry(child.name.clone());
            extensions.push(Extension {
                keyword: e.keyword,
                value: ExtensionValue::Text(e.value),
                unit: e.unit,
            });
        } else {
            return Err(child.unexpected());
        }
    }
    if mode == ParseMode::Strict && segments.is_empty() {
        return Err(body.error(ErrorKind::UnexpectedEnd {
            expected: "segment",
        }));
    }
    Ok(Tdm {
        version,
        header,
        segments,
        extensions,
    })
}

fn read_segment(
    seg: &Element,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<TdmSegment> {
    let metadata =
        layout::read_metadata(seg.required_child("metadata")?.block()?, mode, extensions)?;
    let data = read_data(seg.required_child("data")?, mode, extensions)?;
    Ok(TdmSegment { metadata, data })
}

fn read_data(el: &Element, mode: ParseMode, extensions: &mut Vec<Extension>) -> Result<TdmData> {
    let mut data = TdmData::default();
    for child in &el.children {
        match child.name.as_str() {
            "COMMENT" if child.children.is_empty() => {
                data.comments.push(child.text.trim().to_owned());
            }
            "observation" => read_observation(child, &mut data.observations, mode)?,
            _ if mode == ParseMode::Lenient && child.children.is_empty() => {
                let mut b = Block::starting_at(Some(child.at));
                b.entries.push(child.entry(child.name.clone()));
                Fields::new(b, mode)?.1.finish(extensions)?;
            }
            _ => return Err(child.unexpected()),
        }
    }
    Ok(data)
}

fn read_observation(
    obs_el: &Element,
    out: &mut Vec<TdmObservation>,
    mode: ParseMode,
) -> Result<()> {
    let mut epoch = None;
    let mut obs_entries = Vec::new();
    for child in &obs_el.children {
        if !child.children.is_empty() {
            return Err(child.unexpected());
        }
        if child.name == "EPOCH" {
            if epoch.is_some() {
                return Err(child
                    .error(ErrorKind::DuplicateKeyword)
                    .with_keyword("EPOCH"));
            }
            epoch = Some(child.entry("EPOCH".to_owned()).epoch(mode)?);
        } else if child.name != "COMMENT" {
            obs_entries.push(child.entry(child.name.clone()));
        }
    }
    let epoch = epoch.ok_or_else(|| {
        obs_el
            .error(ErrorKind::MissingKeyword)
            .with_keyword("EPOCH")
    })?;
    if obs_entries.is_empty() && mode == ParseMode::Strict {
        return Err(obs_el.error(ErrorKind::UnexpectedEnd {
            expected: "tracking observable",
        }));
    }
    for entry in obs_entries {
        let kind = layout::parse_observable_keyword(&entry, mode)?;
        let value = layout::parse_scalar_value(entry.value.trim(), &entry.keyword, entry.at, mode)?;
        out.push(TdmObservation {
            epoch: epoch.clone(),
            kind,
            value,
        });
    }
    Ok(())
}

pub(super) fn write(
    tdm: &Tdm,
    out: &mut XmlWriter,
    options: WriteOptions,
    standalone: bool,
) -> Result<()> {
    let mut attributes = xml::root_attributes(tdm.version, standalone);
    attributes.push(("id", VERSION_KEY));
    attributes.push(("version", tdm.version.as_str()));
    out.open("tdm", &attributes);
    out.block("header", &tdm.header.write(options), None);
    out.open("body", &[]);
    for seg in &tdm.segments {
        out.open("segment", &[]);
        out.block(
            "metadata",
            &layout::write_metadata(&seg.metadata, options)?,
            None,
        );
        out.open("data", &[]);
        out.comments(&seg.data.comments);
        for obs in &seg.data.observations {
            let key = obs.kind.as_str();
            let val_str = number::format_f64(obs.value)
                .ok_or_else(|| Error::new(ErrorKind::NonFiniteNumber).with_keyword(key.clone()))?;
            out.open("observation", &[]);
            out.leaf("EPOCH", &[], &obs.epoch.to_string());
            out.leaf(&key, &[], &val_str);
            out.close("observation");
        }
        out.close("data");
        out.close("segment");
    }
    if !tdm.extensions.is_empty() {
        out.entries(&extension::write_extensions(&tdm.extensions), None);
    }
    out.close("body");
    out.close("tdm");
    Ok(())
}
