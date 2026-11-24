//! The OEM in XML: `<segment>`s of `<metadata>` and `<data>`, the data
//! holding `<stateVector>` and `<covarianceMatrix>` elements.

use super::layout::{self, VERSION_KEY};
use super::{Oem, OemData, OemSegment};
use crate::block::Fields;
use crate::error::{ErrorKind, Result};
use crate::extension::Extension;
use crate::header::{Header, Version};
use crate::xml::flat::version_entry;
use crate::xml::tree::Element;
use crate::xml::writer::XmlWriter;
use crate::{ParseMode, WriteOptions, xml};

pub(super) fn read(root: &Element, mode: ParseMode) -> Result<Oem> {
    if root.name != "oem" {
        return Err(root.error(ErrorKind::WrongMessage {
            expected: "OEM",
            found: root.name.clone(),
        }));
    }
    let version = Version::parse(&version_entry(root, VERSION_KEY, mode)?, mode)?;
    let mut extensions = Vec::new();
    let header = Header::read(
        root.required_child("header")?.block()?,
        mode,
        &mut extensions,
    )?;
    let body = root.required_child("body")?;
    let mut segments = Vec::new();
    for segment in &body.children {
        if segment.name != "segment" {
            return Err(segment.unexpected());
        }
        let metadata = layout::read_metadata(
            segment.required_child("metadata")?.block()?,
            mode,
            &mut extensions,
        )?;
        let data = read_data(segment.required_child("data")?, mode, &mut extensions)?;
        segments.push(OemSegment { metadata, data });
    }
    if segments.is_empty() {
        return Err(body.error(ErrorKind::MissingElement { name: "segment" }));
    }
    Ok(Oem {
        version,
        header,
        segments,
        extensions,
    })
}

fn read_data(data: &Element, mode: ParseMode, extensions: &mut Vec<Extension>) -> Result<OemData> {
    let mut out = OemData::default();
    for child in &data.children {
        match child.name.as_str() {
            "COMMENT" => out.comments.push(child.text.trim().to_owned()),
            "stateVector" => {
                let (comments, mut fields) = Fields::new(child.block()?, mode)?;
                // State vectors have no comments of their own in this model.
                out.comments.extend(comments);
                out.states.push(layout::read_state(&mut fields)?);
                fields.finish(extensions)?;
            }
            "covarianceMatrix" => {
                let (comments, mut fields) = Fields::new(child.block()?, mode)?;
                out.covariances
                    .push(layout::read_covariance(comments, &mut fields)?);
                fields.finish(extensions)?;
            }
            _ if mode == ParseMode::Lenient && child.children.is_empty() => {
                let mut extra = crate::block::Block::default();
                extra.entries.push(child.entry(child.name.clone()));
                Fields::new(extra, mode)?.1.finish(extensions)?;
            }
            _ => return Err(child.unexpected()),
        }
    }
    Ok(out)
}

pub(super) fn write(
    oem: &Oem,
    out: &mut XmlWriter,
    options: WriteOptions,
    standalone: bool,
) -> Result<()> {
    let mut attributes = xml::root_attributes(oem.version, standalone);
    attributes.push(("id", VERSION_KEY));
    attributes.push(("version", oem.version.as_str()));
    out.open("oem", &attributes);
    out.block("header", &oem.header.write(options), None);
    out.open("body", &[]);
    let last = oem.segments.len().saturating_sub(1);
    for (index, segment) in oem.segments.iter().enumerate() {
        out.open("segment", &[]);
        out.block(
            "metadata",
            &layout::write_metadata(&segment.metadata, options),
            None,
        );
        out.open("data", &[]);
        out.comments(&segment.data.comments);
        for state in &segment.data.states {
            out.block("stateVector", &layout::write_state(state, options)?, None);
        }
        for covariance in &segment.data.covariances {
            let block = layout::write_covariance(covariance, options)?;
            out.block("covarianceMatrix", &block, None);
        }
        if index == last && !oem.extensions.is_empty() {
            out.entries(&crate::extension::write_extensions(&oem.extensions), None);
        }
        out.close("data");
        out.close("segment");
    }
    out.close("body");
    out.close("oem");
    Ok(())
}
