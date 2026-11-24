//! CDM XML reader and writer (`<cdm>`).

use super::cov::CdmCovariance;
use super::kvn::validate_segments;
use super::layout::{self, VERSION_KEY};
use super::segment;
use super::{
    Cdm, CdmAdditionalParameters, CdmData, CdmOdParameters, CdmRelativeMetadata, CdmSegment,
    CdmStateVector,
};
use crate::block::{Block, BlockWriter, Fields};
use crate::error::{ErrorKind, Result};
use crate::extension::{self, Extension, ExtensionValue};
use crate::xml::flat::version_entry;
use crate::xml::tree::Element;
use crate::xml::writer::XmlWriter;
use crate::{ParseMode, WriteOptions, xml};

pub(super) fn read(root: &Element, mode: ParseMode) -> Result<Cdm> {
    if root.name != "cdm" {
        return Err(root.error(ErrorKind::WrongMessage {
            expected: "CDM",
            found: root.name.clone(),
        }));
    }
    let version = layout::check_cdm_version(&version_entry(root, VERSION_KEY, mode)?, mode)?;
    let mut extensions = Vec::new();
    let header = layout::read_header(
        root.required_child("header")?.block()?,
        mode,
        &mut extensions,
    )?;
    let body = root.required_child("body")?;
    let relative_metadata = read_relative_xml(
        body.required_child("relativeMetadataData")?,
        mode,
        &mut extensions,
    )?;
    let mut segs = Vec::new();
    let mut user_defined = None;
    for child in &body.children {
        match child.name.as_str() {
            "relativeMetadataData" => {}
            "segment" => segs.push(read_segment(child, mode, &mut extensions)?),
            "userDefinedParameters" if user_defined.is_none() => {
                user_defined = Some(extension::read(
                    read_user_block(child)?,
                    mode,
                    &mut extensions,
                )?);
            }
            _ if mode == ParseMode::Lenient && child.children.is_empty() => {
                let e = child.entry(child.name.clone());
                extensions.push(Extension {
                    keyword: e.keyword,
                    value: ExtensionValue::Text(e.value),
                    unit: e.unit,
                });
            }
            _ => return Err(child.unexpected()),
        }
    }
    let mut seg_iter = segs.into_iter();
    let seg1 = seg_iter.next().ok_or_else(|| {
        body.error(ErrorKind::UnexpectedEnd {
            expected: "segment",
        })
    })?;
    let seg2 = seg_iter.next().ok_or_else(|| {
        body.error(ErrorKind::UnexpectedEnd {
            expected: "segment",
        })
    })?;
    if seg_iter.next().is_some() {
        return Err(body.unexpected());
    }
    validate_segments(&seg1, &seg2, root.at)?;
    Ok(Cdm {
        version,
        header,
        relative_metadata,
        object1: seg1,
        object2: seg2,
        user_defined,
        extensions,
    })
}

fn read_relative_xml(
    el: &Element,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<CdmRelativeMetadata> {
    let mut block = Block::starting_at(Some(el.at));
    for child in &el.children {
        if child.name == "COMMENT" && child.children.is_empty() {
            block.comments.push(child.text.trim().to_owned());
        } else if child.name == "relativeStateVector" {
            let sub = child.block()?;
            block.comments.extend(sub.comments);
            block.entries.extend(sub.entries);
        } else if child.children.is_empty() {
            block.entries.push(child.entry(child.name.clone()));
        } else {
            return Err(child.unexpected());
        }
    }
    let (comments, mut fields) = Fields::new(block, mode)?;
    let rel = layout::read_relative(comments, &mut fields, mode)?;
    fields.finish(extensions)?;
    Ok(rel)
}

fn read_segment(
    seg: &Element,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<CdmSegment> {
    let metadata =
        segment::read_metadata(seg.required_child("metadata")?.block()?, mode, extensions)?;
    let data = read_data(seg.required_child("data")?, mode, extensions)?;
    Ok(CdmSegment { metadata, data })
}

struct PartialData {
    comments: Vec<String>,
    od: Option<CdmOdParameters>,
    additional: Option<CdmAdditionalParameters>,
    state: Option<CdmStateVector>,
    cov: Option<CdmCovariance>,
}

fn read_data(el: &Element, mode: ParseMode, extensions: &mut Vec<Extension>) -> Result<CdmData> {
    let mut p = PartialData {
        comments: Vec::new(),
        od: None,
        additional: None,
        state: None,
        cov: None,
    };
    for child in &el.children {
        match child.name.as_str() {
            "COMMENT" if child.children.is_empty() => p.comments.push(child.text.trim().to_owned()),
            "odParameters" => {
                p.od = Some(parse_child(child, mode, extensions, segment::read_od)?);
            }
            "additionalParameters" => {
                p.additional = Some(parse_child(
                    child,
                    mode,
                    extensions,
                    segment::read_additional,
                )?);
            }
            "stateVector" => {
                p.state = Some(parse_child(
                    child,
                    mode,
                    extensions,
                    layout::read_state_vector,
                )?);
            }
            "covarianceMatrix" => {
                p.cov = Some(parse_child(child, mode, extensions, |c, f| {
                    CdmCovariance::read(c, f, mode)
                })?);
            }
            _ if mode == ParseMode::Lenient && child.children.is_empty() => {
                let e = child.entry(child.name.clone());
                extensions.push(Extension {
                    keyword: e.keyword,
                    value: ExtensionValue::Text(e.value),
                    unit: e.unit,
                });
            }
            _ => return Err(child.unexpected()),
        }
    }
    let state_vector = p
        .state
        .ok_or_else(|| el.error(ErrorKind::MissingKeyword).with_keyword("X"))?;
    let covariance = p
        .cov
        .ok_or_else(|| el.error(ErrorKind::MissingKeyword).with_keyword("CR_R"))?;
    Ok(CdmData {
        comments: p.comments,
        od_parameters: p.od,
        additional_parameters: p.additional,
        state_vector,
        covariance,
    })
}

fn parse_child<T>(
    child: &Element,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
    parse: impl FnOnce(Vec<String>, &mut Fields) -> Result<T>,
) -> Result<T> {
    let (comments, mut fields) = Fields::new(child.block()?, mode)?;
    let val = parse(comments, &mut fields)?;
    fields.finish(extensions)?;
    Ok(val)
}

fn read_user_block(element: &Element) -> Result<Block> {
    let mut block = Block::starting_at(Some(element.at));
    for child in &element.children {
        match (child.name.as_str(), child.attribute("parameter")) {
            ("COMMENT", _) if child.children.is_empty() => {
                block.comments.push(child.text.trim().to_owned());
            }
            ("USER_DEFINED", Some(parameter)) if child.children.is_empty() => {
                block
                    .entries
                    .push(child.entry(format!("{}{parameter}", extension::PREFIX)));
            }
            _ => return Err(child.unexpected()),
        }
    }
    Ok(block)
}

pub(super) fn write(
    cdm: &Cdm,
    out: &mut XmlWriter,
    options: WriteOptions,
    standalone: bool,
) -> Result<()> {
    let mut attributes = xml::root_attributes(cdm.version, standalone);
    attributes.push(("id", VERSION_KEY));
    attributes.push(("version", cdm.version.as_str()));
    out.open("cdm", &attributes);
    out.block("header", &layout::write_header(&cdm.header, options), None);
    out.open("body", &[]);
    write_relative_xml(out, &cdm.relative_metadata, options)?;
    write_segment_xml(out, &cdm.object1, options)?;
    write_segment_xml(out, &cdm.object2, options)?;
    if let Some(user) = &cdm.user_defined {
        out.block(
            "userDefinedParameters",
            &extension::write(user, options),
            Some(extension::PREFIX),
        );
    }
    if !cdm.extensions.is_empty() {
        out.entries(&extension::write_extensions(&cdm.extensions), None);
    }
    out.close("body");
    out.close("cdm");
    Ok(())
}

fn write_relative_xml(
    out: &mut XmlWriter,
    rel: &CdmRelativeMetadata,
    options: WriteOptions,
) -> Result<()> {
    out.open("relativeMetadataData", &[]);
    let mut head = BlockWriter::new(&rel.comments, options);
    layout::write_relative_head(rel, &mut head)?;
    out.entries(&head.finish(), None);
    if let Some(state) = &rel.relative_state {
        let mut sv = BlockWriter::new(&[], options);
        layout::write_relative_state(state, &mut sv)?;
        out.block("relativeStateVector", &sv.finish(), None);
    }
    let mut tail = BlockWriter::new(&[], options);
    layout::write_relative_tail(rel, &mut tail)?;
    out.entries(&tail.finish(), None);
    out.close("relativeMetadataData");
    Ok(())
}

fn write_segment_xml(out: &mut XmlWriter, seg: &CdmSegment, options: WriteOptions) -> Result<()> {
    out.open("segment", &[]);
    out.block(
        "metadata",
        &segment::write_metadata(&seg.metadata, options),
        None,
    );
    out.open("data", &[]);
    out.comments(&seg.data.comments);
    if let Some(od) = &seg.data.od_parameters {
        out.block("odParameters", &segment::write_od(od, options)?, None);
    }
    if let Some(add) = &seg.data.additional_parameters {
        out.block(
            "additionalParameters",
            &segment::write_additional(add, options)?,
            None,
        );
    }
    out.block(
        "stateVector",
        &layout::write_state_vector(&seg.data.state_vector, options)?,
        None,
    );
    out.block(
        "covarianceMatrix",
        &seg.data.covariance.write(options)?,
        None,
    );
    out.close("data");
    out.close("segment");
    Ok(())
}
