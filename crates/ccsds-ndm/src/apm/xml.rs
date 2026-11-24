//! APM XML reader and writer (`<apm>`).

use super::layout::{self, VERSION_KEY};
use super::{Apm, ApmData, ApmEuler, ApmQuaternion};
use crate::adm::check_adm_version;
use crate::block::{Block, BlockWriter, Fields};
use crate::error::{ErrorKind, Result};
use crate::extension::{self, Extension, ExtensionValue};
use crate::header::{Header, Version};
use crate::state::write_vector;
use crate::xml::flat::version_entry;
use crate::xml::tree::Element;
use crate::xml::writer::XmlWriter;
use crate::{ParseMode, WriteOptions, xml};

pub(super) fn read(root: &Element, mode: ParseMode) -> Result<Apm> {
    if root.name != "apm" {
        return Err(root.error(ErrorKind::WrongMessage {
            expected: "APM",
            found: root.name.clone(),
        }));
    }
    let version = check_adm_version(&version_entry(root, VERSION_KEY, mode)?, mode)?;
    let mut extensions = Vec::new();
    let header = Header::read(
        root.required_child("header")?.block()?,
        mode,
        &mut extensions,
    )?;
    let body = root.required_child("body")?;
    let segment = body.required_child("segment")?;
    let metadata = layout::read_metadata(
        segment.required_child("metadata")?.block()?,
        mode,
        &mut extensions,
    )?;
    let data = read_data(segment.required_child("data")?, mode, &mut extensions)?;
    if mode == ParseMode::Lenient {
        for child in &body.children {
            if child.name != "segment" && child.children.is_empty() {
                let e = child.entry(child.name.clone());
                extensions.push(Extension {
                    keyword: e.keyword,
                    value: ExtensionValue::Text(e.value),
                    unit: e.unit,
                });
            }
        }
    }
    layout::validate_data(&data, version, root.at, mode)?;
    Ok(Apm {
        version,
        header,
        metadata,
        data,
        extensions,
    })
}

fn read_data(el: &Element, mode: ParseMode, extensions: &mut Vec<Extension>) -> Result<ApmData> {
    let mut data = ApmData::default();
    let mut pending = Vec::new();
    for child in &el.children {
        read_data_child(child, &mut data, &mut pending, mode, extensions)?;
    }
    data.comments.append(&mut pending);
    Ok(data)
}

fn read_data_child(
    child: &Element,
    data: &mut ApmData,
    pending: &mut Vec<String>,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<()> {
    match child.name.as_str() {
        "COMMENT" => pending.push(child.text.trim().to_owned()),
        "EPOCH" if child.children.is_empty() => read_data_epoch(child, data, pending, mode)?,
        "quaternionState" => {
            if data.quaternion.is_some() {
                return Err(child.unexpected());
            }
            let block = flatten_nested(child, &["quaternion", "quaternionDot", "quaternionRate"])?;
            data.quaternion = Some(parse_block(
                block,
                pending,
                mode,
                extensions,
                layout::read_quaternion,
            )?);
        }
        "eulerAngleState" | "eulerElementsThree" => {
            if data.euler.is_some() {
                return Err(child.unexpected());
            }
            let block = flatten_nested(child, &["rotationAngles", "rotationRates"])?;
            data.euler = Some(parse_block(
                block,
                pending,
                mode,
                extensions,
                layout::read_euler,
            )?);
        }
        "angVel"
        | "angularVelocity"
        | "spin"
        | "eulerElementsSpin"
        | "inertia"
        | "spacecraftParameters"
        | "maneuverParameters"
        | "userDefinedParameters" => {
            read_data_component(child, data, pending, mode, extensions)?;
        }
        _ if mode == ParseMode::Lenient && child.children.is_empty() => {
            let mut extra = Block::starting_at(Some(child.at));
            extra.entries.push(child.entry(child.name.clone()));
            Fields::new(extra, mode)?.1.finish(extensions)?;
        }
        _ => return Err(child.unexpected()),
    }
    Ok(())
}

fn read_data_epoch(
    child: &Element,
    data: &mut ApmData,
    pending: &mut Vec<String>,
    mode: ParseMode,
) -> Result<()> {
    if data.epoch.is_some() {
        return Err(child
            .error(ErrorKind::DuplicateKeyword)
            .with_keyword("EPOCH"));
    }
    let entry = child.entry("EPOCH".to_owned());
    if entry.is_empty() {
        if mode == ParseMode::Strict {
            return Err(entry.error(ErrorKind::EmptyValue));
        }
    } else {
        data.comments.append(pending);
        data.epoch = Some(entry.epoch(mode)?);
    }
    Ok(())
}

fn read_data_component(
    child: &Element,
    data: &mut ApmData,
    pending: &mut Vec<String>,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
) -> Result<()> {
    match child.name.as_str() {
        "angVel" | "angularVelocity" => set_once(
            &mut data.ang_vel,
            child,
            pending,
            mode,
            extensions,
            layout::read_ang_vel,
        ),
        "spin" | "eulerElementsSpin" => set_once(
            &mut data.spin,
            child,
            pending,
            mode,
            extensions,
            layout::read_spin,
        ),
        "inertia" | "spacecraftParameters" => set_once(
            &mut data.inertia,
            child,
            pending,
            mode,
            extensions,
            layout::read_inertia,
        ),
        "maneuverParameters" => {
            data.maneuvers.push(read_simple(
                child,
                pending,
                mode,
                extensions,
                layout::read_maneuver,
            )?);
            Ok(())
        }
        "userDefinedParameters" => {
            if data.user_defined.is_some() {
                return Err(child.unexpected());
            }
            let mut parsed = extension::read(read_user_block(child)?, mode, extensions)?;
            prepend_comments(&mut parsed.comments, pending);
            data.user_defined = Some(parsed);
            Ok(())
        }
        _ => Err(child.unexpected()),
    }
}

fn set_once<T>(
    slot: &mut Option<T>,
    child: &Element,
    pending: &mut Vec<String>,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
    parse: impl FnOnce(Vec<String>, &mut Fields) -> Result<T>,
) -> Result<()> {
    if slot.is_some() {
        return Err(child.unexpected());
    }
    *slot = Some(read_simple(child, pending, mode, extensions, parse)?);
    Ok(())
}

fn parse_block<T>(
    mut block: Block,
    pending: &mut Vec<String>,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
    parse: impl FnOnce(Vec<String>, &mut Fields) -> Result<T>,
) -> Result<T> {
    prepend_comments(&mut block.comments, pending);
    let (c, mut f) = Fields::new(block, mode)?;
    let value = parse(c, &mut f)?;
    f.finish(extensions)?;
    Ok(value)
}

fn read_simple<T>(
    child: &Element,
    pending: &mut Vec<String>,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
    parse: impl FnOnce(Vec<String>, &mut Fields) -> Result<T>,
) -> Result<T> {
    parse_block(child.block()?, pending, mode, extensions, parse)
}

fn prepend_comments(target: &mut Vec<String>, pending: &mut Vec<String>) {
    if !pending.is_empty() {
        pending.append(target);
        *target = std::mem::take(pending);
    }
}

fn flatten_nested(el: &Element, nested: &[&str]) -> Result<Block> {
    let mut block = Block::starting_at(Some(el.at));
    for child in &el.children {
        if child.name == "COMMENT" && child.children.is_empty() {
            block.comments.push(child.text.trim().to_owned());
        } else if nested.contains(&child.name.as_str()) {
            let sub = child.block()?;
            block.comments.extend(sub.comments);
            block.entries.extend(sub.entries);
        } else if child.children.is_empty() {
            block.entries.push(child.entry(child.name.clone()));
        } else {
            return Err(child.unexpected());
        }
    }
    Ok(block)
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
    apm: &Apm,
    out: &mut XmlWriter,
    options: WriteOptions,
    standalone: bool,
) -> Result<()> {
    let mut attributes = xml::root_attributes(apm.version, standalone);
    attributes.push(("id", VERSION_KEY));
    attributes.push(("version", apm.version.as_str()));
    out.open("apm", &attributes);
    out.block("header", &apm.header.write(options), None);
    out.open("body", &[]);
    out.open("segment", &[]);
    out.block(
        "metadata",
        &layout::write_metadata(&apm.metadata, options),
        None,
    );
    out.open("data", &[]);
    write_data_children(out, apm, options)?;
    out.close("data");
    out.close("segment");
    if !apm.extensions.is_empty() {
        out.entries(&extension::write_extensions(&apm.extensions), None);
    }
    out.close("body");
    out.close("apm");
    Ok(())
}

fn write_data_children(out: &mut XmlWriter, apm: &Apm, options: WriteOptions) -> Result<()> {
    let data = &apm.data;
    out.comments(&data.comments);
    if let Some(epoch) = &data.epoch {
        out.leaf("EPOCH", &[], &epoch.to_string());
    }
    if let Some(q) = &data.quaternion {
        write_quat_xml(out, q, apm.version, options)?;
    }
    if let Some(euler) = &data.euler {
        write_euler_xml(out, euler, apm.version, options)?;
    }
    if let Some(av) = &data.ang_vel {
        out.block(
            "angVel",
            &layout::write_ang_vel(av, apm.version, options)?,
            None,
        );
    }
    if let Some(spin) = &data.spin {
        let tag = if apm.version == Version::V1 {
            "eulerElementsSpin"
        } else {
            "spin"
        };
        out.block(tag, &layout::write_spin(spin, apm.version, options)?, None);
    }
    if let Some(inertia) = &data.inertia {
        let tag = if apm.version == Version::V1 {
            "spacecraftParameters"
        } else {
            "inertia"
        };
        out.block(
            tag,
            &layout::write_inertia(inertia, apm.version, options)?,
            None,
        );
    }
    for man in &data.maneuvers {
        out.block(
            "maneuverParameters",
            &layout::write_maneuver(man, options)?,
            None,
        );
    }
    if let Some(user) = &data.user_defined {
        out.block(
            "userDefinedParameters",
            &extension::write(user, options),
            Some(extension::PREFIX),
        );
    }
    Ok(())
}

fn write_quat_xml(
    out: &mut XmlWriter,
    q: &ApmQuaternion,
    version: Version,
    options: WriteOptions,
) -> Result<()> {
    out.open("quaternionState", &[]);
    out.comments(&q.comments);
    if let Some(epoch) = &q.epoch {
        out.leaf("EPOCH", &[], &epoch.to_string());
    }
    let (fa, fb) = if version == Version::V1 {
        ("Q_FRAME_A", "Q_FRAME_B")
    } else {
        ("REF_FRAME_A", "REF_FRAME_B")
    };
    out.leaf(fa, &[], q.ref_frame_a.as_str());
    out.leaf(fb, &[], q.ref_frame_b.as_str());
    if let Some(dir) = q.dir {
        out.leaf("Q_DIR", &[], dir.as_str());
    }
    let mut qb = BlockWriter::new(&[], options);
    qb.real("Q1", q.quaternion.q1)?;
    qb.real("Q2", q.quaternion.q2)?;
    qb.real("Q3", q.quaternion.q3)?;
    qb.real("QC", q.quaternion.qc)?;
    out.block("quaternion", &qb.finish(), None);
    if let Some(rate) = q.rate {
        let mut rb = BlockWriter::new(&[], options);
        rb.quantity("Q1_DOT", rate.q1_dot)?;
        rb.quantity("Q2_DOT", rate.q2_dot)?;
        rb.quantity("Q3_DOT", rate.q3_dot)?;
        rb.quantity("QC_DOT", rate.qc_dot)?;
        let rate_tag = if version == Version::V1 {
            "quaternionRate"
        } else {
            "quaternionDot"
        };
        out.block(rate_tag, &rb.finish(), None);
    }
    out.close("quaternionState");
    Ok(())
}

fn write_euler_xml(
    out: &mut XmlWriter,
    euler: &ApmEuler,
    version: Version,
    options: WriteOptions,
) -> Result<()> {
    if version == Version::V1 {
        out.open("eulerElementsThree", &[]);
        out.comments(&euler.comments);
        if let Some(epoch) = &euler.epoch {
            out.leaf("EPOCH", &[], &epoch.to_string());
        }
        out.leaf("EULER_FRAME_A", &[], euler.ref_frame_a.as_str());
        out.leaf("EULER_FRAME_B", &[], euler.ref_frame_b.as_str());
        if let Some(dir) = euler.dir {
            out.leaf("EULER_DIR", &[], dir.as_str());
        }
        out.leaf("EULER_ROT_SEQ", &[], euler.rot_seq.as_str());
        if let Some(rf) = &euler.rate_frame {
            out.leaf("RATE_FRAME", &[], rf.as_str());
        }
        let mut ab = BlockWriter::new(&[], options);
        write_vector(&mut ab, ["X_ANGLE", "Y_ANGLE", "Z_ANGLE"], &euler.angles)?;
        out.block("rotationAngles", &ab.finish(), None);
        if let Some(rates) = &euler.rates {
            let mut rb = BlockWriter::new(&[], options);
            write_vector(&mut rb, ["X_RATE", "Y_RATE", "Z_RATE"], rates)?;
            out.block("rotationRates", &rb.finish(), None);
        }
        out.close("eulerElementsThree");
        return Ok(());
    }
    let mut bw = BlockWriter::new(&euler.comments, options);
    layout::write_euler_fields(&mut bw, euler, version)?;
    out.block("eulerAngleState", &bw.finish(), None);
    Ok(())
}
