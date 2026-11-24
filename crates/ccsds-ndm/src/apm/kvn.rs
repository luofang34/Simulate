//! APM KVN reader and writer.

use super::layout::{
    self, ANGVEL_DISCRIMINATORS, EULER_DISCRIMINATORS, INERTIA_KEYWORDS, MANEUVER_KEYWORDS,
    METADATA_KEYWORDS, QUAT_DISCRIMINATORS, SPIN_DISCRIMINATORS, VERSION_KEY,
};
use super::{Apm, ApmData};
use crate::adm::check_adm_version;
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::extension::{self, Extension};
use crate::header::{HEADER_KEYWORDS, Header};
use crate::kvn::flat::describe;
use crate::kvn::lexer::{Line, lex};
use crate::kvn::writer::KvnWriter;
use crate::{ParseMode, WriteOptions};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Section {
    Header,
    Metadata,
    DataEpoch,
    Quaternion,
    Euler,
    AngVel,
    Spin,
    Inertia,
    Maneuver,
    User,
}

struct ApmKvnBuilder {
    header: Block,
    metadata: Block,
    data_comments: Vec<String>,
    data_epoch: Option<Entry>,
    quat: Option<Block>,
    euler: Option<Block>,
    ang_vel: Option<Block>,
    spin: Option<Block>,
    inertia: Option<Block>,
    maneuvers: Vec<Block>,
    user: Option<Block>,
    extensions: Block,
    pending_comments: Vec<String>,
    pending_shared: Vec<Entry>,
    current: Section,
}

impl ApmKvnBuilder {
    fn new(start: Location) -> Self {
        Self {
            header: Block::starting_at(Some(start)),
            metadata: Block::starting_at(Some(start)),
            data_comments: Vec::new(),
            data_epoch: None,
            quat: None,
            euler: None,
            ang_vel: None,
            spin: None,
            inertia: None,
            maneuvers: Vec::new(),
            user: None,
            extensions: Block::default(),
            pending_comments: Vec::new(),
            pending_shared: Vec::new(),
            current: Section::Header,
        }
    }

    fn flush_shared(&mut self, section: Section) {
        let target = match section {
            Section::Quaternion => self.quat.get_or_insert_with(Block::default),
            Section::Euler => self.euler.get_or_insert_with(Block::default),
            Section::AngVel => self.ang_vel.get_or_insert_with(Block::default),
            Section::Spin => self.spin.get_or_insert_with(Block::default),
            _ => return,
        };
        if target.at.is_none() {
            target.at = self.pending_shared.first().and_then(|e| e.at);
        }
        target.comments.append(&mut self.pending_comments);
        target.entries.append(&mut self.pending_shared);
    }

    fn push_to(&mut self, section: Section, entry: Entry, mode: ParseMode) -> Result<()> {
        if !matches!(
            section,
            Section::Quaternion | Section::Euler | Section::AngVel | Section::Spin
        ) && !self.pending_shared.is_empty()
        {
            if mode == ParseMode::Strict {
                let first = self.pending_shared.first().map_or_else(
                    || entry.error(ErrorKind::UnknownKeyword),
                    |e| e.error(ErrorKind::UnknownKeyword),
                );
                return Err(first);
            }
            self.extensions.entries.append(&mut self.pending_shared);
        }
        self.flush_shared(section);
        self.current = section;
        let block = match section {
            Section::Header => &mut self.header,
            Section::Metadata => &mut self.metadata,
            Section::Quaternion => self
                .quat
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            Section::Euler => self
                .euler
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            Section::AngVel => self
                .ang_vel
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            Section::Spin => self
                .spin
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            Section::Inertia => self
                .inertia
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            Section::Maneuver => {
                let open_new = entry.keyword == "MAN_EPOCH_START"
                    || self.maneuvers.last().is_none_or(|b| b.has(&entry.keyword));
                if open_new {
                    self.maneuvers.push(Block::starting_at(entry.at));
                }
                self.maneuvers.last_mut().ok_or_else(|| {
                    entry.error(ErrorKind::UnexpectedLine {
                        text: entry.keyword.clone(),
                    })
                })?
            }
            Section::User => self
                .user
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            Section::DataEpoch => &mut self.extensions,
        };
        block.comments.append(&mut self.pending_comments);
        block.entries.push(entry);
        Ok(())
    }

    fn route(&mut self, entry: Entry, mode: ParseMode) -> Result<()> {
        let kw = entry.keyword.as_str();
        if HEADER_KEYWORDS.contains(&kw) {
            return self.check_order(Section::Header, entry, mode);
        }
        if METADATA_KEYWORDS.contains(&kw) {
            return self.check_order(Section::Metadata, entry, mode);
        }
        if matches!(kw, "EPOCH" | "REF_FRAME_A" | "REF_FRAME_B") {
            return self.route_shared(entry, mode);
        }
        if QUAT_DISCRIMINATORS.contains(&kw) {
            return self.check_order(Section::Quaternion, entry, mode);
        }
        if EULER_DISCRIMINATORS.contains(&kw) {
            return self.check_order(Section::Euler, entry, mode);
        }
        if ANGVEL_DISCRIMINATORS.contains(&kw) {
            return self.check_order(Section::AngVel, entry, mode);
        }
        if SPIN_DISCRIMINATORS.contains(&kw) {
            return self.check_order(Section::Spin, entry, mode);
        }
        if INERTIA_KEYWORDS.contains(&kw) {
            return self.check_order(Section::Inertia, entry, mode);
        }
        if MANEUVER_KEYWORDS.contains(&kw) {
            return self.check_order(Section::Maneuver, entry, mode);
        }
        if kw.starts_with(extension::PREFIX) {
            return self.check_order(Section::User, entry, mode);
        }
        if mode == ParseMode::Strict {
            return Err(entry.error(ErrorKind::UnknownKeyword));
        }
        self.extensions.comments.append(&mut self.pending_comments);
        self.extensions.entries.push(entry);
        Ok(())
    }

    fn route_shared(&mut self, entry: Entry, mode: ParseMode) -> Result<()> {
        if entry.keyword == "EPOCH"
            && section_rank(self.current) <= section_rank(Section::Metadata)
            && self.data_epoch.is_none()
            && self.pending_shared.is_empty()
        {
            self.data_comments.append(&mut self.pending_comments);
            self.data_epoch = Some(entry);
            self.current = Section::DataEpoch;
            return Ok(());
        }
        if section_rank(self.current) > section_rank(Section::Spin) {
            if mode == ParseMode::Strict {
                return Err(entry.error(ErrorKind::KeywordOutOfOrder));
            }
            self.extensions.entries.push(entry);
            return Ok(());
        }
        if self
            .pending_shared
            .iter()
            .any(|e| e.keyword == entry.keyword)
        {
            return Err(entry.error(ErrorKind::DuplicateKeyword));
        }
        self.pending_shared.push(entry);
        Ok(())
    }

    fn check_order(&mut self, target: Section, entry: Entry, mode: ParseMode) -> Result<()> {
        let cur_rank = section_rank(self.current);
        let tgt_rank = section_rank(target);
        if tgt_rank < cur_rank && mode == ParseMode::Strict {
            return Err(entry.error(ErrorKind::KeywordOutOfOrder));
        }
        self.push_to(target, entry, mode)
    }
}

const fn section_rank(section: Section) -> u8 {
    match section {
        Section::Header => 0,
        Section::Metadata => 1,
        Section::DataEpoch => 2,
        Section::Quaternion => 3,
        Section::Euler => 4,
        Section::AngVel => 5,
        Section::Spin => 6,
        Section::Inertia => 7,
        Section::Maneuver => 8,
        Section::User => 9,
    }
}

pub(super) fn read(text: &str, mode: ParseMode) -> Result<Apm> {
    let mut lines = lex(text, mode).into_iter();
    let version_entry = match lines.next() {
        Some(Line::Pair(entry)) if entry.keyword == VERSION_KEY => entry,
        Some(other) => {
            return Err(Error::new(ErrorKind::WrongMessage {
                expected: "APM",
                found: describe(&other),
            })
            .at(other.at()));
        }
        None => return Err(Error::new(ErrorKind::Empty)),
    };
    let version = check_adm_version(&version_entry, mode)?;
    let start = version_entry.at.unwrap_or(Location::new(1, 1));
    let mut builder = ApmKvnBuilder::new(start);
    for line in lines {
        match line {
            Line::Comment { text, .. } => builder.pending_comments.push(text),
            Line::Bare { text, at } => {
                return Err(Error::new(ErrorKind::UnexpectedLine { text }).at(at));
            }
            Line::Pair(entry) => builder.route(entry, mode)?,
        }
    }
    finish_apm(version, start, builder, mode)
}

fn finish_apm(
    version: crate::Version,
    start: Location,
    mut builder: ApmKvnBuilder,
    mode: ParseMode,
) -> Result<Apm> {
    let mut extensions = Vec::new();
    if !builder.pending_shared.is_empty() {
        if mode == ParseMode::Strict {
            let err = builder.pending_shared.first().map_or_else(
                || {
                    Error::new(ErrorKind::UnexpectedEnd {
                        expected: "attitude state",
                    })
                    .at(start)
                },
                |e| e.error(ErrorKind::KeywordOutOfOrder),
            );
            return Err(err);
        }
        builder
            .extensions
            .entries
            .append(&mut builder.pending_shared);
    }
    if !builder.pending_comments.is_empty() {
        builder.data_comments.append(&mut builder.pending_comments);
    }
    let header = Header::read(builder.header, mode, &mut extensions)?;
    let metadata = layout::read_metadata(builder.metadata, mode, &mut extensions)?;
    let epoch = builder
        .data_epoch
        .map(|e| {
            if e.is_empty() {
                Err(e.error(ErrorKind::EmptyValue))
            } else {
                e.epoch(mode)
            }
        })
        .transpose()?;
    let quaternion = read_opt_block(builder.quat, mode, &mut extensions, layout::read_quaternion)?;
    let euler = read_opt_block(builder.euler, mode, &mut extensions, layout::read_euler)?;
    let ang_vel = read_opt_block(builder.ang_vel, mode, &mut extensions, layout::read_ang_vel)?;
    let spin = read_opt_block(builder.spin, mode, &mut extensions, layout::read_spin)?;
    let inertia = read_opt_block(builder.inertia, mode, &mut extensions, layout::read_inertia)?;
    let mut maneuvers = Vec::new();
    for block in builder.maneuvers {
        let (c, mut f) = Fields::new(block, mode)?;
        maneuvers.push(layout::read_maneuver(c, &mut f)?);
        f.finish(&mut extensions)?;
    }
    let user_defined = builder
        .user
        .map(|b| extension::read(b, mode, &mut extensions))
        .transpose()?;
    Fields::new(builder.extensions, mode)?
        .1
        .finish(&mut extensions)?;
    let data = ApmData {
        comments: builder.data_comments,
        epoch,
        quaternion,
        euler,
        ang_vel,
        spin,
        inertia,
        maneuvers,
        user_defined,
    };
    layout::validate_data(&data, version, start, mode)?;
    Ok(Apm {
        version,
        header,
        metadata,
        data,
        extensions,
    })
}

fn read_opt_block<T>(
    block: Option<Block>,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
    parse: impl FnOnce(Vec<String>, &mut Fields) -> Result<T>,
) -> Result<Option<T>> {
    let Some(block) = block else {
        return Ok(None);
    };
    let (comments, mut fields) = Fields::new(block, mode)?;
    let value = parse(comments, &mut fields)?;
    fields.finish(extensions)?;
    Ok(Some(value))
}

pub(super) fn write(apm: &Apm, options: WriteOptions) -> Result<String> {
    let mut out = KvnWriter::default();
    out.pair(VERSION_KEY, apm.version.as_str());
    out.blank();
    out.block(&apm.header.write(options))?;
    out.blank();
    out.block(&layout::write_metadata(&apm.metadata, options))?;
    let data = &apm.data;
    if !data.comments.is_empty() || data.epoch.is_some() {
        out.blank();
        let mut bw = BlockWriter::new(&data.comments, options);
        bw.text_opt("EPOCH", data.epoch.as_ref());
        out.block(&bw.finish())?;
    }
    if let Some(q) = &data.quaternion {
        out.blank();
        let mut bw = BlockWriter::new(&q.comments, options);
        layout::write_quaternion_fields(&mut bw, q, apm.version)?;
        out.block(&bw.finish())?;
    }
    if let Some(euler) = &data.euler {
        out.blank();
        let mut bw = BlockWriter::new(&euler.comments, options);
        layout::write_euler_fields(&mut bw, euler, apm.version)?;
        out.block(&bw.finish())?;
    }
    if let Some(av) = &data.ang_vel {
        out.blank();
        out.block(&layout::write_ang_vel(av, apm.version, options)?)?;
    }
    if let Some(spin) = &data.spin {
        out.blank();
        out.block(&layout::write_spin(spin, apm.version, options)?)?;
    }
    if let Some(inertia) = &data.inertia {
        out.blank();
        out.block(&layout::write_inertia(inertia, apm.version, options)?)?;
    }
    for man in &data.maneuvers {
        out.blank();
        out.block(&layout::write_maneuver(man, options)?)?;
    }
    if let Some(user) = &data.user_defined {
        out.blank();
        out.block(&extension::write(user, options))?;
    }
    if !apm.extensions.is_empty() {
        out.blank();
        out.block(&extension::write_extensions(&apm.extensions))?;
    }
    Ok(out.finish())
}
