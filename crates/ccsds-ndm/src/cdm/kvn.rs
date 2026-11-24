//! CDM KVN reader and writer.

use super::cov::{COVARIANCE_KEYWORDS, CdmCovariance};
use super::layout::{self, HEADER_KEYWORDS, RELATIVE_KEYWORDS, STATE_VECTOR_KEYWORDS, VERSION_KEY};
use super::segment::{
    self, ADDITIONAL_KEYWORDS, METADATA_KEYWORDS, OD_KEYWORDS, write_additional, write_od,
};
use super::{Cdm, CdmData, CdmObject, CdmRelativeMetadata, CdmSegment};
use crate::block::{Block, BlockWriter, Entry, Fields};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::extension::{self, Extension};
use crate::kvn::flat::describe;
use crate::kvn::lexer::{Line, lex};
use crate::kvn::writer::KvnWriter;
use crate::{ParseMode, WriteOptions};

#[derive(Clone, Copy, PartialEq, Eq)]
enum SegSection {
    Meta,
    Od,
    Additional,
    State,
    Cov,
}

struct SegBuilder {
    metadata: Block,
    od: Option<Block>,
    additional: Option<Block>,
    state: Block,
    cov: Block,
    section: SegSection,
}

impl SegBuilder {
    fn new(entry: Entry, pending: &mut Vec<String>) -> Self {
        let mut metadata = Block::starting_at(entry.at);
        metadata.comments.append(pending);
        metadata.entries.push(entry);
        Self {
            metadata,
            od: None,
            additional: None,
            state: Block::default(),
            cov: Block::default(),
            section: SegSection::Meta,
        }
    }

    fn push(
        &mut self,
        target: SegSection,
        entry: Entry,
        pending: &mut Vec<String>,
        mode: ParseMode,
    ) -> Result<()> {
        if (target as u8) < (self.section as u8) && mode == ParseMode::Strict {
            return Err(entry.error(ErrorKind::KeywordOutOfOrder));
        }
        self.section = target;
        let block = match target {
            SegSection::Meta => &mut self.metadata,
            SegSection::Od => self.od.get_or_insert_with(|| Block::starting_at(entry.at)),
            SegSection::Additional => self
                .additional
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            SegSection::State => {
                if self.state.at.is_none() {
                    self.state.at = entry.at;
                }
                &mut self.state
            }
            SegSection::Cov => {
                if self.cov.at.is_none() {
                    self.cov.at = entry.at;
                }
                &mut self.cov
            }
        };
        block.comments.append(pending);
        block.entries.push(entry);
        Ok(())
    }

    fn finish(self, mode: ParseMode, extensions: &mut Vec<Extension>) -> Result<CdmSegment> {
        let metadata = segment::read_metadata(self.metadata, mode, extensions)?;
        let od_parameters = read_opt_block(self.od, mode, extensions, segment::read_od)?;
        let additional_parameters =
            read_opt_block(self.additional, mode, extensions, segment::read_additional)?;
        let (sc, mut sf) = Fields::new(self.state, mode)?;
        let state_vector = layout::read_state_vector(sc, &mut sf)?;
        sf.finish(extensions)?;
        let (cc, mut cf) = Fields::new(self.cov, mode)?;
        let covariance = CdmCovariance::read(cc, &mut cf, mode)?;
        cf.finish(extensions)?;
        Ok(CdmSegment {
            metadata,
            data: CdmData {
                comments: Vec::new(),
                od_parameters,
                additional_parameters,
                state_vector,
                covariance,
            },
        })
    }
}

fn read_opt_block<T>(
    block: Option<Block>,
    mode: ParseMode,
    extensions: &mut Vec<Extension>,
    parse: impl FnOnce(Vec<String>, &mut Fields) -> Result<T>,
) -> Result<Option<T>> {
    let Some(b) = block else {
        return Ok(None);
    };
    let (c, mut f) = Fields::new(b, mode)?;
    let val = parse(c, &mut f)?;
    f.finish(extensions)?;
    Ok(Some(val))
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum TopSection {
    Header,
    Relative,
    Segment,
    User,
}

struct CdmKvnBuilder {
    header: Block,
    relative: Block,
    segments: Vec<SegBuilder>,
    user: Option<Block>,
    extensions: Block,
    pending: Vec<String>,
    section: TopSection,
}

impl CdmKvnBuilder {
    fn new(start: Location) -> Self {
        Self {
            header: Block::starting_at(Some(start)),
            relative: Block::starting_at(Some(start)),
            segments: Vec::new(),
            user: None,
            extensions: Block::default(),
            pending: Vec::new(),
            section: TopSection::Header,
        }
    }

    fn route(&mut self, entry: Entry, mode: ParseMode) -> Result<()> {
        let kw = entry.keyword.as_str();
        if HEADER_KEYWORDS.contains(&kw) {
            return self.push_top(TopSection::Header, entry, mode);
        }
        if RELATIVE_KEYWORDS.contains(&kw) {
            return self.push_top(TopSection::Relative, entry, mode);
        }
        if kw == "OBJECT" {
            if (TopSection::Segment as u8) < (self.section as u8) && mode == ParseMode::Strict {
                return Err(entry.error(ErrorKind::KeywordOutOfOrder));
            }
            self.section = TopSection::Segment;
            let seg = SegBuilder::new(entry, &mut self.pending);
            self.segments.push(seg);
            return Ok(());
        }
        if kw.starts_with(extension::PREFIX) {
            return self.push_top(TopSection::User, entry, mode);
        }
        if self.section == TopSection::Segment
            && let Some(seg) = self.segments.last_mut()
            && let Some(target) = segment_target(kw)
        {
            return seg.push(target, entry, &mut self.pending, mode);
        }
        if mode == ParseMode::Strict {
            return Err(entry.error(ErrorKind::UnknownKeyword));
        }
        self.extensions.comments.append(&mut self.pending);
        self.extensions.entries.push(entry);
        Ok(())
    }

    fn push_top(&mut self, target: TopSection, entry: Entry, mode: ParseMode) -> Result<()> {
        if (target as u8) < (self.section as u8) && mode == ParseMode::Strict {
            return Err(entry.error(ErrorKind::KeywordOutOfOrder));
        }
        self.section = target;
        let block = match target {
            TopSection::Header => &mut self.header,
            TopSection::Relative => &mut self.relative,
            TopSection::User => self
                .user
                .get_or_insert_with(|| Block::starting_at(entry.at)),
            TopSection::Segment => &mut self.extensions,
        };
        block.comments.append(&mut self.pending);
        block.entries.push(entry);
        Ok(())
    }
}

fn segment_target(kw: &str) -> Option<SegSection> {
    if METADATA_KEYWORDS.contains(&kw) {
        Some(SegSection::Meta)
    } else if OD_KEYWORDS.contains(&kw) {
        Some(SegSection::Od)
    } else if ADDITIONAL_KEYWORDS.contains(&kw) {
        Some(SegSection::Additional)
    } else if STATE_VECTOR_KEYWORDS.contains(&kw) {
        Some(SegSection::State)
    } else if COVARIANCE_KEYWORDS.contains(&kw) {
        Some(SegSection::Cov)
    } else {
        None
    }
}

pub(super) fn read(text: &str, mode: ParseMode) -> Result<Cdm> {
    let mut lines = lex(text, mode).into_iter();
    let version_entry = match lines.next() {
        Some(Line::Pair(entry)) if entry.keyword == VERSION_KEY => entry,
        Some(other) => {
            return Err(Error::new(ErrorKind::WrongMessage {
                expected: "CDM",
                found: describe(&other),
            })
            .at(other.at()));
        }
        None => return Err(Error::new(ErrorKind::Empty)),
    };
    let version = layout::check_cdm_version(&version_entry, mode)?;
    let start = version_entry.at.unwrap_or(Location::new(1, 1));
    let mut builder = CdmKvnBuilder::new(start);
    for line in lines {
        match line {
            Line::Comment { text, .. } => builder.pending.push(text),
            Line::Bare { text, at } => {
                return Err(Error::new(ErrorKind::UnexpectedLine { text }).at(at));
            }
            Line::Pair(entry) => builder.route(entry, mode)?,
        }
    }
    finish_cdm(version, start, builder, mode)
}

fn finish_cdm(
    version: crate::Version,
    start: Location,
    mut builder: CdmKvnBuilder,
    mode: ParseMode,
) -> Result<Cdm> {
    let mut extensions = Vec::new();
    let header = layout::read_header(builder.header, mode, &mut extensions)?;
    let (rc, mut rf) = Fields::new(builder.relative, mode)?;
    let relative_metadata = layout::read_relative(rc, &mut rf, mode)?;
    rf.finish(&mut extensions)?;
    let mut seg_iter = builder.segments.into_iter();
    let seg1 = seg_iter
        .next()
        .ok_or_else(|| {
            Error::new(ErrorKind::MissingKeyword)
                .with_keyword("OBJECT")
                .at(start)
        })?
        .finish(mode, &mut extensions)?;
    let seg2 = seg_iter
        .next()
        .ok_or_else(|| {
            Error::new(ErrorKind::MissingKeyword)
                .with_keyword("OBJECT")
                .at(start)
        })?
        .finish(mode, &mut extensions)?;
    if seg_iter.next().is_some() {
        return Err(Error::new(ErrorKind::DuplicateKeyword)
            .with_keyword("OBJECT")
            .at(start));
    }
    let user_defined = builder
        .user
        .map(|b| extension::read(b, mode, &mut extensions))
        .transpose()?;
    if !builder.pending.is_empty() {
        builder.extensions.comments.append(&mut builder.pending);
    }
    Fields::new(builder.extensions, mode)?
        .1
        .finish(&mut extensions)?;
    validate_segments(&seg1, &seg2, start)?;
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

pub(super) fn validate_segments(seg1: &CdmSegment, seg2: &CdmSegment, at: Location) -> Result<()> {
    if seg1.metadata.object != CdmObject::Object1 {
        return Err(Error::new(ErrorKind::InvalidValue {
            text: seg1.metadata.object.as_str().to_owned(),
            expected: "OBJECT1",
        })
        .at(at));
    }
    if seg2.metadata.object != CdmObject::Object2 {
        return Err(Error::new(ErrorKind::InvalidValue {
            text: seg2.metadata.object.as_str().to_owned(),
            expected: "OBJECT2",
        })
        .at(at));
    }
    Ok(())
}

pub(super) fn write(cdm: &Cdm, options: WriteOptions) -> Result<String> {
    let mut out = KvnWriter::default();
    out.pair(VERSION_KEY, cdm.version.as_str());
    out.blank();
    out.block(&layout::write_header(&cdm.header, options))?;
    out.blank();
    out.block(&write_relative_block(&cdm.relative_metadata, options)?)?;
    write_segment(&mut out, &cdm.object1, options)?;
    write_segment(&mut out, &cdm.object2, options)?;
    if let Some(user) = &cdm.user_defined {
        out.blank();
        out.block(&extension::write(user, options))?;
    }
    if !cdm.extensions.is_empty() {
        out.blank();
        out.block(&extension::write_extensions(&cdm.extensions))?;
    }
    Ok(out.finish())
}

fn write_relative_block(rel: &CdmRelativeMetadata, options: WriteOptions) -> Result<Block> {
    let mut bw = BlockWriter::new(&rel.comments, options);
    layout::write_relative_head(rel, &mut bw)?;
    if let Some(state) = &rel.relative_state {
        layout::write_relative_state(state, &mut bw)?;
    }
    layout::write_relative_tail(rel, &mut bw)?;
    Ok(bw.finish())
}

fn write_segment(out: &mut KvnWriter, seg: &CdmSegment, options: WriteOptions) -> Result<()> {
    out.blank();
    out.block(&segment::write_metadata(&seg.metadata, options))?;
    if !seg.data.comments.is_empty() {
        out.comments(&seg.data.comments)?;
    }
    if let Some(od) = &seg.data.od_parameters {
        out.blank();
        out.block(&write_od(od, options)?)?;
    }
    if let Some(add) = &seg.data.additional_parameters {
        out.blank();
        out.block(&write_additional(add, options)?)?;
    }
    out.blank();
    out.block(&layout::write_state_vector(
        &seg.data.state_vector,
        options,
    )?)?;
    out.blank();
    out.block(&seg.data.covariance.write(options)?)?;
    Ok(())
}
