//! TDM KVN reader and writer.

use super::layout::{self, VERSION_KEY};
use super::{Tdm, TdmData, TdmMetadata, TdmObservation, TdmSegment};
use crate::block::{Block, Entry};
use crate::error::{Error, ErrorKind, Location, Result};
use crate::extension::{self, Extension, ExtensionValue};
use crate::header::Header;
use crate::kvn::flat::describe;
use crate::kvn::lexer::{Line, lex};
use crate::kvn::writer::KvnWriter;
use crate::{Epoch, ParseMode, WriteOptions, number};

enum Phase {
    Header,
    Meta(Block),
    WaitData(TdmMetadata),
    Data(TdmMetadata, TdmData),
    BetweenSegments,
}

struct Reader {
    mode: ParseMode,
    header: Block,
    segments: Vec<TdmSegment>,
    pending_comments: Vec<String>,
    extensions: Vec<Extension>,
    phase: Phase,
}

impl Reader {
    fn new(start: Location, mode: ParseMode) -> Self {
        Self {
            mode,
            header: Block::starting_at(Some(start)),
            segments: Vec::new(),
            pending_comments: Vec::new(),
            extensions: Vec::new(),
            phase: Phase::Header,
        }
    }

    fn step(&mut self, line: Line) -> Result<()> {
        match line {
            Line::Comment { text, .. } => {
                self.pending_comments.push(text);
                Ok(())
            }
            Line::Bare { text, at } => self.on_bare(&text, at),
            Line::Pair(entry) => self.on_pair(entry),
        }
    }

    fn on_bare(&mut self, text: &str, at: Location) -> Result<()> {
        match text.trim() {
            "META_START" => self.start_meta(at),
            "META_STOP" => self.stop_meta(at),
            "DATA_START" => self.start_data(at),
            "DATA_STOP" => self.stop_data(at),
            _ => Err(Error::new(ErrorKind::UnexpectedLine {
                text: text.to_owned(),
            })
            .at(at)),
        }
    }

    fn start_meta(&mut self, at: Location) -> Result<()> {
        self.flush(at)?;
        if !matches!(self.phase, Phase::Header | Phase::BetweenSegments) {
            return Err(Error::new(ErrorKind::UnexpectedLine {
                text: "META_START".to_owned(),
            })
            .at(at));
        }
        let mut block = Block::starting_at(Some(at));
        block.comments.append(&mut self.pending_comments);
        self.phase = Phase::Meta(block);
        Ok(())
    }

    fn stop_meta(&mut self, at: Location) -> Result<()> {
        let prev = std::mem::replace(&mut self.phase, Phase::BetweenSegments);
        let Phase::Meta(mut block) = prev else {
            return Err(Error::new(ErrorKind::UnexpectedLine {
                text: "META_STOP".to_owned(),
            })
            .at(at));
        };
        block.comments.append(&mut self.pending_comments);
        let meta = layout::read_metadata(block, self.mode, &mut self.extensions)?;
        self.phase = Phase::WaitData(meta);
        Ok(())
    }

    fn start_data(&mut self, at: Location) -> Result<()> {
        let prev = std::mem::replace(&mut self.phase, Phase::BetweenSegments);
        let Phase::WaitData(meta) = prev else {
            return Err(Error::new(ErrorKind::UnexpectedLine {
                text: "DATA_START".to_owned(),
            })
            .at(at));
        };
        let mut data = TdmData::default();
        data.comments.append(&mut self.pending_comments);
        self.phase = Phase::Data(meta, data);
        Ok(())
    }

    fn stop_data(&mut self, at: Location) -> Result<()> {
        let prev = std::mem::replace(&mut self.phase, Phase::BetweenSegments);
        let Phase::Data(meta, mut data) = prev else {
            return Err(Error::new(ErrorKind::UnexpectedLine {
                text: "DATA_STOP".to_owned(),
            })
            .at(at));
        };
        data.comments.append(&mut self.pending_comments);
        self.segments.push(TdmSegment {
            metadata: meta,
            data,
        });
        self.phase = Phase::BetweenSegments;
        Ok(())
    }

    fn on_pair(&mut self, entry: Entry) -> Result<()> {
        match &mut self.phase {
            Phase::Header => {
                self.header.comments.append(&mut self.pending_comments);
                self.header.entries.push(entry);
                Ok(())
            }
            Phase::Meta(block) => {
                block.comments.append(&mut self.pending_comments);
                block.entries.push(entry);
                Ok(())
            }
            Phase::Data(_, data) => {
                data.comments.append(&mut self.pending_comments);
                data.observations.push(parse_obs_entry(entry, self.mode)?);
                Ok(())
            }
            _ if self.mode == ParseMode::Lenient => {
                self.extensions.push(Extension {
                    keyword: entry.keyword,
                    value: ExtensionValue::Text(entry.value),
                    unit: entry.unit,
                });
                Ok(())
            }
            _ => Err(entry.error(ErrorKind::UnknownKeyword)),
        }
    }

    fn flush(&mut self, at: Location) -> Result<()> {
        match std::mem::replace(&mut self.phase, Phase::BetweenSegments) {
            Phase::Header | Phase::BetweenSegments => Ok(()),
            Phase::Meta(_) => Err(Error::new(ErrorKind::UnexpectedEnd {
                expected: "META_STOP",
            })
            .at(at)),
            Phase::WaitData(_) => Err(Error::new(ErrorKind::UnexpectedEnd {
                expected: "DATA_START",
            })
            .at(at)),
            Phase::Data(_, _) => Err(Error::new(ErrorKind::UnexpectedEnd {
                expected: "DATA_STOP",
            })
            .at(at)),
        }
    }
}

fn parse_obs_entry(entry: Entry, mode: ParseMode) -> Result<TdmObservation> {
    let kind = layout::parse_observable_keyword(&entry, mode)?;
    if let Some(unit) = &entry.unit
        && mode == ParseMode::Strict
    {
        return Err(entry.error(ErrorKind::UnitMismatch {
            expected: "",
            found: unit.clone(),
        }));
    }
    let mut parts = entry.value.split_whitespace();
    let Some(epoch_tok) = parts.next() else {
        return Err(entry.error(ErrorKind::EmptyValue));
    };
    let Some(val_tok) = parts.next() else {
        return Err(entry.error(ErrorKind::ColumnCount {
            expected: "2",
            found: 1,
        }));
    };
    if parts.next().is_some() {
        return Err(entry.error(ErrorKind::ColumnCount {
            expected: "2",
            found: 3,
        }));
    }
    let epoch = Epoch::parse(epoch_tok).map_err(|reason| {
        let err = Error::new(ErrorKind::InvalidEpoch {
            text: epoch_tok.to_owned(),
            reason,
        });
        match entry.at {
            Some(loc) => err.at(loc),
            None => err,
        }
    })?;
    let value = layout::parse_scalar_value(val_tok, &entry.keyword, entry.at, mode)?;
    Ok(TdmObservation { epoch, kind, value })
}

pub(super) fn read(text: &str, mode: ParseMode) -> Result<Tdm> {
    let mut lines = lex(text, mode).into_iter();
    let version_entry = match lines.next() {
        Some(Line::Pair(entry)) if entry.keyword == VERSION_KEY => entry,
        Some(other) => {
            return Err(Error::new(ErrorKind::WrongMessage {
                expected: "TDM",
                found: describe(&other),
            })
            .at(other.at()));
        }
        None => return Err(Error::new(ErrorKind::Empty)),
    };
    let version = layout::check_tdm_version(&version_entry, mode)?;
    let start = version_entry.at.unwrap_or(Location::new(1, 1));
    let mut reader = Reader::new(start, mode);
    for line in lines {
        reader.step(line)?;
    }
    reader.flush(start)?;
    if mode == ParseMode::Strict && reader.segments.is_empty() {
        return Err(Error::new(ErrorKind::UnexpectedEnd {
            expected: "META_START",
        })
        .at(start));
    }
    let header = Header::read(reader.header, mode, &mut reader.extensions)?;
    Ok(Tdm {
        version,
        header,
        segments: reader.segments,
        extensions: reader.extensions,
    })
}

pub(super) fn write(tdm: &Tdm, options: WriteOptions) -> Result<String> {
    let mut out = KvnWriter::default();
    out.pair(VERSION_KEY, tdm.version.as_str());
    out.blank();
    out.block(&tdm.header.write(options))?;
    for seg in &tdm.segments {
        out.blank();
        out.line("META_START");
        out.block(&layout::write_metadata(&seg.metadata, options)?)?;
        out.line("META_STOP");
        out.blank();
        out.line("DATA_START");
        out.comments(&seg.data.comments)?;
        for obs in &seg.data.observations {
            let key = obs.kind.as_str();
            let val_str = number::format_f64(obs.value)
                .ok_or_else(|| Error::new(ErrorKind::NonFiniteNumber).with_keyword(key.clone()))?;
            out.pair(&key, &format!("{} {val_str}", obs.epoch));
        }
        out.line("DATA_STOP");
    }
    if !tdm.extensions.is_empty() {
        out.blank();
        out.block(&extension::write_extensions(&tdm.extensions))?;
    }
    Ok(out.finish())
}
