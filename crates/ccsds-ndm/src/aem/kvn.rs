//! AEM KVN reader and writer.

use super::layout::{self, VERSION_KEY};
use super::{Aem, AemCovariance, AemData, AemMetadata, AemSegment};
use crate::adm::check_adm_version;
use crate::block::{Block, BlockWriter, Entry};
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
    WaitData(AemMetadata),
    Data(AemMetadata, AemData),
    Cov(AemMetadata, AemData, CovBuilder),
    BetweenSegments,
}

#[derive(Default)]
struct CovBuilder {
    pending: Vec<String>,
    current: Option<PartialCov>,
}

struct PartialCov {
    comments: Vec<String>,
    epoch: Option<Epoch>,
    ref_frame: Option<crate::RefFrame>,
    values: Vec<f64>,
    at: Location,
}

struct Reader {
    version: crate::Version,
    mode: ParseMode,
    header: Block,
    segments: Vec<AemSegment>,
    pending_comments: Vec<String>,
    extensions: Vec<Extension>,
    phase: Phase,
}

impl Reader {
    fn new(version: crate::Version, start: Location, mode: ParseMode) -> Self {
        Self {
            version,
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
            "COVARIANCE_START" => self.start_cov(at),
            "COVARIANCE_STOP" => self.stop_cov(at),
            _ => self.on_data_row(text, at),
        }
    }

    fn start_meta(&mut self, at: Location) -> Result<()> {
        self.flush_segment(at)?;
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
        let meta = layout::read_metadata(block, self.version, self.mode, &mut self.extensions)?;
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
        let mut data = AemData::default();
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
        if self.mode == ParseMode::Strict && data.states.is_empty() {
            return Err(Error::new(ErrorKind::UnexpectedEnd {
                expected: "at least one AEM attitude state",
            })
            .at(at));
        }
        self.segments.push(AemSegment {
            metadata: meta,
            data,
        });
        self.phase = Phase::BetweenSegments;
        Ok(())
    }

    fn start_cov(&mut self, at: Location) -> Result<()> {
        if !matches!(self.phase, Phase::BetweenSegments) {
            return Err(Error::new(ErrorKind::UnexpectedLine {
                text: "COVARIANCE_START".to_owned(),
            })
            .at(at));
        }
        let Some(seg) = self.segments.pop() else {
            return Err(Error::new(ErrorKind::UnexpectedLine {
                text: "COVARIANCE_START".to_owned(),
            })
            .at(at));
        };
        let mut cov = CovBuilder::default();
        cov.pending.append(&mut self.pending_comments);
        self.phase = Phase::Cov(seg.metadata, seg.data, cov);
        Ok(())
    }

    fn stop_cov(&mut self, at: Location) -> Result<()> {
        let prev = std::mem::replace(&mut self.phase, Phase::BetweenSegments);
        let Phase::Cov(meta, mut data, mut cov) = prev else {
            return Err(Error::new(ErrorKind::UnexpectedLine {
                text: "COVARIANCE_STOP".to_owned(),
            })
            .at(at));
        };
        if let Some(partial) = cov.current.take() {
            data.covariances.push(finish_cov(partial, self.mode)?);
        }
        self.segments.push(AemSegment {
            metadata: meta,
            data,
        });
        Ok(())
    }

    fn on_data_row(&mut self, text: &str, at: Location) -> Result<()> {
        match &mut self.phase {
            Phase::Data(meta, data) => {
                let mut tokens = text.split_whitespace();
                let Some(epoch_tok) = tokens.next() else {
                    return Ok(());
                };
                let epoch = Epoch::parse(epoch_tok).map_err(|reason| {
                    Error::new(ErrorKind::InvalidEpoch {
                        text: epoch_tok.to_owned(),
                        reason,
                    })
                    .at(at)
                })?;
                let mut cols = Vec::new();
                for tok in tokens {
                    cols.push(layout::parse_real_col(tok, "AEM_STATE", at, self.mode)?);
                }
                let expected = meta.attitude_type.column_count();
                if cols.len() != expected {
                    return Err(Error::new(ErrorKind::ColumnCount {
                        expected: col_expected_label(meta.attitude_type),
                        found: cols.len(),
                    })
                    .at(at));
                }
                if !self.pending_comments.is_empty() {
                    data.comments.append(&mut self.pending_comments);
                }
                data.states
                    .push(layout::parse_state_columns(epoch, &cols, meta, Some(at))?);
                Ok(())
            }
            Phase::Cov(_, data, cov) => {
                let mut partial = cov.current.take().unwrap_or_else(|| PartialCov {
                    comments: std::mem::take(&mut cov.pending),
                    epoch: None,
                    ref_frame: None,
                    values: Vec::new(),
                    at,
                });
                partial.comments.append(&mut self.pending_comments);
                for tok in text.split_whitespace() {
                    partial
                        .values
                        .push(layout::parse_real_col(tok, "COV", at, self.mode)?);
                }
                if is_full_lower_tri(partial.values.len()) {
                    data.covariances.push(finish_cov(partial, self.mode)?);
                } else {
                    cov.current = Some(partial);
                }
                Ok(())
            }
            _ => Err(Error::new(ErrorKind::UnexpectedLine {
                text: text.to_owned(),
            })
            .at(at)),
        }
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
            Phase::Cov(_, data, cov) => {
                on_cov_pair(entry, data, cov, &mut self.pending_comments, self.mode)
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

    fn flush_segment(&mut self, at: Location) -> Result<()> {
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
            Phase::Cov(_, _, _) => Err(Error::new(ErrorKind::UnexpectedEnd {
                expected: "COVARIANCE_STOP",
            })
            .at(at)),
        }
    }
}

fn on_cov_pair(
    entry: Entry,
    data: &mut AemData,
    cov: &mut CovBuilder,
    pending: &mut Vec<String>,
    mode: ParseMode,
) -> Result<()> {
    let at = entry.at.unwrap_or(Location::new(1, 1));
    match entry.keyword.as_str() {
        "EPOCH" => {
            if let Some(prev) = cov.current.take() {
                data.covariances.push(finish_cov(prev, mode)?);
            }
            let mut comments = std::mem::take(&mut cov.pending);
            comments.append(pending);
            cov.current = Some(PartialCov {
                comments,
                epoch: Some(entry.epoch(mode)?),
                ref_frame: None,
                values: Vec::new(),
                at,
            });
            Ok(())
        }
        "COV_REF_FRAME" => {
            let partial = cov.current.get_or_insert_with(|| PartialCov {
                comments: std::mem::take(&mut cov.pending),
                epoch: None,
                ref_frame: None,
                values: Vec::new(),
                at,
            });
            partial.comments.append(pending);
            partial.ref_frame = Some(entry.frame(mode)?);
            Ok(())
        }
        _ if mode == ParseMode::Lenient => Ok(()),
        _ => Err(entry.error(ErrorKind::UnknownKeyword)),
    }
}

fn is_full_lower_tri(len: usize) -> bool {
    matches!(len, 6 | 21)
}

const fn col_expected_label(kind: crate::adm::AttitudeType) -> &'static str {
    match kind.column_count() {
        3 => "3",
        4 => "4",
        6 => "6",
        7 => "7",
        8 => "8",
        _ => "attitude state columns",
    }
}

fn finish_cov(partial: PartialCov, mode: ParseMode) -> Result<AemCovariance> {
    if mode == ParseMode::Strict && !is_full_lower_tri(partial.values.len()) {
        return Err(Error::new(ErrorKind::ColumnCount {
            expected: "6 or 21",
            found: partial.values.len(),
        })
        .at(partial.at));
    }
    Ok(AemCovariance {
        comments: partial.comments,
        epoch: partial.epoch,
        ref_frame: partial.ref_frame,
        lower_triangle: partial.values,
    })
}

pub(super) fn read(text: &str, mode: ParseMode) -> Result<Aem> {
    let mut lines = lex(text, mode).into_iter();
    let version_entry = match lines.next() {
        Some(Line::Pair(entry)) if entry.keyword == VERSION_KEY => entry,
        Some(other) => {
            return Err(Error::new(ErrorKind::WrongMessage {
                expected: "AEM",
                found: describe(&other),
            })
            .at(other.at()));
        }
        None => return Err(Error::new(ErrorKind::Empty)),
    };
    let version = check_adm_version(&version_entry, mode)?;
    let start = version_entry.at.unwrap_or(Location::new(1, 1));
    let mut reader = Reader::new(version, start, mode);
    for line in lines {
        reader.step(line)?;
    }
    reader.flush_segment(start)?;
    if mode == ParseMode::Strict && reader.segments.is_empty() {
        return Err(Error::new(ErrorKind::UnexpectedEnd {
            expected: "META_START",
        })
        .at(start));
    }
    let header = Header::read(reader.header, mode, &mut reader.extensions)?;
    Ok(Aem {
        version,
        header,
        segments: reader.segments,
        extensions: reader.extensions,
    })
}

pub(super) fn write(aem: &Aem, options: WriteOptions) -> Result<String> {
    let mut out = KvnWriter::default();
    out.pair(VERSION_KEY, aem.version.as_str());
    out.blank();
    out.block(&aem.header.write(options))?;
    for seg in &aem.segments {
        out.blank();
        out.line("META_START");
        out.block(&layout::write_metadata(&seg.metadata, aem.version, options))?;
        out.line("META_STOP");
        out.blank();
        out.line("DATA_START");
        out.comments(&seg.data.comments)?;
        for st in &seg.data.states {
            let cols = layout::state_columns(&st.state, seg.metadata.quaternion_type)?;
            let mut row = st.epoch.to_string();
            for c in cols {
                row.push(' ');
                row.push_str(&c);
            }
            out.line(&row);
        }
        out.line("DATA_STOP");
        write_covariances(&mut out, &seg.data.covariances, options)?;
    }
    if !aem.extensions.is_empty() {
        out.blank();
        out.block(&extension::write_extensions(&aem.extensions))?;
    }
    Ok(out.finish())
}

fn write_covariances(
    out: &mut KvnWriter,
    covs: &[AemCovariance],
    options: WriteOptions,
) -> Result<()> {
    if covs.is_empty() {
        return Ok(());
    }
    out.blank();
    out.line("COVARIANCE_START");
    for cov in covs {
        let mut bw = BlockWriter::new(&cov.comments, options);
        bw.text_opt("EPOCH", cov.epoch.as_ref());
        bw.text_opt("COV_REF_FRAME", cov.ref_frame.as_ref());
        out.block(&bw.finish())?;
        write_cov_rows(out, &cov.lower_triangle)?;
    }
    out.line("COVARIANCE_STOP");
    Ok(())
}

fn write_cov_rows(out: &mut KvnWriter, values: &[f64]) -> Result<()> {
    let mut idx = 0usize;
    let mut row_len = 1usize;
    while idx < values.len() {
        let mut line = String::new();
        for col in 0..row_len {
            let Some(&val) = values.get(idx) else {
                break;
            };
            if col > 0 {
                line.push(' ');
            }
            let s = number::format_f64(val)
                .ok_or_else(|| Error::new(ErrorKind::NonFiniteNumber).with_keyword("COV"))?;
            line.push_str(&s);
            idx = idx.wrapping_add(1);
        }
        out.line(&line);
        row_len = row_len.wrapping_add(1);
    }
    Ok(())
}
