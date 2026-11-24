//! The OEM in KVN: a header, then segments of `META_START`…`META_STOP`
//! metadata, ephemeris lines, and an optional `COVARIANCE_START`…
//! `COVARIANCE_STOP` section.

use std::iter::Peekable;
use std::vec::IntoIter;

use super::layout::{self, VERSION_KEY};
use super::{EphemerisCovariance, EphemerisState, Oem, OemData, OemSegment};
use crate::block::{Block, Fields};
use crate::covariance::Covariance6;
use crate::error::{Error, ErrorKind, Location, Result};
use crate::extension::Extension;
use crate::header::{Header, Version};
use crate::kvn::flat::describe;
use crate::kvn::lexer::{Line, lex};
use crate::kvn::writer::KvnWriter;
use crate::{Epoch, ParseMode, WriteOptions, number};

type Lines = Peekable<IntoIter<Line>>;

fn is_marker(line: Option<&Line>, marker: &str) -> bool {
    matches!(line, Some(Line::Bare { text, .. }) if text == marker)
}

fn end(expected: &'static str, at: Location) -> Error {
    Error::new(ErrorKind::UnexpectedEnd { expected }).at(at)
}

pub(super) fn read(text: &str, mode: ParseMode) -> Result<Oem> {
    let mut lines = lex(text, mode).into_iter().peekable();
    let version_entry = match lines.next() {
        Some(Line::Pair(entry)) if entry.keyword == VERSION_KEY => entry,
        Some(other) => {
            return Err(Error::new(ErrorKind::WrongMessage {
                expected: "OEM",
                found: describe(&other),
            })
            .at(other.at()));
        }
        None => return Err(Error::new(ErrorKind::Empty)),
    };
    let version = Version::parse(&version_entry, mode)?;
    let start = version_entry.at.unwrap_or(Location::new(1, 1));
    let mut extensions = Vec::new();
    let header_block = keywords_until(&mut lines, "META_START", start)?;
    let header = Header::read(header_block, mode, &mut extensions)?;
    let mut segments = Vec::new();
    let mut pending = Vec::new();
    while let Some(line) = lines.next() {
        match line {
            Line::Comment { text, .. } => pending.push(text),
            Line::Bare { ref text, at } if text == "META_START" => {
                let mut block = keywords_until(&mut lines, "META_STOP", at)?;
                lines.next();
                pending.append(&mut block.comments);
                block.comments = std::mem::take(&mut pending);
                let metadata = layout::read_metadata(block, mode, &mut extensions)?;
                let data = read_data(&mut lines, mode, &mut pending, &mut extensions)?;
                segments.push(OemSegment { metadata, data });
            }
            other => {
                return Err(Error::new(ErrorKind::UnexpectedLine {
                    text: describe(&other),
                })
                .at(other.at()));
            }
        }
    }
    if segments.is_empty() {
        return Err(end("META_START", start));
    }
    if let Some(last) = segments.last_mut() {
        last.data.comments.append(&mut pending);
    }
    Ok(Oem {
        version,
        header,
        segments,
        extensions,
    })
}

/// Collects comments and keywords up to, not including, `marker`.
fn keywords_until(lines: &mut Lines, marker: &'static str, start: Location) -> Result<Block> {
    let mut block = Block::starting_at(Some(start));
    loop {
        if is_marker(lines.peek(), marker) {
            return Ok(block);
        }
        match lines.next() {
            Some(Line::Comment { text, .. }) => block.comments.push(text),
            Some(Line::Pair(entry)) => block.entries.push(entry),
            Some(other) => {
                return Err(Error::new(ErrorKind::UnexpectedLine {
                    text: describe(&other),
                })
                .at(other.at()));
            }
            None => return Err(end(marker, start)),
        }
    }
}

/// Reads ephemeris lines and covariances up to the next segment. Comments
/// after the last line are left in `pending` for the next segment.
fn read_data(
    lines: &mut Lines,
    mode: ParseMode,
    pending: &mut Vec<String>,
    extensions: &mut Vec<Extension>,
) -> Result<OemData> {
    let mut data = OemData::default();
    loop {
        let next = lines.peek();
        if next.is_none() || is_marker(next, "META_START") {
            return Ok(data);
        }
        match lines.next() {
            Some(Line::Comment { text, .. }) => pending.push(text),
            Some(Line::Bare { text, at }) if text == "COVARIANCE_START" => {
                data.comments.append(pending);
                let covariances = read_covariances(lines, mode, at, extensions)?;
                data.covariances.extend(covariances);
            }
            Some(Line::Bare { text, at }) => {
                // Comments before the first state belong to the data block;
                // later ones join it too, as the notation has no other place.
                data.comments.append(pending);
                data.states.push(state_line(&text, at, mode)?);
            }
            Some(Line::Pair(entry)) if mode == ParseMode::Lenient => {
                data.comments.append(pending);
                let mut extra = Block::starting_at(entry.at);
                extra.entries.push(entry);
                Fields::new(extra, mode)?.1.finish(extensions)?;
            }
            Some(other) => {
                return Err(Error::new(ErrorKind::UnexpectedLine {
                    text: describe(&other),
                })
                .at(other.at()));
            }
            None => return Ok(data),
        }
    }
}

fn real_at(token: &str, at: Location, mode: ParseMode) -> Result<f64> {
    number::parse_f64(token, mode).ok_or_else(|| {
        Error::new(ErrorKind::InvalidNumber {
            text: token.to_owned(),
        })
        .at(at)
    })
}

fn state_line(text: &str, at: Location, mode: ParseMode) -> Result<EphemerisState> {
    let tokens: Vec<&str> = text.split_whitespace().collect();
    let (epoch, values) = match tokens.split_first() {
        Some((epoch, values)) if values.len() == 6 || values.len() == 9 => (epoch, values),
        _ => {
            return Err(Error::new(ErrorKind::ColumnCount {
                expected: "an epoch and 6 or 9 values",
                found: tokens.len(),
            })
            .at(at));
        }
    };
    let epoch = Epoch::parse(epoch).map_err(|reason| {
        Error::new(ErrorKind::InvalidEpoch {
            text: (*epoch).to_owned(),
            reason,
        })
        .at(at)
    })?;
    let mut values = values.iter();
    let mut next3 = || -> Result<[f64; 3]> {
        let mut triple = [0.0; 3];
        for slot in &mut triple {
            let token = values.next().copied().unwrap_or_default();
            *slot = real_at(token, at, mode)?;
        }
        Ok(triple)
    };
    let position = crate::unit::vector3(next3()?);
    let velocity = crate::unit::vector3(next3()?);
    let acceleration = if tokens.len() == 10 {
        Some(crate::unit::vector3(next3()?))
    } else {
        None
    };
    Ok(EphemerisState {
        epoch,
        position,
        velocity,
        acceleration,
    })
}

fn read_covariances(
    lines: &mut Lines,
    mode: ParseMode,
    start: Location,
    extensions: &mut Vec<Extension>,
) -> Result<Vec<EphemerisCovariance>> {
    let mut out = Vec::new();
    let mut comments = Vec::new();
    let mut current: Option<(Block, Vec<f64>, usize)> = None;
    loop {
        match lines.next() {
            None => return Err(end("COVARIANCE_STOP", start)),
            Some(Line::Comment { text, .. }) => comments.push(text),
            Some(Line::Bare { text, .. }) if text == "COVARIANCE_STOP" => {
                if let Some(open) = current.take() {
                    out.push(finish_covariance(open, mode, start, extensions)?);
                }
                return Ok(out);
            }
            Some(Line::Pair(entry)) => {
                let restart = entry.keyword == "EPOCH"
                    || current.as_ref().is_some_and(|(block, rows, _)| {
                        !rows.is_empty() || block.has(&entry.keyword)
                    });
                if restart && let Some(open) = current.take() {
                    out.push(finish_covariance(open, mode, start, extensions)?);
                }
                let (block, _, _) =
                    current.get_or_insert_with(|| (Block::starting_at(entry.at), Vec::new(), 0));
                block.comments.append(&mut comments);
                block.entries.push(entry);
            }
            Some(Line::Bare { text, at }) => {
                let Some((_, terms, rows)) = current.as_mut() else {
                    return Err(Error::new(ErrorKind::UnexpectedLine { text }).at(at));
                };
                *rows = rows.saturating_add(1);
                matrix_row(&text, at, *rows, mode, terms)?;
            }
        }
    }
}

/// One lower-triangle row: row `n` holds `n` terms (strict mode).
fn matrix_row(
    text: &str,
    at: Location,
    row: usize,
    mode: ParseMode,
    terms: &mut Vec<f64>,
) -> Result<()> {
    let tokens: Vec<&str> = text.split_whitespace().collect();
    if mode == ParseMode::Strict && tokens.len() != row {
        return Err(Error::new(ErrorKind::ColumnCount {
            expected: "row n of the lower triangle to hold n values",
            found: tokens.len(),
        })
        .at(at));
    }
    for token in tokens {
        terms.push(real_at(token, at, mode)?);
    }
    Ok(())
}

fn finish_covariance(
    (block, terms, _): (Block, Vec<f64>, usize),
    mode: ParseMode,
    start: Location,
    extensions: &mut Vec<Extension>,
) -> Result<EphemerisCovariance> {
    let at = block.at.unwrap_or(start);
    let (comments, mut fields) = Fields::new(block, mode)?;
    let epoch = fields.required("EPOCH", crate::block::Entry::epoch)?;
    let ref_frame = fields.optional("COV_REF_FRAME", crate::block::Entry::frame)?;
    fields.finish(extensions)?;
    let lower: [f64; 21] = terms.try_into().map_err(|terms: Vec<f64>| {
        Error::new(ErrorKind::ColumnCount {
            expected: "21 lower-triangle values",
            found: terms.len(),
        })
        .at(at)
    })?;
    Ok(EphemerisCovariance {
        comments,
        epoch,
        ref_frame,
        matrix: Covariance6::from_lower(lower),
    })
}

pub(super) fn write(oem: &Oem, options: WriteOptions) -> Result<String> {
    let mut out = KvnWriter::default();
    out.pair(VERSION_KEY, oem.version.as_str());
    out.block(&oem.header.write(options))?;
    for segment in &oem.segments {
        out.blank();
        out.line("META_START");
        out.block(&layout::write_metadata(&segment.metadata, options))?;
        out.line("META_STOP");
        let data = &segment.data;
        out.blank();
        out.comments(&data.comments)?;
        for state in &data.states {
            out.line(&state_text(state)?);
        }
        if !data.covariances.is_empty() {
            out.blank();
            out.line("COVARIANCE_START");
            for covariance in &data.covariances {
                write_covariance(&mut out, covariance)?;
            }
            out.line("COVARIANCE_STOP");
        }
    }
    if !oem.extensions.is_empty() {
        out.blank();
        out.block(&crate::extension::write_extensions(&oem.extensions))?;
    }
    Ok(out.finish())
}

fn formatted(value: f64) -> Result<String> {
    number::format_f64(value).ok_or_else(|| Error::new(ErrorKind::NonFiniteNumber))
}

fn state_text(state: &EphemerisState) -> Result<String> {
    let mut text = state.epoch.to_string();
    let position = state.position.iter().map(|q| q.value());
    let velocity = state.velocity.iter().map(|q| q.value());
    let acceleration = state.acceleration.iter().flatten().map(|q| q.value());
    for value in position.chain(velocity).chain(acceleration) {
        text.push(' ');
        text.push_str(&formatted(value)?);
    }
    Ok(text)
}

fn write_covariance(out: &mut KvnWriter, covariance: &EphemerisCovariance) -> Result<()> {
    let mut block = crate::block::BlockWriter::new(&covariance.comments, WriteOptions::default());
    block.text("EPOCH", &covariance.epoch);
    block.text_opt("COV_REF_FRAME", covariance.ref_frame.as_ref());
    out.block(&block.finish())?;
    let mut terms = covariance.matrix.lower().iter();
    for row in 1..=6 {
        let values = terms
            .by_ref()
            .take(row)
            .map(|value| formatted(*value))
            .collect::<Result<Vec<_>>>()?;
        out.line(&values.join(" "));
    }
    Ok(())
}
