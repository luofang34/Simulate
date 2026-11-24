//! Parsing a DAF byte image into the in-memory model.

use alloc::string::String;
use alloc::vec::Vec;

use super::layout::{self, RECORD_BYTES, RECORD_TEXT_CHARS, SUMMARY_AREA_WORDS};
use super::{Daf, DafArray, Endian};
use crate::error::Error;

/// Shape of the file's summaries, fixed by the file record.
struct Shape {
    endian: Endian,
    nd: usize,
    ni: usize,
    summary_words: usize,
}

pub(super) fn parse(bytes: &[u8]) -> Result<Daf, Error> {
    let file_record = layout::window(bytes, 0, RECORD_BYTES)?;
    let id_word = layout::window(file_record, 0, 8)?;
    if !id_word.starts_with(layout::ID_WORD_PREFIX) {
        return Err(Error::NotDaf {
            found: layout::field_text(id_word),
        });
    }
    let kernel_type =
        layout::field_text(id_word.get(layout::ID_WORD_PREFIX.len()..).unwrap_or(&[]));
    let endian = endian_of(file_record)?;
    check_ftp_string(file_record)?;

    let nd = layout::read_i32(file_record, layout::OFFSET_ND, endian)?;
    let ni = layout::read_i32(file_record, layout::OFFSET_NI, endian)?;
    let invalid = || Error::InvalidSummaryFormat {
        nd: i64::from(nd),
        ni: i64::from(ni),
    };
    let nd = usize::try_from(nd).map_err(|_| invalid())?;
    let ni = usize::try_from(ni).map_err(|_| invalid())?;
    let shape = Shape {
        endian,
        nd,
        ni,
        summary_words: layout::summary_words(nd, ni)?,
    };
    let internal_name = layout::field_text(layout::window(
        file_record,
        layout::OFFSET_INTERNAL_NAME,
        layout::INTERNAL_NAME_CHARS,
    )?);
    let forward = layout::read_i32(file_record, layout::OFFSET_FORWARD, endian)?;
    let first_summary = usize::try_from(forward)
        .ok()
        .filter(|&record| record >= 2)
        .ok_or(Error::CorruptSummaryChain { record: 0 })?;

    let comment_lines = comment_lines(bytes, first_summary)?;
    let text_records = layout::comment_text(&comment_lines)
        .len()
        .div_ceil(RECORD_TEXT_CHARS);
    Ok(Daf {
        kernel_type,
        nd,
        ni,
        internal_name,
        comment_lines,
        spare_comment_records: first_summary.saturating_sub(2).saturating_sub(text_records),
        arrays: arrays(bytes, first_summary, &shape)?,
    })
}

fn endian_of(file_record: &[u8]) -> Result<Endian, Error> {
    let format = layout::window(file_record, layout::OFFSET_FORMAT, 8)?;
    if format == layout::FORMAT_LITTLE {
        Ok(Endian::Little)
    } else if format == layout::FORMAT_BIG {
        Ok(Endian::Big)
    } else {
        Err(Error::UnsupportedBinaryFormat {
            found: layout::field_text(format),
        })
    }
}

/// A file written before the validation string existed has none, which is
/// fine; one whose string is present but altered went through a text-mode
/// transfer and its numeric data cannot be trusted.
fn check_ftp_string(file_record: &[u8]) -> Result<(), Error> {
    let slot = layout::window(file_record, layout::OFFSET_FTP, layout::FTP_STRING.len())?;
    if slot == layout::FTP_STRING {
        return Ok(());
    }
    let tail = file_record.get(layout::OFFSET_FORMAT..).unwrap_or(&[]);
    let has = |marker: &[u8]| {
        tail.windows(marker.len())
            .any(|candidate| candidate == marker)
    };
    if has(layout::FTP_START) || has(layout::FTP_END) {
        Err(Error::FtpCorruption)
    } else {
        Ok(())
    }
}

/// One-based record number to byte offset.
fn record_offset(record: usize) -> usize {
    record.saturating_sub(1).saturating_mul(RECORD_BYTES)
}

/// The comment area's lines: the text of every record between the file
/// record and the first summary record, up to the end-of-text marker. An
/// area without the marker holds reserved space only.
fn comment_lines(bytes: &[u8], first_summary: usize) -> Result<Vec<String>, Error> {
    let mut text = Vec::new();
    for record in 2..first_summary {
        text.extend_from_slice(layout::window(
            bytes,
            record_offset(record),
            RECORD_TEXT_CHARS,
        )?);
    }
    let Some(end) = text.iter().position(|&byte| byte == layout::COMMENT_EOT) else {
        return Ok(Vec::new());
    };
    let body = text.get(..end).unwrap_or(&[]);
    let mut lines: Vec<String> = body
        .split(|&byte| byte == layout::COMMENT_EOL)
        .map(|line| line.iter().map(|&byte| char::from(byte)).collect())
        .collect();
    // Every line ends with the end-of-line marker, so the split leaves one
    // empty piece after the last line.
    if lines.last().is_some_and(String::is_empty) {
        lines.pop();
    }
    Ok(lines)
}

/// A control word of a summary record: an integer stored as a double.
fn control_word(
    bytes: &[u8],
    offset: usize,
    endian: Endian,
    record: usize,
) -> Result<usize, Error> {
    let value = layout::read_f64(bytes, offset, endian)?;
    // The cast saturates and maps NaN to zero, so comparing back rejects
    // every value that is not a small non-negative integer.
    let count = value as usize;
    if count as f64 == value && count <= i32::MAX as usize {
        Ok(count)
    } else {
        Err(Error::CorruptSummaryChain { record })
    }
}

fn arrays(bytes: &[u8], first_summary: usize, shape: &Shape) -> Result<Vec<DafArray>, Error> {
    let total_records = bytes.len() / RECORD_BYTES;
    let per_record = SUMMARY_AREA_WORDS / shape.summary_words;
    let mut out = Vec::new();
    let mut record = first_summary;
    let mut visited = 0_usize;
    while record != 0 {
        // A chain longer than the file has records revisits one of them.
        visited = visited.wrapping_add(1);
        if record < 2 || record >= total_records || visited > total_records {
            return Err(Error::CorruptSummaryChain { record });
        }
        let base = record_offset(record);
        let next = control_word(bytes, base, shape.endian, record)?;
        let count = control_word(bytes, base.saturating_add(16), shape.endian, record)?;
        if count > per_record {
            return Err(Error::CorruptSummaryChain { record });
        }
        for index in 0..count {
            out.push(array(bytes, record, index, shape)?);
        }
        record = next;
    }
    Ok(out)
}

fn array(bytes: &[u8], record: usize, index: usize, shape: &Shape) -> Result<DafArray, Error> {
    let summary_bytes = shape.summary_words.saturating_mul(8);
    let summary_at = record_offset(record)
        .saturating_add(24)
        .saturating_add(index.saturating_mul(summary_bytes));
    let mut doubles = Vec::with_capacity(shape.nd);
    for k in 0..shape.nd {
        let offset = summary_at.saturating_add(k.saturating_mul(8));
        doubles.push(layout::read_f64(bytes, offset, shape.endian)?);
    }
    let integers_at = summary_at.saturating_add(shape.nd.saturating_mul(8));
    let mut integers = Vec::with_capacity(shape.ni);
    for k in 0..shape.ni {
        let offset = integers_at.saturating_add(k.saturating_mul(4));
        integers.push(layout::read_i32(bytes, offset, shape.endian)?);
    }
    let name_at =
        record_offset(record.saturating_add(1)).saturating_add(index.saturating_mul(summary_bytes));
    let name = layout::field_text(layout::window(bytes, name_at, summary_bytes)?);

    let end = integers.pop().unwrap_or(0);
    let start = integers.pop().unwrap_or(0);
    let invalid = || Error::InvalidAddressRange {
        name: name.clone(),
        start: i64::from(start),
        end: i64::from(end),
    };
    let first = usize::try_from(start).ok().filter(|&word| word >= 1);
    let last = usize::try_from(end).ok();
    let (Some(first), Some(last)) = (first, last) else {
        return Err(invalid());
    };
    if last < first {
        return Err(invalid());
    }
    let words = last.saturating_sub(first).saturating_add(1);
    let raw = layout::window(
        bytes,
        first.saturating_sub(1).saturating_mul(8),
        words.saturating_mul(8),
    )
    .map_err(|_| invalid())?;
    let mut data = Vec::with_capacity(words);
    for offset in (0..raw.len()).step_by(8) {
        data.push(layout::read_f64(raw, offset, shape.endian)?);
    }
    Ok(DafArray {
        name,
        doubles,
        integers,
        data,
    })
}
