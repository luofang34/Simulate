//! Laying a DAF model out as records and writing the byte image.
//!
//! The layout is the one SPICE produces when arrays are added in order to a
//! new file: the file record, the comment records, then for each group of
//! arrays that fills one summary record that summary record, its name record
//! and the group's data, each group starting on a record boundary.

use alloc::vec::Vec;

use super::layout::{self, RECORD_BYTES, RECORD_TEXT_CHARS, SUMMARY_AREA_WORDS, WORDS_PER_RECORD};
use super::{Daf, DafArray, Endian};
use crate::error::Error;

/// Where one summary record and the data of its arrays go.
struct Group<'a> {
    arrays: &'a [DafArray],
    /// One-based number of the summary record.
    record: usize,
    /// One-based address of the group's first data word.
    first_word: usize,
    /// Data words of all the group's arrays.
    words: usize,
}

pub(super) fn encode(daf: &Daf, endian: Endian) -> Result<Vec<u8>, Error> {
    let summary_words = layout::summary_words(daf.nd, daf.ni)?;
    let comment = comment_area(daf);
    let first_summary = (comment.len() / RECORD_BYTES).saturating_add(2);
    let groups = plan(daf, summary_words, first_summary);
    let free = groups
        .last()
        .map_or(1, |group| group.first_word.saturating_add(group.words));
    let last_summary = groups.last().map_or(first_summary, |group| group.record);

    let mut out = Vec::new();
    push_file_record(
        &mut out,
        daf,
        endian,
        [
            address(first_summary)?,
            address(last_summary)?,
            address(free)?,
        ],
    );
    out.extend_from_slice(&comment);
    for (index, group) in groups.iter().enumerate() {
        let previous = index
            .checked_sub(1)
            .and_then(|at| groups.get(at))
            .map_or(0, |other| other.record);
        let next = groups
            .get(index.saturating_add(1))
            .map_or(0, |other| other.record);
        push_summary_record(&mut out, daf, group, endian, [next, previous])?;
        push_name_record(&mut out, group, summary_words.saturating_mul(8));
        for array in group.arrays {
            for &word in &array.data {
                layout::push_f64(&mut out, word, endian);
            }
        }
        layout::pad_record(&mut out, 0);
    }
    Ok(out)
}

/// DAF addresses and record numbers are 32-bit integers in the file.
fn address(value: usize) -> Result<i32, Error> {
    i32::try_from(value).map_err(|_| Error::TooLarge { words: value })
}

fn plan(daf: &Daf, summary_words: usize, first_summary: usize) -> Vec<Group<'_>> {
    let per_record = (SUMMARY_AREA_WORDS / summary_words).max(1);
    let mut chunks: Vec<&[DafArray]> = daf.arrays.chunks(per_record).collect();
    if chunks.is_empty() {
        // A DAF without arrays still has one summary and one name record.
        chunks.push(&[]);
    }
    let mut record = first_summary;
    let mut groups = Vec::with_capacity(chunks.len());
    for arrays in chunks {
        let words = arrays
            .iter()
            .fold(0_usize, |sum, array| sum.saturating_add(array.data.len()));
        // The data starts in the record after the name record.
        let first_word = record
            .saturating_add(1)
            .saturating_mul(WORDS_PER_RECORD)
            .saturating_add(1);
        groups.push(Group {
            arrays,
            record,
            first_word,
            words,
        });
        record = record
            .saturating_add(2)
            .saturating_add(words.div_ceil(WORDS_PER_RECORD));
    }
    groups
}

/// The comment records: the text in 1000-character records padded with
/// blanks, then the spare records as NULs.
fn comment_area(daf: &Daf) -> Vec<u8> {
    let text = layout::comment_text(&daf.comment_lines);
    let mut out = Vec::new();
    for chunk in text.chunks(RECORD_TEXT_CHARS) {
        let start = out.len();
        out.extend_from_slice(chunk);
        out.resize(start.saturating_add(RECORD_TEXT_CHARS), b' ');
        layout::pad_record(&mut out, 0);
    }
    let spare = daf.spare_comment_records.saturating_mul(RECORD_BYTES);
    out.resize(out.len().saturating_add(spare), 0);
    out
}

fn push_file_record(out: &mut Vec<u8>, daf: &Daf, endian: Endian, pointers: [i32; 3]) {
    out.extend_from_slice(layout::ID_WORD_PREFIX);
    layout::push_padded(out, &daf.kernel_type, layout::KERNEL_TYPE_CHARS);
    layout::push_i32(out, i32::try_from(daf.nd).unwrap_or(0), endian);
    layout::push_i32(out, i32::try_from(daf.ni).unwrap_or(0), endian);
    layout::push_padded(out, &daf.internal_name, layout::INTERNAL_NAME_CHARS);
    for pointer in pointers {
        layout::push_i32(out, pointer, endian);
    }
    out.extend_from_slice(match endian {
        Endian::Little => layout::FORMAT_LITTLE,
        Endian::Big => layout::FORMAT_BIG,
    });
    out.resize(layout::OFFSET_FTP, 0);
    out.extend_from_slice(layout::FTP_STRING);
    layout::pad_record(out, 0);
}

fn push_summary_record(
    out: &mut Vec<u8>,
    daf: &Daf,
    group: &Group<'_>,
    endian: Endian,
    [next, previous]: [usize; 2],
) -> Result<(), Error> {
    // Control words are integers stored as doubles; record numbers were
    // checked against the 32-bit range, so the conversion is exact.
    for control in [next, previous, group.arrays.len()] {
        layout::push_f64(out, f64::from(address(control)?), endian);
    }
    let mut first = group.first_word;
    for array in group.arrays {
        let last = first.saturating_add(array.data.len()).saturating_sub(1);
        for &value in &array.doubles {
            layout::push_f64(out, value, endian);
        }
        for &value in &array.integers {
            layout::push_i32(out, value, endian);
        }
        layout::push_i32(out, address(first)?, endian);
        layout::push_i32(out, address(last)?, endian);
        if daf.ni % 2 == 1 {
            // An odd integer count leaves half a word unused.
            layout::push_i32(out, 0, endian);
        }
        first = last.saturating_add(1);
    }
    layout::pad_record(out, 0);
    Ok(())
}

fn push_name_record(out: &mut Vec<u8>, group: &Group<'_>, name_chars: usize) {
    let start = out.len();
    for array in group.arrays {
        layout::push_padded(out, &array.name, name_chars);
    }
    out.resize(start.saturating_add(RECORD_TEXT_CHARS), b' ');
    layout::pad_record(out, 0);
}
