//! Fixed offsets and sizes of the DAF record layout, and the byte-level
//! readers and writers shared by decoding and encoding.

use alloc::string::String;
use alloc::vec::Vec;

use super::Endian;
use crate::error::Error;

pub(super) const RECORD_BYTES: usize = 1024;
pub(super) const WORDS_PER_RECORD: usize = 128;
/// Text characters a comment or name record holds; the rest of the record is
/// not text.
pub(super) const RECORD_TEXT_CHARS: usize = 1000;
/// Words a summary record gives to summaries after its three control words.
pub(super) const SUMMARY_AREA_WORDS: usize = 125;

pub(super) const ID_WORD_PREFIX: &[u8; 4] = b"DAF/";
pub(super) const KERNEL_TYPE_CHARS: usize = 4;
pub(super) const INTERNAL_NAME_CHARS: usize = 60;

pub(super) const OFFSET_ND: usize = 8;
pub(super) const OFFSET_NI: usize = 12;
pub(super) const OFFSET_INTERNAL_NAME: usize = 16;
pub(super) const OFFSET_FORWARD: usize = 76;
pub(super) const OFFSET_FORMAT: usize = 88;
pub(super) const OFFSET_FTP: usize = 699;

pub(super) const FORMAT_LITTLE: &[u8; 8] = b"LTL-IEEE";
pub(super) const FORMAT_BIG: &[u8; 8] = b"BIG-IEEE";

/// Byte sequence that a text-mode transfer would alter: every line-ending
/// convention, a NUL and bytes with the high bit set.
pub(super) const FTP_STRING: &[u8; 28] = b"FTPSTR:\r:\n:\r\n:\r\x00:\x81:\x10\xce:ENDFTP";
pub(super) const FTP_START: &[u8; 6] = b"FTPSTR";
pub(super) const FTP_END: &[u8; 6] = b"ENDFTP";

/// End-of-line and end-of-text markers of the comment area.
pub(super) const COMMENT_EOL: u8 = 0;
pub(super) const COMMENT_EOT: u8 = 4;

/// Words one summary occupies: `ND + (NI + 1) / 2`.
pub(super) fn summary_words(nd: usize, ni: usize) -> Result<usize, Error> {
    let invalid = || Error::InvalidSummaryFormat {
        nd: i64::try_from(nd).unwrap_or(i64::MAX),
        ni: i64::try_from(ni).unwrap_or(i64::MAX),
    };
    if nd > 124 || !(2..=250).contains(&ni) {
        return Err(invalid());
    }
    let words = nd.saturating_add(ni.div_ceil(2));
    if words > SUMMARY_AREA_WORDS {
        return Err(invalid());
    }
    Ok(words)
}

/// Rejects text the fixed-width name fields cannot store: SPICE allows
/// printable ASCII only.
pub(super) fn check_printable(field: &'static str, text: &str) -> Result<(), Error> {
    match text.chars().find(|c| !(' '..='~').contains(c)) {
        Some(character) => Err(Error::InvalidCharacter { field, character }),
        None => Ok(()),
    }
}

/// Rejects a comment line the comment area cannot store as one byte per
/// character, or that would be read back as a line or text terminator.
pub(super) fn check_comment_line(line: &str) -> Result<(), Error> {
    let storable =
        |c: &char| u8::try_from(*c).is_ok_and(|byte| byte != COMMENT_EOL && byte != COMMENT_EOT);
    match line.chars().find(|c| !storable(c)) {
        Some(character) => Err(Error::InvalidCharacter {
            field: "comment line",
            character,
        }),
        None => Ok(()),
    }
}

/// The comment area's text: each line followed by the end-of-line marker,
/// then the end-of-text marker. No lines give no text at all, which readers
/// take as an empty area.
pub(super) fn comment_text(lines: &[String]) -> Vec<u8> {
    let mut text = Vec::new();
    for line in lines {
        text.extend(line.chars().map(|c| u8::try_from(c).unwrap_or(b' ')));
        text.push(COMMENT_EOL);
    }
    if !text.is_empty() {
        text.push(COMMENT_EOT);
    }
    text
}

/// Text of a fixed-width field: one character per byte, trailing blanks and
/// NULs removed (writers pad with either).
pub(super) fn field_text(bytes: &[u8]) -> String {
    let used = bytes
        .iter()
        .rposition(|&byte| byte != b' ' && byte != 0)
        .map_or(0, |last| last.saturating_add(1));
    bytes
        .iter()
        .take(used)
        .map(|&byte| char::from(byte))
        .collect()
}

/// A bounds-checked window into `bytes`.
pub(super) fn window(bytes: &[u8], offset: usize, len: usize) -> Result<&[u8], Error> {
    let end = offset.saturating_add(len);
    bytes.get(offset..end).ok_or(Error::Truncated {
        len: bytes.len(),
        needed: end,
    })
}

fn array<const N: usize>(bytes: &[u8], offset: usize) -> Result<[u8; N], Error> {
    let mut out = [0_u8; N];
    out.copy_from_slice(window(bytes, offset, N)?);
    Ok(out)
}

pub(super) fn read_i32(bytes: &[u8], offset: usize, endian: Endian) -> Result<i32, Error> {
    let raw = array::<4>(bytes, offset)?;
    Ok(match endian {
        Endian::Little => i32::from_le_bytes(raw),
        Endian::Big => i32::from_be_bytes(raw),
    })
}

pub(super) fn read_f64(bytes: &[u8], offset: usize, endian: Endian) -> Result<f64, Error> {
    let raw = array::<8>(bytes, offset)?;
    Ok(match endian {
        Endian::Little => f64::from_le_bytes(raw),
        Endian::Big => f64::from_be_bytes(raw),
    })
}

pub(super) fn push_i32(out: &mut Vec<u8>, value: i32, endian: Endian) {
    match endian {
        Endian::Little => out.extend_from_slice(&value.to_le_bytes()),
        Endian::Big => out.extend_from_slice(&value.to_be_bytes()),
    }
}

pub(super) fn push_f64(out: &mut Vec<u8>, value: f64, endian: Endian) {
    match endian {
        Endian::Little => out.extend_from_slice(&value.to_le_bytes()),
        Endian::Big => out.extend_from_slice(&value.to_be_bytes()),
    }
}

/// Appends `text` as bytes, one per character, padded with blanks to `width`.
pub(super) fn push_padded(out: &mut Vec<u8>, text: &str, width: usize) {
    let start = out.len();
    out.extend(
        text.chars()
            .take(width)
            .map(|c| u8::try_from(c).unwrap_or(b' ')),
    );
    out.resize(start.saturating_add(width), b' ');
}

/// Pads `out` with `fill` up to the next record boundary.
pub(super) fn pad_record(out: &mut Vec<u8>, fill: u8) {
    let padded = out
        .len()
        .div_ceil(RECORD_BYTES)
        .saturating_mul(RECORD_BYTES);
    out.resize(padded, fill);
}
