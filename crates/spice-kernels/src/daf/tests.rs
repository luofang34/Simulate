use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use super::{Daf, DafArray, Endian, encode, layout};
use crate::error::Error;

fn i32_at(bytes: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn f64_at(bytes: &[u8], offset: usize) -> f64 {
    f64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}

/// `count` arrays of differing lengths, so that data crosses record
/// boundaries and, past 25 arrays, summaries cross summary records.
fn spk_shaped(count: usize) -> Daf {
    let mut daf = Daf::new("SPK", 2, 6, "unit test file").unwrap();
    for index in 0..count {
        let data = (0..37 + 11 * index)
            .map(|k| index as f64 + k as f64 * 0.125)
            .collect();
        let array = DafArray::new(
            &format!("array {index}"),
            vec![index as f64, index as f64 + 1.0],
            vec![index as i32, -1, 1, 9],
            data,
        )
        .unwrap();
        daf.push(array).unwrap();
    }
    daf
}

#[test]
fn file_record_has_the_documented_layout() {
    let bytes = spk_shaped(1).to_bytes().unwrap();
    assert_eq!(bytes.len() % 1024, 0);
    assert_eq!(&bytes[0..8], b"DAF/SPK ");
    assert_eq!(i32_at(&bytes, 8), 2);
    assert_eq!(i32_at(&bytes, 12), 6);
    assert_eq!(&bytes[16..30], b"unit test file");
    assert!(bytes[30..76].iter().all(|&byte| byte == b' '));
    // No comment records: the summary record is record 2, its name record 3,
    // data starts at word 385 and 37 words follow.
    assert_eq!(i32_at(&bytes, 76), 2);
    assert_eq!(i32_at(&bytes, 80), 2);
    assert_eq!(i32_at(&bytes, 84), 385 + 37);
    assert_eq!(&bytes[88..96], b"LTL-IEEE");
    assert!(bytes[96..699].iter().all(|&byte| byte == 0));
    assert_eq!(
        &bytes[699..727],
        b"FTPSTR:\r:\n:\r\n:\r\x00:\x81:\x10\xce:ENDFTP"
    );
    assert!(bytes[727..1024].iter().all(|&byte| byte == 0));
}

#[test]
fn summary_and_name_records_have_the_documented_layout() {
    let bytes = spk_shaped(1).to_bytes().unwrap();
    let summary = &bytes[1024..2048];
    assert_eq!(f64_at(summary, 0), 0.0);
    assert_eq!(f64_at(summary, 8), 0.0);
    assert_eq!(f64_at(summary, 16), 1.0);
    assert_eq!(f64_at(summary, 24), 0.0);
    assert_eq!(f64_at(summary, 32), 1.0);
    let integers: Vec<i32> = (0..6).map(|k| i32_at(summary, 40 + 4 * k)).collect();
    assert_eq!(integers, [0, -1, 1, 9, 385, 421]);
    assert!(summary[64..].iter().all(|&byte| byte == 0));
    let names = &bytes[2048..3072];
    assert_eq!(&names[0..7], b"array 0");
    assert!(names[7..1000].iter().all(|&byte| byte == b' '));
    assert!(names[1000..].iter().all(|&byte| byte == 0));
    assert_eq!(f64_at(&bytes, 3072 + 8), 0.125);
}

#[test]
fn summary_records_chain_when_one_fills() {
    let daf = spk_shaped(30);
    let bytes = daf.to_bytes().unwrap();
    let first = i32_at(&bytes, 76) as usize;
    let last = i32_at(&bytes, 80) as usize;
    assert_eq!(first, 2);
    assert!(last > first);
    let first_record = &bytes[(first - 1) * 1024..];
    let last_record = &bytes[(last - 1) * 1024..];
    assert_eq!(f64_at(first_record, 0), last as f64);
    assert_eq!(f64_at(first_record, 16), 25.0);
    assert_eq!(f64_at(last_record, 0), 0.0);
    assert_eq!(f64_at(last_record, 8), first as f64);
    assert_eq!(f64_at(last_record, 16), 5.0);
    // The free pointer is one past the last array's last word.
    let last_end = i32_at(last_record, 24 + 4 * 40 + 16 + 20);
    assert_eq!(i32_at(&bytes, 84), last_end + 1);
    assert_eq!(Daf::parse(&bytes).unwrap(), daf);
}

#[test]
fn both_byte_orders_parse_to_the_same_model() {
    let mut daf = spk_shaped(30);
    daf.set_comment_lines(vec!["big and little".into()])
        .unwrap();
    let little = encode::encode(&daf, Endian::Little).unwrap();
    let big = encode::encode(&daf, Endian::Big).unwrap();
    assert_ne!(little, big);
    assert_eq!(&big[88..96], b"BIG-IEEE");
    // Integers swap as four-byte units, doubles as eight-byte units.
    assert_eq!(&big[8..12], &[0, 0, 0, 2]);
    assert_eq!(&big[2048 + 16..2048 + 24], &25.0_f64.to_be_bytes());
    let from_little = Daf::parse(&little).unwrap();
    let from_big = Daf::parse(&big).unwrap();
    assert_eq!(from_little, from_big);
    assert_eq!(from_big.arrays(), daf.arrays());
}

#[test]
fn comment_area_round_trips_across_records() {
    let mut daf = spk_shaped(2);
    let mut lines: Vec<String> = (0..40)
        .map(|k| format!("line {k} {}", "x".repeat(k * 3)))
        .collect();
    lines.push(String::new());
    lines.push("after a blank line".into());
    lines.push(String::new());
    daf.set_comment_lines(lines.clone()).unwrap();
    let bytes = daf.to_bytes().unwrap();
    let first_summary = i32_at(&bytes, 76) as usize;
    assert!(first_summary > 3, "the text needs more than one record");
    // Text occupies 1000 characters per record; the other 24 bytes are NUL.
    assert!(bytes[1024 + 1000..2048].iter().all(|&byte| byte == 0));
    let parsed = Daf::parse(&bytes).unwrap();
    assert_eq!(parsed.comment_lines(), lines);
    assert_eq!(parsed, daf);
    assert_eq!(parsed.to_bytes().unwrap(), bytes);

    // Spare records follow the text and survive the round trip.
    daf.set_spare_comment_records(2);
    let spared = daf.to_bytes().unwrap();
    assert_eq!(i32_at(&spared, 76) as usize, first_summary + 2);
    assert_eq!(Daf::parse(&spared).unwrap(), daf);
}

#[test]
fn spare_comment_records_without_text_read_as_no_comment() {
    let mut daf = spk_shaped(1);
    daf.set_spare_comment_records(3);
    let bytes = daf.to_bytes().unwrap();
    assert_eq!(i32_at(&bytes, 76), 5);
    assert!(bytes[1024..4096].iter().all(|&byte| byte == 0));
    let parsed = Daf::parse(&bytes).unwrap();
    assert!(parsed.comment_lines().is_empty());
    assert_eq!(parsed.spare_comment_records(), 3);
    assert_eq!(parsed, daf);
}

#[test]
fn odd_integer_count_and_empty_file_round_trip() {
    // The binary PCK shape: five integers leave half a summary word unused.
    let mut pck = Daf::new("PCK", 2, 5, "odd").unwrap();
    for index in 0..3 {
        let array = DafArray::new(
            "orientation",
            vec![0.0, 1.0],
            vec![index, 1, 2],
            vec![1.5; 200],
        )
        .unwrap();
        pck.push(array).unwrap();
    }
    let bytes = pck.to_bytes().unwrap();
    assert_eq!(&bytes[0..8], b"DAF/PCK ");
    assert_eq!(Daf::parse(&bytes).unwrap(), pck);

    let empty = Daf::new("CK", 2, 6, "").unwrap();
    let bytes = empty.to_bytes().unwrap();
    assert_eq!(bytes.len(), 3 * 1024);
    assert_eq!(i32_at(&bytes, 84), 385);
    assert_eq!(Daf::parse(&bytes).unwrap(), empty);
}

#[test]
fn damaged_ftp_string_is_rejected_and_absent_one_accepted() {
    let bytes = spk_shaped(1).to_bytes().unwrap();
    // A text-mode transfer turns the carriage-return line-feed pair into a
    // bare line feed, shifting what follows.
    let mut damaged = bytes.clone();
    damaged.remove(699 + 11);
    damaged.insert(1023, 0);
    assert!(matches!(Daf::parse(&damaged), Err(Error::FtpCorruption)));

    let mut absent = bytes.clone();
    absent[699..727].fill(0);
    assert!(Daf::parse(&absent).is_ok());
}

#[test]
fn structural_faults_are_typed_errors() {
    let bytes = spk_shaped(30).to_bytes().unwrap();
    assert!(matches!(
        Daf::parse(&bytes[..1000]),
        Err(Error::Truncated { .. })
    ));

    let mut not_daf = bytes.clone();
    not_daf[0..8].copy_from_slice(b"NAIF/DAF");
    assert!(matches!(Daf::parse(&not_daf), Err(Error::NotDaf { .. })));

    let mut vax = bytes.clone();
    vax[88..96].copy_from_slice(b"VAX-GFLT");
    assert!(matches!(
        Daf::parse(&vax),
        Err(Error::UnsupportedBinaryFormat { .. })
    ));

    let mut shape = bytes.clone();
    shape[8..12].copy_from_slice(&200_i32.to_le_bytes());
    assert!(matches!(
        Daf::parse(&shape),
        Err(Error::InvalidSummaryFormat { nd: 200, ni: 6 })
    ));

    // A summary record that names itself as its successor.
    let mut cyclic = bytes.clone();
    cyclic[1024..1032].copy_from_slice(&2.0_f64.to_le_bytes());
    assert!(matches!(
        Daf::parse(&cyclic),
        Err(Error::CorruptSummaryChain { record: 2 })
    ));

    let mut outside = bytes.clone();
    outside[1024..1032].copy_from_slice(&1.0e6_f64.to_le_bytes());
    assert!(matches!(
        Daf::parse(&outside),
        Err(Error::CorruptSummaryChain { .. })
    ));

    let mut reversed = bytes.clone();
    reversed[1024 + 24 + 32..1024 + 24 + 36].copy_from_slice(&9_000_000_i32.to_le_bytes());
    assert!(matches!(
        Daf::parse(&reversed),
        Err(Error::InvalidAddressRange { .. })
    ));

    // Losing the last data record leaves the last array short.
    let cut = &bytes[..bytes.len() - 1024];
    assert!(matches!(
        Daf::parse(cut),
        Err(Error::InvalidAddressRange { .. })
    ));
}

#[test]
fn builder_rejects_what_the_format_cannot_store() {
    assert!(matches!(
        Daf::new("SPK", 2, 1, ""),
        Err(Error::InvalidSummaryFormat { .. })
    ));
    assert!(matches!(
        Daf::new("SPK", 124, 6, ""),
        Err(Error::InvalidSummaryFormat { .. })
    ));
    assert!(matches!(
        Daf::new("TOOLONG", 2, 6, ""),
        Err(Error::TextTooLong { .. })
    ));
    assert!(matches!(
        Daf::new("SPK", 2, 6, &"n".repeat(61)),
        Err(Error::TextTooLong { max: 60, .. })
    ));
    assert!(matches!(
        Daf::new("SPK", 2, 6, "tab\there"),
        Err(Error::InvalidCharacter {
            character: '\t',
            ..
        })
    ));
    assert!(matches!(
        DafArray::new("empty", vec![], vec![], vec![]),
        Err(Error::EmptyArray { .. })
    ));

    let mut daf = Daf::new("SPK", 2, 6, "").unwrap();
    let short = DafArray::new("short", vec![0.0], vec![1, 2, 3, 4], vec![1.0]).unwrap();
    assert!(matches!(
        daf.push(short),
        Err(Error::SummaryShapeMismatch { .. })
    ));
    let long = DafArray::new(&"n".repeat(41), vec![0.0, 1.0], vec![1, 2, 3, 4], vec![1.0]).unwrap();
    assert!(matches!(
        daf.push(long),
        Err(Error::TextTooLong { max: 40, .. })
    ));
    assert!(matches!(
        daf.set_comment_lines(vec!["end\u{4}of text".into()]),
        Err(Error::InvalidCharacter { .. })
    ));
    assert!(matches!(
        daf.set_comment_lines(vec!["\u{3bc}".into()]),
        Err(Error::InvalidCharacter { .. })
    ));
}

#[test]
fn field_text_trims_blank_and_nul_padding() {
    assert_eq!(layout::field_text(b"name  \0\0"), "name");
    assert_eq!(layout::field_text(b"a b\0\0\0  "), "a b");
    assert_eq!(layout::field_text(b"        "), "");
}
