//! Deterministic mutation sweep across KVN, XML and JSON fixtures:
//! never panics, and every accepted mutation round-trips with byte-stable writes.

use ccsds_ndm::{Ndm, Oem, Omm, Opm, ParseMode, WriteOptions};

#[derive(Clone, Copy)]
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn index(&mut self, len: usize) -> usize {
        if len == 0 {
            0
        } else {
            (self.next() as usize) % len
        }
    }
}

const MODES: [ParseMode; 2] = [ParseMode::Strict, ParseMode::Lenient];

const TRICKS: [&str; 12] = [
    "NaN",
    "Inf",
    "-inf",
    "1e999",
    "1e-999",
    "[furlongs]",
    "\u{0000}",
    "\u{feff}",
    "\r\n",
    "VENDOR_EXT = 1\n",
    "<x><y><z>",
    "\"\\",
];

fn char_boundary_at(text: &str, approx: usize) -> usize {
    let mut idx = approx.min(text.len());
    while idx > 0 && !text.is_char_boundary(idx) {
        idx -= 1;
    }
    idx
}

fn mutate_text(seed_text: &str, rng: &mut SplitMix64, step: usize) -> String {
    let lines: Vec<&str> = seed_text.lines().collect();
    match step % 8 {
        0 => String::new(),
        1 => seed_text[..char_boundary_at(seed_text, rng.index(seed_text.len() + 1))].to_owned(),
        2 => seed_text[char_boundary_at(seed_text, rng.index(seed_text.len()))..].to_owned(),
        3 if lines.len() > 1 => {
            let drop = rng.index(lines.len());
            lines
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != drop)
                .map(|(_, l)| *l)
                .collect::<Vec<_>>()
                .join("\n")
        }
        4 if lines.len() > 1 => {
            let mut copy = lines;
            let a = rng.index(copy.len());
            let b = rng.index(copy.len());
            copy.swap(a, b);
            copy.join("\n")
        }
        5 => {
            let cut = char_boundary_at(seed_text, rng.index(seed_text.len() + 1));
            let trick = TRICKS[rng.index(TRICKS.len())];
            format!("{}{}{}", &seed_text[..cut], trick, &seed_text[cut..])
        }
        6 => format!("\u{feff}{}", seed_text.replace('\n', "\r\n")),
        _ => {
            let mut out = seed_text.as_bytes().to_vec();
            for _ in 0..3 {
                if out.is_empty() {
                    break;
                }
                let pos = rng.index(out.len());
                out[pos] = ((rng.next() % 95) + 32) as u8;
            }
            String::from_utf8_lossy(&out).into_owned()
        }
    }
}

fn check_opm_kvn(text: &str) {
    let opts = WriteOptions::default();
    for mode in MODES {
        if let Ok(msg) = Opm::from_kvn(text, mode)
            && let Ok(written) = msg.to_kvn(opts)
        {
            let again = Opm::from_kvn(&written, mode).expect("re-parse kvn");
            assert_eq!(again, msg);
            assert_eq!(again.to_kvn(opts).unwrap(), written);
        }
        drop(Omm::from_kvn(text, mode));
        drop(Oem::from_kvn(text, mode));
    }
}

fn check_omm_kvn(text: &str) {
    let opts = WriteOptions::default();
    for mode in MODES {
        if let Ok(msgs) = Omm::from_kvn_all(text, mode)
            && !msgs.is_empty()
        {
            let first = msgs[0].clone();
            if let Ok(written) = first.to_kvn(opts) {
                let again = Omm::from_kvn(&written, mode).expect("re-parse omm kvn");
                assert_eq!(again, first);
                assert_eq!(again.to_kvn(opts).unwrap(), written);
            }
        }
        drop(Omm::from_kvn(text, mode));
        drop(Opm::from_kvn(text, mode));
    }
}

fn check_oem_kvn(text: &str) {
    let opts = WriteOptions::default();
    for mode in MODES {
        if let Ok(msg) = Oem::from_kvn(text, mode)
            && let Ok(written) = msg.to_kvn(opts)
        {
            let again = Oem::from_kvn(&written, mode).expect("re-parse oem kvn");
            assert_eq!(again, msg);
            assert_eq!(again.to_kvn(opts).unwrap(), written);
        }
    }
}

fn check_xml(text: &str) {
    let opts = WriteOptions::default();
    for mode in MODES {
        if let Ok(ndm) = Ndm::from_xml(text, mode)
            && let Ok(written) = ndm.to_xml(opts)
        {
            let again = Ndm::from_xml(&written, mode).expect("re-parse ndm xml");
            assert_eq!(again, ndm);
            assert_eq!(again.to_xml(opts).unwrap(), written);
        }
        if let Ok(opm) = Opm::from_xml(text, mode)
            && let Ok(written) = opm.to_xml(opts)
        {
            let again = Opm::from_xml(&written, mode).expect("re-parse opm xml");
            assert_eq!(again, opm);
            assert_eq!(again.to_xml(opts).unwrap(), written);
        }
        if let Ok(omms) = Omm::from_xml_all(text, mode)
            && omms.len() == 1
            && let Ok(written) = omms[0].to_xml(opts)
        {
            let again = Omm::from_xml(&written, mode).expect("re-parse omm xml");
            assert_eq!(again, omms[0]);
            assert_eq!(again.to_xml(opts).unwrap(), written);
        }
        if let Ok(oem) = Oem::from_xml(text, mode)
            && let Ok(written) = oem.to_xml(opts)
        {
            let again = Oem::from_xml(&written, mode).expect("re-parse oem xml");
            assert_eq!(again, oem);
            assert_eq!(again.to_xml(opts).unwrap(), written);
        }
    }
}

#[cfg(feature = "json")]
fn check_json(text: &str) {
    let opts = WriteOptions::default();
    for mode in MODES {
        if let Ok(omms) = Omm::from_json_all(text, mode)
            && let Ok(written) = Omm::to_json_all(&omms, opts)
        {
            let again = Omm::from_json_all(&written, mode).expect("re-parse omm json");
            assert_eq!(again, omms);
            assert_eq!(Omm::to_json_all(&again, opts).unwrap(), written);
        }
        drop(Omm::from_json(text, mode));
    }
}

fn sweep_kind(fixtures: &[&str], seed: u64, cases_per_fixture: usize, check: fn(&str)) {
    let mut rng = SplitMix64(seed);
    for fixture in fixtures {
        check(fixture);
        for step in 0..cases_per_fixture {
            let mutated = mutate_text(fixture, &mut rng, step);
            check(&mutated);
        }
    }
}

#[test]
fn kvn_fixtures_survive_deterministic_mutations() {
    sweep_kind(
        &[
            include_str!("../fixtures/bluebook/opm_g01.kvn"),
            include_str!("../fixtures/bluebook/opm_g02.kvn"),
            include_str!("../fixtures/bluebook/opm_g03.kvn"),
            include_str!("../fixtures/bluebook/opm_g04.kvn"),
        ],
        0x51a0_0001,
        64,
        check_opm_kvn,
    );
    sweep_kind(
        &[
            include_str!("../fixtures/bluebook/omm_g07.kvn"),
            include_str!("../fixtures/bluebook/omm_g08.kvn"),
            include_str!("../fixtures/bluebook/omm_g09.kvn"),
            include_str!("../fixtures/celestrak/iss_25544.kvn"),
            include_str!("../fixtures/celestrak/intdes_2010-005.kvn"),
        ],
        0x51a0_0002,
        64,
        check_omm_kvn,
    );
    sweep_kind(
        &[
            include_str!("../fixtures/bluebook/oem_g11.kvn"),
            include_str!("../fixtures/bluebook/oem_g12.kvn"),
            include_str!("../fixtures/bluebook/oem_g13.kvn"),
        ],
        0x51a0_0003,
        64,
        check_oem_kvn,
    );
}

#[test]
fn xml_and_pathological_nesting_survive_deterministic_mutations() {
    sweep_kind(
        &[
            include_str!("../fixtures/bluebook/opm_g05.xml"),
            include_str!("../fixtures/bluebook/omm_g10.xml"),
            include_str!("../fixtures/bluebook/oem_g14.xml"),
            include_str!("../fixtures/bluebook/ndm_g21.xml"),
            include_str!("../fixtures/celestrak/iss_25544.xml"),
            include_str!("../fixtures/celestrak/intdes_2010-005.xml"),
        ],
        0x51a0_0004,
        64,
        check_xml,
    );
    let deep = format!("{}<leaf/>{}", "<a>".repeat(256), "</a>".repeat(256));
    check_xml(&deep);
    drop(Omm::from_kvn(
        &"COMMENT loop\n".repeat(512),
        ParseMode::Lenient,
    ));
}

#[cfg(feature = "json")]
#[test]
fn json_fixtures_survive_deterministic_mutations() {
    sweep_kind(
        &[
            include_str!("../fixtures/celestrak/iss_25544.json"),
            include_str!("../fixtures/celestrak/intdes_2010-005.json"),
            include_str!("../fixtures/spacetrack/iss_25544_gp.json"),
        ],
        0x51a0_0005,
        80,
        check_json,
    );
}
