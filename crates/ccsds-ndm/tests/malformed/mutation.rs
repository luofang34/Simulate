//! Deterministic mutation sweep across KVN, XML and JSON fixtures:
//! never panics, and every accepted mutation round-trips with byte-stable writes.

use ccsds_ndm::{Aem, Apm, Cdm, Ndm, Oem, Omm, Opm, ParseMode, Tdm, WriteOptions};

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

fn check_xml_variant<T: PartialEq + std::fmt::Debug>(
    parse: fn(&str, ParseMode) -> ccsds_ndm::Result<T>,
    write: fn(&T, WriteOptions) -> ccsds_ndm::Result<String>,
    text: &str,
    mode: ParseMode,
) {
    let opts = WriteOptions::default();
    if let Ok(msg) = parse(text, mode)
        && let Ok(written) = write(&msg, opts)
    {
        let again = parse(&written, mode).expect("re-parse xml");
        assert_eq!(again, msg);
        assert_eq!(write(&again, opts).unwrap(), written);
    }
}

fn check_xml(text: &str) {
    let opts = WriteOptions::default();
    for mode in MODES {
        check_xml_variant(Ndm::from_xml, Ndm::to_xml, text, mode);
        check_xml_variant(Opm::from_xml, Opm::to_xml, text, mode);
        check_xml_variant(Oem::from_xml, Oem::to_xml, text, mode);
        check_xml_variant(Apm::from_xml, Apm::to_xml, text, mode);
        check_xml_variant(Aem::from_xml, Aem::to_xml, text, mode);
        check_xml_variant(Cdm::from_xml, Cdm::to_xml, text, mode);
        check_xml_variant(Tdm::from_xml, Tdm::to_xml, text, mode);
        if let Ok(omms) = Omm::from_xml_all(text, mode)
            && omms.len() == 1
            && let Ok(written) = omms[0].to_xml(opts)
        {
            let again = Omm::from_xml(&written, mode).expect("re-parse omm xml");
            assert_eq!(again, omms[0]);
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

fn check_kvn_roundtrip<T: PartialEq + std::fmt::Debug>(
    parse: fn(&str, ParseMode) -> ccsds_ndm::Result<T>,
    write: fn(&T, WriteOptions) -> ccsds_ndm::Result<String>,
    text: &str,
) {
    let opts = WriteOptions::default();
    for mode in MODES {
        if let Ok(msg) = parse(text, mode)
            && let Ok(written) = write(&msg, opts)
        {
            let again = parse(&written, mode).expect("re-parse kvn");
            assert_eq!(again, msg);
            assert_eq!(write(&again, opts).unwrap(), written);
        }
    }
}

#[test]
fn apm_aem_cdm_tdm_survive_deterministic_mutations() {
    let apm_kvn = "CCSDS_APM_VERS = 2.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = SIM\nOBJECT_NAME = SAT\nOBJECT_ID = 2026-001A\nTIME_SYSTEM = UTC\nEPOCH = 2026-01-01T00:00:00\nREF_FRAME_A = GCRF\nREF_FRAME_B = SC_BODY_1\nQ1 = 0\nQ2 = 0\nQ3 = 0\nQC = 1\nMAN_EPOCH_START = 2026-01-01T00:01:00\nMAN_DURATION = 5 [s]\nMAN_REF_FRAME = SC_BODY_1\nMAN_TOR_1 = 0.1 [N*m]\nMAN_TOR_2 = 0 [N*m]\nMAN_TOR_3 = 0 [N*m]\n";
    let aem_kvn = "CCSDS_AEM_VERS = 2.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = SIM\nMETA_START\nOBJECT_NAME = SAT\nOBJECT_ID = 2026-001A\nREF_FRAME_A = GCRF\nREF_FRAME_B = SC_BODY_1\nTIME_SYSTEM = UTC\nSTART_TIME = 2026-01-01T00:00:00\nSTOP_TIME = 2026-01-01T00:02:00\nATTITUDE_TYPE = QUATERNION/ANGVEL\nINTERPOLATION_METHOD = HERMITE\nINTERPOLATION_DEGREE = 3\nMETA_STOP\nDATA_START\n2026-01-01T00:00:00 0 0 0 1 0.01 0 0\nDATA_STOP\n";
    let cdm_kvn = "CCSDS_CDM_VERS = 1.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = SIM\nMESSAGE_ID = M1\nTCA = 2026-01-02T00:00:00\nMISS_DISTANCE = 150 [m]\nOBJECT = OBJECT1\nOBJECT_DESIGNATOR = 1\nCATALOG_NAME = SATCAT\nOBJECT_NAME = A\nINTERNATIONAL_DESIGNATOR = 2026-001A\nEPHEMERIS_NAME = NONE\nCOVARIANCE_METHOD = CALCULATED\nMANEUVERABLE = YES\nREF_FRAME = EME2000\nX = 7000\nY = 0\nZ = 0\nX_DOT = 0\nY_DOT = 7.5\nZ_DOT = 0\nCR_R = 10\nCT_R = 0\nCT_T = 10\nCN_R = 0\nCN_T = 0\nCN_N = 10\nCRDOT_R = 0\nCRDOT_T = 0\nCRDOT_N = 0\nCRDOT_RDOT = 0.01\nCTDOT_R = 0\nCTDOT_T = 0\nCTDOT_N = 0\nCTDOT_RDOT = 0\nCTDOT_TDOT = 0.01\nCNDOT_R = 0\nCNDOT_T = 0\nCNDOT_N = 0\nCNDOT_RDOT = 0\nCNDOT_TDOT = 0\nCNDOT_NDOT = 0.01\nOBJECT = OBJECT2\nOBJECT_DESIGNATOR = 2\nCATALOG_NAME = SATCAT\nOBJECT_NAME = B\nINTERNATIONAL_DESIGNATOR = 2026-002A\nEPHEMERIS_NAME = NONE\nCOVARIANCE_METHOD = CALCULATED\nMANEUVERABLE = NO\nREF_FRAME = EME2000\nX = 7000.15\nY = 0\nZ = 0\nX_DOT = 0\nY_DOT = -7.5\nZ_DOT = 0\nCR_R = 20\nCT_R = 0\nCT_T = 20\nCN_R = 0\nCN_T = 0\nCN_N = 20\nCRDOT_R = 0\nCRDOT_T = 0\nCRDOT_N = 0\nCRDOT_RDOT = 0.02\nCTDOT_R = 0\nCTDOT_T = 0\nCTDOT_N = 0\nCTDOT_RDOT = 0\nCTDOT_TDOT = 0.02\nCNDOT_R = 0\nCNDOT_T = 0\nCNDOT_N = 0\nCNDOT_RDOT = 0\nCNDOT_TDOT = 0\nCNDOT_NDOT = 0.02\n";
    let tdm_kvn = "CCSDS_TDM_VERS = 2.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = SIM\nMETA_START\nTIME_SYSTEM = UTC\nPARTICIPANT_1 = DSS-14\nPARTICIPANT_2 = SAT\nMODE = SEQUENTIAL\nPATH = 1,2,1\nRANGE_MODE = COHERENT\nRANGE_UNITS = km\nMETA_STOP\nDATA_START\nRANGE = 2026-01-01T00:00:00 384400\nDOPPLER_INTEGRATED = 2026-01-01T00:00:00 -0.5\nANGLE_1 = 2026-01-01T00:00:00 120.5\nANGLE_2 = 2026-01-01T00:00:00 45.25\nDATA_STOP\n";
    sweep_kind(&[apm_kvn], 0x51a0_0006, 64, |t| {
        check_kvn_roundtrip(Apm::from_kvn, Apm::to_kvn, t)
    });
    sweep_kind(&[aem_kvn], 0x51a0_0007, 64, |t| {
        check_kvn_roundtrip(Aem::from_kvn, Aem::to_kvn, t)
    });
    sweep_kind(&[cdm_kvn], 0x51a0_0008, 64, |t| {
        check_kvn_roundtrip(Cdm::from_kvn, Cdm::to_kvn, t)
    });
    sweep_kind(&[tdm_kvn], 0x51a0_0009, 64, |t| {
        check_kvn_roundtrip(Tdm::from_kvn, Tdm::to_kvn, t)
    });

    let apm_xml = Apm::from_kvn(apm_kvn, ParseMode::Strict)
        .unwrap()
        .to_xml(WriteOptions::default())
        .unwrap();
    let aem_xml = Aem::from_kvn(aem_kvn, ParseMode::Strict)
        .unwrap()
        .to_xml(WriteOptions::default())
        .unwrap();
    let cdm_xml = Cdm::from_kvn(cdm_kvn, ParseMode::Strict)
        .unwrap()
        .to_xml(WriteOptions::default())
        .unwrap();
    let tdm_xml = Tdm::from_kvn(tdm_kvn, ParseMode::Strict)
        .unwrap()
        .to_xml(WriteOptions::default())
        .unwrap();
    sweep_kind(
        &[&apm_xml, &aem_xml, &cdm_xml, &tdm_xml],
        0x51a0_000a,
        64,
        check_xml,
    );
}
