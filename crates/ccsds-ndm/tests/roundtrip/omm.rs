//! OMM: the annex G examples, real CelesTrak downloads, and a Space-Track
//! layout sample.
//!
//! Fixture sources:
//! - `bluebook/omm_g07.kvn` … `omm_g10.xml`: figures G-7 to G-10 of
//!   CCSDS 502.0-B-3, extracted as described in `opm.rs`.
//! - `celestrak/iss_25544.{kvn,xml,json}`: CelesTrak GP API,
//!   `gp.php?CATNR=25544&FORMAT=KVN|XML|JSON`, fetched 2026-11-24.
//! - `celestrak/intdes_2010-005.{kvn,xml,json}`: CelesTrak GP API,
//!   `gp.php?INTDES=2010-005&FORMAT=…`, fetched 2026-11-24 (two objects; the
//!   KVN has CRLF line ends, as served).
//! - `spacetrack/iss_25544_gp.json`: not a download (Space-Track needs an
//!   account). Written for this test in the shape of Space-Track's `gp`
//!   class: every value a string, plus its own members; the element values
//!   are the CelesTrak ISS set above.

#[cfg(feature = "json")]
use ccsds_ndm::ExtensionValue;
use ccsds_ndm::{
    CenterName, ErrorKind, MeanElementTheory, Omm, OrbitSize, ParseMode, Quantity, RefFrame,
    TleDrag, WriteOptions,
};

use super::content;

const KVN: [(&str, &str); 3] = [
    ("G-7", include_str!("../fixtures/bluebook/omm_g07.kvn")),
    ("G-8", include_str!("../fixtures/bluebook/omm_g08.kvn")),
    ("G-9", include_str!("../fixtures/bluebook/omm_g09.kvn")),
];
const XML_G10: &str = include_str!("../fixtures/bluebook/omm_g10.xml");
const ISS_KVN: &str = include_str!("../fixtures/celestrak/iss_25544.kvn");
const ISS_XML: &str = include_str!("../fixtures/celestrak/iss_25544.xml");
#[cfg(feature = "json")]
const ISS_JSON: &str = include_str!("../fixtures/celestrak/iss_25544.json");
const PAIR_KVN: &str = include_str!("../fixtures/celestrak/intdes_2010-005.kvn");
const PAIR_XML: &str = include_str!("../fixtures/celestrak/intdes_2010-005.xml");
#[cfg(feature = "json")]
const PAIR_JSON: &str = include_str!("../fixtures/celestrak/intdes_2010-005.json");
#[cfg(feature = "json")]
const SPACE_TRACK: &str = include_str!("../fixtures/spacetrack/iss_25544_gp.json");

fn options() -> [WriteOptions; 2] {
    [
        WriteOptions::default(),
        WriteOptions::default().with_units(true),
    ]
}

/// Every notation of `omm` reads back to `omm`, and each is a fixed point.
fn assert_round_trips(omm: &Omm, mode: ParseMode, label: &str) {
    for options in options() {
        let kvn = omm.to_kvn(options).unwrap();
        let again = Omm::from_kvn(&kvn, mode).unwrap();
        assert_eq!(&again, omm, "{label} via KVN");
        assert_eq!(
            again.to_kvn(options).unwrap(),
            kvn,
            "{label} KVN not stable"
        );
        let xml = omm.to_xml(options).unwrap();
        let again = Omm::from_xml(&xml, mode).unwrap();
        assert_eq!(&again, omm, "{label} via XML");
        assert_eq!(
            again.to_xml(options).unwrap(),
            xml,
            "{label} XML not stable"
        );
    }
}

#[test]
fn bluebook_kvn_examples_round_trip() {
    for (figure, text) in KVN {
        let omm =
            Omm::from_kvn(text, ParseMode::Strict).unwrap_or_else(|e| panic!("{figure}: {e}"));
        assert_round_trips(&omm, ParseMode::Strict, figure);
        let written = omm.to_kvn(WriteOptions::default()).unwrap();
        assert_eq!(
            content::kvn(&written),
            content::kvn(text),
            "{figure} content"
        );
    }
}

#[test]
fn bluebook_xml_example_round_trips() {
    let omm = Omm::from_xml(XML_G10, ParseMode::Strict).unwrap();
    assert_round_trips(&omm, ParseMode::Strict, "G-10");
    let written = omm.to_xml(WriteOptions::default()).unwrap();
    assert_eq!(content::xml(&written), content::xml(XML_G10));
    assert_eq!(omm.header.classification.as_deref(), Some("CUI"));
    assert_eq!(omm.header.message_id.as_deref(), Some("OMM 202013719185"));
}

#[test]
fn g9_units_and_user_defined() {
    let omm = Omm::from_kvn(KVN[2].1, ParseMode::Strict).unwrap();
    let tle = omm.data.tle.as_ref().unwrap();
    assert_eq!(tle.element_set_no, Some(925));
    assert_eq!(tle.drag, Some(TleDrag::Bstar(Quantity::new(0.0001))));
    assert_eq!(
        omm.data.mean_elements.size,
        OrbitSize::MeanMotion(Quantity::new(1.00273272))
    );
    let user = omm.data.user_defined.as_ref().unwrap();
    assert_eq!(user.parameters[0].name, "EARTH_MODEL");
    let with_units = omm
        .to_kvn(WriteOptions::default().with_units(true))
        .unwrap();
    assert!(with_units.contains("[rev/day**2]"), "{with_units}");
    assert!(with_units.contains("[1/ER]"));
}

#[test]
fn celestrak_samples_need_lenient_mode() {
    let error = Omm::from_kvn(ISS_KVN, ParseMode::Strict).err().unwrap();
    assert!(matches!(error.kind(), ErrorKind::EmptyValue), "{error}");
    assert_eq!(error.keyword(), Some("CREATION_DATE"));
    assert_eq!(error.location().map(|l| l.line), Some(2));
    let error = Omm::from_xml(ISS_XML, ParseMode::Strict).err().unwrap();
    assert!(
        matches!(error.kind(), ErrorKind::WrongMessage { .. }),
        "{error}"
    );
    #[cfg(feature = "json")]
    {
        let error = Omm::from_json(ISS_JSON, ParseMode::Strict).err().unwrap();
        assert!(matches!(error.kind(), ErrorKind::MissingKeyword), "{error}");
    }
}

#[test]
fn celestrak_notations_agree() {
    let kvn = Omm::from_kvn_all(ISS_KVN, ParseMode::Lenient).unwrap();
    let xml = Omm::from_xml_all(ISS_XML, ParseMode::Lenient).unwrap();
    assert_eq!((kvn.len(), xml.len()), (1, 1));
    let (kvn, xml) = (&kvn[0], &xml[0]);
    assert_eq!(kvn.header.creation_date, None);
    assert_eq!(kvn.metadata.center_name, Some(CenterName::Earth));
    assert_eq!(kvn.metadata.ref_frame, Some(RefFrame::Teme));
    // The theory is spelled differently per notation by the producer.
    assert_eq!(
        kvn.metadata.mean_element_theory,
        Some(MeanElementTheory::SgpSgp4)
    );
    assert_eq!(
        xml.metadata.mean_element_theory,
        Some(MeanElementTheory::Sgp4)
    );
    assert_eq!(kvn.data, xml.data);
    #[cfg(feature = "json")]
    {
        let json = Omm::from_json_all(ISS_JSON, ParseMode::Lenient).unwrap();
        assert_eq!(json.len(), 1);
        let json = &json[0];
        assert_eq!(json.metadata.mean_element_theory, None);
        assert_eq!(kvn.data, json.data);
        assert_eq!(kvn.metadata.object_name, json.metadata.object_name);
        assert_eq!(xml.metadata.object_id, json.metadata.object_id);
    }
    for (label, omm) in [("KVN", kvn), ("XML", xml)] {
        assert_round_trips(omm, ParseMode::Lenient, label);
    }
}

#[test]
fn celestrak_groups_read_every_message() {
    let kvn = Omm::from_kvn_all(PAIR_KVN, ParseMode::Lenient).unwrap();
    let xml = Omm::from_xml_all(PAIR_XML, ParseMode::Lenient).unwrap();
    assert_eq!(kvn.len(), 2);
    assert_eq!(xml.len(), 2);
    for (k, x) in kvn.iter().zip(&xml) {
        assert_eq!(k.data, x.data);
    }
    assert_eq!(kvn[1].metadata.object_name, "ATLAS 5 CENTAUR R/B");
    #[cfg(feature = "json")]
    {
        let json = Omm::from_json_all(PAIR_JSON, ParseMode::Lenient).unwrap();
        assert_eq!(json.len(), 2);
        for (k, j) in kvn.iter().zip(&json) {
            assert_eq!(k.data, j.data);
        }
        let written = Omm::to_json_all(&json, WriteOptions::default()).unwrap();
        assert_eq!(written, PAIR_JSON.trim_end());
    }
}

#[cfg(feature = "json")]
#[test]
fn json_round_trip_is_bit_exact() {
    let all = Omm::from_json_all(PAIR_JSON, ParseMode::Lenient).unwrap();
    let written = Omm::to_json_all(&all, WriteOptions::default()).unwrap();
    let again = Omm::from_json_all(&written, ParseMode::Lenient).unwrap();
    assert_eq!(again, all);
    let bits = |omm: &Omm| omm.data.mean_elements.eccentricity.to_bits();
    assert_eq!(bits(&again[1]), bits(&all[1]));
}

#[cfg(feature = "json")]
#[test]
fn space_track_layout_keeps_its_extra_members() {
    let error = Omm::from_json(SPACE_TRACK, ParseMode::Strict)
        .err()
        .unwrap();
    assert!(
        matches!(error.kind(), ErrorKind::JsonType { .. }),
        "{error}"
    );
    let omm = Omm::from_json(SPACE_TRACK, ParseMode::Lenient).unwrap();
    assert_eq!(omm.header.comments, ["GENERATED VIA SPACE-TRACK.ORG API"]);
    assert_eq!(omm.header.originator.as_deref(), Some("18 SPCS"));
    let celestrak = Omm::from_json(ISS_JSON, ParseMode::Lenient).unwrap();
    assert_eq!(omm.data, celestrak.data);
    let keyword = |name: &str| omm.extensions.iter().find(|e| e.keyword == name);
    assert_eq!(keyword("DECAY_DATE").unwrap().value, ExtensionValue::Null);
    assert_eq!(
        keyword("GP_ID").unwrap().value,
        ExtensionValue::Text("291234567".to_owned())
    );
    assert_eq!(omm.extensions.len(), 15);
    let written = omm.to_json(WriteOptions::default()).unwrap();
    assert_eq!(Omm::from_json(&written, ParseMode::Lenient).unwrap(), omm);
    let kvn = omm.to_kvn(WriteOptions::default()).unwrap();
    let from_kvn = Omm::from_kvn(&kvn, ParseMode::Lenient).unwrap();
    assert_eq!(from_kvn.data, omm.data);
    assert_eq!(from_kvn.extensions.len(), omm.extensions.len());
}
