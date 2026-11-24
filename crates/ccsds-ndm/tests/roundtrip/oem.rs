//! OEM: the annex G examples.
//!
//! Fixture sources: `bluebook/oem_g11.kvn` … `oem_g14.xml` are figures G-11
//! to G-14 of CCSDS 502.0-B-3, extracted as described in `opm.rs`. The
//! figures elide ephemeris lines with a `< intervening data records omitted
//! here >` placeholder line, which is not KVN and was removed.

use ccsds_ndm::{Interpolation, Oem, ParseMode, RefFrame, WriteOptions};

use super::content;

const KVN: [(&str, &str); 3] = [
    ("G-11", include_str!("../fixtures/bluebook/oem_g11.kvn")),
    ("G-12", include_str!("../fixtures/bluebook/oem_g12.kvn")),
    ("G-13", include_str!("../fixtures/bluebook/oem_g13.kvn")),
];
const XML_G14: &str = include_str!("../fixtures/bluebook/oem_g14.xml");

fn assert_round_trips(oem: &Oem, label: &str) {
    for options in [
        WriteOptions::default(),
        WriteOptions::default().with_units(true),
    ] {
        let kvn = oem.to_kvn(options).unwrap();
        let again = Oem::from_kvn(&kvn, ParseMode::Strict).unwrap();
        assert_eq!(&again, oem, "{label} via KVN");
        assert_eq!(
            again.to_kvn(options).unwrap(),
            kvn,
            "{label} KVN not stable"
        );
        let xml = oem.to_xml(options).unwrap();
        let again = Oem::from_xml(&xml, ParseMode::Strict).unwrap();
        assert_eq!(&again, oem, "{label} via XML");
        assert_eq!(
            again.to_xml(options).unwrap(),
            xml,
            "{label} XML not stable"
        );
    }
}

#[test]
fn kvn_examples_round_trip() {
    for (figure, text) in KVN {
        let oem =
            Oem::from_kvn(text, ParseMode::Strict).unwrap_or_else(|e| panic!("{figure}: {e}"));
        assert_round_trips(&oem, figure);
        let written = oem.to_kvn(WriteOptions::default()).unwrap();
        assert_eq!(
            content::kvn(&written),
            content::kvn(text),
            "{figure} content"
        );
    }
}

#[test]
fn xml_example_round_trips() {
    let oem = Oem::from_xml(XML_G14, ParseMode::Strict).unwrap();
    assert_round_trips(&oem, "G-14");
    let written = oem.to_xml(WriteOptions::default()).unwrap();
    assert_eq!(content::xml(&written), content::xml(XML_G14));
}

#[test]
fn g11_has_two_segments_of_states() {
    let oem = Oem::from_kvn(KVN[0].1, ParseMode::Strict).unwrap();
    assert_eq!(oem.segments.len(), 2);
    let first = &oem.segments[0];
    assert_eq!(first.metadata.interpolation, Some(Interpolation::Hermite));
    assert_eq!(first.metadata.interpolation_degree, Some(7));
    assert_eq!(first.data.comments.len(), 2);
    assert_eq!(first.data.states.len(), 4);
    assert_eq!(first.data.states[0].position[0].value(), 2789.619);
    assert!(first.data.states[0].acceleration.is_none());
    let second = &oem.segments[1];
    assert_eq!(second.data.states[0].position[1].value(), -63.042);
    assert_eq!(
        second
            .metadata
            .useable_start_time
            .as_ref()
            .unwrap()
            .to_string(),
        "2019-12-28T22:08:02.5"
    );
}

#[test]
fn g12_accelerations_and_g13_covariances() {
    let g12 = Oem::from_kvn(KVN[1].1, ParseMode::Strict).unwrap();
    let state = &g12.segments[0].data.states[0];
    assert_eq!(state.acceleration.unwrap()[2].value(), -0.159);
    let g13 = Oem::from_kvn(KVN[2].1, ParseMode::Strict).unwrap();
    let covariances = &g13.segments[0].data.covariances;
    assert_eq!(covariances.len(), 2);
    assert_eq!(covariances[0].ref_frame, Some(RefFrame::Eme2000));
    assert_eq!(covariances[1].matrix.get(5, 5), Some(6.2244443e-10));
    assert_eq!(covariances[1].epoch.to_string(), "2019-12-29T21:00:00");
}
