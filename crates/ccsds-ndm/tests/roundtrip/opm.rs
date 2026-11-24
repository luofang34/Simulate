//! ODM 502.0-B-3 annex G OPM examples.
//!
//! Fixture sources: `tests/fixtures/bluebook/opm_g0N.*` is figure G-N of
//! CCSDS 502.0-B-3 (April 2023, https://public.ccsds.org/Pubs/502x0b3e1.pdf),
//! extracted with `pdftotext -layout`; page headers and footers removed, and
//! the schema URL the page wraps rejoined. The extraction matches the
//! independent transcription in the ccsds-ndm repository
//! (`ccsds-ndm/data/kvn/opm_g*.kvn`) keyword for keyword.

use ccsds_ndm::{Anomaly, CenterName, ParseMode, RefFrame, TimeSystem, WriteOptions};
use ccsds_ndm::{Opm, Quantity};

use super::content;

const KVN: [(&str, &str); 4] = [
    ("G-1", include_str!("../fixtures/bluebook/opm_g01.kvn")),
    ("G-2", include_str!("../fixtures/bluebook/opm_g02.kvn")),
    ("G-3", include_str!("../fixtures/bluebook/opm_g03.kvn")),
    ("G-4", include_str!("../fixtures/bluebook/opm_g04.kvn")),
];
const XML_G5: &str = include_str!("../fixtures/bluebook/opm_g05.xml");

fn options() -> [WriteOptions; 2] {
    [
        WriteOptions::default(),
        WriteOptions::default().with_units(true),
    ]
}

#[test]
fn kvn_examples_round_trip() {
    for (figure, text) in KVN {
        let opm =
            Opm::from_kvn(text, ParseMode::Strict).unwrap_or_else(|e| panic!("{figure}: {e}"));
        for options in options() {
            let written = opm.to_kvn(options).unwrap();
            let again = Opm::from_kvn(&written, ParseMode::Strict).unwrap();
            assert_eq!(again, opm, "{figure}");
            assert_eq!(
                again.to_kvn(options).unwrap(),
                written,
                "{figure} not byte-stable"
            );
            assert_eq!(
                content::kvn(&written),
                content::kvn(text),
                "{figure} content"
            );
            let xml = opm.to_xml(options).unwrap();
            assert_eq!(
                Opm::from_xml(&xml, ParseMode::Strict).unwrap(),
                opm,
                "{figure} via XML"
            );
        }
    }
}

#[test]
fn xml_example_round_trips() {
    let opm = Opm::from_xml(XML_G5, ParseMode::Strict).unwrap();
    for options in options() {
        let written = opm.to_xml(options).unwrap();
        let again = Opm::from_xml(&written, ParseMode::Strict).unwrap();
        assert_eq!(again, opm);
        assert_eq!(again.to_xml(options).unwrap(), written);
        assert_eq!(content::xml(&written), content::xml(XML_G5));
        let kvn = opm.to_kvn(options).unwrap();
        assert_eq!(Opm::from_kvn(&kvn, ParseMode::Strict).unwrap(), opm);
    }
}

#[test]
fn g2_fields_are_typed() {
    let opm = Opm::from_kvn(KVN[1].1, ParseMode::Strict).unwrap();
    assert_eq!(opm.header.comments.len(), 2);
    assert_eq!(opm.metadata.ref_frame, RefFrame::Tod);
    assert_eq!(opm.metadata.center_name, CenterName::Earth);
    assert_eq!(opm.metadata.time_system, TimeSystem::Utc);
    assert_eq!(opm.data.state.comments, ["State Vector"]);
    assert_eq!(opm.data.state.position[1], Quantity::new(-40218.5751));
    let kepler = opm.data.keplerian.as_ref().unwrap();
    assert_eq!(kepler.anomaly, Anomaly::True(Quantity::new(41.922339)));
    assert_eq!(kepler.gm.value(), 398600.4415);
    assert_eq!(opm.data.maneuvers.len(), 2);
    let second = &opm.data.maneuvers[1];
    assert_eq!(second.ref_frame, RefFrame::Rtn);
    assert_eq!(second.comments.len(), 2);
    assert_eq!(opm.data.maneuvers[0].comments.len(), 3);
    assert_eq!(second.delta_v[1].value(), -0.001873);
}

#[test]
fn g4_user_defined_and_covariance() {
    let opm = Opm::from_kvn(KVN[3].1, ParseMode::Strict).unwrap();
    let user = opm.data.user_defined.as_ref().unwrap();
    assert_eq!(user.parameters[0].name, "EARTH_MODEL");
    assert_eq!(user.parameters[0].value, "WGS-84");
    let covariance = opm.data.covariance.as_ref().unwrap();
    assert_eq!(covariance.ref_frame, Some(RefFrame::Rtn));
    assert_eq!(covariance.matrix.get(5, 5), Some(6.2244443386355e-10));
}
