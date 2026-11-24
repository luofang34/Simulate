//! NDM combined instantiation.
//!
//! Fixture source: `bluebook/ndm_g21.xml` is figure G-21 of CCSDS
//! 502.0-B-3, extracted as described in `opm.rs`; the page also wraps each
//! `TLE_LINE2` value onto a second line, which was rejoined. Figure G-22
//! holds an OCM, which this crate does not read yet.

use ccsds_ndm::{Ndm, NdmMessage, Omm, ParseMode, WriteOptions};

use super::content;

const G21: &str = include_str!("../fixtures/bluebook/ndm_g21.xml");

#[test]
fn g21_holds_three_omms_and_round_trips() {
    let ndm = Ndm::from_xml(G21, ParseMode::Strict).unwrap();
    assert_eq!(ndm.messages.len(), 3);
    let NdmMessage::Omm(first) = &ndm.messages[0] else {
        panic!("{:?}", ndm.messages[0]);
    };
    let user = first.data.user_defined.as_ref().unwrap();
    assert_eq!(user.parameters.len(), 3);
    assert!(user.parameters[2].value.ends_with("15.05566242001761"));
    let written = ndm.to_xml(WriteOptions::default()).unwrap();
    let again = Ndm::from_xml(&written, ParseMode::Strict).unwrap();
    assert_eq!(again, ndm);
    assert_eq!(again.to_xml(WriteOptions::default()).unwrap(), written);
    assert_eq!(content::xml(&written), content::xml(G21));
    let omms = Omm::from_xml_all(G21, ParseMode::Strict).unwrap();
    assert_eq!(omms.len(), 3);
    assert_eq!(NdmMessage::Omm(omms[2].clone()), ndm.messages[2]);
}

#[test]
fn unsupported_messages_are_named() {
    let text = "<ndm><ocm id=\"CCSDS_OCM_VERS\" version=\"3.0\"/></ndm>";
    let error = Ndm::from_xml(text, ParseMode::Lenient).err().unwrap();
    assert!(error.to_string().contains("<ocm>"), "{error}");
}
