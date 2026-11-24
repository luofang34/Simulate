//! NDM combined instantiation.
//!
//! Fixture source: `bluebook/ndm_g21.xml` is figure G-21 of CCSDS
//! 502.0-B-3, extracted as described in `opm.rs`; the page also wraps each
//! `TLE_LINE2` value onto a second line, which was rejoined. Figure G-22
//! holds an OCM, which this crate does not read yet.

use ccsds_ndm::{Aem, Apm, Cdm, Ndm, NdmMessage, Oem, Omm, Opm, ParseMode, Tdm, WriteOptions};

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

#[test]
fn all_seven_message_types_round_trip_in_ndm() {
    let opm = Opm::from_kvn(
        include_str!("../fixtures/bluebook/opm_g01.kvn"),
        ParseMode::Strict,
    )
    .unwrap();
    let omm = Omm::from_kvn(
        include_str!("../fixtures/bluebook/omm_g07.kvn"),
        ParseMode::Strict,
    )
    .unwrap();
    let oem = Oem::from_kvn(
        include_str!("../fixtures/bluebook/oem_g11.kvn"),
        ParseMode::Strict,
    )
    .unwrap();
    let apm = Apm::from_kvn(
        "CCSDS_APM_VERS = 2.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = TEST\nOBJECT_NAME = SAT\nOBJECT_ID = 2026-001A\nTIME_SYSTEM = UTC\nEPOCH = 2026-01-01T00:00:00\nREF_FRAME_A = GCRF\nREF_FRAME_B = SC_BODY_1\nQ1 = 0\nQ2 = 0\nQ3 = 0\nQC = 1\n",
        ParseMode::Strict,
    )
    .unwrap();
    let aem = Aem::from_kvn(
        "CCSDS_AEM_VERS = 2.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = TEST\nMETA_START\nOBJECT_NAME = SAT\nOBJECT_ID = 2026-001A\nREF_FRAME_A = GCRF\nREF_FRAME_B = SC_BODY_1\nTIME_SYSTEM = UTC\nSTART_TIME = 2026-01-01T00:00:00\nSTOP_TIME = 2026-01-01T00:01:00\nATTITUDE_TYPE = QUATERNION\nMETA_STOP\nDATA_START\n2026-01-01T00:00:00 0 0 0 1\nDATA_STOP\n",
        ParseMode::Strict,
    )
    .unwrap();
    let cdm = Cdm::from_kvn(
        "CCSDS_CDM_VERS = 1.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = TEST\nMESSAGE_ID = CDM-1\nTCA = 2026-01-02T00:00:00\nMISS_DISTANCE = 100\nOBJECT = OBJECT1\nOBJECT_DESIGNATOR = 1\nCATALOG_NAME = SATCAT\nOBJECT_NAME = A\nINTERNATIONAL_DESIGNATOR = 2026-001A\nEPHEMERIS_NAME = NONE\nCOVARIANCE_METHOD = CALCULATED\nMANEUVERABLE = YES\nREF_FRAME = EME2000\nX = 7000\nY = 0\nZ = 0\nX_DOT = 0\nY_DOT = 7.5\nZ_DOT = 0\nCR_R = 1\nCT_R = 0\nCT_T = 1\nCN_R = 0\nCN_T = 0\nCN_N = 1\nCRDOT_R = 0\nCRDOT_T = 0\nCRDOT_N = 0\nCRDOT_RDOT = 0.001\nCTDOT_R = 0\nCTDOT_T = 0\nCTDOT_N = 0\nCTDOT_RDOT = 0\nCTDOT_TDOT = 0.001\nCNDOT_R = 0\nCNDOT_T = 0\nCNDOT_N = 0\nCNDOT_RDOT = 0\nCNDOT_TDOT = 0\nCNDOT_NDOT = 0.001\nOBJECT = OBJECT2\nOBJECT_DESIGNATOR = 2\nCATALOG_NAME = SATCAT\nOBJECT_NAME = B\nINTERNATIONAL_DESIGNATOR = 2026-002A\nEPHEMERIS_NAME = NONE\nCOVARIANCE_METHOD = CALCULATED\nMANEUVERABLE = NO\nREF_FRAME = EME2000\nX = 7000.1\nY = 0\nZ = 0\nX_DOT = 0\nY_DOT = -7.5\nZ_DOT = 0\nCR_R = 1\nCT_R = 0\nCT_T = 1\nCN_R = 0\nCN_T = 0\nCN_N = 1\nCRDOT_R = 0\nCRDOT_T = 0\nCRDOT_N = 0\nCRDOT_RDOT = 0.001\nCTDOT_R = 0\nCTDOT_T = 0\nCTDOT_N = 0\nCTDOT_RDOT = 0\nCTDOT_TDOT = 0.001\nCNDOT_R = 0\nCNDOT_T = 0\nCNDOT_N = 0\nCNDOT_RDOT = 0\nCNDOT_TDOT = 0\nCNDOT_NDOT = 0.001\n",
        ParseMode::Strict,
    )
    .unwrap();
    let tdm = Tdm::from_kvn(
        "CCSDS_TDM_VERS = 2.0\nCREATION_DATE = 2026-01-01T00:00:00\nORIGINATOR = TEST\nMETA_START\nTIME_SYSTEM = UTC\nPARTICIPANT_1 = DSS-14\nMETA_STOP\nDATA_START\nRANGE = 2026-01-01T00:00:00 384400\nDATA_STOP\n",
        ParseMode::Strict,
    )
    .unwrap();
    let ndm = Ndm {
        comments: vec!["All seven CCSDS message types".to_owned()],
        messages: vec![
            NdmMessage::Opm(opm),
            NdmMessage::Omm(omm),
            NdmMessage::Oem(oem),
            NdmMessage::Apm(apm),
            NdmMessage::Aem(aem),
            NdmMessage::Cdm(cdm),
            NdmMessage::Tdm(tdm),
        ],
    };
    let xml = ndm.to_xml(WriteOptions::default()).unwrap();
    let reparsed = Ndm::from_xml(&xml, ParseMode::Strict).unwrap();
    assert_eq!(reparsed, ndm);
    assert_eq!(reparsed.to_xml(WriteOptions::default()).unwrap(), xml);
}
