//! One defect per case, with the error kind, keyword and line it must name.

use ccsds_ndm::{
    Error, ErrorKind, ExtensionValue, Ndm, Oem, Omm, Opm, ParseMode, Quantity, WriteOptions,
};

const OPM: &str = "CCSDS_OPM_VERS = 3.0
CREATION_DATE = 2022-11-06T09:23:57
ORIGINATOR = JAXA
OBJECT_NAME = OSPREY 5
OBJECT_ID = 1998-999A
CENTER_NAME = EARTH
REF_FRAME = ITRF2000
TIME_SYSTEM = UTC
EPOCH = 2022-12-18T14:28:15.1172
X = 6503.514
Y = 1239.647
Z = -717.49
X_DOT = -0.87316
Y_DOT = 8.74042
Z_DOT = -4.191076
";

const OEM: &str = "CCSDS_OEM_VERS = 3.0
CREATION_DATE = 1996-11-04T17:22:31
ORIGINATOR = NASA/JPL
META_START
OBJECT_NAME = MGS
OBJECT_ID = 1996-062A
CENTER_NAME = MARS BARYCENTER
REF_FRAME = EME2000
TIME_SYSTEM = UTC
START_TIME = 2019-12-18T12:00:00.331
STOP_TIME = 2019-12-28T21:28:00.331
META_STOP
2019-12-18T12:00:00.331 2789.619 -280.045 -1746.755 4.73372 -2.49586 -1.04195
";

fn opm_with(line: usize, replacement: &str) -> String {
    let mut lines: Vec<&str> = OPM.lines().collect();
    lines[line - 1] = replacement;
    lines.join("\n")
}

fn expect(
    result: Result<impl std::fmt::Debug, Error>,
    line: Option<usize>,
    keyword: Option<&str>,
) -> Error {
    let error = result.expect_err("input must be rejected");
    assert_eq!(error.location().map(|l| l.line), line, "{error}");
    assert_eq!(error.keyword(), keyword, "{error}");
    error
}

#[test]
fn baseline_inputs_are_valid() {
    Opm::from_kvn(OPM, ParseMode::Strict).unwrap();
    Oem::from_kvn(OEM, ParseMode::Strict).unwrap();
}

#[test]
fn keyword_level_defects() {
    let missing = OPM.replace("OBJECT_ID = 1998-999A\n", "");
    let e = expect(
        Opm::from_kvn(&missing, ParseMode::Lenient),
        Some(4),
        Some("OBJECT_ID"),
    );
    assert!(matches!(e.kind(), ErrorKind::MissingKeyword));

    let e = expect(
        Opm::from_kvn(&opm_with(10, "X = 1.2.3"), ParseMode::Strict),
        Some(10),
        Some("X"),
    );
    assert!(matches!(e.kind(), ErrorKind::InvalidNumber { text } if text == "1.2.3"));
    assert_eq!(e.location().unwrap().column, 5);

    let e = expect(
        Opm::from_kvn(&opm_with(10, "X = 1 [m]"), ParseMode::Lenient),
        Some(10),
        Some("X"),
    );
    assert!(matches!(
        e.kind(),
        ErrorKind::UnitMismatch { expected: "km", .. }
    ));

    let upper = opm_with(10, "X = 1 [KM]");
    let e = expect(
        Opm::from_kvn(&upper, ParseMode::Strict),
        Some(10),
        Some("X"),
    );
    assert!(matches!(e.kind(), ErrorKind::UnitMismatch { .. }));
    assert_eq!(
        Opm::from_kvn(&upper, ParseMode::Lenient)
            .unwrap()
            .data
            .state
            .position[0],
        Quantity::new(1.0)
    );

    let e = expect(
        Opm::from_kvn(
            &opm_with(9, "EPOCH = 2022-13-18T14:28:15"),
            ParseMode::Strict,
        ),
        Some(9),
        Some("EPOCH"),
    );
    assert!(matches!(e.kind(), ErrorKind::InvalidEpoch { .. }));
}

#[test]
fn value_level_defects() {
    let e = expect(
        Opm::from_kvn(&opm_with(8, "TIME_SYSTEM = UTX"), ParseMode::Lenient),
        Some(8),
        Some("TIME_SYSTEM"),
    );
    assert!(matches!(e.kind(), ErrorKind::InvalidValue { .. }));

    let e = expect(
        Opm::from_kvn(&opm_with(1, "CCSDS_OPM_VERS = 4.0"), ParseMode::Lenient),
        Some(1),
        Some("CCSDS_OPM_VERS"),
    );
    assert!(matches!(e.kind(), ErrorKind::UnsupportedVersion { .. }));

    let e = expect(
        Opm::from_kvn(&opm_with(11, "X = 2"), ParseMode::Lenient),
        Some(11),
        Some("X"),
    );
    assert!(matches!(e.kind(), ErrorKind::DuplicateKeyword));

    let e = expect(
        Opm::from_kvn(&opm_with(3, "ORIGINATOR = "), ParseMode::Strict),
        Some(3),
        Some("ORIGINATOR"),
    );
    assert!(matches!(e.kind(), ErrorKind::EmptyValue));
    let lenient = Opm::from_kvn(&opm_with(3, "ORIGINATOR = "), ParseMode::Lenient).unwrap();
    assert_eq!(lenient.header.originator, None);
}

#[test]
fn undefined_keywords_are_strict_errors_and_lenient_extensions() {
    let text = opm_with(15, "Z_DOT = -4.191076\nVENDOR_NOTE = hello");
    let e = expect(
        Opm::from_kvn(&text, ParseMode::Strict),
        Some(16),
        Some("VENDOR_NOTE"),
    );
    assert!(matches!(e.kind(), ErrorKind::UnknownKeyword));
    let opm = Opm::from_kvn(&text, ParseMode::Lenient).unwrap();
    assert_eq!(
        opm.extensions[0].value,
        ExtensionValue::Text("hello".to_owned())
    );
    let again = Opm::from_kvn(
        &opm.to_kvn(WriteOptions::default()).unwrap(),
        ParseMode::Lenient,
    )
    .unwrap();
    assert_eq!(again, opm);
}

#[test]
fn structure_defects() {
    let e = expect(
        Opm::from_kvn(
            &opm_with(9, "EPOCH 2022-12-18T14:28:15"),
            ParseMode::Lenient,
        ),
        Some(9),
        None,
    );
    assert!(matches!(e.kind(), ErrorKind::UnexpectedLine { .. }));

    let e = expect(Opm::from_kvn("", ParseMode::Lenient), None, None);
    assert!(matches!(e.kind(), ErrorKind::Empty));

    let e = expect(Opm::from_kvn(OEM, ParseMode::Lenient), Some(1), None);
    assert!(matches!(e.kind(), ErrorKind::WrongMessage { .. }));

    let short = OEM.replace(" -1.04195", "");
    let e = expect(Oem::from_kvn(&short, ParseMode::Lenient), Some(13), None);
    assert!(matches!(e.kind(), ErrorKind::ColumnCount { found: 6, .. }));

    let open = OEM.replace("META_STOP\n", "");
    let e = expect(Oem::from_kvn(&open, ParseMode::Lenient), Some(12), None);
    assert!(matches!(e.kind(), ErrorKind::UnexpectedLine { .. }));

    let covariance = format!(
        "{OEM}COVARIANCE_START\nEPOCH = 2019-12-18T12:00:00.331\n1\n2 3\n4 5 6\n7 8 9 10\n11 12 13 14 15\n16 17 18 19 20\nCOVARIANCE_STOP\n"
    );
    let e = expect(
        Oem::from_kvn(&covariance, ParseMode::Strict),
        Some(21),
        None,
    );
    assert!(matches!(e.kind(), ErrorKind::ColumnCount { found: 5, .. }));
    let e = expect(
        Oem::from_kvn(&covariance, ParseMode::Lenient),
        Some(15),
        None,
    );
    assert!(matches!(e.kind(), ErrorKind::ColumnCount { found: 20, .. }));
}

#[test]
fn xml_and_json_defects() {
    let e = expect(
        Opm::from_xml(
            "<opm id=\"CCSDS_OPM_VERS\" version=\"3.0\">\n<header>",
            ParseMode::Lenient,
        ),
        Some(2),
        None,
    );
    assert!(
        matches!(
            e.kind(),
            ErrorKind::UnexpectedEnd { .. } | ErrorKind::Xml(_)
        ),
        "{e}"
    );

    let no_body = "<opm id=\"CCSDS_OPM_VERS\" version=\"3.0\"><header/></opm>";
    let e = expect(Opm::from_xml(no_body, ParseMode::Lenient), Some(1), None);
    assert!(matches!(
        e.kind(),
        ErrorKind::MissingElement { name: "body" }
    ));

    let wrong_id = "<opm id=\"X\" version=\"3.0\"/>";
    let e = expect(Opm::from_xml(wrong_id, ParseMode::Strict), Some(1), None);
    assert!(matches!(e.kind(), ErrorKind::InvalidValue { .. }));

    let e = expect(Ndm::from_xml("<omm/>", ParseMode::Lenient), Some(1), None);
    assert!(matches!(e.kind(), ErrorKind::WrongMessage { .. }));

    let e = expect(
        Omm::from_json("{\"OBJECT_NAME\": [1]}", ParseMode::Lenient),
        None,
        Some("OBJECT_NAME"),
    );
    assert!(matches!(e.kind(), ErrorKind::JsonType { .. }));
}

#[test]
fn writers_refuse_what_the_notation_cannot_carry() {
    let mut opm = Opm::from_kvn(OPM, ParseMode::Strict).unwrap();
    opm.data.state.position[0] = Quantity::new(f64::NAN);
    let e = opm.to_kvn(WriteOptions::default()).err().unwrap();
    assert!(matches!(e.kind(), ErrorKind::NonFiniteNumber));
    assert_eq!(e.keyword(), Some("X"));

    let mut opm = Opm::from_kvn(OPM, ParseMode::Strict).unwrap();
    opm.metadata.object_name = "TWO\nLINES".to_owned();
    let e = opm.to_kvn(WriteOptions::default()).err().unwrap();
    assert!(matches!(e.kind(), ErrorKind::InvalidValue { .. }));
    let xml = opm.to_xml(WriteOptions::default()).unwrap();
    assert_eq!(Opm::from_xml(&xml, ParseMode::Strict).unwrap(), opm);

    let json = "{\"OBJECT_NAME\":\"A\",\"OBJECT_ID\":\"B\",\"EPOCH\":\"2020-01-01T00:00:00\",\"MEAN_MOTION\":1,\"ECCENTRICITY\":0,\"INCLINATION\":0,\"RA_OF_ASC_NODE\":0,\"ARG_OF_PERICENTER\":0,\"MEAN_ANOMALY\":0}";
    let omm = Omm::from_json(json, ParseMode::Lenient).unwrap();
    let e = omm.to_kvn(WriteOptions::default()).err().unwrap();
    assert_eq!(e.keyword(), Some("CCSDS_OMM_VERS"));
    assert_eq!(omm.to_json(WriteOptions::default()).unwrap(), json);
}
