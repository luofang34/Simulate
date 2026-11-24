use super::*;

const CELESTRAK: &str = r#"[{"OBJECT_NAME":"ISS (ZARYA)","OBJECT_ID":"1998-067A","EPOCH":"2026-11-23T21:54:38.454912","MEAN_MOTION":15.49568558,"ECCENTRICITY":0.00073794,"INCLINATION":51.6322,"RA_OF_ASC_NODE":229.652,"ARG_OF_PERICENTER":58.6021,"MEAN_ANOMALY":301.5689,"EPHEMERIS_TYPE":0,"CLASSIFICATION_TYPE":"U","NORAD_CAT_ID":25544,"ELEMENT_SET_NO":999,"REV_AT_EPOCH":59651,"BSTAR":0.00010921139,"MEAN_MOTION_DOT":5.691e-5,"MEAN_MOTION_DDOT":0}]"#;

#[test]
fn celestrak_layout_reads_leniently_and_writes_back() {
    let omm = Omm::from_json(CELESTRAK, ParseMode::Lenient).unwrap();
    assert_eq!(omm.version, None);
    assert_eq!(omm.metadata.center_name, None);
    assert_eq!(omm.data.tle.as_ref().unwrap().norad_cat_id, Some(25544));
    let written = Omm::to_json_all(std::slice::from_ref(&omm), WriteOptions::default()).unwrap();
    assert_eq!(written, CELESTRAK);
}

#[test]
fn strict_mode_names_the_first_missing_keyword() {
    let error = Omm::from_json(CELESTRAK, ParseMode::Strict).err().unwrap();
    assert!(matches!(error.kind(), ErrorKind::MissingKeyword));
    assert_eq!(error.keyword(), Some("CREATION_DATE"));
    assert_eq!(error.message_index(), Some(0));
}

#[test]
fn string_numbers_only_in_lenient_mode() {
    let text = CELESTRAK.replace("15.49568558", "\"15.49568558\"");
    let lenient = Omm::from_json(&text, ParseMode::Lenient).unwrap();
    assert_eq!(
        lenient.data.mean_elements.size,
        crate::OrbitSize::MeanMotion(crate::Quantity::new(15.49568558))
    );
}

#[test]
fn malformed_json_carries_its_position() {
    let error = Omm::from_json("[{\"OBJECT_NAME\":}]", ParseMode::Lenient)
        .err()
        .unwrap();
    assert!(matches!(error.kind(), ErrorKind::Json(_)));
    assert_eq!(error.location().unwrap().line, 1);
    for text in ["", "3", "[3]", "[]", "[{},{}]", "{\"OBJECT_NAME\":true}"] {
        assert!(Omm::from_json(text, ParseMode::Lenient).is_err(), "{text}");
    }
}
