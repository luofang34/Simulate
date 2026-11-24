use super::*;

#[test]
fn both_date_forms_write_back_exactly() {
    for text in [
        "2022-12-18T14:28:15.1172",
        "2020-064T10:34:41.4264",
        "2021-06-03T00:00:00.000",
        "2002-204T15:56:23Z",
        "2026-11-23T21:54:38.454912",
        "2016-12-31T23:59:60.5",
        "2022-11-06T09:23:57",
    ] {
        let epoch = Epoch::parse(text).unwrap();
        assert_eq!(epoch.to_string(), text);
    }
}

#[test]
fn fields_are_exposed_as_parsed() {
    let epoch = Epoch::parse("2020-064T10:34:41.4264").unwrap();
    assert_eq!(epoch.year(), 2020);
    assert_eq!(epoch.date(), DateForm::Ordinal { day_of_year: 64 });
    assert_eq!((epoch.hour(), epoch.minute(), epoch.second()), (10, 34, 41));
    assert_eq!(epoch.fraction_digits(), "4264");
    assert!(!epoch.is_zulu());
}

#[test]
fn rejects_malformed_and_out_of_range() {
    for text in [
        "",
        "2020-01-01",
        "20-01-01T00:00:00",
        "2020-1-01T00:00:00",
        "2020-13-01T00:00:00",
        "2021-02-29T00:00:00",
        "2021-366T00:00:00",
        "2020-000T00:00:00",
        "2020-01-01T24:00:00",
        "2020-01-01T00:60:00",
        "2020-01-01T00:00:61",
        "2020-01-01T00:00:00.",
        "2020-01-01T00:00:00.1a",
        "2020-01-01T00:00",
        "2020-01-01T00:00:00:00",
        "2020-01-01 00:00:00",
        "2020-01-01T00:00:00ZZ",
        "+2020-01-01T00:00:00",
    ] {
        assert!(Epoch::parse(text).is_err(), "{text:?}");
    }
    assert!(Epoch::parse("2020-366T00:00:00").is_ok());
    assert!(Epoch::parse("2000-02-29T00:00:00").is_ok());
    assert!(Epoch::parse("1900-02-29T00:00:00").is_err());
}
