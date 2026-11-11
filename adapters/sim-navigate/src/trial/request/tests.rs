use super::super::epoch::parse_utc;

#[test]
fn an_iso_epoch_reads_as_unix_seconds() {
    let utc = parse_utc("2026-03-20T02:00:00Z").unwrap();
    assert_eq!(utc.unix_seconds(), 1_773_972_000.0);
    let fractional = parse_utc("1970-01-01T00:00:01.5Z").unwrap();
    assert_eq!(fractional.unix_seconds(), 1.5);
    assert!(parse_utc("2026-03-20 02:00:00").is_err());
    assert!(parse_utc("2026-13-20T02:00:00Z").is_err());
}
