use super::{tai_minus_utc_s, utc_from_tai_unix_s};

/// `1969-07-20T20:17:40Z`, the Apollo 11 landing.
const APOLLO_11_LANDING_UTC_UNIX_S: f64 = -14_182_940.0;
/// `1972-01-01T00:00:00Z`.
const LEAP_TABLE_START_UTC_UNIX_S: f64 = 63_072_000.0;

#[test]
fn apollo_era_offset_follows_the_1968_segment() {
    let mjd = APOLLO_11_LANDING_UTC_UNIX_S / 86_400.0 + 40_587.0;
    let expected = 4.213_170_0 + (mjd - 39_126.0) * 0.002_592;
    let got = tai_minus_utc_s(APOLLO_11_LANDING_UTC_UNIX_S).unwrap_or(f64::NAN);
    assert!((got - expected).abs() < 1e-12, "got {got}, want {expected}");
    assert!(
        (7.5..7.6).contains(&got),
        "mid-1969 TAI-UTC is about 7.57 s, got {got}"
    );
}

#[test]
fn final_steered_offset_steps_to_ten_seconds_by_the_documented_amount() {
    let last = tai_minus_utc_s(LEAP_TABLE_START_UTC_UNIX_S - 1e-3).unwrap_or(f64::NAN);
    assert!(
        (10.0 - last - 0.107_758).abs() < 1e-5,
        "UTC stepped 0.107758 s into 1972, got TAI-UTC {last} before the step"
    );
    assert_eq!(tai_minus_utc_s(LEAP_TABLE_START_UTC_UNIX_S), None);
}

#[test]
fn table_starts_in_1961() {
    let start = (37_300.0 - 40_587.0) * 86_400.0;
    assert!(tai_minus_utc_s(start).is_some());
    assert_eq!(tai_minus_utc_s(start - 1.0), None);
    assert_eq!(tai_minus_utc_s(f64::NAN), None);
}

#[test]
fn tai_to_utc_inverts_utc_to_tai_inside_every_segment() {
    for utc in [
        (37_400.0 - 40_587.0) * 86_400.0,
        (38_000.0 - 40_587.0) * 86_400.0,
        (38_900.0 - 40_587.0) * 86_400.0,
        APOLLO_11_LANDING_UTC_UNIX_S,
        LEAP_TABLE_START_UTC_UNIX_S - 10.0,
    ] {
        let tai = utc + tai_minus_utc_s(utc).unwrap_or(f64::NAN);
        let back = utc_from_tai_unix_s(tai).unwrap_or(f64::NAN);
        assert!(
            (back - utc).abs() < 1e-6,
            "utc {utc} -> tai {tai} -> {back}"
        );
    }
}
