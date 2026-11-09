use sim_geo::UtcTime;

use super::SimEpoch;
use crate::clock::{SimInstant, TICKS_PER_SECOND, TickSpan};
use crate::hash::ContentHash;
use crate::time::builtin::{BUILTIN_LSK_BYTES, BUILTIN_LSK_SHA256};

/// `1969-07-20T20:17:40Z`.
const APOLLO_11_LANDING_UTC_UNIX_S: f64 = -14_182_940.0;
/// `2026-07-01T00:00:00Z`.
const MID_2026_UTC_UNIX_S: f64 = 1_782_864_000.0;

fn after_seconds(seconds: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(seconds, 0))
}

#[test]
fn builtin_leap_seconds_are_the_pinned_kernel() {
    let expected = ContentHash::from_hex(BUILTIN_LSK_SHA256).unwrap_or(ContentHash([0; 32]));
    assert_eq!(ContentHash::of_bytes(BUILTIN_LSK_BYTES), expected);
}

#[test]
fn start_utc_round_trips_at_apollo_and_present_epochs() {
    for start in [APOLLO_11_LANDING_UTC_UNIX_S, MID_2026_UTC_UNIX_S] {
        let epoch = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(start));
        let utc = epoch.and_then(|e| e.start_utc()).map(|u| u.unix_seconds());
        let got = utc.unwrap_or(f64::NAN);
        assert!((got - start).abs() < 1e-6, "start {start} came back {got}");
    }
}

#[test]
fn utc_advances_with_ticks_and_maps_back_to_the_same_tick() {
    let epoch = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(MID_2026_UTC_UNIX_S));
    let Ok(epoch) = epoch else {
        panic!("builtin epoch: {epoch:?}");
    };
    let later = after_seconds(3_600);
    let utc = epoch
        .utc_at(later)
        .map(|u| u.unix_seconds())
        .unwrap_or(f64::NAN);
    assert!(
        (utc - (MID_2026_UTC_UNIX_S + 3_600.0)).abs() < 1e-6,
        "utc {utc}"
    );
    let back = epoch.instant_at_utc(UtcTime::from_unix_seconds(utc));
    let Ok(Some(back)) = back else {
        panic!("instant_at_utc: {back:?}");
    };
    let error_ticks = back.ticks_u128().abs_diff(later.ticks_u128());
    assert!(
        error_ticks < u128::from(TICKS_PER_SECOND) / 1_000_000,
        "tick round trip off by {error_ticks} ticks"
    );
}

#[test]
fn civil_time_across_the_1972_step_keeps_tdb_uniform() {
    // 1971-12-31T23:59:50Z, twenty TDB seconds before 1972-01-01T00:00:00Z
    // plus the 0.107758 s UTC step back.
    let start = 63_072_000.0 - 10.0;
    let epoch = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(start));
    let Ok(epoch) = epoch else {
        panic!("builtin epoch: {epoch:?}");
    };
    let utc = epoch
        .utc_at(after_seconds(20))
        .map(|u| u.unix_seconds())
        .unwrap_or(f64::NAN);
    let expected = start + 20.0 - 0.107_758;
    assert!((utc - expected).abs() < 1e-5, "got {utc}, want {expected}");
}

#[test]
fn epochs_compare_by_anchor_and_record() {
    let a = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(MID_2026_UTC_UNIX_S));
    let b = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(MID_2026_UTC_UNIX_S));
    let c = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(MID_2026_UTC_UNIX_S + 1.0));
    let (Ok(a), Ok(b), Ok(c)) = (a, b, c) else {
        panic!("builtin epochs");
    };
    assert_eq!(a, b);
    assert_eq!(a.content_hash(), b.content_hash());
    assert_ne!(a, c);
    assert_ne!(a.content_hash(), c.content_hash());
}

#[test]
fn tt_runs_with_tdb_to_within_its_annual_term() {
    let epoch = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(MID_2026_UTC_UNIX_S));
    let Ok(epoch) = epoch else {
        panic!("builtin epoch: {epoch:?}");
    };
    let start_tt = epoch.tt_j2000_seconds_at(SimInstant::START);
    let start_tdb = epoch.tdb_at(SimInstant::START).as_j2000_seconds_f64();
    assert!(
        (start_tdb - start_tt).abs() < 1.7e-3,
        "TDB - TT {}",
        start_tdb - start_tt
    );
    let day_tt = epoch.tt_j2000_seconds_at(after_seconds(86_400)) - start_tt;
    // TDB - TT changes by at most 2π·1.657 ms/yr ≈ 28.5 µs a day.
    assert!(
        (day_tt - 86_400.0).abs() < 3e-5,
        "one TDB day is {day_tt} s of TT"
    );
    assert!(
        (day_tt - 86_400.0).abs() > 0.0,
        "TT must not simply copy TDB"
    );
}
