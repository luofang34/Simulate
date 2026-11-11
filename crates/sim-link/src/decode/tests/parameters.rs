use super::{assert_refusals, with};
use crate::{
    Means, Milli, NavSet, NumberFault, Parameter, Reason, StartTime, Vehicle, decode_fragment,
};

#[test]
fn the_time_is_a_real_utc_second() {
    assert_eq!(
        decode_fragment(&with("t", "20240229T235959Z"))
            .unwrap()
            .time,
        StartTime {
            year: 2024,
            month: 2,
            day: 29,
            hour: 23,
            minute: 59,
            second: 59
        }
    );
    assert_refusals(
        "t",
        Parameter::T,
        &[
            ("2026-06-15T12:00:00Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20260615T120000", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20260615 120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("2026061xT120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20261315T120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20260230T120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20250229T120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("21000229T120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20260600T120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20260615T240000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20260615T126000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("20260630T235960Z", None, |r| {
                matches!(
                    r,
                    Reason::BadTime {
                        why: "a leap second cannot be a start time",
                        ..
                    }
                )
            }),
            ("20260615T120061Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("00000615T120000Z", None, |r| {
                matches!(r, Reason::BadTime { .. })
            }),
            ("", None, |r| matches!(r, Reason::BadTime { .. })),
        ],
    );
    assert_eq!(
        decode_fragment(&with("t", "20000229T000000Z"))
            .unwrap()
            .time
            .iso8601(),
        "2000-02-29T00:00:00Z"
    );
}

#[test]
fn vehicle_and_datum_are_words_of_the_format() {
    assert_eq!(
        decode_fragment(&with("veh", "x500")).unwrap().vehicle,
        Vehicle::X500
    );
    assert_refusals(
        "veh",
        Parameter::Veh,
        &[
            ("b737", None, |r| {
                matches!(
                    r,
                    Reason::UnknownWord {
                        what: "vehicle",
                        ..
                    }
                )
            }),
            ("C172", None, |r| matches!(r, Reason::UnknownWord { .. })),
            ("", None, |r| matches!(r, Reason::UnknownWord { .. })),
        ],
    );
    assert_refusals(
        "hd",
        Parameter::Hd,
        &[("agl", None, |r| {
            matches!(r, Reason::UnknownWord { what: "datum", .. })
        })],
    );
}

#[test]
fn speed_and_seed_are_held_to_their_ranges() {
    let link = decode_fragment(&with("spd", "62.125")).unwrap();
    assert_eq!(link.cruise_speed, Some(Milli::from_scaled(62_125)));
    assert_refusals(
        "spd",
        Parameter::Spd,
        &[
            ("0.999", None, |r| {
                matches!(r, Reason::OutOfRange { what: "speed", .. })
            }),
            ("350.001", None, |r| matches!(r, Reason::OutOfRange { .. })),
            ("55.1234", None, |r| {
                matches!(
                    r,
                    Reason::BadNumber {
                        fault: NumberFault::TooManyDecimals,
                        ..
                    }
                )
            }),
            ("-55", None, |r| matches!(r, Reason::BadNumber { .. })),
            ("5.5e1", None, |r| matches!(r, Reason::BadNumber { .. })),
        ],
    );
    assert_eq!(
        decode_fragment(&with("seed", "18446744073709551615"))
            .unwrap()
            .seed,
        u64::MAX
    );
    assert_eq!(decode_fragment(&with("seed", "0")).unwrap().seed, 0);
    assert_refusals(
        "seed",
        Parameter::Seed,
        &[
            ("18446744073709551616", None, |r| {
                matches!(r, Reason::OutOfRange { what: "seed", .. })
            }),
            ("-1", None, |r| matches!(r, Reason::Malformed { .. })),
            ("1.0", None, |r| matches!(r, Reason::Malformed { .. })),
            ("", None, |r| matches!(r, Reason::Malformed { .. })),
        ],
    );
}

#[test]
fn budget_and_auto_start_are_held_to_their_forms() {
    assert_eq!(
        decode_fragment(&with("mb", "65536")).unwrap().budget_mb,
        65_536
    );
    assert_refusals(
        "mb",
        Parameter::Mb,
        &[
            ("0", None, |r| {
                matches!(r, Reason::OutOfRange { what: "budget", .. })
            }),
            ("65537", None, |r| matches!(r, Reason::OutOfRange { .. })),
            ("99999999999999999999999", None, |r| {
                matches!(r, Reason::OutOfRange { .. })
            }),
            ("2e2", None, |r| matches!(r, Reason::Malformed { .. })),
        ],
    );
    assert_refusals(
        "go",
        Parameter::Go,
        &[
            ("0", None, |r| matches!(r, Reason::Malformed { .. })),
            ("true", None, |r| matches!(r, Reason::Malformed { .. })),
            ("", None, |r| matches!(r, Reason::Malformed { .. })),
        ],
    );
}

#[test]
fn sets_keep_their_means_in_the_order_given() {
    let link = decode_fragment(&with("nav", "ins.stars.baro;ins;ins.baro.visual")).unwrap();
    assert_eq!(
        link.sets,
        vec![
            NavSet(vec![Means::Ins, Means::Stars, Means::Baro]),
            NavSet(vec![Means::Ins]),
            NavSet(vec![Means::Ins, Means::Baro, Means::Visual]),
        ]
    );
    assert_eq!(link.steering_set, 2);
    assert_eq!(link.steering().unwrap().text(), "ins.baro.visual");
}

#[test]
fn malformed_sets_are_refused() {
    assert_refusals(
        "nav",
        Parameter::Nav,
        &[
            ("", None, |r| matches!(r, Reason::Empty { .. })),
            (
                "ins.baro.visual;baro.gnss",
                Some(2),
                |r| matches!(r, Reason::SetWithoutIns { set } if set == "baro.gnss"),
            ),
            ("ins.baro.visual;ins.gnss.gnss", Some(2), |r| {
                matches!(r, Reason::RepeatedMeans { means: "gnss", .. })
            }),
            (
                "ins.baro.visual;ins.baro;baro.ins",
                Some(3),
                |r| matches!(r, Reason::RepeatedSet { set } if set == "baro.ins"),
            ),
            ("ins.baro.visual;ins.radar", Some(2), |r| {
                matches!(r, Reason::UnknownWord { what: "means", .. })
            }),
            ("ins.baro.visual;ins.Baro", Some(2), |r| {
                matches!(r, Reason::UnknownWord { .. })
            }),
            ("ins.baro.visual;ins..baro", Some(2), |r| {
                matches!(r, Reason::Empty { .. })
            }),
            ("ins.baro.visual;", Some(2), |r| {
                matches!(r, Reason::Empty { .. })
            }),
        ],
    );
}

#[test]
fn the_steering_set_is_one_of_the_sets_in_any_order_of_means() {
    assert_eq!(
        decode_fragment(&with("fly", "gnss.ins.baro"))
            .unwrap()
            .steering_set,
        2
    );
    assert_eq!(
        decode_fragment(&with("fly", "baro.ins"))
            .unwrap()
            .steering_set,
        1
    );
    assert_refusals(
        "fly",
        Parameter::Fly,
        &[
            (
                "ins.gnss",
                None,
                |r| matches!(r, Reason::FlyNotInNav { set } if set == "ins.gnss"),
            ),
            ("ins", None, |r| matches!(r, Reason::FlyNotInNav { .. })),
            ("baro", None, |r| matches!(r, Reason::SetWithoutIns { .. })),
            ("ins.sonar", None, |r| {
                matches!(r, Reason::UnknownWord { .. })
            }),
            ("", None, |r| matches!(r, Reason::Empty { .. })),
        ],
    );
}
