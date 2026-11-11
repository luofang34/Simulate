use super::{FULL, assert_refusals, refusal, with, without};
use crate::{
    HeightDatum, MAX_WAYPOINTS, Micro, Milli, NumberFault, Parameter, Reason, Turn, decode_fragment,
};

#[test]
fn waypoints_carry_position_height_turn_and_name() {
    let link = decode_fragment(&with("r", "41.5,-1.25,1500,o,A-1;-41,2,,b;0,180,-500.0")).unwrap();
    let [first, second, third] = link.route.as_slice() else {
        panic!("three waypoints");
    };
    assert_eq!(first.lat, Micro::from_scaled(41_500_000));
    assert_eq!(first.lon_deg(), -1.25);
    assert_eq!(first.height, Some(Milli::from_whole(1500)));
    assert_eq!(first.turn, Turn::FlyOver);
    assert_eq!(first.name.as_deref(), Some("A-1"));
    assert_eq!(
        (second.height, second.turn, &second.name),
        (None, Turn::FlyBy, &None)
    );
    assert_eq!(third.height, Some(Milli::from_whole(-500)));
    assert_eq!(
        link.height_or_default(second.height),
        Some(Milli::from_scaled(1_981_200))
    );
    assert_eq!(link.datum, HeightDatum::MeanSeaLevel);
}

#[test]
fn trailing_empty_fields_and_repeated_names_are_accepted() {
    let link = decode_fragment(&with("r", "41.5,2.0,,,;41.6,2.0,,,HOME;41.5,2.0,,,HOME")).unwrap();
    assert_eq!(link.route[0].name, None);
    assert_eq!(link.route[1].name, link.route[2].name);
}

#[test]
fn a_route_needs_two_to_three_hundred_waypoints() {
    let one = refusal(&with("r", "41.5,2.0"));
    assert_eq!(one.reason, Reason::TooFewWaypoints { count: 1 });
    let many = vec!["41.5,2.0"; MAX_WAYPOINTS + 1].join(";");
    let refused = refusal(&with("r", &many));
    assert_eq!(
        refused.reason,
        Reason::TooManyWaypoints {
            count: MAX_WAYPOINTS + 1,
            limit: MAX_WAYPOINTS
        }
    );
    let most = vec!["41.5,2.0"; MAX_WAYPOINTS].join(";");
    assert_eq!(
        decode_fragment(&with("r", &most)).unwrap().route.len(),
        MAX_WAYPOINTS
    );
}

#[test]
fn waypoint_positions_outside_the_format_are_refused() {
    assert_refusals(
        "r",
        Parameter::R,
        &[
            ("", None, |r| matches!(r, Reason::Empty { .. })),
            ("41.5,2.0;91,2.0", Some(2), |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "latitude",
                        ..
                    }
                )
            }),
            ("41.5,2.0;-90.000001,2.0", Some(2), |r| {
                matches!(r, Reason::OutOfRange { .. })
            }),
            ("41.5,180.000001;41.5,2.0", Some(1), |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "longitude",
                        ..
                    }
                )
            }),
            ("4e1,2.0;41.5,2.0", Some(1), |r| {
                matches!(
                    r,
                    Reason::BadNumber {
                        fault: NumberFault::NotDecimal,
                        ..
                    }
                )
            }),
            ("nan,2.0;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::BadNumber { .. })
            }),
            ("41.5,+2.0;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::BadNumber { .. })
            }),
            ("41.5000001,2.0;41.5,2.0", Some(1), |r| {
                matches!(
                    r,
                    Reason::BadNumber {
                        fault: NumberFault::TooManyDecimals,
                        decimals: 6,
                        ..
                    }
                )
            }),
        ],
    );
}

#[test]
fn malformed_waypoint_fields_are_refused() {
    assert_refusals(
        "r",
        Parameter::R,
        &[
            ("41.5,2.0;41.5,2.0,30000.001", Some(2), |r| {
                matches!(r, Reason::OutOfRange { what: "height", .. })
            }),
            ("41.5,2.0;41.5,2.0,1.0001", Some(2), |r| {
                matches!(
                    r,
                    Reason::BadNumber {
                        fault: NumberFault::TooManyDecimals,
                        decimals: 3,
                        ..
                    }
                )
            }),
            ("41.5,2.0,,x;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::UnknownWord { what: "turn", .. })
            }),
            ("41.5,2.0,,,Z 2;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::BadName { .. })
            }),
            (
                "41.5,2.0,,,ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456;41.5,2.0",
                Some(1),
                |r| matches!(r, Reason::BadName { .. }),
            ),
            ("41.5,2.0,,,A,B;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            ("41.5;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            ("41.5,2.0;", Some(2), |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            (",2.0;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::Empty { .. })
            }),
            ("41.5,;41.5,2.0", Some(1), |r| {
                matches!(r, Reason::Empty { .. })
            }),
        ],
    );
}

#[test]
fn a_name_of_thirty_two_characters_is_accepted() {
    let name = "ABCDEFGHIJKLMNOPQRSTUVWXYZ_-0123";
    assert_eq!(name.len(), 32);
    let link = decode_fragment(&with("r", &format!("41.5,2.0,,,{name};41.6,2.0"))).unwrap();
    assert_eq!(link.route[0].name.as_deref(), Some(name));
}

#[test]
fn a_refusal_points_at_the_field_at_fault() {
    let fragment = with("r", "41.5,2.0;41.5,2.0,1.0,q,NAME");
    let refused = refusal(&fragment);
    assert_eq!(refused.offset, fragment.find(",q,").unwrap() + 1);
    assert_eq!(refused.item, Some(2));
}

#[test]
fn a_waypoint_without_a_height_needs_the_default_height() {
    let refused = refusal(&without("h"));
    assert_eq!(
        (refused.parameter, refused.item, &refused.reason),
        (Parameter::R, Some(1), &Reason::MissingHeight)
    );
    let fragment = without("h").replace(",,,LEZG", ",900,,LEZG");
    assert_eq!(refusal(&fragment).item, Some(2));
}

#[test]
fn the_default_height_may_be_absent_when_nothing_needs_it() {
    let fragment = "v=1&r=41.5,2.0,1500;41.6,2.0,1600.5&hd=hae&start=41.4,2.0,1400\
&t=20260615T120000Z&veh=x500&seed=7&nav=ins&mb=1";
    let link = decode_fragment(fragment).unwrap();
    assert_eq!(link.default_height, None);
    assert_eq!(link.datum, HeightDatum::Ellipsoid);
    let refused = refusal(&fragment.replace("start=41.4,2.0,1400", "start=41.4,2.0"));
    assert_eq!(
        (refused.parameter, &refused.reason),
        (Parameter::Start, &Reason::MissingHeight)
    );
}

#[test]
fn the_default_height_is_held_to_its_range() {
    assert_refusals(
        "h",
        Parameter::H,
        &[
            ("-500.1", None, |r| {
                matches!(r, Reason::OutOfRange { what: "height", .. })
            }),
            ("30000.5", None, |r| matches!(r, Reason::OutOfRange { .. })),
            ("high", None, |r| matches!(r, Reason::BadNumber { .. })),
            ("", None, |r| matches!(r, Reason::Empty { .. })),
        ],
    );
    assert_eq!(
        decode_fragment(&with("h", "-500")).unwrap().default_height,
        Some(Milli::from_whole(-500))
    );
}

#[test]
fn the_start_carries_position_height_and_heading() {
    let start = decode_fragment(FULL).unwrap().start.unwrap();
    assert_eq!((start.lat_deg(), start.lon_deg()), (41.66625, -1.04155));
    assert_eq!(
        (start.height, start.heading),
        (None, Some(Milli::from_whole(75)))
    );
    let start = decode_fragment(&with("start", "-41.5,170.25,1800.0"))
        .unwrap()
        .start
        .unwrap();
    assert_eq!(
        (start.height, start.heading),
        (Some(Milli::from_whole(1800)), None)
    );
    let north = decode_fragment(&with("start", "0,0,,0"))
        .unwrap()
        .start
        .unwrap();
    assert_eq!(north.heading, Some(Milli::from_whole(0)));
}

#[test]
fn a_malformed_start_is_refused() {
    assert_refusals(
        "start",
        Parameter::Start,
        &[
            ("", None, |r| matches!(r, Reason::Empty { .. })),
            ("41.5", None, |r| matches!(r, Reason::Malformed { .. })),
            ("41.5,2.0,,90,extra", None, |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            ("91,2.0", None, |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "latitude",
                        ..
                    }
                )
            }),
            ("41.5,2.0,,360", None, |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "heading",
                        ..
                    }
                )
            }),
            ("41.5,2.0,,-1", None, |r| {
                matches!(r, Reason::BadNumber { .. })
            }),
            ("41.5,2.0,,359.9999", None, |r| {
                matches!(r, Reason::BadNumber { .. })
            }),
            ("41.5,2.0,40000", None, |r| {
                matches!(r, Reason::OutOfRange { what: "height", .. })
            }),
        ],
    );
    assert_eq!(
        decode_fragment(&with("start", "41.5,2.0,,359.999"))
            .unwrap()
            .start
            .unwrap()
            .heading,
        Some(Milli::from_scaled(359_999))
    );
}
