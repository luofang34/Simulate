//! `h` and `start`.

use super::{FULL, assert_refusals, refusal, with};
use crate::{HeightDatum, Milli, Parameter, Reason, decode_fragment};

#[test]
fn the_default_height_may_be_absent_when_nothing_needs_it() {
    let fragment = "v=2&r=IF,41.5/2,a1500;TF,41.6/2,a1600.5&hd=hae&start=41.4,2.0,1400\
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
