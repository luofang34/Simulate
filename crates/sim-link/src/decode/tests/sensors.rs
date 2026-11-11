use super::{FULL, assert_refusals, refusal, with};
use crate::{Camera, Milli, NumberFault, Outage, Parameter, Reason, Sensor, Span, decode_fragment};

#[test]
fn outages_keep_their_entries_and_spans() {
    let seconds = |text: &str| Milli::parse(text, false).unwrap();
    let link = decode_fragment(&with("out", "cam:10.5-20.25;gnss:0-300,300-600,900-")).unwrap();
    assert_eq!(
        link.outages,
        vec![
            Outage {
                sensor: Sensor::Cam,
                spans: vec![Span {
                    start: seconds("10.5"),
                    end: Some(seconds("20.25"))
                }],
            },
            Outage {
                sensor: Sensor::Gnss,
                spans: vec![
                    Span {
                        start: seconds("0"),
                        end: Some(seconds("300"))
                    },
                    Span {
                        start: seconds("300"),
                        end: Some(seconds("600"))
                    },
                    Span {
                        start: seconds("900"),
                        end: None
                    },
                ],
            },
        ]
    );
}

#[test]
fn malformed_outages_are_refused() {
    assert_refusals(
        "out",
        Parameter::Out,
        &[
            ("", None, |r| matches!(r, Reason::Empty { .. })),
            ("radar:1-2", Some(1), |r| {
                matches!(r, Reason::UnknownWord { what: "sensor", .. })
            }),
            (
                "gnss:5-5",
                Some(1),
                |r| matches!(r, Reason::ReversedSpan { span } if span == "5-5"),
            ),
            ("baro:1-2;gnss:5-3.5", Some(2), |r| {
                matches!(r, Reason::ReversedSpan { .. })
            }),
            (
                "gnss:1-5,4-6",
                Some(1),
                |r| matches!(r, Reason::OverlappingSpans { span } if span == "4-6"),
            ),
            ("gnss:10-20,1-2", Some(1), |r| {
                matches!(r, Reason::OverlappingSpans { .. })
            }),
            ("gnss:1-,5-6", Some(1), |r| {
                matches!(r, Reason::OverlappingSpans { .. })
            }),
            ("gnss:1-2;baro:1-2;gnss:3-4", Some(3), |r| {
                matches!(r, Reason::RepeatedSensor { sensor: "gnss" })
            }),
            ("gnss", Some(1), |r| matches!(r, Reason::Malformed { .. })),
            ("gnss:5", Some(1), |r| matches!(r, Reason::Malformed { .. })),
            ("gnss:", Some(1), |r| matches!(r, Reason::Empty { .. })),
            ("gnss:1-2,", Some(1), |r| matches!(r, Reason::Empty { .. })),
            ("gnss:-1-2", Some(1), |r| matches!(r, Reason::Empty { .. })),
            ("gnss:1.0001-2", Some(1), |r| {
                matches!(
                    r,
                    Reason::BadNumber {
                        fault: NumberFault::TooManyDecimals,
                        decimals: 3,
                        ..
                    }
                )
            }),
            ("gnss:1-2e3", Some(1), |r| {
                matches!(r, Reason::BadNumber { .. })
            }),
            ("gnss:1-2;", Some(2), |r| {
                matches!(r, Reason::Malformed { .. })
            }),
        ],
    );
}

#[test]
fn the_camera_carries_size_view_and_rate() {
    assert_eq!(
        decode_fragment(FULL).unwrap().camera,
        Some(Camera {
            width_px: 800,
            height_px: 600,
            fov_deg: Milli::from_scaled(67_400),
            depression_deg: Milli::from_whole(90),
            rate_hz: Milli::from_whole(1),
        })
    );
    assert!(decode_fragment(&with("cam", "16x8192,1,0,0.1")).is_ok());
    assert!(decode_fragment(&with("cam", "8192x16,179,90,120")).is_ok());
}

#[test]
fn a_camera_outside_its_ranges_is_refused() {
    assert_refusals(
        "cam",
        Parameter::Cam,
        &[
            ("", None, |r| matches!(r, Reason::Empty { .. })),
            ("800x600,67.4,90", None, |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            ("800x600,67.4,90,1,2", None, |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            ("800*600,67.4,90,1", None, |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            ("800x,67.4,90,1", None, |r| {
                matches!(r, Reason::Malformed { .. })
            }),
            ("15x600,67.4,90,1", None, |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "image width",
                        ..
                    }
                )
            }),
            ("800x8193,67.4,90,1", None, |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "image height",
                        ..
                    }
                )
            }),
            ("800x600,0.999,90,1", None, |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "field of view",
                        ..
                    }
                )
            }),
            ("800x600,179.001,90,1", None, |r| {
                matches!(r, Reason::OutOfRange { .. })
            }),
            ("800x600,67.4,90.001,1", None, |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "depression",
                        ..
                    }
                )
            }),
            ("800x600,67.4,90,0.099", None, |r| {
                matches!(
                    r,
                    Reason::OutOfRange {
                        what: "frame rate",
                        ..
                    }
                )
            }),
            ("800x600,67.4,90,120.001", None, |r| {
                matches!(r, Reason::OutOfRange { .. })
            }),
            ("800x600,67.4444,90,1", None, |r| {
                matches!(r, Reason::BadNumber { .. })
            }),
        ],
    );
}

#[test]
fn a_refusal_in_a_later_outage_entry_points_at_its_span() {
    let fragment = with("out", "baro:1-2;gnss:7-9,8-10");
    let refused = refusal(&fragment);
    assert_eq!(refused.offset, fragment.find("8-10").unwrap());
}
