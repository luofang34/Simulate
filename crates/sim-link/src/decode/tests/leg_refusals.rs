//! `r`: every way a leg can be malformed is refused, pointing into the leg.

use super::legs::{EVERY_PATH, route_with};
use super::{Case, assert_refusals, refusal, with};
use crate::{Key, NumberFault, Parameter, PathTerminator, Reason, shape};

/// The legs of `leg` without the attribute of `key`.
fn without_key(leg: &str, key: Key) -> String {
    let fields: Vec<&str> = leg.split(',').collect();
    let fix_fields = 1 + usize::from(fields.first().is_some_and(|code| {
        PathTerminator::ALL
            .iter()
            .any(|path| path.code() == *code && shape(*path).fix)
    }));
    fields
        .iter()
        .enumerate()
        .filter(|(index, field)| *index < fix_fields || !field.starts_with(key.letter()))
        .map(|(_, field)| *field)
        .collect::<Vec<_>>()
        .join(",")
}

#[test]
fn a_leg_without_a_key_its_path_terminator_needs_is_refused() {
    for (path, leg) in EVERY_PATH {
        for key in shape(path).required {
            let route = route_with(path, &without_key(leg, *key));
            let refused = refusal(&with("r", &route));
            assert_eq!(
                refused.reason,
                Reason::MissingKey {
                    key: key.letter(),
                    leg: path.code(),
                    meaning: key.meaning(),
                },
                "{route}"
            );
        }
    }
}

#[test]
fn a_leg_with_a_key_its_path_terminator_does_not_take_is_refused() {
    for (path, leg) in EVERY_PATH {
        for key in Key::ALL.into_iter().filter(|key| !shape(path).allows(*key)) {
            let route = route_with(path, &format!("{leg},{}1", key.letter()));
            let refused = refusal(&with("r", &route));
            assert_eq!(
                refused.reason,
                Reason::KeyNotAllowed {
                    key: key.letter(),
                    leg: path.code()
                },
                "{route}"
            );
        }
    }
}

#[test]
fn malformed_legs_and_fixes_are_refused() {
    let cases: &[Case] = &[
        ("", None, |r| matches!(r, Reason::Empty { .. })),
        ("XX,A:41.5/2;TF,A", Some(1), |r| {
            matches!(
                r,
                Reason::UnknownWord {
                    what: "path terminator",
                    ..
                }
            )
        }),
        ("if,A:41.5/2;TF,A", Some(1), |r| {
            matches!(r, Reason::UnknownWord { .. })
        }),
        ("IF,A:41.5/2;TF", Some(2), |r| {
            matches!(r, Reason::MissingFix { leg: "TF" })
        }),
        ("IF,A:41.5/2;", Some(2), |r| {
            matches!(r, Reason::Empty { .. })
        }),
        ("IF,A:41.5/2;TF,", Some(2), |r| {
            matches!(r, Reason::Empty { .. })
        }),
        ("IF,A:41.5/2;TF,Q", Some(2), |r| {
            *r == Reason::UnknownFix { name: "Q".into() }
        }),
        ("IF,A:41.5/2;TF,B:91/2", Some(2), |r| {
            matches!(
                r,
                Reason::OutOfRange {
                    what: "latitude",
                    ..
                }
            )
        }),
        ("IF,A:41.5/2;TF,B:41.5/180.000001", Some(2), |r| {
            matches!(
                r,
                Reason::OutOfRange {
                    what: "longitude",
                    ..
                }
            )
        }),
        ("IF,A:41.5/2;TF,B:41.5000001/2", Some(2), |r| {
            matches!(
                r,
                Reason::BadNumber {
                    fault: NumberFault::TooManyDecimals,
                    decimals: 6,
                    ..
                }
            )
        }),
        ("IF,A:41.5,2;TF,A", Some(1), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5;TF,A", Some(1), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,Z 2:41.5/2;TF,A", Some(1), |r| {
            matches!(r, Reason::BadName { .. })
        }),
        ("IF,:41.5/2;TF,A", Some(1), |r| {
            matches!(r, Reason::BadName { .. })
        }),
        (
            "IF,ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456:41.5/2;TF,A",
            Some(1),
            |r| matches!(r, Reason::BadName { .. }),
        ),
    ];
    assert_refusals("r", Parameter::R, cases);
}

#[test]
fn malformed_attributes_are_refused() {
    let cases: &[Case] = &[
        ("IF,A:41.5/2,;TF,A", Some(1), |r| {
            matches!(r, Reason::Empty { .. })
        }),
        ("IF,A:41.5/2,q1;TF,A", Some(1), |r| {
            matches!(
                r,
                Reason::UnknownWord {
                    what: "attribute key",
                    ..
                }
            )
        }),
        ("IF,A:41.5/2,o,o;TF,A", Some(1), |r| {
            *r == Reason::RepeatedKey { key: 'o' }
        }),
        ("IF,A:41.5/2,o1;TF,A", Some(1), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2,rFOO;TF,A", Some(1), |r| {
            matches!(r, Reason::UnknownWord { what: "role", .. })
        }),
        ("IF,A:41.5/2,rIAF.IAF;TF,A", Some(1), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2,r;TF,A", Some(1), |r| {
            matches!(r, Reason::Empty { .. })
        }),
        ("IF,A:41.5/2,tX;TF,A", Some(1), |r| {
            matches!(
                r,
                Reason::UnknownWord {
                    what: "turn direction",
                    ..
                }
            )
        }),
    ];
    assert_refusals("r", Parameter::R, cases);
}

#[test]
fn malformed_altitudes_are_refused() {
    let cases: &[Case] = &[
        ("IF,A:41.5/2,a;TF,A", Some(1), |r| {
            matches!(r, Reason::Empty { .. })
        }),
        ("IF,A:41.5/2,a1500..1000;TF,A", Some(1), |r| {
            matches!(r, Reason::ReversedLimit { .. })
        }),
        ("IF,A:41.5/2,a1500..1500;TF,A", Some(1), |r| {
            matches!(r, Reason::ReversedLimit { .. })
        }),
        ("IF,A:41.5/2,a1500m;TF,A", Some(1), |r| {
            matches!(r, Reason::BadNumber { .. })
        }),
        ("IF,A:41.5/2,a30000.001;TF,A", Some(1), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2,a98426ft;TF,A", Some(1), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2,aFL985;TF,A", Some(1), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2,aFL12.5;TF,A", Some(1), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2,a1.0001;TF,A", Some(1), |r| {
            matches!(
                r,
                Reason::BadNumber {
                    fault: NumberFault::TooManyDecimals,
                    decimals: 3,
                    ..
                }
            )
        }),
    ];
    assert_refusals("r", Parameter::R, cases);
}

#[test]
fn malformed_speeds_bearings_and_lengths_are_refused() {
    let cases: &[Case] = &[
        ("IF,A:41.5/2,xFL10;TF,A", Some(1), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2,gQ100;TF,A", Some(1), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2,v-90.5;TF,A", Some(1), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2,s0;TF,A", Some(1), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2,s681kt;TF,A", Some(1), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2,s-5;TF,A", Some(1), |r| {
            matches!(r, Reason::BadNumber { .. })
        }),
        ("IF,A:41.5/2;CF,A,c360T", Some(2), |r| {
            matches!(
                r,
                Reason::OutOfRange {
                    what: "bearing",
                    ..
                }
            )
        }),
        ("IF,A:41.5/2;CF,A,c90", Some(2), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2;CF,A,c90X", Some(2), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2;FC,A,c90T,d0", Some(2), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2;FC,A,c90T,d541nm", Some(2), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2;HM,A,tR,c90T,l60", Some(2), |r| {
            matches!(r, Reason::Malformed { .. })
        }),
        ("IF,A:41.5/2;HM,A,tR,c90T,l61min", Some(2), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2;HM,A,tR,c90T,l0s", Some(2), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
        ("IF,A:41.5/2,p0;TF,A", Some(1), |r| {
            matches!(r, Reason::OutOfRange { .. })
        }),
    ];
    assert_refusals("r", Parameter::R, cases);
}

#[test]
fn a_bearing_or_navaid_distance_needs_its_navaid() {
    let cases: &[Case] = &[
        ("IF,A:41.5/2,b10T;TF,A", Some(1), |r| {
            matches!(r, Reason::MissingKey { key: 'n', .. })
        }),
        ("IF,A:41.5/2,d2nm;TF,A", Some(1), |r| {
            matches!(r, Reason::MissingKey { key: 'n', .. })
        }),
        ("IF,A:41.5/2;CF,A,c1T,d2nm;TF,A", Some(2), |r| {
            matches!(
                r,
                Reason::MissingKey {
                    key: 'n',
                    leg: "CF",
                    ..
                }
            )
        }),
    ];
    assert_refusals("r", Parameter::R, cases);
    for along in ["IF,A:41.5/2;FC,A,c1T,d2nm", "IF,A:41.5/2;PI,A,tL,c1T,d10nm"] {
        crate::decode_fragment(&with("r", along))
            .unwrap_or_else(|refusal| panic!("{along}: {refusal}"));
    }
}

#[test]
fn a_leg_where_its_path_terminator_cannot_stand_is_refused() {
    let misplaced = |route: &str| refusal(&with("r", route));
    for (route, leg, item) in [
        ("IF,A:41.5/2;IF,B:41.6/2", "IF", 2),
        ("TF,A:41.5/2;TF,B:41.6/2", "TF", 1),
        ("IF,A:41.5/2;HM,A,tR,c90T,l1min;TF,B:41.6/2", "HM", 2),
        ("IF,A:41.5/2;FM,A,c90T;TF,B:41.6/2", "FM", 2),
        ("IF,A:41.5/2;VM,h90T;TF,B:41.6/2", "VM", 2),
        ("IF,A:41.5/2;CI,c90T", "CI", 2),
        ("IF,A:41.5/2;VI,h90T", "VI", 2),
    ] {
        let refused = misplaced(route);
        assert!(
            matches!(refused.reason, Reason::MisplacedLeg { leg: found, .. } if found == leg),
            "{route}: {refused}"
        );
        assert_eq!(refused.item, Some(item), "{route}");
    }
}

#[test]
fn a_refusal_points_at_the_attribute_at_fault() {
    let fragment = with("r", "IF,A:41.5/2;TF,B:41.6/2,o,tQ");
    let refused = refusal(&fragment);
    assert_eq!(refused.offset, fragment.find(",tQ").unwrap() + 2);
    assert_eq!(refused.item, Some(2));
}
