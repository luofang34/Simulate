//! `r`: every path terminator, attribute values and fix references.

use super::{FULL, refusal, with};
use crate::{
    Bearing, Distance, DistanceUnit, Duration, Height, HeightUnit, HoldLength, Key, Limit,
    MAX_LEGS, Micro, Milli, North, Parameter, PathTerminator, Reason, Role, SecondAltitude,
    SecondAltitudeKind, Speed, SpeedUnit, TimeUnit, TurnDirection, decode_fragment, shape,
};

/// One leg of every path terminator with only the keys it needs, in the
/// written form, after `IF,A:41.5/2`.
pub(super) const EVERY_PATH: [(PathTerminator, &str); 23] = [
    (PathTerminator::If, "IF,B:41.6/2"),
    (PathTerminator::Tf, "TF,B:41.6/2"),
    (PathTerminator::Cf, "CF,B:41.6/2,c10T"),
    (PathTerminator::Df, "DF,B:41.6/2"),
    (PathTerminator::Fa, "FA,A,a1500+,c90T"),
    (PathTerminator::Fc, "FC,A,c90T,d2nm"),
    (PathTerminator::Fd, "FD,A,c90T,d5nm,nV:41.4/2.1"),
    (PathTerminator::Fm, "FM,A,c90M"),
    (PathTerminator::Ca, "CA,a300+,c40T"),
    (PathTerminator::Cd, "CD,c40T,d5nm,nV:41.4/2.1"),
    (PathTerminator::Ci, "CI,c40T"),
    (PathTerminator::Cr, "CR,c40T,b120M,nV:41.4/2.1"),
    (PathTerminator::Rf, "RF,B:41.6/2.1,tL,zC:41.6/2"),
    (
        PathTerminator::Af,
        "AF,B:41.6/2.1,tR,d10nm,b350M,nV:41.5/2.1",
    ),
    (PathTerminator::Va, "VA,a300+,h40M"),
    (PathTerminator::Vd, "VD,h40M,d5nm,nV:41.4/2.1"),
    (PathTerminator::Vi, "VI,h40M"),
    (PathTerminator::Vm, "VM,h40M"),
    (PathTerminator::Vr, "VR,h40M,b120M,nV:41.4/2.1"),
    (PathTerminator::Pi, "PI,A,tL,c270T,d10nm"),
    (PathTerminator::Ha, "HA,A,tR,a1500,c90T,l1min"),
    (PathTerminator::Hf, "HF,A,tR,c90T,l4nm"),
    (PathTerminator::Hm, "HM,A,tL,c90T,l60s"),
];

/// A route that flies `leg` where its path terminator may stand.
pub(super) fn route_with(path: PathTerminator, leg: &str) -> String {
    match path {
        PathTerminator::If => format!("{leg};TF,A:41.5/2"),
        _ if path.intercepts_next() => format!("IF,A:41.5/2;{leg};TF,D:41.7/2"),
        _ => format!("IF,A:41.5/2;{leg}"),
    }
}

#[test]
fn every_path_terminator_decodes_and_writes_back_unchanged() {
    assert_eq!(EVERY_PATH.len(), PathTerminator::ALL.len());
    for (path, leg) in EVERY_PATH {
        let route = route_with(path, leg);
        let fragment = with("r", &route);
        let link =
            decode_fragment(&fragment).unwrap_or_else(|refusal| panic!("{route}: {refusal}"));
        let index = usize::from(path != PathTerminator::If);
        assert_eq!(link.route[index].path, path, "{route}");
        assert_eq!(link.route[index].fix.is_some(), shape(path).fix, "{route}");
        assert_eq!(link.fragment().unwrap(), fragment, "{route}");
    }
}

#[test]
fn the_full_link_carries_three_named_legs() {
    let link = decode_fragment(FULL).unwrap();
    let names: Vec<_> = link
        .route
        .iter()
        .map(|leg| leg.fix.as_ref().unwrap().name.as_deref().unwrap())
        .collect();
    assert_eq!(names, ["LEZG", "Z1", "Z_2_A"]);
    assert_eq!(
        link.route[1].fix.as_ref().unwrap().lat,
        Micro::from_scaled(41_750_000)
    );
}

#[test]
fn every_attribute_reads_its_value_and_unit() {
    let route = "IF,A:41.5/2;TF,RW04:38.68/-75.36,o,m,rRWY.MAP,tR,a1800ft..FL120,gG1800ft,\
v-3.1,x50ft,s180kt-,d4.5nm,b123.4M,nV:41.4/2.1,p0.3nm";
    let fragment = with("r", route);
    let link = decode_fragment(&fragment).unwrap();
    let leg = &link.route[1];
    assert!(leg.fly_over && leg.missed_approach);
    assert_eq!(leg.roles, [Role::Rwy, Role::Map]);
    assert_eq!(leg.turn, Some(TurnDirection::Right));
    let feet = |feet: i64| Height {
        value: Milli::from_whole(feet),
        unit: HeightUnit::Foot,
    };
    assert_eq!(
        leg.altitude,
        Some(Limit::Between {
            lower: feet(1800),
            upper: Height {
                value: Milli::from_whole(120),
                unit: HeightUnit::FlightLevel
            }
        })
    );
    assert_eq!(
        leg.second_altitude,
        Some(SecondAltitude {
            kind: SecondAltitudeKind::GlidePath,
            height: feet(1800)
        })
    );
    assert_eq!(leg.vertical_angle, Some(Milli::from_scaled(-3_100)));
    assert_eq!(leg.crossing_height, Some(feet(50)));
    assert_eq!(
        leg.speed,
        Some(Limit::AtOrBelow(Speed {
            value: Milli::from_whole(180),
            unit: SpeedUnit::Knot
        }))
    );
    assert_eq!(leg.distance.map(Distance::metres), Some(4.5 * 1852.0));
    assert_eq!(
        leg.theta,
        Some(Bearing {
            degrees: Milli::from_scaled(123_400),
            north: North::Magnetic
        })
    );
    assert_eq!(leg.navaid.as_ref().unwrap().name.as_deref(), Some("V"));
    assert_eq!(
        leg.rnp,
        Some(Distance {
            value: Milli::from_scaled(300),
            unit: DistanceUnit::NauticalMile
        })
    );
    assert_eq!(link.fragment().unwrap(), fragment);
}

#[test]
fn altitudes_speeds_and_hold_lengths_take_every_form() {
    let altitude = |text: &str| {
        let fragment = with("r", &format!("IF,A:41.5/2,a{text};TF,B:41.6/2"));
        let link = decode_fragment(&fragment).unwrap_or_else(|refusal| panic!("{text}: {refusal}"));
        assert_eq!(link.fragment().unwrap(), fragment);
        link.route[0].altitude.unwrap()
    };
    let metres = |value: i64| Height {
        value: Milli::from_whole(value),
        unit: HeightUnit::Metre,
    };
    assert_eq!(altitude("1500"), Limit::At(metres(1500)));
    assert_eq!(altitude("1500+"), Limit::AtOrAbove(metres(1500)));
    assert_eq!(altitude("-200-"), Limit::AtOrBelow(metres(-200)));
    assert_eq!(altitude("-200"), Limit::At(metres(-200)));
    assert_eq!(
        altitude("100..200"),
        Limit::Between {
            lower: metres(100),
            upper: metres(200)
        }
    );
    assert_eq!(altitude("FL80").text(Height::text), "FL80");
    assert_eq!(altitude("4500.5ft").text(Height::text), "4500.5ft");

    let speed = decode_fragment(&with("r", "IF,A:41.5/2,s60..70;TF,B:41.6/2")).unwrap();
    assert_eq!(
        speed.route[0].speed.map(|limit| limit.text(Speed::text)),
        Some("60..70".to_owned())
    );

    for (text, length) in [
        (
            "60s",
            HoldLength::Time(Duration {
                value: Milli::from_whole(60),
                unit: TimeUnit::Second,
            }),
        ),
        (
            "1.5min",
            HoldLength::Time(Duration {
                value: Milli::from_scaled(1_500),
                unit: TimeUnit::Minute,
            }),
        ),
        (
            "4nm",
            HoldLength::Distance(Distance {
                value: Milli::from_whole(4),
                unit: DistanceUnit::NauticalMile,
            }),
        ),
        (
            "7400m",
            HoldLength::Distance(Distance {
                value: Milli::from_whole(7400),
                unit: DistanceUnit::Metre,
            }),
        ),
    ] {
        let fragment = with("r", &format!("IF,A:41.5/2;HM,A,tR,c90T,l{text}"));
        let link = decode_fragment(&fragment).unwrap_or_else(|refusal| panic!("{text}: {refusal}"));
        assert_eq!(link.route[1].hold_length, Some(length), "{text}");
        assert_eq!(link.fragment().unwrap(), fragment);
    }
}

#[test]
fn a_bare_name_is_the_latest_fix_of_that_name() {
    let fragment = with(
        "r",
        "IF,A:41.5/2;TF,A:41.6/2;TF,B:41.7/2;TF,A;CF,C:41.8/2,c0T,nB;DF,1.5/-2",
    );
    let link = decode_fragment(&fragment).unwrap();
    let at = |index: usize| {
        let fix = link.route[index].fix.as_ref().unwrap();
        (fix.name.clone(), fix.lat.text(0))
    };
    assert_eq!(at(0), (Some("A".to_owned()), "41.5".to_owned()));
    assert_eq!(at(3), (Some("A".to_owned()), "41.6".to_owned()));
    assert_eq!(at(5), (None, "1.5".to_owned()));
    let navaid = link.route[4].navaid.as_ref().unwrap();
    assert_eq!(navaid.lat.text(0), "41.7");
    assert_eq!(link.fragment().unwrap(), fragment);
}

#[test]
fn a_fix_given_twice_at_one_place_is_written_by_name_the_second_time() {
    let link = decode_fragment(&with("r", "IF,A:41.5/2;TF,B:41.6/2;TF,A:41.50/2.0")).unwrap();
    let fragment = link.fragment().unwrap();
    assert!(
        fragment.contains("r=IF,A:41.5/2;TF,B:41.6/2;TF,A&"),
        "{fragment}"
    );
    assert_eq!(decode_fragment(&fragment), Ok(link));
}

#[test]
fn a_route_needs_two_to_three_hundred_legs() {
    let one = refusal(&with("r", "IF,A:41.5/2"));
    assert_eq!(one.reason, Reason::TooFewLegs { count: 1 });
    let legs = |count: usize| {
        let mut legs = vec!["IF,A:41.5/2".to_owned()];
        legs.extend((1..count).map(|_| "TF,A".to_owned()));
        legs.join(";")
    };
    let refused = refusal(&with("r", &legs(MAX_LEGS + 1)));
    assert_eq!(
        (refused.parameter, refused.reason),
        (
            Parameter::R,
            Reason::TooManyLegs {
                count: MAX_LEGS + 1,
                limit: MAX_LEGS
            }
        )
    );
    assert_eq!(
        decode_fragment(&with("r", &legs(MAX_LEGS)))
            .unwrap()
            .route
            .len(),
        MAX_LEGS
    );
}

#[test]
fn a_name_of_thirty_two_characters_is_accepted() {
    let name = "ABCDEFGHIJKLMNOPQRSTUVWXYZ_-0123";
    assert_eq!(name.len(), 32);
    let link = decode_fragment(&with("r", &format!("IF,{name}:41.5/2;TF,{name}"))).unwrap();
    assert_eq!(
        link.route[1].fix.as_ref().unwrap().name.as_deref(),
        Some(name)
    );
}

#[test]
fn every_key_has_a_letter_and_a_meaning() {
    let letters: Vec<char> = Key::ALL.iter().map(|key| key.letter()).collect();
    assert_eq!(letters.iter().collect::<String>(), "omrtagvxschdbnzlp");
    assert!(Key::ALL.iter().all(|key| !key.meaning().is_empty()));
}
