use sim_link::{PathTerminator, SimulationLink, TurnDirection, decode_fragment};

use crate::{HostError, chain_spec, flyable_route, hosted_request};

/// The parameters after `r`, for a link that only changes its legs.
const REST: &str = "&h=2000&hd=hae&t=20260615T120000Z&veh=c172&seed=1&nav=ins.baro&mb=200";

/// A square over land with a final course of exactly 0° true.
const SQUARE: &str = "IF,ZA1:41.7/-1.05;TF,ZA2:41.7/-0.8,o;TF,ZA3:41.8/-0.8,a1500";

fn link(route: &str) -> SimulationLink {
    decode_fragment(&format!("v=2&r={route}{REST}"))
        .unwrap_or_else(|refusal| panic!("{route}: {refusal}"))
}

fn refusal(route: &str) -> (usize, &'static str, String) {
    match flyable_route(&link(route)) {
        Err(HostError::Unflyable {
            leg, code, reason, ..
        }) => (leg, code, reason),
        other => panic!("{route}: {other:?}"),
    }
}

#[test]
fn fixes_keep_their_order_names_turns_and_heights() {
    let route = flyable_route(&link(SQUARE)).unwrap();
    let names: Vec<_> = route.fixes.iter().map(|fix| fix.name.as_deref()).collect();
    assert_eq!(names, [Some("ZA1"), Some("ZA2"), Some("ZA3")]);
    let over: Vec<bool> = route.fixes.iter().map(|fix| fix.fly_over).collect();
    assert_eq!(over, [false, true, false]);
    assert_eq!(route.fixes[2].height_m, Some(1500.0));
    assert_eq!(route.fixes[1].height_m, None);
    assert_eq!(route.hold, None);
}

#[test]
fn a_height_in_feet_is_flown_in_metres() {
    let route = flyable_route(&link("IF,A:41.7/-1.05,a5000ft;TF,B:41.7/-0.8")).unwrap();
    assert_eq!(route.fixes[0].height_m, Some(5000.0 * 0.3048));
}

#[test]
fn a_hold_at_the_last_fix_on_the_last_course_is_flown() {
    let route = flyable_route(&link(&format!("{SQUARE};HM,ZA3,tL,a1600,c0.5T,l90s"))).unwrap();
    let hold = route.hold.unwrap();
    assert_eq!(hold.turn, TurnDirection::Left);
    assert_eq!(
        (hold.leg_time_s, hold.inbound_course_deg, hold.height_m),
        (90.0, 0.5, Some(1600.0))
    );
    assert_eq!(route.fixes.len(), 3);

    let hosted = hosted_request(&link(&format!("{SQUARE};HM,ZA3,tR,c0T,l1.5min"))).unwrap();
    let spec = chain_spec(&link(SQUARE), &hosted, None).unwrap();
    let hold = spec.hold.unwrap();
    assert_eq!(hold.leg_time_s, 90.0);
    assert_eq!(hold.turn, sim_chain_trial::HoldTurn::Right);
    assert_eq!(hold.altitude_m, None);
}

#[test]
fn a_link_without_a_hold_leaves_the_standard_hold() {
    let hosted = hosted_request(&link(SQUARE)).unwrap();
    assert_eq!(hosted.hold, None);
    assert_eq!(chain_spec(&link(SQUARE), &hosted, None).unwrap().hold, None);
}

#[test]
fn a_hold_the_mission_cannot_fly_as_stated_is_refused() {
    for (hold, words) in [
        ("HM,ZA2,tR,c0T,l1min", "only at the last fix"),
        ("HM,ZA3,tR,c90T,l1min", "last leg's course"),
        ("HM,ZA3,tR,c0M,l1min", "magnetic"),
        ("HM,ZA3,tR,c0T,l4nm", "distance"),
        ("HM,ZA3,tR,c0T,l1min,s50", "speed"),
        ("HM,ZA3,tR,c0T,l1min,a1500+", "only an altitude to cross at"),
    ] {
        let (leg, code, reason) = refusal(&format!("{SQUARE};{hold}"));
        assert_eq!((leg, code), (4, "HM"), "{hold}");
        assert!(reason.contains(words), "{hold}: {reason}");
    }
    let (_, _, reason) = refusal("IF,A:41.7/-1.05;HM,A,tR,c0T,l1min");
    assert!(reason.contains("two fixes"), "{reason}");
}

#[test]
fn a_leg_or_attribute_not_flown_yet_is_refused_by_number_and_code() {
    for (route, leg, code, words) in [
        (
            "IF,A:41.7/-1.05;CF,B:41.7/-0.8,c90T",
            2,
            "CF",
            "course-to-fix",
        ),
        ("IF,A:41.7/-1.05;TF,B:41.7/-0.8;DF,A", 3, "DF", "direct-to"),
        (
            "IF,A:41.7/-1.05;RF,B:41.7/-0.8,tL,zC:41.6/-0.9",
            2,
            "RF",
            "arc",
        ),
        (
            "IF,A:41.7/-1.05;CA,a300+,c40T;TF,B:41.7/-0.8",
            2,
            "CA",
            "course-to-altitude",
        ),
        ("IF,A:41.7/-1.05;TF,B:41.7/-0.8,s60", 2, "TF", "speed"),
        (
            "IF,A:41.7/-1.05;TF,B:41.7/-0.8,tL",
            2,
            "TF",
            "turn direction",
        ),
        ("IF,A:41.7/-1.05;TF,B:41.7/-0.8,v-3", 2, "TF", "vertical"),
        (
            "IF,A:41.7/-1.05,aFL80;TF,B:41.7/-0.8",
            1,
            "IF",
            "flight levels",
        ),
        (
            "IF,A:41.7/-1.05,a100..200;TF,B:41.7/-0.8",
            1,
            "IF",
            "window",
        ),
    ] {
        let found = refusal(route);
        assert_eq!((found.0, found.1), (leg, code), "{route}: {}", found.2);
        assert!(found.2.contains(words), "{route}: {}", found.2);
    }
}

#[test]
fn a_refusal_names_the_leg_its_fix_and_says_the_link_is_valid() {
    let refused = flyable_route(&link("IF,A:41.7/-1.05;CF,B:41.7/-0.8,c90T")).unwrap_err();
    let words = refused.to_string();
    assert!(
        words.starts_with("leg 2 (CF B): course-to-fix legs are not flown yet"),
        "{words}"
    );
    assert!(words.contains("the link is valid"), "{words}");
}

#[test]
fn reference_data_and_roles_do_not_stop_a_track_to_a_fix() {
    let route = flyable_route(&link(
        "DF,A:41.7/-1.05,rIAF;TF,B:41.7/-0.8,m,rMAP,b90T,d5nm,nV:41.6/-0.9,p0.3nm",
    ))
    .unwrap();
    assert_eq!(route.fixes.len(), 2);
    assert_eq!(route.fixes[1].roles.len(), 1);
    assert_eq!(
        link("DF,A:41.7/-1.05;TF,B:41.7/-0.8").route[0].path,
        PathTerminator::Df
    );
}

#[test]
fn the_page_flies_only_the_c172_at_its_cruise_speeds() {
    let x500 = decode_fragment(&format!("v=2&r={SQUARE}{}", REST.replace("c172", "x500"))).unwrap();
    let hosted = hosted_request(&x500).unwrap();
    assert!(matches!(
        chain_spec(&x500, &hosted, None),
        Err(HostError::ChainVehicle { .. })
    ));
    for speed in ["29.9", "120.1"] {
        let slow = decode_fragment(&format!("v=2&r={SQUARE}{REST}&spd={speed}")).unwrap();
        let hosted = hosted_request(&slow).unwrap();
        assert!(
            matches!(
                chain_spec(&slow, &hosted, None),
                Err(HostError::ChainVehicle { .. })
            ),
            "{speed}"
        );
    }
}
