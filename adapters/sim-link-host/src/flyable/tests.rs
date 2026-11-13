use sim_chain_trial::{
    ChainAltitudeSpec, ChainHoldLengthSpec, ChainLegRole, ChainPathSpec, ChainTrial, FlightPhase,
    HoldTurn,
};
use sim_link::{PathTerminator, SimulationLink, TurnDirection, decode_fragment};
use sim_navigate::trial::DirectoryStores;

use crate::{
    FlyableAltitude, FlyableLegRole, FlyablePath, HostError, chain_spec, flyable_route,
    hosted_request, package_route_msl, plan_summary,
};

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

    let parsed = link(SQUARE);
    let hosted = hosted_request(&parsed).unwrap();
    let spec = chain_spec(&parsed, &hosted, None).unwrap();
    assert!(
        spec.planned_legs.is_empty(),
        "plain IF/TF route omits planned_legs so trial digests stay unchanged"
    );
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
    assert_eq!(hold.turn, HoldTurn::Right);
    assert_eq!(hold.altitude_m, None);
}

#[test]
fn a_link_without_a_hold_leaves_the_standard_hold() {
    let hosted = hosted_request(&link(SQUARE)).unwrap();
    assert_eq!(hosted.hold, None);
    assert_eq!(chain_spec(&link(SQUARE), &hosted, None).unwrap().hold, None);
}

#[test]
fn cifp_cf_ca_df_holds_windows_magnetic_courses_and_forced_turns_are_flown() {
    let procedure = concat!(
        "IF,IAF1:41.70/-1.05,rIAF,a2200+;",
        "HF,IAF1,tR,c95M,l4nm;",
        "CF,FAF1:41.70/-0.92,rFAF,a1800..2100,c95M;",
        "CF,RW09:41.70/-0.82,rMAP.RWY,tL,a1400,c95M,p0.3nm;",
        "CA,m,a1900+,c95M;",
        "DF,MAHP1:41.76/-0.76,rMAHF,a2200-;",
        "HA,MAHP1,tL,a2400+,c275M,l1min;",
        "HM,MAHP1,tL,a2400+,c275M,l1min"
    );
    let parsed = link(procedure);
    let route = flyable_route(&parsed).unwrap();
    assert_eq!(route.fixes.len(), 8);
    assert_eq!(route.hold, None);
    assert_eq!(route.fixes[0].leg_role, Some(FlyableLegRole::Arrival));
    assert_eq!(route.fixes[2].leg_role, Some(FlyableLegRole::Arrival));
    assert_eq!(route.fixes[3].leg_role, Some(FlyableLegRole::Approach));
    assert_eq!(route.fixes[3].rnp_nm, Some(0.3));
    assert_eq!(route.fixes[3].turn_direction, Some(TurnDirection::Left));
    assert_eq!(
        route.fixes[4].leg_role,
        Some(FlyableLegRole::MissedApproach)
    );
    assert!(matches!(
        route.fixes[2].altitude,
        Some(FlyableAltitude::Window {
            lower_m: 1800.0,
            upper_m: 2100.0
        })
    ));
    assert!(matches!(
        route.fixes[4].path,
        FlyablePath::CourseToAltitude { magnetic: true, .. }
    ));

    let corridor = package_route_msl(&parsed).unwrap();
    assert!(
        corridor
            .windows(2)
            .all(|w| { (w[0][0] - w[1][0]).hypot(w[0][1] - w[1][1]) > 1e-5 })
    );
    let hosted = hosted_request(&parsed).unwrap();
    let summary = plan_summary(&parsed, &hosted).unwrap();
    assert!(summary.length_m > 5_000.0);

    let spec = chain_spec(&parsed, &hosted, None).unwrap();
    assert_eq!(spec.planned_legs.len(), 8);
    assert_eq!(spec.planned_legs[0].role, Some(ChainLegRole::Arrival));
    assert_eq!(spec.planned_legs[3].role, Some(ChainLegRole::Approach));
    assert_eq!(
        spec.planned_legs[4].role,
        Some(ChainLegRole::MissedApproach)
    );
    assert!(matches!(
        spec.planned_legs[1].path,
        ChainPathSpec::HoldToFix { hold }
            if hold.magnetic && matches!(hold.length, ChainHoldLengthSpec::DistanceM { .. })
    ));
    assert!(matches!(
        spec.planned_legs[2].altitude,
        ChainAltitudeSpec::Window { .. }
    ));

    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    trial.step_until(5.0).unwrap();
    let display = trial.navigation().expect("Pilotage navigation display");
    assert_eq!(display.phase, FlightPhase::Terminal);
    assert!((display.rnp_m - 1_852.0).abs() < 1e-6);
}

#[test]
fn approach_leg_activates_pilotage_approach_phase_and_rnp_scale() {
    let parsed = link("IF,FAF1:41.70/-1.05,rFAF,a1800;CF,RW09:41.70/-0.90,rMAP.RWY,a1400,c90T");
    let hosted = hosted_request(&parsed).unwrap();
    let spec = chain_spec(&parsed, &hosted, None).unwrap();
    let mut trial = ChainTrial::new(&spec, &DirectoryStores).unwrap();
    trial.step_until(5.0).unwrap();
    let display = trial.navigation().expect("Pilotage navigation display");
    assert_eq!(display.phase, FlightPhase::Approach);
    assert!((display.rnp_m - 0.3 * 1_852.0).abs() < 1e-6);
}

#[test]
fn unflown_legs_and_attributes_are_still_refused_by_number_and_code() {
    for (route, leg, code, words) in [
        (
            "IF,A:41.7/-1.05;RF,B:41.7/-0.8,tL,zC:41.6/-0.9",
            2,
            "RF",
            "arc",
        ),
        (
            "IF,A:41.7/-1.05;AF,B:41.7/-0.8,tR,d5nm,b180T,nV:41.6/-0.9",
            2,
            "AF",
            "arc",
        ),
        (
            "IF,A:41.7/-1.05;PI,A,tR,c90T,d10nm;TF,B:41.7/-0.8",
            2,
            "PI",
            "procedure-turn",
        ),
        ("IF,A:41.7/-1.05;TF,B:41.7/-0.8,s60", 2, "TF", "speed"),
        ("IF,A:41.7/-1.05;TF,B:41.7/-0.8,v-3", 2, "TF", "vertical"),
        (
            "IF,A:41.7/-1.05,aFL80;TF,B:41.7/-0.8",
            1,
            "IF",
            "flight levels",
        ),
        (
            &format!("{SQUARE};HM,ZA3,tR,c0T,l1min,s50"),
            4,
            "HM",
            "speed",
        ),
        (
            &format!("{SQUARE};HM,ZA3,tR,c0T,l1min,aFL80"),
            4,
            "HM",
            "flight levels",
        ),
    ] {
        let found = refusal(route);
        assert_eq!((found.0, found.1), (leg, code), "{route}: {}", found.2);
        assert!(found.2.contains(words), "{route}: {}", found.2);
    }
    let (_, _, reason) = refusal("IF,A:41.7/-1.05;HM,A,tR,c0T,l1min");
    assert!(reason.contains("two fixes"), "{reason}");
}

#[test]
fn a_refusal_names_the_leg_its_fix_and_says_the_link_is_valid() {
    let refused =
        flyable_route(&link("IF,A:41.7/-1.05;RF,B:41.7/-0.8,tL,zC:41.6/-0.9")).unwrap_err();
    let words = refused.to_string();
    assert!(
        words.starts_with("leg 2 (RF B): radius-to-fix arc legs are not flown yet"),
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
    let c172s =
        decode_fragment(&format!("v=2&r={SQUARE}{}", REST.replace("c172", "c172s"))).unwrap();
    let hosted_s = hosted_request(&c172s).unwrap();
    assert_eq!(
        chain_spec(&c172s, &hosted_s, None).unwrap().variant,
        sim_chain_trial::ChainC172Variant::C172s
    );
    let c172p =
        decode_fragment(&format!("v=2&r={SQUARE}{}", REST.replace("c172", "c172p"))).unwrap();
    let hosted_p = hosted_request(&c172p).unwrap();
    assert_eq!(
        chain_spec(&c172p, &hosted_p, None).unwrap().variant,
        sim_chain_trial::ChainC172Variant::C172p
    );
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
