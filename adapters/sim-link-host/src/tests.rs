//! Every link the reference encoder printed, through the whole host path.

use sim_link::decode_url;

use crate::{hosted_request, plan_lines, plan_summary};

const REFERENCE_LINKS: &str = include_str!("../../../crates/sim-link/fixtures/reference-v2.txt");
const OTHER_FORMS: &str =
    include_str!("../../../crates/sim-link/fixtures/reference-v2-other-forms.txt");

#[test]
fn every_reference_link_maps_and_summarises() {
    let links: Vec<&str> = REFERENCE_LINKS
        .lines()
        .chain(OTHER_FORMS.lines())
        .filter(|line| !line.is_empty())
        .collect();
    assert_eq!(links.len(), 9);
    for text in links {
        let link = decode_url(text).unwrap().unwrap();
        let request = hosted_request(&link).unwrap_or_else(|error| panic!("{text}: {error}"));
        let summary =
            plan_summary(&link, &request).unwrap_or_else(|error| panic!("{text}: {error}"));
        assert_eq!(summary.legs.len() + 1, summary.sun.len(), "{text}");
        assert!(
            summary.length_m > 10_000.0 && summary.duration_s > 60.0,
            "{text}"
        );
        let lines = plan_lines(&link, &request, &summary);
        assert!(lines.iter().any(|line| line.label == "Sun"), "{text}");
    }
}

#[test]
fn every_name_the_link_grammar_allows_is_a_waypoint_the_route_takes() {
    let name = "aB_cdefghij-klmnopqrstuvwxyz0123";
    assert_eq!(name.len(), 32);
    let link = decode_url(&format!(
        "http://localhost/#v=2&r=IF,{name}:41.5/2;TF,-:41.6/2.1;TF,_:41.7/2&h=1500&hd=hae\
&t=20260615T120000Z&veh=c172&seed=1&nav=ins&mb=1"
    ))
    .unwrap()
    .unwrap();
    let request = hosted_request(&link).unwrap();
    let summary = plan_summary(&link, &request).unwrap();
    assert_eq!(summary.sun[0].at, name);
    assert_eq!(summary.legs[1].to, "_");
}

/// Holds `fpl_to_simulate_url.py --hold` wrote, on the course it computes
/// for the last leg rounded to a tenth of a degree, over legs east, north
/// (one rounding to 0°), long and far north, south, across the
/// antimeridian, and west: the host takes each as the hold at the last fix.
const CONVERTER_HOLDS: &str = include_str!("../fixtures/converter-holds.txt");

#[test]
fn every_hold_the_converter_writes_is_flown_at_the_last_fix() {
    let links: Vec<&str> = CONVERTER_HOLDS
        .lines()
        .filter(|line| !line.is_empty())
        .collect();
    assert_eq!(links.len(), 7);
    for text in links {
        let link = decode_url(text).unwrap().unwrap();
        let hosted = hosted_request(&link).unwrap_or_else(|error| panic!("{text}: {error}"));
        let hold = hosted.hold.unwrap_or_else(|| panic!("{text}: no hold"));
        let fixes = &hosted.route.fixes;
        let last_course = crate::final_course_deg(&fixes[fixes.len() - 2], &fixes[fixes.len() - 1]);
        let off = (hold.stated.inbound_course_deg - last_course + 540.0).rem_euclid(360.0) - 180.0;
        assert!(off.abs() <= 0.05 + 1e-9, "{text}: {off}");
        let spec = crate::chain_spec(&link, &hosted, None).unwrap();
        assert_eq!(
            spec.hold.map(|hold| hold.leg_time_s),
            Some(hold.stated.leg_time_s)
        );
    }
}
