//! Every link the reference encoder printed, through the whole host path.

use sim_link::decode_url;

use crate::{hosted_request, plan_lines, plan_summary};

const REFERENCE_LINKS: &str = include_str!("../../../crates/sim-link/fixtures/reference-v1.txt");
const OTHER_FORMS: &str =
    include_str!("../../../crates/sim-link/fixtures/reference-v1-other-forms.txt");

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
        "http://localhost/#v=1&r=41.5,2.0,,,{name};41.6,2.1,,,-;41.7,2.0,,,_&h=1500&hd=hae\
&t=20260615T120000Z&veh=c172&seed=1&nav=ins&mb=1"
    ))
    .unwrap()
    .unwrap();
    let request = hosted_request(&link).unwrap();
    let summary = plan_summary(&link, &request).unwrap();
    assert_eq!(summary.sun[0].at, name);
    assert_eq!(summary.legs[1].to, "_");
}
