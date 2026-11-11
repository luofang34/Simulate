use sim_link::{SimulationLink, decode_fragment};

use crate::{Light, PlanSummary, hosted_request, plan_summary};

const ROUTE: &str = "v=1&r=41.666242,-1.041553,,,LEZG;41.750000,-0.800000,,,Z1;41.600000,-0.620000,,,Z2\
&h=2000&hd=msl";
const REST: &str = "&veh=c172&seed=1&nav=ins.baro&mb=200";

fn summary(fragment: &str) -> (SimulationLink, PlanSummary) {
    let link = decode_fragment(fragment).unwrap();
    let request = hosted_request(&link).unwrap();
    let summary = plan_summary(&link, &request).unwrap();
    (link, summary)
}

/// Great-circle distance on the mean sphere, metres: an independent check
/// of the geodesic lengths, good to half a percent.
fn sphere_m(from: (f64, f64), to: (f64, f64)) -> f64 {
    let (lat1, lon1, lat2, lon2) = (
        from.0.to_radians(),
        from.1.to_radians(),
        to.0.to_radians(),
        to.1.to_radians(),
    );
    let h = ((lat2 - lat1) / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * ((lon2 - lon1) / 2.0).sin().powi(2);
    2.0 * 6_371_008.8 * h.sqrt().asin()
}

#[test]
fn legs_follow_the_route_at_the_cruise_speed() {
    let (_, summary) = summary(&format!("{ROUTE}&t=20260615T120000Z{REST}"));
    let names: Vec<(&str, &str)> = summary
        .legs
        .iter()
        .map(|leg| (leg.from.as_str(), leg.to.as_str()))
        .collect();
    assert_eq!(names, [("LEZG", "Z1"), ("Z1", "Z2")]);
    let expected = [
        sphere_m((41.666242, -1.041553), (41.75, -0.8)),
        sphere_m((41.75, -0.8), (41.6, -0.62)),
    ];
    for (leg, expected) in summary.legs.iter().zip(expected) {
        assert!(
            (leg.length_m / expected - 1.0).abs() < 0.005,
            "{} vs {expected}",
            leg.length_m
        );
        assert_eq!(leg.speed_m_s, 55.0);
        assert_eq!(leg.duration_s, leg.length_m / 55.0);
        // One height above mean sea level is two heights above the
        // ellipsoid: the geoid slopes a little between the waypoints.
        assert!(leg.altitude_change_m != 0.0 && leg.altitude_change_m.abs() < 1.0);
    }
    assert_eq!(
        summary.length_m,
        summary.legs[0].length_m + summary.legs[1].length_m
    );
    assert!((summary.duration_s - summary.length_m / 55.0).abs() < 1e-9);
    assert!(
        summary.length_m > 40_000.0 && summary.length_m < 45_000.0,
        "{}",
        summary.length_m
    );
}

#[test]
fn a_start_position_adds_the_leg_to_the_first_waypoint() {
    let (_, without) = summary(&format!("{ROUTE}&t=20260615T120000Z{REST}"));
    let (_, with) = summary(&format!(
        "{ROUTE}&start=41.666242,-1.241553&t=20260615T120000Z{REST}"
    ));
    assert_eq!(with.legs.len(), without.legs.len() + 1);
    let first = &with.legs[0];
    assert_eq!((first.from.as_str(), first.to.as_str()), ("START", "LEZG"));
    let expected = sphere_m((41.666242, -1.241553), (41.666242, -1.041553));
    assert!((first.length_m / expected - 1.0).abs() < 0.005);
    assert_eq!(with.sun.len(), 4);
    assert_eq!(with.sun[0].time_s, 0.0);
    assert_eq!(with.sun[1].time_s, first.duration_s);
}

#[test]
fn the_sun_is_high_at_a_june_noon_over_zaragoza() {
    let (_, summary) = summary(&format!("{ROUTE}&t=20260615T120000Z{REST}"));
    assert_eq!(summary.sun.len(), 3);
    assert_eq!(summary.sun[0].time_s, 0.0);
    assert!(summary.sun[2].time_s > summary.sun[1].time_s);
    // Solar noon there is near 12:04 UTC; the noon elevation is 90° minus the
    // latitude plus the Sun's declination, about 71.6°.
    for sample in &summary.sun {
        assert!((70.0..73.0).contains(&sample.elevation_deg), "{sample:?}");
        assert_eq!(sample.light(), Light::Day);
    }
    assert_eq!(summary.light_range(), Some((Light::Day, Light::Day)));
}

#[test]
fn the_sky_is_dark_half_a_day_away() {
    let (_, summary) = summary(&format!("{ROUTE}&t=20260615T003000Z{REST}"));
    for sample in &summary.sun {
        // At local midnight the Sun is 90° minus the latitude minus the
        // declination below the horizon, about 25°.
        assert!((-27.0..-22.0).contains(&sample.elevation_deg), "{sample:?}");
        assert_eq!(sample.light(), Light::Night);
    }
}

#[test]
fn the_sun_moves_with_the_planned_time_of_passing() {
    // Dawn over Zaragoza in June is near 04:30 UTC: the Sun climbs about a
    // sixth of a degree a minute, so it is higher at each later waypoint of
    // a slow flight eastward.
    let fragment = format!("{ROUTE}&t=20260615T043000Z&veh=c172&spd=30&seed=1&nav=ins.baro&mb=200");
    let (_, summary) = summary(&fragment);
    assert!(
        summary.sun[1].elevation_deg > summary.sun[0].elevation_deg + 0.5,
        "{:?}",
        summary.sun
    );
    assert!(summary.sun[2].elevation_deg > summary.sun[1].elevation_deg + 0.5);
}
