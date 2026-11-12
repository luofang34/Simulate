use super::super::*;
use crate::geodesy::destination;

/// Metres between two `[lat, lon]` points a few kilometres apart.
fn metres([lat_a, lon_a]: [f64; 2], [lat_b, lon_b]: [f64; 2]) -> f64 {
    let north = (lat_b - lat_a).to_radians() * 6_371_008.8;
    let east = (lon_b - lon_a).to_radians() * 6_371_008.8 * lat_a.to_radians().cos();
    north.hypot(east)
}

fn hold(right_turns: bool) -> HoldPattern {
    HoldPattern {
        fix_deg: [41.74, -1.05],
        inbound_course_deg: 0.0,
        right_turns,
        turn_radius_m: 1_000.0,
        leg_length_m: 4_000.0,
    }
}

#[test]
fn the_racetrack_closes_and_its_inbound_leg_ends_at_the_fix() {
    let pattern = hold(true);
    let track = pattern.racetrack();
    let (first, last) = (track[0], track[track.len() - 1]);
    assert!(metres(first, last) < 1.0, "closed: {first:?} {last:?}");
    assert!(
        metres(track[1], pattern.fix_deg) < 1.0,
        "inbound leg ends at the fix"
    );
    assert!((metres(first, pattern.fix_deg) - 4_000.0).abs() < 1.0);
    // The far end of the turn past the fix lies one turn radius beyond it.
    let beyond = track
        .iter()
        .map(|p| p[0] - pattern.fix_deg[0])
        .fold(f64::MIN, f64::max)
        .to_radians()
        * 6_371_008.8;
    assert!((beyond - 1_000.0).abs() < 5.0, "{beyond}");
}

#[test]
fn right_turns_hold_on_the_right_of_the_inbound_course_and_left_turns_on_the_left() {
    // Inbound due north: the right is east.
    let fix = hold(true).fix_deg;
    let east_of_fix = |p: &[f64; 2]| p[1] > fix[1] + 1e-9;
    let west_of_fix = |p: &[f64; 2]| p[1] < fix[1] - 1e-9;
    assert!(hold(true).racetrack().iter().all(|p| !west_of_fix(p)));
    assert!(hold(true).racetrack().iter().any(east_of_fix));
    assert!(hold(false).racetrack().iter().all(|p| !east_of_fix(p)));
    // The outbound leg runs two turn radii off the inbound line.
    let [lon, lat] = destination([fix[1], fix[0]], 90.0, 2_000.0);
    let nearest = hold(true)
        .racetrack()
        .into_iter()
        .map(|p| metres(p, [lat, lon]))
        .fold(f64::MAX, f64::min);
    assert!(nearest < 5.0, "{nearest}");
}

#[test]
fn a_held_plan_draws_its_racetrack_as_the_active_part() {
    let plan = FlightPlan {
        origin: None,
        fixes: vec![RouteFix {
            id: FixId(0),
            ident: "ZA5".to_owned(),
            lat_deg: 41.74,
            lon_deg: -1.05,
            altitude_m: None,
        }],
    };
    let mut progress = RouteProgress::after_sequenced(Some((0, true)));
    assert!(!plan.geojson(progress).to_string().contains("\"hold\""));
    progress.hold = Some(hold(true));
    let document = plan.geojson(progress);
    let holds: Vec<&serde_json::Value> = document["features"]
        .as_array()
        .expect("features")
        .iter()
        .filter(|f| f["properties"]["kind"] == "hold")
        .collect();
    assert_eq!(holds.len(), 1);
    assert_eq!(holds[0]["properties"]["state"], "active");
    assert_eq!(holds[0]["geometry"]["type"], "LineString");
}
