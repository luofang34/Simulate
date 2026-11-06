use serde_json::Value;

use super::features::{airport_features, runway_features};
use super::{GlobeAirportMarker, GlobeRunwayEndOverlay};

fn marker(code: &str, towered: bool, selected: bool, tier: u8) -> GlobeAirportMarker {
    GlobeAirportMarker {
        code: code.to_owned(),
        label: code.to_owned(),
        lat_deg: 40.69,
        lon_deg: -74.17,
        tier,
        towered,
        selected,
    }
}

fn features(collection: &Value) -> &Vec<Value> {
    collection["features"].as_array().unwrap()
}

#[test]
fn markers_are_classed_by_selection_then_tower_and_selected_ones_come_last() {
    let collection = airport_features(&[
        marker("KEWR", true, true, 1),
        marker("47N", false, false, 9),
        marker("KTEB", true, false, 2),
    ]);
    let kinds: Vec<&str> = features(&collection)
        .iter()
        .map(|f| f["properties"]["kind"].as_str().unwrap())
        .collect();
    assert_eq!(kinds, ["untowered", "towered", "selected"]);
    assert_eq!(features(&collection)[0]["properties"]["tier"], 4);
}

#[test]
fn non_finite_markers_are_dropped() {
    let mut bad = marker("NAN", false, false, 1);
    bad.lat_deg = f64::NAN;
    assert!(features(&airport_features(&[bad])).is_empty());
}

#[test]
fn a_selected_runway_end_gets_a_five_mile_feather_before_its_threshold_and_a_bar() {
    let end = GlobeRunwayEndOverlay {
        designator: "22L".to_owned(),
        threshold_lat_deg: 40.7,
        threshold_lon_deg: -74.17,
        opposite_lat_deg: 40.67,
        opposite_lon_deg: -74.19,
        width_m: 45.0,
        selected: true,
    };
    let unselected = GlobeRunwayEndOverlay {
        selected: false,
        ..end.clone()
    };
    let collection = runway_features(&[end, unselected]);
    let kinds: Vec<&str> = features(&collection)
        .iter()
        .map(|f| f["properties"]["kind"].as_str().unwrap())
        .collect();
    assert_eq!(kinds, ["pavement", "feather", "threshold", "pavement"]);
    let feather = &features(&collection)[1]["geometry"]["coordinates"];
    let start_lat = feather[0][1].as_f64().unwrap();
    // The runway runs south-west from the threshold, so the approach comes from the north-east.
    assert!(
        start_lat > 40.7 + 0.05,
        "feather starts north of the threshold: {start_lat}"
    );
}

#[test]
fn polar_markers_and_runways_bypass_mercator_geojson_for_overlay_rendering() {
    let nzsp = GlobeAirportMarker {
        code: "NZSP".to_owned(),
        label: "NZSP".to_owned(),
        lat_deg: -89.9975,
        lon_deg: 139.2729,
        tier: 1,
        towered: true,
        selected: true,
    };
    let rwy_02t = GlobeRunwayEndOverlay {
        designator: "02T".to_owned(),
        threshold_lat_deg: -89.9975,
        threshold_lon_deg: 139.2729,
        opposite_lat_deg: -89.9666,
        opposite_lon_deg: 145.67,
        width_m: 46.0,
        selected: true,
    };
    assert!(features(&airport_features(std::slice::from_ref(&nzsp))).is_empty());
    assert!(features(&runway_features(std::slice::from_ref(&rwy_02t))).is_empty());
    assert!(super::polar_overlay::is_polar_marker(&nzsp));
    assert!(super::polar_overlay::is_polar_runway(&rwy_02t));
}
