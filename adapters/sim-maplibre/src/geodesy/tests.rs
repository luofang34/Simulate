use super::*;

#[test]
fn a_destination_due_east_on_the_equator_moves_only_longitude() {
    let [lon, lat] = destination([0.0, 0.0], 90.0, METRES_PER_NM * 60.0);
    assert!(lat.abs() < 1e-9, "{lat}");
    assert!((lon - 1.0).abs() < 2e-3, "60 NM is about a degree: {lon}");
}

#[test]
fn the_bearing_back_from_a_destination_is_the_reciprocal() {
    let from = [-74.17, 40.69];
    let to = destination(from, 220.0, 9.0 * METRES_PER_NM);
    let out = initial_bearing_deg(from, to);
    let back = initial_bearing_deg(to, from);
    assert!((out - 220.0).abs() < 1e-6, "{out}");
    assert!(
        ((back - out).rem_euclid(360.0) - 180.0).abs() < 0.1,
        "{back}"
    );
}

#[test]
fn longitudes_wrap_across_the_antimeridian() {
    let [lon, _] = destination([179.9, 0.0], 90.0, 50_000.0);
    assert!((-180.0..-179.0).contains(&lon), "{lon}");
}

#[test]
fn destinations_cross_the_south_pole_without_singularity() {
    let [due_south_lon, due_south_lat] =
        destination([139.2729, -89.9975], 180.0, 3.0 * METRES_PER_NM);
    assert!(due_south_lat.is_finite() && due_south_lon.is_finite());
    assert!(
        due_south_lat > -90.0 && due_south_lat < -89.9,
        "{due_south_lat}"
    );
    assert!((due_south_lon - (-40.7271)).abs() < 1e-3, "{due_south_lon}");

    let [oblique_lon, oblique_lat] = destination([139.2729, -89.9975], 200.0, 3.0 * METRES_PER_NM);
    assert!(oblique_lat > -90.0 && (oblique_lon - (-19.7)).abs() < 0.2);

    let [pole_lon, pole_lat] = destination([0.0, -90.0], 45.0, METRES_PER_NM);
    assert!((pole_lon - 45.0).abs() < 1e-6 && pole_lat > -90.0);
}
