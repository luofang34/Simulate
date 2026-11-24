//! Recorded state of the carried spacecraft after its five revolutions and
//! after it comes back to the local tier.

use super::*;

/// Fails with every `(name, now, recorded, tolerance)` row that left its
/// tolerance, each printed in full.
#[cfg(test)]
fn assert_rows(rows: &[(&str, f64, f64, f64)]) {
    let off: Vec<String> = rows
        .iter()
        .filter(|(_, now, recorded, tolerance)| {
            let error = (now - recorded).abs();
            error.is_nan() || error > *tolerance
        })
        .map(|(name, now, recorded, tolerance)| {
            format!("{name}: {now:?} against {recorded:?} (tolerance {tolerance:e})")
        })
        .collect();
    assert!(off.is_empty(), "\n{}", off.join("\n"));
}

#[test]
fn the_carried_flight_ends_where_it_was_recorded() {
    let launch = launch();
    let mut universe = universe_with(&launch, &Rc::new(Cell::new(0)), true, true);
    universe.advance_to(seconds(DEMOTED_S)).unwrap();
    let aloft = internals(&universe);
    let orbit = universe.vehicle_state(SHIP).unwrap();
    universe.demote_to_local(SHIP).unwrap();
    universe.advance_to(seconds(DEMOTED_S + 5)).unwrap();
    let back = internals(&universe);
    let local = universe.vehicle_state(SHIP).unwrap();
    // The vessel's own block flies the first ten seconds and the last five;
    // in between the orbital tier integrates the thrust the carried
    // internals report. Integration truncation over those ten seconds of a
    // 7.7 km/s orbit is micrometres, which five revolutions stretch along
    // the track.
    assert_rows(&[
        ("aloft_tank_kg", aloft.tank_kg, 19.860298208412942, 1e-9),
        ("aloft_battery_wh", aloft.battery_wh, 600.0, 1e-6),
        (
            "aloft_temperature_k",
            aloft.temperature_k,
            265.2739090566565,
            1e-6,
        ),
        ("orbit_px", orbit.position_ecef.x, -2642806.449817557, 1e-2),
        ("orbit_py", orbit.position_ecef.y, 6141144.317798923, 1e-2),
        ("orbit_pz", orbit.position_ecef.z, -1162273.1337914562, 1e-2),
        ("orbit_vx", orbit.velocity_ecef.x, 617.7919555536288, 1e-5),
        ("orbit_vy", orbit.velocity_ecef.y, 1687.7520351740657, 1e-5),
        ("orbit_vz", orbit.velocity_ecef.z, 7515.846512738615, 1e-5),
        ("back_tank_kg", back.tank_kg, 19.860267013778298, 1e-9),
        ("back_thrust_n", back.thrust_n, 0.12236594555735139, 1e-9),
        ("local_px", local.position_ecef.x, -2639672.4385853084, 1e-2),
        ("local_py", local.position_ecef.y, 6149484.265621668, 1e-2),
        ("local_pz", local.position_ecef.z, -1124675.4906987264, 1e-2),
        ("local_vx", local.velocity_ecef.x, 635.8044520574708, 1e-5),
        ("local_vy", local.velocity_ecef.y, 1648.2159693425178, 1e-5),
        ("local_vz", local.velocity_ecef.z, 7523.170612640122, 1e-5),
    ]);
}
