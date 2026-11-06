use super::*;
// Datum conversions are tested here because they are defined by the geoid.
use crate::{Geodetic, OrthometricHeightM, ellipsoidal_to_msl_egm96, msl_egm96_to_ellipsoidal};

fn at(lat_deg: f64, lon_deg: f64) -> f64 {
    egm96_undulation_m(lat_deg.to_radians(), lon_deg.to_radians())
}

#[test]
fn the_us_east_coast_sits_about_33_m_below_the_ellipsoid() {
    for (name, lat, lon) in [("KDOV", 39.13, -75.47), ("KEWR", 40.69, -74.17)] {
        let n = at(lat, lon);
        assert!((-37.0..=-30.0).contains(&n), "{name}: {n}");
    }
}

#[test]
fn the_indian_ocean_low_is_deep() {
    let n = at(4.5, 78.5);
    assert!(n < -95.0, "{n}");
}

#[test]
fn the_new_guinea_and_north_atlantic_highs_stand_up() {
    assert!(at(-6.0, 147.0) > 60.0, "{}", at(-6.0, 147.0));
    assert!(at(60.0, -25.0) > 50.0, "{}", at(60.0, -25.0));
}

#[test]
fn the_grid_nodes_are_exact() {
    // Row 10 is 40°N; column 57 is 285°E = 75°W.
    let node = f64::from(EGM96_DECIMETRES[10 * COLUMNS + 57]) * 0.1;
    assert_eq!(at(40.0, -75.0), node);
    assert_eq!(at(40.0, 285.0), node);
}

#[test]
fn the_antimeridian_is_continuous() {
    let east = at(12.3, 179.99);
    let west = at(12.3, -179.99);
    assert!((east - west).abs() < 0.05, "{east} vs {west}");
    assert!((at(12.3, 180.0) - at(12.3, -180.0)).abs() < 1e-9);
}

#[test]
fn each_pole_has_one_value_and_latitude_clamps_past_it() {
    for pole in [90.0, -90.0] {
        let reference = at(pole, 0.0);
        for lon in [-170.0, -45.0, 33.0, 120.0] {
            assert!((at(pole, lon) - reference).abs() < 1e-9);
        }
        assert_eq!(at(pole * 1.0001, 10.0), at(pole, 10.0));
    }
    assert!((at(90.0, 0.0) - 13.6).abs() < 0.1);
    assert!((at(-90.0, 0.0) + 29.5).abs() < 0.2);
}

#[test]
fn non_finite_input_is_zero_not_nan() {
    assert_eq!(egm96_undulation_m(f64::NAN, 0.0), 0.0);
    assert_eq!(egm96_undulation_m(0.0, f64::INFINITY), 0.0);
}

#[test]
fn the_field_is_bounded_like_the_real_geoid() {
    for lat in (-90..=90).step_by(3) {
        for lon in (-180..180).step_by(7) {
            let n = at(f64::from(lat), f64::from(lon));
            assert!((-110.0..=90.0).contains(&n), "{lat},{lon}: {n}");
        }
    }
}

#[test]
fn msl_and_ellipsoidal_heights_round_trip_exactly() {
    for (lat, lon, h) in [
        (39.13, -75.47, 250.0),
        (4.5, 78.5, -12.0),
        (-6.0, 147.0, 3_000.0),
        (89.9, 179.99, 0.0),
        (-90.0, -179.99, 11_000.0),
    ] {
        let position = Geodetic::from_degrees(lat, lon, h);
        let msl = ellipsoidal_to_msl_egm96(&position);
        let back = msl_egm96_to_ellipsoidal(position.lat_rad, position.lon_rad, msl);
        assert!((back.0 - position.height_m).abs() <= 1e-12, "{lat},{lon}");
    }
}

#[test]
fn msl_is_above_the_ellipsoid_where_the_geoid_is_below_it() {
    let dover = Geodetic::from_degrees(39.13, -75.47, 0.0);
    let msl = ellipsoidal_to_msl_egm96(&dover);
    assert!(msl.0 > 30.0, "{msl:?}");
    assert!(msl > OrthometricHeightM::default());
}
