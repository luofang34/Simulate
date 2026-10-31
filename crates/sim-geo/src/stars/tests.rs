use std::f64::consts::{FRAC_PI_2, PI};

use super::*;

#[test]
fn every_direction_is_a_unit_vector() {
    for star in &APOLLO_NAV_STARS {
        let length = star.direction_j2000.length();
        assert!((length - 1.0).abs() < 1e-12, "{} {length}", star.name);
    }
}

#[test]
fn codes_run_contiguously_from_0o01_to_0o45() {
    assert_eq!(APOLLO_NAV_STARS.len(), 0o45);
    for (index, star) in APOLLO_NAV_STARS.iter().enumerate() {
        assert_eq!(
            usize::from(star.apollo_octal_id),
            index + 1,
            "{}",
            star.name
        );
    }
}

#[test]
fn lookup_uses_the_octal_code_the_crew_keyed() {
    let name = |code| nav_star_by_octal_id(code).map(|star| star.name);
    assert_eq!(name(0o01), Some("Alpheratz"));
    assert_eq!(name(0o15), Some("Sirius"));
    assert_eq!(name(0o45), Some("Fomalhaut"));
    assert_eq!(name(0), None);
    assert_eq!(name(0o46), None);
}

#[test]
fn catalog_is_the_flown_list() {
    let name_is = |wanted| APOLLO_NAV_STARS.iter().any(|star| star.name == wanted);
    for wanted in ["Navi", "Regor", "Dnoces", "Vega", "Canopus", "Enif"] {
        assert!(name_is(wanted), "{wanted}");
    }
    assert!(!name_is("Betelgeuse"));
}

#[test]
fn polaris_sits_next_to_the_j2000_pole() {
    let names: Vec<_> = stars_in_cone(DVec3::Z, 1.0_f64.to_radians())
        .map(|star| star.name)
        .collect();
    assert_eq!(names, ["Polaris"]);
}

#[test]
fn cone_on_a_star_finds_it_at_any_boresight_length() {
    let Some(sirius) = nav_star_by_octal_id(0o15) else {
        panic!("Sirius is in the catalog");
    };
    let hits: Vec<_> = stars_in_cone(sirius.direction_j2000 * 40.0, 1e-3).collect();
    assert_eq!(hits, [sirius]);
}

#[test]
fn hemispheres_split_the_catalog_by_declination() {
    assert_eq!(stars_in_cone(DVec3::Z, FRAC_PI_2).count(), 19);
    assert_eq!(stars_in_cone(-DVec3::Z, FRAC_PI_2).count(), 18);
    assert_eq!(stars_in_cone(DVec3::X, PI).count(), APOLLO_NAV_STARS.len());
}

#[test]
fn degenerate_queries_see_nothing() {
    assert_eq!(stars_in_cone(DVec3::ZERO, PI).count(), 0);
    assert_eq!(stars_in_cone(DVec3::NAN, PI).count(), 0);
    assert_eq!(stars_in_cone(DVec3::Z, -1.0).count(), 0);
    assert_eq!(stars_in_cone(DVec3::Z, f64::NAN).count(), 0);
}
