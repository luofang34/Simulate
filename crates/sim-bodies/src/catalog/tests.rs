use std::sync::Arc;

use sim_geo::{BodyEllipsoid, GravityField};
use sim_spacetime::NaifId;

use super::BodyCatalog;
use crate::{BodyDescriptor, HarmonicGravity, HarmonicMagnetism, RotationSource};

fn venus() -> BodyDescriptor {
    let ellipsoid = BodyEllipsoid {
        semi_major_m: 6_051_800.0,
        flattening: 0.0,
        mu_m3_s2: 3.248_585_92e14,
        omega_rad_s: -2.992_4e-7,
        j2: 0.0,
    };
    BodyDescriptor::new(NaifId::VENUS, "Venus", ellipsoid)
        .unwrap()
        .with_sphere_of_influence(616_000_000.0, NaifId::SSB)
        .with_regime(250_000.0, 200_000.0)
}

#[test]
fn modelled_bodies_map_to_their_identifiers_and_back() {
    let bodies = BodyCatalog::standard_solar_system().unwrap();
    assert_eq!(bodies.naif_of(&BodyEllipsoid::WGS84), Some(NaifId::EARTH));
    assert_eq!(bodies.naif_of(&BodyEllipsoid::MOON_IAU), Some(NaifId::MOON));
    assert_eq!(bodies.naif_of(&BodyEllipsoid::MARS_IAU), Some(NaifId::MARS));
    for body in [NaifId::EARTH, NaifId::MOON, NaifId::MARS] {
        let ellipsoid = bodies.ellipsoid_of(body).unwrap();
        assert_eq!(bodies.naif_of(&ellipsoid), Some(body));
    }
    assert_eq!(bodies.ellipsoid_of(NaifId::SUN), None);
    assert_eq!(bodies.ellipsoid_of(NaifId::SSB), None);
    assert_eq!(bodies.ellipsoid_of(NaifId::VENUS), None);
}

#[test]
fn a_registered_body_exists_for_every_lookup() {
    let bodies = BodyCatalog::standard_solar_system().unwrap().with(venus());
    let described = bodies.get(NaifId::VENUS).unwrap();
    assert_eq!(bodies.naif_of(&described.ellipsoid), Some(NaifId::VENUS));
    assert_eq!(described.rotation, RotationSource::IauModel);
    let under_the_sun: Vec<NaifId> = bodies.children_of(NaifId::SSB).map(|b| b.naif).collect();
    assert_eq!(under_the_sun, [NaifId::VENUS, NaifId::EARTH, NaifId::MARS]);
    let under_earth: Vec<NaifId> = bodies.children_of(NaifId::EARTH).map(|b| b.naif).collect();
    assert_eq!(under_earth, [NaifId::MOON]);
    let occulters: Vec<NaifId> = bodies
        .solar_flux_model()
        .occulters
        .iter()
        .map(|sphere| sphere.body)
        .collect();
    assert_eq!(
        occulters,
        [NaifId::VENUS, NaifId::MOON, NaifId::EARTH, NaifId::MARS]
    );
}

#[test]
fn the_standard_bodies_carry_the_reference_fields() {
    let bodies = BodyCatalog::standard_solar_system().unwrap();
    for body in bodies.iter() {
        let fixed = glam::DVec3::new(1.3, 0.2, 0.4) * body.ellipsoid.semi_major_m;
        let closed = body.ellipsoid.gravity_fixed(fixed);
        let field = body.gravity_field().attraction_fixed(fixed);
        assert!((field - closed).length() / closed.length() < 1e-14);
    }
    let earth = bodies.get(NaifId::EARTH).unwrap();
    let at = glam::DVec3::new(6_378_137.0, 0.0, 0.0);
    assert!(
        earth
            .magnetic_field()
            .field_fixed_nt(at, 2026.5)
            .unwrap()
            .length()
            > 20_000.0
    );
    let moon = bodies.get(NaifId::MOON).unwrap();
    assert_eq!(
        moon.magnetic_field().field_fixed_nt(at, 2026.5).unwrap(),
        glam::DVec3::ZERO
    );
    let low = sim_geo::Geodetic::from_degrees(0.0, 0.0, 10_000.0);
    let utc = sim_geo::UtcTime::from_unix_seconds(1.8e9);
    let density =
        |body: &crate::BodyDescriptor| body.atmosphere().sample(&low, utc).unwrap().density_kg_m3;
    assert!(density(earth) > 0.3 && density(moon) == 0.0);
}

/// Everything a trajectory depends on is in the hash, so two catalogs that
/// fly differently can never share a manifest.
#[test]
fn the_hash_tells_catalogs_apart() {
    let standard = || BodyCatalog::standard_solar_system().unwrap();
    let base = standard().content_hash();
    assert_eq!(base, standard().content_hash());
    assert_ne!(base, standard().with(venus()).content_hash());
    let detailed = BodyDescriptor::earth()
        .unwrap()
        .with_gravity(HarmonicGravity::egm96_degree_36().unwrap());
    assert_ne!(base, standard().with(detailed).content_hash());
    let dipole = HarmonicMagnetism::dipole(6_371_200.0, [-29_400.0, 0.0, 0.0], 1900.0, 2100.0);
    let other_field = BodyDescriptor::earth()
        .unwrap()
        .with_magnetic_field("axial-dipole", Arc::new(dipole.unwrap()));
    assert_ne!(base, standard().with(other_field).content_hash());
    let wider = BodyDescriptor::moon()
        .unwrap()
        .with_regime(30_000.0, 15_000.0);
    assert_ne!(base, standard().with(wider).content_hash());
    let surveyed = |down: f64| {
        let geometry = crate::GridGeometry {
            south_deg: 38.0,
            west_deg: -77.0,
            latitude_step_deg: 1.0,
            longitude_step_deg: 1.0,
            rows: 2,
            columns: 2,
        };
        let values = (0..4).flat_map(|_| [0.0, 0.0, down]).collect();
        let outside = crate::OutsideCoverage::Zero;
        let grid = crate::AnomalyOverlay::new(geometry, values, &BodyEllipsoid::WGS84, outside);
        BodyDescriptor::earth()
            .unwrap()
            .with_gravity_anomalies(vec![grid.unwrap()])
    };
    let with_grid = standard().with(surveyed(3.0e-4)).content_hash();
    assert_ne!(base, with_grid);
    assert_ne!(with_grid, standard().with(surveyed(3.1e-4)).content_hash());
    // A new global model keeps the surveyed anomalies on top of it.
    let both = surveyed(3.0e-4).with_gravity(HarmonicGravity::egm96_degree_36().unwrap());
    let inside =
        BodyEllipsoid::WGS84.geodetic_to_fixed(&sim_geo::Geodetic::from_degrees(38.5, -76.5, 0.0));
    let added =
        both.gravity_field().attraction_fixed(inside) - both.gravity().attraction_fixed(inside);
    assert!((added.length() - 3.0e-4).abs() < 1e-12);
    let smaller = BodyDescriptor::mars()
        .unwrap()
        .with_sphere_of_influence(5.0e8, NaifId::SSB);
    assert_ne!(base, standard().with(smaller).content_hash());
}

/// A detailed field is one instance for every consumer of the body.
#[test]
fn a_detailed_gravity_is_shared_by_field_and_coefficients() {
    let earth = BodyDescriptor::earth()
        .unwrap()
        .with_gravity(HarmonicGravity::egm96_degree_36().unwrap());
    let fixed = glam::DVec3::new(4.0e6, -3.0e6, 4.2e6);
    assert_eq!(
        earth.gravity_field().attraction_fixed(fixed),
        earth.gravity().attraction_fixed(fixed)
    );
    assert_eq!(earth.gravity().truncation().degree, 36);
}
