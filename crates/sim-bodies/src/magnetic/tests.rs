use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, MagneticField, MagneticFieldError};
use world_core::{DecimalYear, GeodeticPosition};
use world_geomag::wmm2025;

use super::{HarmonicMagnetism, NoMagneticField};

fn ned_nt(field: &dyn MagneticField, site: &Geodetic, year: f64) -> DVec3 {
    let fixed = BodyEllipsoid::WGS84.geodetic_to_fixed(site);
    LocalFrame::at(site).ecef_to_ned_vector(field.field_fixed_nt(fixed, year).unwrap())
}

/// IGRF-14 and WMM2025 are independent fits of the same 2025 main field to
/// degree 13 and 12. They agree to a few tens of nanotesla at the surface
/// at their common epoch, which is the size of the unmodelled crustal field
/// and far below the 25,000 to 65,000 nT of the field itself.
#[test]
fn igrf14_agrees_with_wmm2025_at_their_common_epoch() {
    let igrf = HarmonicMagnetism::igrf14().unwrap();
    let (mut sum_sq, mut largest, mut count) = (0.0_f64, 0.0_f64, 0.0_f64);
    for lat in (-80..=80).step_by(10) {
        for lon in (-180..180).step_by(20) {
            let (lat, lon) = (f64::from(lat), f64::from(lon));
            let site = Geodetic::from_degrees(lat, lon, 0.0);
            let reference =
                wmm2025(GeodeticPosition::wgs84(lat, lon, 0.0), DecimalYear(2025.0)).unwrap();
            let wmm = DVec3::new(reference.north_nt, reference.east_nt, reference.down_nt);
            let difference = (ned_nt(&igrf, &site, 2025.0) - wmm).length();
            sum_sq += difference * difference;
            largest = largest.max(difference);
            count += 1.0;
        }
    }
    let rms = (sum_sq / count).sqrt();
    assert!(rms < 20.0, "{rms}");
    assert!(largest < 60.0, "{largest}");
}

#[test]
fn the_field_at_dover_has_the_charted_strength_and_declination() {
    let igrf = HarmonicMagnetism::igrf14().unwrap();
    let dover = Geodetic::from_degrees(39.13, -75.47, 9.0);
    let field = ned_nt(&igrf, &dover, 2026.5);
    let declination_deg = field.y.atan2(field.x).to_degrees();
    assert!(
        (-13.5..-10.5).contains(&declination_deg),
        "{declination_deg}"
    );
    assert!((49_000.0..53_000.0).contains(&field.length()), "{field}");
}

#[test]
fn a_date_outside_the_model_is_refused_not_extrapolated() {
    let igrf = HarmonicMagnetism::igrf14().unwrap();
    let fixed = DVec3::new(6_378_137.0, 0.0, 0.0);
    assert!(igrf.field_fixed_nt(fixed, 1970.0).is_ok());
    let refused = igrf.field_fixed_nt(fixed, 2031.0).unwrap_err();
    assert!(matches!(
        refused,
        MagneticFieldError::OutsideValidity { from, until, .. } if from == 1900.0 && until == 2030.0
    ));
    assert!(matches!(
        igrf.field_fixed_nt(DVec3::ZERO, 2026.0).unwrap_err(),
        MagneticFieldError::OutsideDomain { .. }
    ));
}

/// A centred axial dipole: on the equator the field is the equatorial
/// strength along the axis, at the pole twice it, and it falls as 1/r³.
#[test]
fn a_dipole_has_the_closed_form_field() {
    let (radius, g10) = (6_371_200.0, -30_000.0);
    let dipole = HarmonicMagnetism::dipole(radius, [g10, 0.0, 0.0], 2000.0, 2100.0).unwrap();
    let equator = dipole
        .field_fixed_nt(DVec3::new(radius, 0.0, 0.0), 2050.0)
        .unwrap();
    assert!(
        (equator - DVec3::new(0.0, 0.0, -g10)).length() < 1e-9,
        "{equator}"
    );
    let pole = dipole
        .field_fixed_nt(DVec3::new(0.0, 0.0, radius), 2050.0)
        .unwrap();
    assert!(
        (pole - DVec3::new(0.0, 0.0, 2.0 * g10)).length() < 1e-9,
        "{pole}"
    );
    let far = dipole
        .field_fixed_nt(DVec3::new(2.0 * radius, 0.0, 0.0), 2050.0)
        .unwrap();
    assert!((far - equator / 8.0).length() < 1e-9);
    let gradient = dipole
        .field_gradient_fixed_nt_per_m(DVec3::new(radius, 0.0, 0.0), 2050.0)
        .unwrap()
        .unwrap();
    // dBz/dx on the equator is -3 Bz / r.
    assert!((gradient.x_axis.z - 3.0 * g10 / radius).abs() < 1e-12);
}

#[test]
fn a_body_without_a_field_reads_zero() {
    let none = NoMagneticField;
    let fixed = DVec3::new(1.8e6, 0.0, 0.0);
    assert_eq!(none.field_fixed_nt(fixed, 1969.5).unwrap(), DVec3::ZERO);
    assert!(
        none.field_gradient_fixed_nt_per_m(fixed, 1969.5)
            .unwrap()
            .is_some()
    );
}
