use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, GravityField, LocalFrame};

use super::{HarmonicGravity, HarmonicTerm};
use crate::TideSystem;

const MGAL: f64 = 1.0e-5;

fn sites() -> Vec<Geodetic> {
    let mut sites = Vec::new();
    for lat in [-80.0, -45.0, -10.0, 0.0, 27.0, 39.0, 61.0, 89.0] {
        for lon in [-150.0, -75.5, 0.0, 77.0, 140.0] {
            for height in [0.0, 12_000.0, 400_000.0] {
                sites.push(Geodetic::from_degrees(lat, lon, height));
            }
        }
    }
    sites
}

/// The reference field must be the closed-form `J2` of the body it is built
/// from to round-off: it is the navigator's model, and a truth world on it
/// has no gravity model error.
#[test]
fn the_reference_field_is_the_closed_form_j2_to_round_off() {
    let mut worst = 0.0_f64;
    for body in [
        BodyEllipsoid::WGS84,
        BodyEllipsoid::MOON_IAU,
        BodyEllipsoid::MARS_IAU,
    ] {
        let field = HarmonicGravity::reference(&body).unwrap();
        for site in sites() {
            let fixed = body.geodetic_to_fixed(&site);
            let closed = body.gravity_fixed(fixed);
            let relative = (field.attraction_fixed(fixed) - closed).length() / closed.length();
            worst = worst.max(relative);
        }
    }
    assert!(worst < 4.0e-15, "{worst:e}");
}

#[test]
fn the_centre_has_no_attraction_and_no_gradient() {
    let field = HarmonicGravity::reference(&BodyEllipsoid::WGS84).unwrap();
    assert_eq!(field.attraction_fixed(DVec3::ZERO), DVec3::ZERO);
    assert!(field.attraction_gradient_fixed(DVec3::ZERO).is_none());
}

/// The gradient tensor is the derivative of the attraction, and traceless
/// outside the body (Laplace).
#[test]
fn the_gradient_tensor_is_the_derivative_of_the_attraction() {
    let field = HarmonicGravity::egm96_degree_36().unwrap();
    let fixed =
        BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic::from_degrees(39.0, -75.5, 5_000.0));
    let tensor = field.attraction_gradient_fixed(fixed).unwrap();
    let step = 50.0;
    for (axis, column) in [
        (DVec3::X, tensor.x_axis),
        (DVec3::Y, tensor.y_axis),
        (DVec3::Z, tensor.z_axis),
    ] {
        let numeric = (field.attraction_fixed(fixed + step * axis)
            - field.attraction_fixed(fixed - step * axis))
            / (2.0 * step);
        assert!((numeric - column).length() < 1e-12, "{numeric} {column}");
    }
    let trace = tensor.x_axis.x + tensor.y_axis.y + tensor.z_axis.z;
    assert!(trace.abs() < 1e-18, "{trace:e}");
    // The vertical gradient of gravity near the surface is about 3.086e-6 1/s².
    let up = LocalFrame::at(&Geodetic::from_degrees(39.0, -75.5, 5_000.0)).up();
    let vertical = up.dot(tensor * up);
    assert!((vertical - 3.08e-6).abs() < 0.03e-6, "{vertical:e}");
}

/// What the navigator's `J2` model leaves out: the degree-36 field differs
/// from it by tens of milligal on the ground, less at height.
#[test]
fn the_degree_36_field_differs_from_j2_by_tens_of_milligal() {
    let reference = HarmonicGravity::reference(&BodyEllipsoid::WGS84).unwrap();
    let detailed = HarmonicGravity::egm96_degree_36().unwrap();
    let anomaly_mgal = |height_m: f64| {
        let (mut sum_sq, mut largest, mut count) = (0.0_f64, 0.0_f64, 0.0_f64);
        for lat in (-85..=85).step_by(5) {
            for lon in (-180..180).step_by(5) {
                let site = Geodetic::from_degrees(f64::from(lat), f64::from(lon), height_m);
                let fixed = BodyEllipsoid::WGS84.geodetic_to_fixed(&site);
                let difference =
                    (detailed.attraction_fixed(fixed) - reference.attraction_fixed(fixed)).length()
                        / MGAL;
                sum_sq += difference * difference;
                largest = largest.max(difference);
                count += 1.0;
            }
        }
        ((sum_sq / count).sqrt(), largest)
    };
    let (ground_rms, ground_max) = anomaly_mgal(0.0);
    let (orbit_rms, orbit_max) = anomaly_mgal(400_000.0);
    assert!((10.0..40.0).contains(&ground_rms), "{ground_rms}");
    assert!((40.0..200.0).contains(&ground_max), "{ground_max}");
    assert!(orbit_rms < ground_rms && orbit_rms > 2.0, "{orbit_rms}");
    assert!(orbit_max < ground_max, "{orbit_max}");
}

#[test]
fn a_cap_keeps_the_lower_of_the_stored_and_asked_degree() {
    let detailed = HarmonicGravity::egm96_degree_36().unwrap();
    let capped = detailed.clone().capped(1, 8, 8).unwrap();
    assert_eq!(capped.truncation().degree, 8);
    assert_eq!(capped.truncation().min_degree, 1);
    let j2_only = HarmonicGravity::reference(&BodyEllipsoid::WGS84)
        .unwrap()
        .capped(1, 8, 8)
        .unwrap();
    assert_eq!(j2_only.truncation().degree, 2);
    // Without its central term a field is the perturbation alone.
    let fixed = DVec3::new(6_900_000.0, 300_000.0, 1_200_000.0);
    let whole = detailed.clone().capped(0, 8, 8).unwrap();
    let central = HarmonicGravity::point_mass(detailed.gm_m3_s2(), detailed.radius_m()).unwrap();
    let parts = capped.attraction_fixed(fixed) + central.attraction_fixed(fixed);
    assert!((whole.attraction_fixed(fixed) - parts).length() < 1e-14);
}

#[test]
fn sparse_terms_fill_a_dense_triangle() {
    let terms = [
        HarmonicTerm::from_unnormalized_j2(1.082_626_68e-3),
        HarmonicTerm {
            degree: 2,
            order: 2,
            c_bar: 2.43e-6,
            s_bar: -1.40e-6,
        },
    ];
    let field =
        HarmonicGravity::from_terms(3.986e14, 6_378_137.0, &terms, TideSystem::Unstated).unwrap();
    assert_eq!(field.coefficient(0, 0), (1.0, 0.0));
    assert_eq!(field.coefficient(2, 2), (2.43e-6, -1.40e-6));
    assert_eq!(field.coefficient(2, 1), (0.0, 0.0));
    let wrong = [HarmonicTerm {
        degree: 2,
        order: 3,
        c_bar: 1.0,
        s_bar: 0.0,
    }];
    assert!(HarmonicGravity::from_terms(3.986e14, 6.4e6, &wrong, TideSystem::Unstated).is_err());
}

/// A field is immutable shared data: every thread gets the same bits.
#[test]
fn threads_evaluate_the_same_bits() {
    let field = std::sync::Arc::new(HarmonicGravity::egm96_degree_36().unwrap());
    let points: Vec<DVec3> = sites()
        .iter()
        .map(|site| BodyEllipsoid::WGS84.geodetic_to_fixed(site))
        .collect();
    let evaluate = |field: &HarmonicGravity, points: &[DVec3]| -> Vec<[u64; 3]> {
        points
            .iter()
            .map(|&p| field.attraction_fixed(p).to_array().map(f64::to_bits))
            .collect()
    };
    let here = evaluate(&field, &points);
    let handles: Vec<_> = (0..4)
        .map(|_| {
            let (field, points) = (field.clone(), points.clone());
            std::thread::spawn(move || evaluate(&field, &points))
        })
        .collect();
    for handle in handles {
        assert_eq!(handle.join().unwrap(), here);
    }
}

const SMALL_GFC: &str = "\
modelname                   test-earth
earth_gravity_constant      0.3986004415E+15
radius                      0.6378136300E+07
max_degree                     3
errors                      formal
norm                        fully_normalized
tide_system                 tide_free
end_of_head ================================
gfc     0   0  1.000000000000e+00  0.000000000000e+00  0.0e+00  0.0e+00
gfc     2   0 -0.484165371736e-03  0.000000000000e+00  0.0e+00  0.0e+00
gfc     2   2  0.243914352398e-05 -0.140016683654e-05  0.0e+00  0.0e+00
gfc     3   0  0.957254173792e-06  0.000000000000e+00  0.0e+00  0.0e+00
";

const SMALL_SHADR: &str = "\
 1.7380000000000000e+03, 4.9028001224453001e+03, 6.4536052689015518e-15,    2,    2,    1, 0.0000000000000000e+00, 0.0000000000000000e+00
    2,    0,-9.0884339347424299e-05, 0.0000000000000000e+00, 7.9734541185906691e-12, 0.0000000000000000e+00
    2,    2, 3.4673798465672399e-05, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00
";

/// A scenario loads a field of any body from the text of a published file.
#[test]
fn published_coefficient_files_load_at_a_chosen_degree() {
    let earth = HarmonicGravity::from_gfc(SMALL_GFC, None).unwrap();
    assert_eq!(earth.truncation().degree, 3);
    assert_eq!(earth.gm_m3_s2(), 0.398_600_441_5e15);
    assert_eq!(earth.tide_system(), TideSystem::TideFree);
    assert_eq!(
        earth.coefficient(2, 2),
        (0.243_914_352_398e-5, -0.140_016_683_654e-5)
    );
    let cut = HarmonicGravity::from_gfc(SMALL_GFC, Some(2)).unwrap();
    assert_eq!(cut.truncation().degree, 2);
    assert_eq!(cut.coefficient(3, 0), (0.0, 0.0));

    let moon = HarmonicGravity::from_shadr(SMALL_SHADR, None).unwrap();
    assert_eq!(moon.radius_m(), 1_738_000.0);
    assert!((moon.gm_m3_s2() - 4.902_800_122_445_3e12).abs() < 1.0);
    assert_eq!(moon.coefficient(2, 0).0, -9.088_433_934_742_43e-5);
    let low_orbit = DVec3::new(1_838_000.0, 0.0, 0.0);
    assert!((moon.attraction_fixed(low_orbit).length() - 1.451).abs() < 2e-3);

    let broken = SMALL_GFC.replace("fully_normalized", "unnormalized");
    assert!(HarmonicGravity::from_gfc(&broken, None).is_err());
}
