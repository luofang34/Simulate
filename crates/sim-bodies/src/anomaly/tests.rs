use std::sync::Arc;

use glam::DVec3;
use sim_geo::{
    BodyEllipsoid, Geodetic, GravityField, LocalFrame, MagneticField, MagneticFieldError,
};

use super::{AnomalyOverlay, GravityWithAnomaly, MagnetismWithAnomaly};
use crate::{GridGeometry, HarmonicGravity, HarmonicMagnetism, OutsideCoverage};

fn geometry() -> GridGeometry {
    GridGeometry {
        south_deg: 38.0,
        west_deg: -77.0,
        latitude_step_deg: 1.0,
        longitude_step_deg: 1.0,
        rows: 3,
        columns: 3,
    }
}

/// A uniform down-only anomaly of `down` on every node.
fn uniform_down(down: f64, outside: OutsideCoverage) -> AnomalyOverlay {
    let values = (0..9).flat_map(|_| [0.0, 0.0, down]).collect();
    AnomalyOverlay::new(geometry(), values, &BodyEllipsoid::WGS84, outside).unwrap()
}

#[test]
fn a_gravity_anomaly_adds_along_the_local_vertical_inside_its_grid() {
    let base: Arc<dyn GravityField> =
        Arc::new(HarmonicGravity::reference(&BodyEllipsoid::WGS84).unwrap());
    let field = GravityWithAnomaly {
        base: base.clone(),
        overlays: vec![uniform_down(30.0e-5, OutsideCoverage::Zero)],
    };
    let inside = Geodetic::from_degrees(39.2, -75.5, 100.0);
    let fixed = BodyEllipsoid::WGS84.geodetic_to_fixed(&inside);
    let added = field.attraction_fixed(fixed) - base.attraction_fixed(fixed);
    let down = -LocalFrame::at(&inside).up();
    assert!((added - 30.0e-5 * down).length() < 1e-12, "{added}");
    // A sum with a grid states no gradient rather than the base's alone.
    assert!(field.attraction_gradient_fixed(fixed).is_none());
    let outside = BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic::from_degrees(10.0, 20.0, 0.0));
    assert_eq!(
        field.attraction_fixed(outside),
        base.attraction_fixed(outside)
    );
}

#[test]
fn a_magnetic_anomaly_adds_and_a_refusing_grid_reports_its_edge() {
    let base: Arc<dyn MagneticField> = Arc::new(HarmonicMagnetism::igrf14().unwrap());
    let field = MagnetismWithAnomaly {
        base: base.clone(),
        overlays: vec![uniform_down(250.0, OutsideCoverage::Refuse)],
    };
    let inside = Geodetic::from_degrees(39.2, -75.5, 100.0);
    let fixed = BodyEllipsoid::WGS84.geodetic_to_fixed(&inside);
    let added =
        field.field_fixed_nt(fixed, 2026.5).unwrap() - base.field_fixed_nt(fixed, 2026.5).unwrap();
    let down = -LocalFrame::at(&inside).up();
    assert!((added - 250.0 * down).length() < 1e-6, "{added}");
    let outside = BodyEllipsoid::WGS84.geodetic_to_fixed(&Geodetic::from_degrees(10.0, 20.0, 0.0));
    assert!(matches!(
        field.field_fixed_nt(outside, 2026.5),
        Err(MagneticFieldError::OutsideDomain { .. })
    ));
}

#[test]
fn a_malformed_grid_is_refused() {
    let short = vec![0.0; 9];
    let refused = AnomalyOverlay::new(
        geometry(),
        short,
        &BodyEllipsoid::WGS84,
        OutsideCoverage::Zero,
    );
    assert!(refused.is_err());
    assert_eq!(
        uniform_down(1.0, OutsideCoverage::Zero)
            .anomaly_fixed(DVec3::new(f64::NAN, 0.0, 0.0))
            .ok(),
        None
    );
}
