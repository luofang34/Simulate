use sim_pack::{builtin_c172_package, builtin_p38g_package};

use super::StructuralDatum;
use crate::UsdAircraftAsset;

/// How far any C172 landmark may sit from its POH station. The embedded mesh's mains
/// stand 4.7 in aft and its propeller 3.1 in forward; a worse mesh shows here.
const C172_RESIDUAL_BOUND_IN: f64 = 6.0;

fn fit_c172() -> StructuralDatum {
    let package = builtin_c172_package().unwrap();
    let landmarks = package.structural_landmarks().unwrap();
    StructuralDatum::fit(landmarks, &UsdAircraftAsset::embedded_c172().unwrap()).unwrap()
}

#[test]
fn the_c172_is_placed_by_its_poh_landmarks_within_bounds() {
    let datum = fit_c172();
    let residuals = datum.residuals_aft_in();
    assert_eq!(residuals.len(), 4);
    let report: Vec<String> = residuals
        .iter()
        .map(|(name, aft)| format!("{name}: {aft:+.2} in aft"))
        .collect();
    tracing::info!(residuals = ?report, origin = ?datum.origin_in(), "C172 structural datum");
    assert!(
        residuals
            .iter()
            .all(|(_, aft)| aft.abs() < C172_RESIDUAL_BOUND_IN),
        "C172 landmarks off their POH stations: {report:?}"
    );
    assert!(residuals.iter().map(|(_, aft)| aft).sum::<f64>().abs() < 1e-9);
    let origin = datum.origin_in();
    assert!((origin.x - 121.27).abs() < 0.05, "{origin}");
    assert!((origin.z + 0.055 / 0.0254).abs() < 0.01, "{origin}");
}

#[test]
fn the_p38g_is_placed_by_its_thrusters_until_drawing_stations_land() {
    let package = builtin_p38g_package().unwrap();
    let landmarks = package.structural_landmarks().unwrap();
    let datum = StructuralDatum::fit(landmarks, &UsdAircraftAsset::p38g().unwrap()).unwrap();
    assert!(
        datum
            .residuals_aft_in()
            .iter()
            .all(|(_, aft)| aft.abs() < 1e-9)
    );
    let origin = datum.origin_in();
    assert!((origin.x - (36.0 + 2.63 / 0.0254)).abs() < 0.01, "{origin}");
    assert!((origin.z + 0.04 / 0.0254).abs() < 0.01, "{origin}");
}

#[test]
fn a_landmark_the_model_lacks_is_refused() {
    let package = builtin_c172_package().unwrap();
    let mut landmarks = package.structural_landmarks().unwrap().clone();
    landmarks.landmarks[0].feature = sim_pack::MeshFeature::PrimBoxCentre {
        prim_path: "/Nowhere".to_owned(),
    };
    let asset = UsdAircraftAsset::embedded_c172().unwrap();
    assert!(StructuralDatum::fit(&landmarks, &asset).is_err());
}
