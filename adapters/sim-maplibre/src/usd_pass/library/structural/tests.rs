use sim_pack::{builtin_c172p_package, builtin_c172s_package, builtin_p38g_package};

use super::StructuralDatum;
use crate::UsdAircraftAsset;

/// How far any C172 landmark may sit from its POH station, fitted on the bare-gear tail
/// airframes the ownship draws. Its gear agrees with itself; its propeller disc stands
/// about 2 in forward of the gear against the POH; a worse mesh shows here.
const C172_RESIDUAL_BOUND_IN: f64 = 4.0;

fn fit_c172s() -> StructuralDatum {
    let package = builtin_c172s_package().unwrap();
    let landmarks = package.structural_landmarks().unwrap();
    StructuralDatum::fit(landmarks, &UsdAircraftAsset::embedded_n642sp().unwrap()).unwrap()
}

fn fit_c172p() -> StructuralDatum {
    let package = builtin_c172p_package().unwrap();
    let landmarks = package.structural_landmarks().unwrap();
    StructuralDatum::fit(
        landmarks,
        &UsdAircraftAsset::embedded_c172p_d_ecjb().unwrap(),
    )
    .unwrap()
}

#[test]
fn the_c172_is_placed_by_its_poh_landmarks_within_bounds() {
    for (label, datum) in [("C172S", fit_c172s()), ("C172P", fit_c172p())] {
        let residuals = datum.residuals_aft_in();
        assert_eq!(residuals.len(), 4);
        let report: Vec<String> = residuals
            .iter()
            .map(|(name, aft)| format!("{name}: {aft:+.2} in aft"))
            .collect();
        tracing::info!(
            variant = label,
            residuals = ?report,
            origin = ?datum.origin_in(),
            "C172 structural datum"
        );
        assert!(
            residuals
                .iter()
                .all(|(_, aft)| aft.abs() < C172_RESIDUAL_BOUND_IN),
            "{label} landmarks off their POH stations: {report:?}"
        );
        assert!(residuals.iter().map(|(_, aft)| aft).sum::<f64>().abs() < 1e-9);
        let origin = datum.origin_in();
        assert!(
            (origin.x - 122.25).abs() < 0.05,
            "{label}: {origin} {report:?}"
        );
        assert!(
            (origin.z + 0.055 / 0.0254).abs() < 0.01,
            "{label}: {origin}"
        );
    }
}

/// How far any P-38G landmark may sit from its AN 01-75F-2 station: the mesh is built
/// on those stations, so only facet sampling separates them.
const P38G_RESIDUAL_BOUND_IN: f64 = 0.25;

#[test]
fn the_p38g_is_placed_by_its_drawing_stations() {
    let package = builtin_p38g_package().unwrap();
    let landmarks = package.structural_landmarks().unwrap();
    let datum = StructuralDatum::fit(landmarks, &UsdAircraftAsset::p38g().unwrap()).unwrap();
    let residuals = datum.residuals_aft_in();
    assert_eq!(residuals.len(), 4);
    assert!(
        residuals
            .iter()
            .all(|(_, aft)| aft.abs() < P38G_RESIDUAL_BOUND_IN),
        "P-38G landmarks off their drawing stations: {residuals:?}"
    );
    let origin = datum.origin_in();
    let (nose_y, frl_z, inch) = (
        f64::from(sim_p38g::drawing::NOSE_Y_M),
        f64::from(sim_p38g::drawing::FRL_Z_M),
        f64::from(sim_p38g::drawing::INCH_M),
    );
    assert!(
        (origin.x - nose_y / inch).abs() < P38G_RESIDUAL_BOUND_IN,
        "{origin}"
    );
    assert!((origin.z + frl_z / inch).abs() < 0.01, "{origin}");
}

#[test]
fn a_landmark_the_model_lacks_is_refused() {
    let package = builtin_c172s_package().unwrap();
    let mut landmarks = package.structural_landmarks().unwrap().clone();
    landmarks.landmarks[0].feature = sim_pack::MeshFeature::PrimBoxCentre {
        prim_path: "/Nowhere".to_owned(),
    };
    let asset = UsdAircraftAsset::embedded_c172().unwrap();
    assert!(StructuralDatum::fit(&landmarks, &asset).is_err());
}
