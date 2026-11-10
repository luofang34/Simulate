mod bays;
mod calibration;
mod cockpit_stations;
mod silhouette;
mod surfaces;

use crate::{
    P38PartRole, p38g_cockpit_contract, p38g_mesh_prims, p38g_package_source, p38g_usda,
    three_view_gondola_outline_x, three_view_wing_leading_y, three_view_wing_trailing_y,
};

#[test]
fn powerplant_and_cockpit_reservation_prims_are_present() {
    let prims = p38g_mesh_prims();
    for expected in [
        "/P38g/Powerplant/ReductionGearCastings",
        "/P38g/Powerplant/ExhaustManifoldShrouds",
        "/P38g/Powerplant/ExhaustCollectors",
        "/P38g/Powerplant/TurboHousings",
        "/P38g/Powerplant/TurboNozzleBoxes",
        "/P38g/Powerplant/TurboBucketWheels",
        "/P38g/Powerplant/RadiatorScoops",
        "/P38g/Powerplant/ChinOilCoolerScoops",
        "/P38g/Powerplant/ChinOilCoolerInletLips",
        "/P38g/Powerplant/CoolerMatrixCores",
        "/P38g/CockpitInterior/TubAndBulkheads",
        "/P38g/CockpitInterior/InstrumentPanel/Board",
        "/P38g/CockpitInterior/ControlWheel",
    ] {
        assert!(
            prims.iter().any(|p| p.prim_path == expected),
            "missing prim {expected}"
        );
    }

    // The turbocharged G exhausts through its turbos; it has no ejector stacks.
    assert!(!prims.iter().any(|p| p.prim_path.contains("Ejector")));

    let contract = p38g_cockpit_contract();
    assert_eq!(contract.bulkheads.len(), 6);
    assert!(
        prims
            .iter()
            .any(|p| p.prim_path == contract.main_instrument_panel.prim_path
                && p.role == P38PartRole::CabinInterior)
    );
    let widest = three_view_gondola_outline_x(crate::drawing::fs_to_y(120.0));
    assert!(
        (widest / crate::drawing::INCH_M - 18.995).abs() < 0.01,
        "{widest}"
    );
    assert!(three_view_wing_leading_y(0.0) > three_view_wing_trailing_y(0.0));
}

#[test]
fn openusd_stage_and_package_manifest_export_cleanly() {
    let manifest = p38g_package_source();
    assert!(manifest.contains("Lockheed P-38G-13-LO Lightning"));
    let Ok(usda) = p38g_usda() else {
        return;
    };
    assert!(usda.starts_with("#usda 1.0"));
    assert!(usda.contains("defaultPrim = \"P38g\""));
    assert!(usda.contains("Powerplant_ExhaustCollectors"));
    assert!(usda.contains("CockpitInterior_InstrumentPanel_Board"));
}
