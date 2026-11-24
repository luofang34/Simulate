use std::path::Path;
use std::sync::Arc;

use super::BuiltinAirframe;
use crate::usd_aircraft::occlusion;
use crate::usd_pass::AircraftLibrary;
use crate::{OwnshipVisualKind, UsdAircraftAsset};

/// The airframes that ship an occlusion table, with the table's file name.
const BAKED: [(BuiltinAirframe, &str); 3] = [
    (BuiltinAirframe::C172, "c172.ao"),
    (BuiltinAirframe::N642sp, "n642sp.ao"),
    (BuiltinAirframe::DEcjb, "d_ecjb.ao"),
];

/// Share of vertices whose shipped occlusion may sit further than [`VALUE_TOLERANCE`]
/// from a bake on the machine running the test: a ray grazing a triangle's edge lands on
/// either side of it as the platform's `sin` and `cos` round.
const GRAZING_SHARE: f64 = 1e-3;
const VALUE_TOLERANCE: f32 = 1e-4;

#[test]
fn ownship_traffic_and_every_other_consumer_share_one_composition() {
    let traffic = UsdAircraftAsset::builtin(BuiltinAirframe::C172).unwrap();
    let mut scene = AircraftLibrary::new(Arc::clone(&traffic));
    scene.select_ownship(OwnshipVisualKind::C172).unwrap();
    let drawn = scene.active_ownship_asset().unwrap();
    // The structural fit, the hangar and a parked NPC ask for the same tail.
    let fitted = UsdAircraftAsset::builtin(BuiltinAirframe::N642sp).unwrap();
    assert!(Arc::ptr_eq(&drawn, &fitted));

    let mut second_scene =
        AircraftLibrary::new(UsdAircraftAsset::builtin(BuiltinAirframe::C172).unwrap());
    second_scene
        .select_ownship(OwnshipVisualKind::C172)
        .unwrap();
    assert!(Arc::ptr_eq(
        &second_scene.active_ownship_asset().unwrap(),
        &drawn
    ));
    assert!(Arc::ptr_eq(
        &UsdAircraftAsset::builtin(BuiltinAirframe::C172).unwrap(),
        &traffic
    ));

    // Every test of this binary draws from the same process-wide copies, so the counts
    // also cover whatever the other tests asked for.
    assert_eq!(BuiltinAirframe::N642sp.compositions(), 1);
    assert_eq!(BuiltinAirframe::C172.compositions(), 1);
}

#[test]
fn a_copy_is_the_shared_airframe_part_for_part() {
    let shared = UsdAircraftAsset::builtin(BuiltinAirframe::N642sp).unwrap();
    let copy = UsdAircraftAsset::embedded_n642sp().unwrap();
    assert_eq!(copy.parts().len(), shared.parts().len());
    for (a, b) in copy.parts().iter().zip(shared.parts()) {
        assert_eq!(a.prim_path, b.prim_path);
        assert_eq!(a.vertices, b.vertices);
        assert_eq!(a.indices, b.indices);
    }
}

#[test]
fn a_shipped_occlusion_table_changes_nothing_but_stands_in_for_the_bake() {
    for (airframe, file) in BAKED {
        let baked_here = airframe.compose(None).unwrap();
        let shipped = UsdAircraftAsset::builtin(airframe).unwrap();
        assert_eq!(baked_here.parts().len(), shipped.parts().len(), "{file}");
        let (mut vertices, mut apart, mut worst) = (0_usize, 0_usize, 0.0_f32);
        for (here, ship) in baked_here.parts().iter().zip(shipped.parts()) {
            assert_eq!(here.prim_path, ship.prim_path);
            assert_eq!(here.role, ship.role);
            assert_eq!(here.material, ship.material);
            assert_eq!(here.double_sided, ship.double_sided);
            assert_eq!(here.rest_to_aircraft, ship.rest_to_aircraft);
            assert_eq!(here.indices, ship.indices);
            assert_eq!(here.vertices.len(), ship.vertices.len());
            for (a, b) in here.vertices.iter().zip(&ship.vertices) {
                assert_eq!(
                    (a.position, a.normal, a.uv, a.tangent),
                    (b.position, b.normal, b.uv, b.tangent)
                );
                let gap = (a.occlusion - b.occlusion).abs();
                vertices += 1;
                apart += usize::from(gap > VALUE_TOLERANCE);
                worst = worst.max(gap);
            }
        }
        assert!(vertices > 1_000, "{file}: {vertices} vertices");
        assert!(
            (apart as f64) <= (vertices as f64) * GRAZING_SHARE,
            "{file}: {apart} of {vertices} vertices differ from a bake, worst {worst}; \
             regenerate the table if the airframe or the bake changed"
        );
        assert_eq!(baked_here.bounding_radius_m(), shipped.bounding_radius_m());
        assert_eq!(baked_here.lowest_point_z_m(), shipped.lowest_point_z_m());
    }
}

#[test]
fn a_table_of_another_size_is_refused_and_the_airframe_baked() {
    let short = [0_u8; 8];
    let asset = BuiltinAirframe::C172.compose(Some(&short)).unwrap();
    let shipped = UsdAircraftAsset::builtin(BuiltinAirframe::C172).unwrap();
    let darkened = |asset: &UsdAircraftAsset| {
        asset
            .parts()
            .iter()
            .flat_map(|part| &part.vertices)
            .filter(|vertex| vertex.occlusion < 0.9)
            .count()
    };
    assert!(darkened(&asset) > 100);
    assert!(darkened(&asset).abs_diff(darkened(&shipped)) <= darkened(&shipped) / 100);
}

#[test]
#[ignore = "rewrites builtin/data/*.ao from a bake on this machine"]
fn regenerate_occlusion_tables() {
    for (airframe, file) in BAKED {
        let asset = airframe.compose(None).unwrap();
        let table = occlusion::baked_table(asset.parts());
        let out = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src/usd_aircraft/builtin/data")
            .join(file);
        std::fs::write(out, table).unwrap();
    }
}
