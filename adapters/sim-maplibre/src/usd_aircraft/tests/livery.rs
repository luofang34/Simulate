use sim_pack::{
    ExternalLiveryBundle, VariantRegistry, builtin_c172p_package, builtin_c172s_package,
};

use super::rest_extremes;
use crate::UsdAircraftAsset;

#[test]
fn c172s_selectable_n172sp_wears_sapphire_swoosh_and_12_inch_registration() {
    let tail = UsdAircraftAsset::embedded_c172s_n172sp().unwrap();
    let part = |suffix: &str| {
        tail.parts()
            .iter()
            .find(|p| p.prim_path.ends_with(suffix))
            .unwrap_or_else(|| panic!("no part {suffix}"))
    };
    let (_, nose) = rest_extremes(&tail, "/NoseWheel/Tire");
    let (_, main) = rest_extremes(&tail, "/WheelLeft/Tire");
    assert!(((nose.y - main.y) / 0.0254 - 65.0).abs() < 0.1);
    for (suffix, side) in [
        ("/LiveryRegistrationPort", -1.0_f32),
        ("/LiveryRegistrationStarboard", 1.0),
    ] {
        let reg = part(suffix);
        assert!(
            reg.vertices
                .iter()
                .all(|v| (0.04..0.6).contains(&(v.position[0] * side)))
        );
        let (lo, hi) = reg
            .vertices
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v.position[2]), hi.max(v.position[2]))
            });
        assert!(((hi - lo) - 0.3048).abs() < 0.01, "{suffix}: {}", hi - lo);
    }
    for stripe in [
        "/LiverySwooshSapphirePort",
        "/LiverySwooshSapphireStarboard",
        "/LiverySwooshSilverPort",
        "/LiveryAccentRubyPort",
    ] {
        assert!(part(stripe).indices.len() >= 3 * 40, "{stripe}");
    }
}

#[test]
fn c172p_default_d_ecjb_wears_cheatlines_german_fin_flash_and_30cm_registration() {
    let tail = UsdAircraftAsset::embedded_c172p_d_ecjb().unwrap();
    let part = |suffix: &str| {
        tail.parts()
            .iter()
            .find(|p| p.prim_path.ends_with(suffix))
            .unwrap_or_else(|| panic!("no part {suffix}"))
    };
    let (_, nose) = rest_extremes(&tail, "/NoseWheel/Tire");
    let (_, main) = rest_extremes(&tail, "/WheelLeft/Tire");
    assert!(((nose.y - main.y) / 0.0254 - 65.0).abs() < 0.1);
    for (suffix, side) in [
        ("/LiveryRegistrationPort", -1.0_f32),
        ("/LiveryRegistrationStarboard", 1.0),
    ] {
        let reg = part(suffix);
        assert!(
            reg.vertices
                .iter()
                .all(|v| (0.02..0.6).contains(&(v.position[0] * side)))
        );
        let (lo, hi) = reg
            .vertices
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v.position[2]), hi.max(v.position[2]))
            });
        // 30 cm ICAO Annex 7 / LuftVZO fuselage marks.
        assert!(((hi - lo) - 0.30).abs() < 0.015, "{suffix}: {}", hi - lo);
    }
    for stripe in [
        "/LiveryCheatlineRedPort",
        "/LiveryCheatlineCrimsonStarboard",
    ] {
        assert!(part(stripe).indices.len() >= 3 * 50, "{stripe}");
    }
    for band in [
        "/LiveryFinFlagBlackPort",
        "/LiveryFinFlagRedPort",
        "/LiveryFinFlagGoldPort",
        "/LiveryFinFlagBlackStarboard",
        "/LiveryFinFlagRedStarboard",
        "/LiveryFinFlagGoldStarboard",
    ] {
        assert!(!part(band).indices.is_empty(), "{band}");
    }
}

#[test]
fn variant_livery_loader_switches_builtin_tails() {
    let registry = VariantRegistry::builtin().unwrap();
    let c172s = registry.variant("c172s").unwrap();
    assert_eq!(c172s.default_livery_id(), Some("n642sp"));
    let n642sp = UsdAircraftAsset::from_variant_livery(c172s, "n642sp").unwrap();
    let n172sp = UsdAircraftAsset::from_variant_livery(c172s, "n172sp").unwrap();
    assert!(
        n642sp
            .parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/LiverySwooshRedPort"))
    );
    assert!(
        n172sp
            .parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/LiverySwooshSapphirePort"))
    );

    let c172p = registry.variant("c172p").unwrap();
    let d_ecjb = UsdAircraftAsset::from_variant_livery(c172p, "d-ecjb").unwrap();
    let n53295 = UsdAircraftAsset::from_variant_livery(c172p, "n53295").unwrap();
    assert!(
        d_ecjb
            .parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/LiveryFinFlagGoldStarboard"))
    );
    assert!(
        !n53295
            .parts()
            .iter()
            .any(|p| p.prim_path.contains("FinFlag"))
    );

    let pkg_s = builtin_c172s_package().unwrap();
    let pkg_p = builtin_c172p_package().unwrap();
    assert!(UsdAircraftAsset::from_syspack_livery(&pkg_s, "n642sp").is_ok());
    assert!(UsdAircraftAsset::from_syspack_livery(&pkg_p, "d-ecjb").is_ok());
}

#[test]
fn variant_livery_loader_composes_external_json_bundle() {
    let mut registry = VariantRegistry::builtin().unwrap();
    let custom_json = r#"{
        "id": "d-egac",
        "tail": "D-EGAC",
        "name": "D-EGAC — Aero-Club Hamburg",
        "aircraft_id": "c172p",
        "base_coat_rgb": [0.95, 0.95, 0.93],
        "materials": {
            "MatCobalt": { "diffuse": [0.06, 0.22, 0.56], "metallic": 0.10, "roughness": 0.28 },
            "MatInk": { "diffuse": [0.04, 0.05, 0.07], "metallic": 0.02, "roughness": 0.35 },
            "MatGold": { "diffuse": [0.90, 0.68, 0.04], "metallic": 0.05, "roughness": 0.30 }
        },
        "skin_overrides": {
            "CabinStripe": "MatWhitePaint",
            "NoseCowlingStripe": "MatWhitePaint"
        },
        "stripes": [
            {
                "name": "StripeCobalt",
                "material": "MatCobalt",
                "upper_yz": [[1.60, -0.04], [0.20, -0.12], [-2.20, -0.19], [-4.00, -0.17]],
                "lower_yz": [[1.60, -0.12], [0.20, -0.23], [-2.20, -0.28], [-4.00, -0.23]]
            }
        ],
        "fin_flash": [
            {
                "name": "FinFlagGold",
                "material": "MatGold",
                "upper_yz": [[-4.85, 0.87], [-5.24, 0.87]],
                "lower_yz": [[-4.78, 0.78], [-5.17, 0.78]]
            }
        ],
        "registration": {
            "text": "D-EGAC",
            "material": "MatInk",
            "center_yz": [-3.10, -0.02],
            "cap_height_m": 0.30,
            "glyph_width_h": 0.6667,
            "stroke_h": 0.1667,
            "gap_h": 0.1667,
            "slant": 0.12
        }
    }"#;
    let bundle = ExternalLiveryBundle::from_spec_bytes(custom_json.as_bytes()).unwrap();
    registry.import_external_livery("c172p", bundle).unwrap();
    let composed =
        UsdAircraftAsset::from_variant_livery(registry.variant("c172p").unwrap(), "d-egac")
            .unwrap();
    assert!(
        composed
            .parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/LiveryStripeCobaltStarboard"))
    );
    assert!(
        composed
            .parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/LiveryRegistrationPort"))
    );
    assert!(
        composed
            .parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/LiveryFinFlagGoldStarboard"))
    );
}
