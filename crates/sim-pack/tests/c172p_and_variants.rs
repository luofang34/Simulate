//! Verification of the Cessna 172P Skyhawk II (`vehicles/c172p`, `D-ECJB` default),
//! the Cessna 172S Skyhawk SP (`vehicles/c172s`, `N642SP` default + `N172SP`),
//! `VariantRegistry` aircraft-family grouping, and external livery / variant import.

use jsbsim::{Fdm, LoadOptions};
use sha2::{Digest, Sha256};
use sim_pack::{
    CalibrationReport, ExternalLiveryBundle, MANIFEST_FILE, PackError, VariantRegistry,
    builtin_c172p_package, builtin_c172s_package, evaluate_package, resolve_variant_alias,
};

const C172P_RECORD: &str = include_str!("../../../vehicles/c172p/performance/validation.json");
const POUNDS_PER_KG: f64 = 2.204_622_621_848_775_7;
const INCHES_PER_METRE: f64 = 39.370_078_740_157_48;
const KG_M2_PER_SLUG_FT2: f64 = 1.355_817_961_892_587_6;

fn listed_c172p(key: &str) -> Vec<String> {
    let record: serde_json::Value = serde_json::from_str(C172P_RECORD).unwrap_or_default();
    let mut names: Vec<String> = record
        .get(key)
        .and_then(serde_json::Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|n| n.as_str().map(str::to_owned))
        .collect();
    names.sort();
    names
}

fn card_rows(report: &CalibrationReport) -> Vec<(String, bool)> {
    let mut rows: Vec<(String, bool)> = report
        .static_runup
        .iter()
        .map(|&(_, _, _, passed)| ("static run-up".to_owned(), passed))
        .collect();
    rows.extend(report.stalls.iter().map(|r| (r.label.clone(), r.passed)));
    rows.extend(
        report
            .climbs
            .iter()
            .map(|r| (format!("climb {:.0} ft", r.pressure_alt_ft), r.passed)),
    );
    rows.extend(report.cruises.iter().map(|r| {
        (
            format!(
                "cruise {:.0} ft {:.0} rpm",
                r.pressure_alt_ft, r.commanded_rpm
            ),
            r.passed,
        )
    }));
    rows.extend(report.takeoffs.iter().map(|r| (r.label.clone(), r.passed)));
    rows
}

#[test]
fn c172s_and_c172p_ship_distinct_verified_packages_and_per_tail_catalogs() {
    let c172s = builtin_c172s_package().unwrap();
    assert_eq!(c172s.manifest().id, "c172s");
    assert_eq!(c172s.manifest().name, "Cessna 172S Skyhawk SP");
    let cat_s = c172s.livery_catalog().expect("c172s livery catalog");
    assert_eq!(cat_s.family.as_deref(), Some("c172"));
    assert_eq!(cat_s.default_livery, "n642sp");
    let n642sp = cat_s.default_entry().expect("N642SP default on c172s");
    assert_eq!(n642sp.tail_code, "N642SP");
    assert_eq!(n642sp.usd_root_layer.as_deref(), Some("usd/n642sp.usda"));
    assert!(cat_s.livery("n172sp").is_some());

    let c172p = builtin_c172p_package().unwrap();
    assert_eq!(c172p.manifest().id, "c172p");
    assert_eq!(c172p.manifest().name, "Cessna 172P Skyhawk II");
    let cat_p = c172p.livery_catalog().expect("c172p livery catalog");
    assert_eq!(cat_p.family.as_deref(), Some("c172"));
    assert_eq!(cat_p.default_livery, "d-ecjb");
    let d_ecjb = cat_p.default_entry().expect("D-ECJB default on c172p");
    assert_eq!(d_ecjb.tail_code, "D-ECJB");
    assert_eq!(d_ecjb.serial_number, "F17202087");
    assert_eq!(d_ecjb.usd_root_layer.as_deref(), Some("usd/d_ecjb.usda"));
    assert!(cat_p.livery("n53295").is_some());

    let env_p = c172p.airspeed_envelope().expect("c172p airspeed envelope");
    assert_eq!(env_p.type_designator, "C172P");
    assert_eq!(
        (env_p.vs0_kt, env_p.vs1_kt, env_p.vfe_kt),
        (33.0, 44.0, 85.0)
    );
    assert_eq!((env_p.vno_kt, env_p.vne_kt), (128.0, 160.0));
    let pec_p = c172p.pitot_static_calibration().expect("c172p PEC");
    assert!((pec_p.kcas_to_kias(1.0, 41.0) - 33.0).abs() < 0.05);
    assert!((pec_p.kcas_to_kias(0.0, 47.0) - 44.0).abs() < 0.05);
}

#[test]
fn c172p_jsbsim_mass_and_inertia_agree_with_syspack_tree() {
    let package = builtin_c172p_package().unwrap();
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_dir, model_name) = package.jsbsim_model().unwrap();
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true)).unwrap();
    fdm.load_model(&files, aircraft_dir, model_name).unwrap();
    fdm.run_ic().unwrap();

    let mass = package.tree().mass_properties().unwrap();
    let weight_lbs = fdm.get_property("inertia/weight-lbs").unwrap();
    assert!(
        (weight_lbs / POUNDS_PER_KG - mass.total_mass_kg).abs() < 1e-2,
        "{weight_lbs} lb vs {} kg",
        mass.total_mass_kg
    );
    let fuel_lbs = fdm.get_property("propulsion/total-fuel-lbs").unwrap();
    assert!((fuel_lbs / POUNDS_PER_KG - 96.0).abs() < 1e-3, "{fuel_lbs}");
    let cg_x = fdm.get_property("inertia/cg-x-in").unwrap() / INCHES_PER_METRE;
    assert!(
        (cg_x + mass.cg_body_m.x).abs() < 1e-3,
        "{cg_x} vs {}",
        mass.cg_body_m.x
    );
    for (property, tree) in [
        ("inertia/ixx-slugs_ft2", mass.inertia_cg_body.x_axis.x),
        ("inertia/iyy-slugs_ft2", mass.inertia_cg_body.y_axis.y),
        ("inertia/izz-slugs_ft2", mass.inertia_cg_body.z_axis.z),
    ] {
        let jsbsim_val = fdm.get_property(property).unwrap() * KG_M2_PER_SLUG_FT2;
        assert!(
            (jsbsim_val - tree).abs() < 0.5,
            "{property}: {jsbsim_val} vs {tree}"
        );
    }
}

#[test]
fn c172p_o320_d2j_delivers_160_bhp_at_2700_rpm_sea_level_isa() {
    let package = builtin_c172p_package().unwrap();
    let (aircraft_dir, model_name) = package.jsbsim_model().unwrap();
    const FDM_XML: &str = include_str!("../../../vehicles/c172p/fdm/c172p.xml");
    let table = FDM_XML.find(r#"<table name="C_POWER""#).unwrap();
    let open = table + FDM_XML[table..].find("<tableData>").unwrap() + "<tableData>".len();
    let close = open + FDM_XML[open..].find("</tableData>").unwrap();
    let run_cp = |cp: f64| -> (f64, f64) {
        let xml = format!(
            "{}\n0.0 {cp}\n2.0 {cp}\n{}",
            &FDM_XML[..open],
            &FDM_XML[close..]
        );
        let mut resolver = package.build_jsbsim_resolver().unwrap();
        resolver.insert("aircraft/c172p/c172p.xml", xml.into_bytes());
        let mut sim = Fdm::new().unwrap();
        sim.load_model(&resolver, aircraft_dir, model_name).unwrap();
        sim.run_ic().unwrap();
        let cg_z_in = sim.get_property("inertia/cg-z-in").unwrap();
        let wheels_z_in = (0..3)
            .map(|n| {
                sim.get_property(&format!("gear/unit[{n}]/z-position"))
                    .unwrap()
            })
            .fold(f64::INFINITY, f64::min);
        for (prop, val) in [
            ("ic/terrain-elevation-ft", 0.0),
            ("ic/h-sl-ft", (cg_z_in - wheels_z_in) / 12.0),
            ("ic/vt-fps", 0.0),
            ("ic/theta-deg", 0.0),
            ("fcs/throttle-cmd-norm", 1.0),
            ("fcs/mixture-cmd-norm", 1.0),
            ("propulsion/magneto_cmd", 3.0),
            ("propulsion/set-running", -1.0),
        ] {
            sim.set_property(prop, val).unwrap();
        }
        sim.run_ic().unwrap();
        for _ in 0..3_600 {
            sim.set_property("fcs/left-brake-cmd-norm", 1.0).unwrap();
            sim.set_property("fcs/right-brake-cmd-norm", 1.0).unwrap();
            sim.run().unwrap();
        }
        (
            sim.get_property("propulsion/engine[0]/engine-rpm").unwrap(),
            sim.get_property("propulsion/engine[0]/power-hp").unwrap(),
        )
    };
    let (mut lo, mut hi) = (0.015_f64, 0.120_f64);
    for _ in 0..22 {
        let mid = 0.5 * (lo + hi);
        let (rpm, _) = run_cp(mid);
        if rpm > 2_700.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (rpm, hp) = run_cp(0.5 * (lo + hi));
    assert!((rpm - 2_700.0).abs() < 5.0, "rpm={rpm}");
    assert!(
        (hp - 160.0).abs() < 3.0,
        "O-320-D2J rated 160 BHP @ 2,700 RPM, got {hp:.2} BHP"
    );
}

#[test]
fn c172p_poh_benchmarks_match_validation_record() {
    let c172p = builtin_c172p_package().unwrap();
    let report = evaluate_package(&c172p).unwrap();
    let md = report.to_markdown();
    let tail = report.tail_profile.as_ref().expect("D-ECJB tail profile");
    assert_eq!(tail.registration, "D-ECJB");
    assert_eq!(tail.serial_number, "F17202087");

    let rows = card_rows(&report);
    let mut passing: Vec<String> = rows.iter().filter(|r| r.1).map(|r| r.0.clone()).collect();
    let mut failing: Vec<String> = rows.iter().filter(|r| !r.1).map(|r| r.0.clone()).collect();
    passing.sort();
    failing.sort();
    assert_eq!(
        passing,
        listed_c172p("passing"),
        "c172p passing rows:\n{md}"
    );
    assert_eq!(
        failing,
        listed_c172p("failing"),
        "c172p failing rows:\n{md}"
    );
    assert!(
        report.stall_kcas_rmse_kt < 1.0,
        "stall KCAS RMSE {:.2} kt\n{md}",
        report.stall_kcas_rmse_kt
    );
    assert!(
        report.climb_rmse_fpm < 35.0,
        "climb RMSE {:.1} fpm\n{md}",
        report.climb_rmse_fpm
    );
    assert!(
        report.cruise_rmse_ktas < 2.5,
        "cruise KTAS RMSE {:.2} kt\n{md}",
        report.cruise_rmse_ktas
    );
}

const CUSTOM_D_EFGH_SPEC: &str = r#"{
  "tail": "D-EFGH",
  "name": "D-EFGH — Aero-Club Custom",
  "aircraft_id": "c172",
  "base_coat_rgb": [0.92, 0.92, 0.90],
  "materials": {
    "StripeCobalt": { "diffuse": [0.06, 0.24, 0.62], "metallic": 0.08, "roughness": 0.28 }
  },
  "stripes": [
    {
      "name": "CustomCheatline",
      "material": "StripeCobalt",
      "upper_yz": [[-1.70, -0.05], [0.50, -0.03], [3.20, 0.02]],
      "lower_yz": [[-1.70, -0.14], [0.50, -0.11], [3.20, -0.05]]
    }
  ],
  "registration": {
    "text": "D-EFGH",
    "material": "StripeCobalt",
    "center_yz": [2.25, 0.06],
    "cap_height_m": 0.30,
    "glyph_width_h": 0.6667,
    "stroke_h": 0.1667,
    "gap_h": 0.1667,
    "slant": 0.0
  }
}"#;

#[test]
fn variant_registry_groups_families_and_imports_external_liveries() {
    assert_eq!(resolve_variant_alias("c172"), "c172s");
    assert_eq!(resolve_variant_alias("c172_class_reference"), "c172s");
    assert_eq!(resolve_variant_alias("c172p"), "c172p");

    let mut registry = VariantRegistry::builtin().unwrap();
    let families = registry.families();
    let c172_fam = families.iter().find(|f| f.family == "c172").unwrap();
    assert_eq!(c172_fam.family_name, "Cessna 172 Skyhawk");
    assert_eq!(c172_fam.variant_ids, vec!["c172s", "c172p"]);
    assert_eq!(registry.variants_in_family("c172").len(), 2);
    assert_eq!(registry.variant("c172").unwrap().id, "c172s");

    let bundle = ExternalLiveryBundle::from_spec_bytes(CUSTOM_D_EFGH_SPEC.as_bytes()).unwrap();
    assert_eq!(bundle.id, "d-efgh");
    assert!(matches!(
        registry.import_external_livery("p38g", bundle.clone()),
        Err(PackError::LiveryImport { .. })
    ));
    let imported = registry.import_external_livery("c172p", bundle).unwrap();
    assert_eq!(imported.id, "d-efgh");
    assert_eq!(imported.tail_code, "D-EFGH");
    assert!(
        registry
            .variant("c172p")
            .unwrap()
            .livery_catalog()
            .unwrap()
            .livery("d-efgh")
            .is_some()
    );
    assert!(
        registry
            .variant("c172p")
            .unwrap()
            .external_livery("d-efgh")
            .is_some()
    );

    let collision_spec = CUSTOM_D_EFGH_SPEC.replace("D-EFGH", "D-ECJB");
    let collision_bundle =
        ExternalLiveryBundle::from_spec_bytes(collision_spec.as_bytes()).unwrap();
    assert!(matches!(
        registry.import_external_livery("c172p", collision_bundle),
        Err(PackError::LiveryImport { .. })
    ));
}

#[test]
fn variant_registry_imports_external_syspack_variant_directory() {
    let mut registry = VariantRegistry::builtin().unwrap();
    let src_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../vehicles/c172p");
    assert!(matches!(
        registry.import_variant_dir(&src_dir),
        Err(PackError::LiveryImport { .. })
    ));

    let temp = tempfile::tempdir().unwrap();
    for entry_key in registry
        .variant("c172p")
        .unwrap()
        .package
        .manifest()
        .assets
        .keys()
    {
        let dst = temp.path().join(entry_key);
        if let Some(parent) = dst.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        std::fs::copy(src_dir.join(entry_key), dst).unwrap();
    }
    let cat_path = temp.path().join("Liveries/catalog.json");
    let cat_edited = std::fs::read_to_string(&cat_path).unwrap().replace(
        "\"aircraft_id\": \"c172p\"",
        "\"aircraft_id\": \"c172r-ext\"",
    );
    std::fs::write(&cat_path, &cat_edited).unwrap();
    let cat_sha: String = Sha256::digest(cat_edited.as_bytes())
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    let mut manifest_val: serde_json::Value =
        serde_json::from_slice(&std::fs::read(src_dir.join(MANIFEST_FILE)).unwrap()).unwrap();
    manifest_val["id"] = serde_json::json!("c172r-ext");
    manifest_val["name"] = serde_json::json!("Cessna 172R Skyhawk (External)");
    manifest_val["assets"]["Liveries/catalog.json"] = serde_json::json!({
        "sha256": cat_sha,
        "size_bytes": cat_edited.len()
    });
    std::fs::write(
        temp.path().join(MANIFEST_FILE),
        serde_json::to_vec_pretty(&manifest_val).unwrap(),
    )
    .unwrap();

    let ext_id = registry.import_variant_dir(temp.path()).unwrap();
    assert_eq!(ext_id, "c172r-ext");
    assert!(registry.variant("c172r-ext").unwrap().is_external);
    assert_eq!(registry.variants_in_family("c172").len(), 3);
}
