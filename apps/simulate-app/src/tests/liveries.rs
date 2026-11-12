//! End-to-end variant, per-tail livery selection, and external livery/variant import tests.

use std::path::PathBuf;

use sim_core::ControlInputs;
use sim_scenario::SimTimeScale;

use crate::error::AppSimError;
use crate::flight::Flight;
use crate::vehicles::FreeFlightVehicle;

#[test]
fn c172s_and_c172p_variants_expose_default_liveries_and_switch_3d_tail_stages()
-> Result<(), AppSimError> {
    let mut flight = Flight::new()?;
    assert_eq!(flight.free_vehicle(), Some(FreeFlightVehicle::JsbsimC172));

    // C-172S defaults to N642SP.
    let (idx_s, def_s) = flight
        .liveries()
        .active_livery("c172s")
        .expect("c172s default livery");
    assert_eq!(idx_s, 0);
    assert_eq!(def_s.tail_code, "N642SP");
    assert_eq!(def_s.id, "n642sp");

    // Switching C-172S to N172SP composes its OpenUSD stage override.
    let chosen_s = flight.select_livery("c172s", "N172SP")?;
    assert_eq!(chosen_s.tail_code, "N172SP");
    let ((aircraft_id, liv_id, _gen), asset_s) = flight
        .active_ownship_livery_override()
        .expect("active USD override for N172SP");
    assert_eq!(aircraft_id, "c172s");
    assert_eq!(liv_id, "n172sp");
    assert!(asset_s.parts().len() > 20);

    // Switch free-flight plant to C-172P Skyhawk II (D-ECJB default).
    flight.select_vehicle(FreeFlightVehicle::JsbsimC172p)?;
    flight.advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
    assert_eq!(flight.free_vehicle(), Some(FreeFlightVehicle::JsbsimC172p));

    let env_p = flight
        .resolved_airspeed_envelope()
        .expect("C-172P airspeed envelope");
    assert!(
        (env_p.vne_kt - 160.0).abs() < 0.5,
        "172P Vne {}",
        env_p.vne_kt
    );
    assert!(
        (env_p.vno_kt - 128.0).abs() < 0.5,
        "172P Vno {}",
        env_p.vno_kt
    );
    assert!(flight.pitot_static_calibration().is_some());

    let (_, def_p) = flight
        .liveries()
        .active_livery("c172p")
        .expect("c172p default livery");
    assert_eq!(def_p.tail_code, "D-ECJB");
    assert_eq!(def_p.id, "d-ecjb");

    // Cycle C-172P to N53295 and verify its composed USD stage.
    let cycled = flight
        .cycle_active_livery(true)?
        .expect("cycled 172P livery");
    assert_eq!(cycled.tail_code, "N53295");
    let ((aircraft_p, liv_p, _), asset_p) = flight
        .active_ownship_livery_override()
        .expect("active USD override for N53295");
    assert_eq!(aircraft_p, "c172p");
    assert_eq!(liv_p, "n53295");
    assert!(asset_p.parts().len() > 20);
    Ok(())
}

const CUSTOM_G_SIMX_JSON: &[u8] = br#"{
    "id": "g-simx",
    "tail": "G-SIMX",
    "name": "G-SIMX Custom Club Scheme",
    "aircraft_id": "c172p",
    "base_coat_rgb": [0.95, 0.95, 0.96],
    "materials": {
        "ClubBlue": { "diffuse": [0.10, 0.36, 0.68], "metallic": 0.10, "roughness": 0.22 },
        "RegDark": { "diffuse": [0.06, 0.08, 0.12], "metallic": 0.0, "roughness": 0.35 }
    },
    "stripes": [
        {
            "name": "ClubCheatline",
            "what": "Port and starboard blue cheatline",
            "material": "ClubBlue",
            "upper_yz": [[-1.60, 0.03], [2.40, 0.19]],
            "lower_yz": [[-1.60, -0.05], [2.40, 0.11]]
        }
    ],
    "registration": {
        "text": "G-SIMX",
        "material": "RegDark",
        "center_yz": [1.55, 0.05],
        "regulation": "UK CAA CAP 523",
        "cap_height_m": 0.30,
        "glyph_width_h": 0.6667,
        "stroke_h": 0.1667,
        "gap_h": 0.1667,
        "slant": 0.0
    }
}"#;

#[test]
fn external_livery_bytes_import_and_compose_onto_usd_mesh() -> Result<(), AppSimError> {
    let mut flight = Flight::new()?;
    let before_count = flight.liveries().liveries_for("c172p").len();
    let (aircraft_id, imported) =
        flight
            .liveries_mut()
            .import_livery_bytes(None, CUSTOM_G_SIMX_JSON, None)?;
    assert_eq!(aircraft_id, "c172p");
    assert_eq!(imported.tail_code, "G-SIMX");
    assert_eq!(
        flight.liveries().liveries_for("c172p").len(),
        before_count + 1
    );

    flight.select_vehicle(FreeFlightVehicle::JsbsimC172p)?;
    let ((ov_aircraft, ov_liv, _), composed_asset) = flight
        .active_ownship_livery_override()
        .expect("imported G-SIMX USD asset cached");
    assert_eq!(ov_aircraft, "c172p");
    assert_eq!(ov_liv, "g-simx");
    assert!(composed_asset.parts().len() > 22);
    Ok(())
}

#[test]
fn external_livery_path_and_variant_packs_import_cleanly() -> Result<(), AppSimError> {
    let mut flight = Flight::new()?;
    let tmp = std::env::temp_dir().join(format!(
        "ext_livery_and_variant_test_{}",
        std::process::id()
    ));
    drop(std::fs::remove_dir_all(&tmp));
    std::fs::create_dir_all(&tmp).expect("create temp dir");
    let json_path = tmp.join("hb_cjb_livery.json");
    let incompatible_json = String::from_utf8_lossy(CUSTOM_G_SIMX_JSON)
        .replace("g-simx", "hb-cjb")
        .replace("G-SIMX", "HB-CJB");
    std::fs::write(&json_path, incompatible_json.as_bytes()).expect("write incompatible json");
    assert!(
        flight
            .liveries_mut()
            .import_livery_path(&json_path, Some("c172s"))
            .is_err(),
        "c172p-targeted livery must reject explicit c172s override"
    );

    let hb_json =
        incompatible_json.replace(r#""aircraft_id": "c172p""#, r#""aircraft_id": "c172""#);
    std::fs::write(&json_path, hb_json.as_bytes()).expect("write hb_cjb_livery.json");
    let (_, hb_liv) = flight
        .liveries_mut()
        .import_livery_path(&json_path, Some("c172s"))?;
    assert_eq!(hb_liv.tail_code, "HB-CJB");
    flight.select_vehicle(FreeFlightVehicle::JsbsimC172)?;
    let ((ov_s, ov_hb, _), hb_asset) = flight
        .active_ownship_livery_override()
        .expect("HB-CJB composed onto c172s USD mesh");
    assert_eq!(ov_s, "c172s");
    assert_eq!(ov_hb, "hb-cjb");
    assert!(hb_asset.parts().len() > 22);

    let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../vehicles/c172p");
    let ext_dir = tmp.join("c172p_club");
    copy_dir_recursive(&src_dir, &ext_dir);
    let manifest_path = ext_dir.join("syspack.json");
    let manifest_txt = std::fs::read_to_string(&manifest_path).expect("read syspack.json");
    let patched = manifest_txt.replacen(r#""id": "c172p""#, r#""id": "c172p_club""#, 1);
    std::fs::write(&manifest_path, patched.as_bytes()).expect("write patched syspack.json");
    let imported_variant = flight.liveries_mut().import_variant_dir(&ext_dir)?;
    assert_eq!(imported_variant, "c172p_club");
    assert!(flight.liveries().registry().variant("c172p_club").is_some());
    drop(std::fs::remove_dir_all(&tmp));
    Ok(())
}

fn copy_dir_recursive(src: &std::path::Path, dst: &std::path::Path) {
    std::fs::create_dir_all(dst).expect("mkdir");
    for entry in std::fs::read_dir(src).expect("readdir").flatten() {
        let path = entry.path();
        let out = dst.join(entry.file_name());
        if path.is_dir() {
            copy_dir_recursive(&path, &out);
        } else {
            std::fs::copy(&path, &out).expect("copy file");
        }
    }
}
