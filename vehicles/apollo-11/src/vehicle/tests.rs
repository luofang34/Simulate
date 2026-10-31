use mbd_core::ComponentId;
use serde_json::json;
use sim_pack::{DeviceKindSpec, DynamicsSpec, SystemKind};

use super::*;
use crate::{PIPA_PULSE_CM_M_S, PIPA_PULSE_LM_M_S};

fn name_of(tree: &mbd_core::ComponentTree, id: ComponentId) -> &str {
    &tree.spec(id).unwrap().name
}

#[test]
fn lm_ids_name_the_components_they_claim() {
    let tree = build_apollo_lm_tree().unwrap();
    for (id, expected) in [
        (LM_ROOT, "LM reference"),
        (LM_ASCENT_STRUCTURE, "Ascent stage structure"),
        (LM_APS_TANK, "APS propellant tank"),
        (LM_RCS_TANK, "RCS propellant tank"),
        (LM_NAV_BASE, "Navigation base"),
        (LM_IMU, "PGNCS IMU"),
        (LM_AOT, "Alignment Optical Telescope"),
        (LM_ASA, "Abort Sensor Assembly"),
        (LM_RGA, "Rate Gyro Assembly"),
        (LM_RENDEZVOUS_RADAR, "Rendezvous radar"),
        (LM_CDR_EYE, "Commander's eye"),
        (LM_DESCENT_STRUCTURE, "Descent stage structure"),
        (LM_DPS_TANK, "DPS propellant tank"),
        (LM_LANDING_RADAR, "Landing radar"),
        (LM_LEG_PLUS_Z, "Forward leg"),
        (LM_LEG_MINUS_Z, "Aft leg"),
        (LM_LEG_PLUS_Y, "Right leg"),
        (LM_LEG_MINUS_Y, "Left leg"),
    ] {
        assert_eq!(name_of(&tree, id), expected);
    }
    let [aft, right, left] = LM_PROBE_TIPS;
    assert_eq!(name_of(&tree, aft), "Aft contact probe");
    assert_eq!(name_of(&tree, right), "Right contact probe");
    assert_eq!(name_of(&tree, left), "Left contact probe");
    assert_eq!(tree.len(), 21);
}

#[test]
fn csm_ids_name_the_components_they_claim() {
    let tree = build_apollo_csm_tree().unwrap();
    for (id, expected) in [
        (CM_ROOT, "CM reference"),
        (CM_STRUCTURE, "Command Module capsule"),
        (CM_NAV_BASE, "Navigation base"),
        (CM_IMU, "PGNCS IMU"),
        (CM_BMAG, "Body-mounted attitude gyros"),
        (CM_OPTICS, "Sextant and Scanning Telescope"),
        (CM_EMS, "Entry Monitor System"),
        (CSM_SERVICE_MODULE, "Service Module"),
        (CSM_SPS_TANK, "SPS propellant tank"),
    ] {
        assert_eq!(name_of(&tree, id), expected);
    }
    assert_eq!(tree.len(), 9);
}

#[test]
fn the_lm_masses_add_up_to_eagle_and_the_cg_sits_inside_it() {
    let tree = build_apollo_lm_tree().unwrap();
    let mass = tree.mass_properties().unwrap();
    assert!(
        (mass.total_mass_kg - 15_102.0).abs() < 1.0,
        "{}",
        mass.total_mass_kg
    );
    assert!(mass.cg_body_m.x < 0.0, "fuelled, it is bottom heavy");
    assert!(mass.cg_body_m.length() < 1.0, "{}", mass.cg_body_m);
    let dps = tree.state(LM_DPS_TANK).unwrap();
    assert!(dps.attached);
    assert!((dps.fluid_mass_kg - 8_200.0).abs() < 100.0);
}

#[test]
fn every_contact_probe_hangs_below_its_footpad() {
    let tree = build_apollo_lm_tree().unwrap();
    for (leg, tip) in [LM_LEG_MINUS_Z, LM_LEG_PLUS_Y, LM_LEG_MINUS_Y]
        .into_iter()
        .zip(LM_PROBE_TIPS)
    {
        let (footpad_m, _) = tree.world_transform_in_root(leg).unwrap();
        let (tip_m, _) = tree.world_transform_in_root(tip).unwrap();
        assert_eq!(footpad_m - tip_m, glam::DVec3::X * PROBE_LENGTH_M);
    }
}

#[test]
fn separating_the_descent_stage_takes_propellant_gear_and_radar_with_it() {
    let mut tree = build_apollo_lm_tree().unwrap();
    let mut stage = *tree.state(LM_DESCENT_STRUCTURE).unwrap();
    stage.attached = false;
    tree.set_state(LM_DESCENT_STRUCTURE, stage).unwrap();
    for id in [
        LM_DPS_TANK,
        LM_LANDING_RADAR,
        LM_LEG_PLUS_Z,
        LM_PROBE_TIPS[0],
    ] {
        assert!(!tree.is_attached(id).unwrap(), "{id}");
    }
    for id in [LM_IMU, LM_ASA, LM_APS_TANK] {
        assert!(tree.is_attached(id).unwrap(), "{id}");
    }
    let mass = tree.mass_properties().unwrap();
    assert!(
        (mass.total_mass_kg - 4_820.0).abs() < 1.0,
        "{}",
        mass.total_mass_kg
    );
    assert!(mass.cg_body_m.x > 1.0);
}

#[test]
fn the_cm_alone_keeps_its_offset_cg_and_the_sm_dominates_the_stack() {
    let mut tree = build_apollo_csm_tree().unwrap();
    let stack = tree.mass_properties().unwrap();
    assert!(stack.cg_body_m.x > 1.0, "{}", stack.cg_body_m);
    let mut service = *tree.state(CSM_SERVICE_MODULE).unwrap();
    service.attached = false;
    tree.set_state(CSM_SERVICE_MODULE, service).unwrap();
    let capsule = tree.mass_properties().unwrap();
    assert_eq!(capsule.total_mass_kg, 5_560.0);
    assert!((capsule.cg_body_m.z.abs() - 0.17).abs() < 1e-12);
}

#[test]
fn the_capsule_trims_near_twenty_degrees() {
    let aero = apollo_cm_aero().unwrap();
    assert_eq!(aero.params().base_radius_m, 1.9558);
    assert_eq!(aero.params().nose_radius_m, 4.69);
    let trim = cm_analytic_trim_incidence_rad().unwrap();
    assert!(trim < 0.0 && (trim.abs() - 20f64.to_radians()).abs() < 2f64.to_radians());
}

#[test]
fn the_aot_shaft_lies_along_the_thrust_axis() {
    let mount = aot_mount();
    assert!((mount * glam::DVec3::Z - glam::DVec3::X).length() < 1e-12);
    assert!((mount * glam::DVec3::X - glam::DVec3::Z).length() < 1e-12);
}

#[test]
fn every_lm_key_resolves_to_its_id_constant() {
    let package = apollo_lm_package().unwrap();
    let [aft, right, left] = LM_PROBE_TIPS;
    for (key, id) in [
        ("lm_root", LM_ROOT),
        ("ascent_stage", LM_ASCENT_STRUCTURE),
        ("aps_tank", LM_APS_TANK),
        ("rcs_tank", LM_RCS_TANK),
        ("nav_base", LM_NAV_BASE),
        ("imu_mount", LM_IMU),
        ("aot", LM_AOT),
        ("asa", LM_ASA),
        ("rga", LM_RGA),
        ("rendezvous_radar", LM_RENDEZVOUS_RADAR),
        ("cdr_eye", LM_CDR_EYE),
        ("descent_stage", LM_DESCENT_STRUCTURE),
        ("dps_tank", LM_DPS_TANK),
        ("landing_radar", LM_LANDING_RADAR),
        ("leg_plus_z", LM_LEG_PLUS_Z),
        ("leg_minus_z", LM_LEG_MINUS_Z),
        ("leg_plus_y", LM_LEG_PLUS_Y),
        ("leg_minus_y", LM_LEG_MINUS_Y),
        ("probe_aft", aft),
        ("probe_right", right),
        ("probe_left", left),
    ] {
        assert_eq!(package.component_id(key).unwrap(), id, "{key}");
    }
}

#[test]
fn every_csm_key_resolves_to_its_id_constant() {
    let package = apollo_csm_package().unwrap();
    for (key, id) in [
        ("cm_root", CM_ROOT),
        ("cm_structure", CM_STRUCTURE),
        ("nav_base", CM_NAV_BASE),
        ("imu_mount", CM_IMU),
        ("bmag", CM_BMAG),
        ("optics", CM_OPTICS),
        ("ems", CM_EMS),
        ("service_module", CSM_SERVICE_MODULE),
        ("sps_tank", CSM_SPS_TANK),
    ] {
        assert_eq!(package.component_id(key).unwrap(), id, "{key}");
    }
}

#[test]
fn the_packages_declare_their_domains_and_dynamics() {
    let lm = apollo_lm_package().unwrap();
    assert_eq!(lm.manifest().kind, SystemKind::LunarLander);
    assert_eq!(lm.manifest().dynamics, DynamicsSpec::RigidBodyVacuum);
    let csm = apollo_csm_package().unwrap();
    assert_eq!(csm.manifest().kind, SystemKind::ReentryCapsule);
    assert_eq!(csm.manifest().dynamics.kind(), "RigidBodyBluntCapsule");
}

/// `(type_id, site, parameters)` of every custom device.
fn custom_devices(
    package: &sim_pack::LoadedSyspack,
) -> Vec<(&str, ComponentId, &serde_json::Value)> {
    package
        .manifest()
        .devices
        .iter()
        .map(|device| {
            let DeviceKindSpec::Custom {
                type_id,
                parameters,
            } = &device.kind
            else {
                panic!("{device:?}");
            };
            let site = package.component_id(&device.site).unwrap();
            (type_id.as_str(), site, parameters)
        })
        .collect()
}

#[test]
fn eagle_declares_its_sensors_on_their_sites() {
    let package = apollo_lm_package().unwrap();
    let expected = [
        ("apollo.pgncs_imu", LM_IMU),
        ("apollo.strapdown_asa", LM_ASA),
        ("apollo.rate_gyro", LM_RGA),
        ("apollo.optical_sight", LM_AOT),
        ("apollo.landing_radar", LM_LANDING_RADAR),
        ("apollo.rendezvous_radar", LM_RENDEZVOUS_RADAR),
        ("apollo.lunar_contact_probes", LM_DESCENT_STRUCTURE),
    ];
    let declared = custom_devices(&package);
    let sites: Vec<_> = declared.iter().map(|&(id, site, _)| (id, site)).collect();
    assert_eq!(sites, expected);
    let parameters = |id| declared.iter().find(|d| d.0 == id).unwrap().2;
    assert_eq!(
        parameters("apollo.pgncs_imu"),
        &json!({ "pipa_m_s_per_pulse": PIPA_PULSE_LM_M_S })
    );
    assert_eq!(
        parameters("apollo.optical_sight"),
        &json!({ "sight": "Detented" })
    );
    let probes = parameters("apollo.lunar_contact_probes");
    assert_eq!(probes["probe_length_m"], json!(PROBE_LENGTH_M));
    let tips: Vec<_> = probes["tips"]
        .as_array()
        .unwrap()
        .iter()
        .map(|key| package.component_id(key.as_str().unwrap()).unwrap())
        .collect();
    assert_eq!(tips, LM_PROBE_TIPS);
}

#[test]
fn columbia_declares_its_sensors_on_their_sites() {
    let package = apollo_csm_package().unwrap();
    let expected = [
        ("apollo.pgncs_imu", CM_IMU),
        ("apollo.rate_gyro", CM_BMAG),
        ("apollo.optical_sight", CM_OPTICS),
        ("apollo.entry_monitor", CM_EMS),
        ("apollo.vhf_ranging", CM_STRUCTURE),
    ];
    let declared = custom_devices(&package);
    let sites: Vec<_> = declared.iter().map(|&(id, site, _)| (id, site)).collect();
    assert_eq!(sites, expected);
    let parameters = |id| declared.iter().find(|d| d.0 == id).unwrap().2;
    assert_eq!(
        parameters("apollo.pgncs_imu"),
        &json!({ "pipa_m_s_per_pulse": PIPA_PULSE_CM_M_S })
    );
    assert_eq!(
        parameters("apollo.optical_sight"),
        &json!({ "sight": "Tracking" })
    );
}

#[test]
fn the_aot_rotation_in_the_package_is_the_sight_mount() {
    let tree = build_apollo_lm_tree().unwrap();
    let rotation = tree.spec(LM_AOT).unwrap().placement.rotation;
    assert_eq!(rotation.to_array(), aot_mount().to_array());
}
