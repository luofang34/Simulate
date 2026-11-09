use glam::{DMat3, DQuat, DVec3};
use sim_core::{AirframeArticulation, ControlInputs, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic};

use super::*;

#[test]
fn embedded_c172_composes_articulation_and_usda_export_roundtrips() {
    let asset = UsdAircraftAsset::embedded_c172();
    assert!(asset.is_ok(), "embedded c172.usdc failed: {asset:?}");
    let Some(asset) = asset.ok() else {
        return;
    };
    assert!(asset.parts().len() >= 18);
    assert!(asset.triangle_count() > 10_000);

    let origin = Geodetic {
        lat_rad: 40.6895_f64.to_radians(),
        lon_rad: (-74.1745_f64).to_radians(),
        height_m: 120.0,
    };
    let state = VehicleState {
        position_ecef: BodyEllipsoid::WGS84.geodetic_to_fixed(&origin),
        velocity_ecef: DVec3::new(0.0, 55.0, 0.0),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let art = AirframeArticulation::from_kinematic(
        &state,
        ControlInputs {
            roll: 0.25,
            pitch: 0.1,
            yaw: -0.15,
            throttle: 0.72,
        },
        4.2,
    );

    let ext_draws = asset.evaluate_draws(&art, SlotCameraRole::External);
    let hud_draws = asset.evaluate_draws(&art, SlotCameraRole::HudForward);
    let cockpit_draws = asset.evaluate_draws(&art, SlotCameraRole::CockpitInterior);
    let nadir_draws = asset.evaluate_draws(&art, SlotCameraRole::BellyNadir);
    assert!(ext_draws.len() >= cockpit_draws.len());
    assert!(hud_draws.is_empty());
    let seen = |role: PartRole| {
        cockpit_draws
            .iter()
            .any(|d| asset.parts()[d.part_index].role == role)
    };
    assert!(seen(PartRole::NoseCowling) && seen(PartRole::WingStructure));
    assert!(nadir_draws.is_empty());

    let usda = export_scene_usda(&kewr_scene(origin, state, art));
    assert!(usda.is_ok(), "USDA scene export failed: {usda:?}");
    let Some(usda_text) = usda.ok() else {
        return;
    };
    assert!(usda_text.contains("sim:engineRpm"));
    assert!(usda_text.contains("c172.usdc"));

    let mut resolver = EmbeddedUsdResolver::single("c172.usdc", EMBEDDED_C172_USDC.into());
    resolver.insert("scene.usda", usda_text.into_bytes().into());
    let composed = openusd::usd::Stage::builder()
        .schema_registry(openusd_schemas::schema_registry())
        .resolver(resolver)
        .open("scene.usda");
    assert!(composed.is_ok(), "composed scene.usda failed: {composed:?}");
    if let Ok(stage) = composed {
        let spinner = stage
            .prim("/World/Ownship/Powerplant/PropellerHub/Spinner")
            .ok()
            .and_then(|p| p.type_name().ok().flatten());
        assert_eq!(spinner.as_ref().map(|t| t.as_str()), Some("Mesh"));
    }
}

fn kewr_scene(
    origin: Geodetic,
    state: VehicleState,
    art: AirframeArticulation,
) -> UsdSceneExportInput {
    UsdSceneExportInput {
        scenario_id: "KEWR".to_owned(),
        sim_time_s: 4.2,
        enu_origin: origin,
        c172_asset_path: "c172.usdc".to_owned(),
        ownship_state: state,
        ownship_articulation: art,
        traffic: Vec::new(),
        runways: vec![UsdRunwaySample {
            ident: "22L".to_owned(),
            threshold_geodetic: origin,
            opposite_geodetic: Geodetic {
                lat_rad: 40.6700_f64.to_radians(),
                lon_rad: (-74.1900_f64).to_radians(),
                height_m: 5.0,
            },
            width_m: 46.0,
        }],
        sun_direction_enu: [0.4, -0.3, 0.866],
        cameras: vec![UsdCameraSample {
            name: "Chase".to_owned(),
            eye_geodetic: origin,
            camera_to_enu_rotation: DMat3::IDENTITY,
            vfov_deg: 50.0,
            aspect: 16.0 / 9.0,
            clipping_range_m: [0.5, 80_000.0],
        }],
    }
}

#[test]
fn exported_ownship_axes_point_right_forward_and_up_in_the_stage_frame() {
    let origin = Geodetic::from_degrees(40.69, -74.17, 10.0);
    let frame = sim_geo::LocalFrame::at(&origin);
    // Level, heading east: forward is East, right is South, up is Up.
    let state = VehicleState::level_flight(&origin, std::f64::consts::FRAC_PI_2, 50.0);
    let q = export::aircraft_usd_to_enu_quat(&frame, state.attitude);
    let close = |a: DVec3, b: DVec3| (a - b).length() < 1e-6;
    assert!(close(q * DVec3::Y, DVec3::X), "{:?}", q * DVec3::Y);
    assert!(close(q * DVec3::X, DVec3::NEG_Y), "{:?}", q * DVec3::X);
    assert!(close(q * DVec3::Z, DVec3::Z), "{:?}", q * DVec3::Z);
}

#[test]
fn embedded_raketa_loads_meshes_and_promotes_porthole_glass() {
    let asset = UsdAircraftAsset::embedded_raketa();
    assert!(
        asset.is_ok(),
        "embedded retro_rocket_v4.usd failed: {asset:?}"
    );
    let Some(asset) = asset.ok() else {
        return;
    };
    assert_eq!(asset.parts().len(), 186);
    assert!(asset.triangle_count() > 20_000);
    let glass = asset
        .parts()
        .iter()
        .find(|p| p.prim_path == "/Rocket/Window/Glass");
    assert!(glass.is_some(), "missing /Rocket/Window/Glass");
    if let Some(glass) = glass {
        assert_eq!(glass.role, PartRole::CabinGlass);
        assert!(glass.material.opacity < 0.5);
    }
}

/// The blur shader reads a propeller disc's texture coordinates as the unit disc about the
/// hub; coordinates outside it draw nothing.
#[test]
fn every_propeller_disc_spans_the_unit_disc_in_texture_coordinates() {
    for (name, asset) in [
        ("c172", UsdAircraftAsset::embedded_c172().unwrap()),
        ("p38g", UsdAircraftAsset::p38g().unwrap()),
    ] {
        let discs: Vec<_> = asset
            .parts()
            .iter()
            .filter(|p| p.role == PartRole::PropellerDisc)
            .collect();
        assert!(!discs.is_empty(), "{name} has a propeller disc");
        for disc in discs {
            let radii: Vec<f32> = disc
                .vertices
                .iter()
                .map(|v| v.uv[0].hypot(v.uv[1]))
                .collect();
            let (least, most) = radii.iter().fold((f32::INFINITY, 0.0_f32), |(lo, hi), r| {
                (lo.min(*r), hi.max(*r))
            });
            // A disc may be an annulus that starts at its spinner.
            assert!(least < 0.2, "{name} {}: hub at {least}", disc.prim_path);
            assert!(
                (0.95..=1.001).contains(&most),
                "{name} {}: rim at {most}",
                disc.prim_path
            );
        }
    }
}

/// A camera with a short exposure records a narrower blade smear than the eye does.
#[test]
fn a_shorter_shutter_smears_the_propeller_over_a_smaller_angle() {
    let asset = UsdAircraftAsset::p38g().unwrap();
    let art = AirframeArticulation {
        propeller_rpm: 1_500.0,
        propeller_right: sim_core::PropellerShaft {
            rpm: 1_500.0,
            ..sim_core::PropellerShaft::default()
        },
        ..AirframeArticulation::default()
    };
    let smear = |shutter_s: f64| {
        asset
            .evaluate_draws_exposed(&art, SlotCameraRole::External, shutter_s)
            .into_iter()
            .find(|d| d.is_prop_disc)
            .map(|d| d.shutter_blur_rad)
            .unwrap()
    };
    let (eye, camera) = (
        smear(UsdAircraftAsset::DISPLAY_SHUTTER_S),
        smear(1.0 / 500.0),
    );
    assert!((camera - 1_500.0 * std::f32::consts::TAU / 60.0 / 500.0).abs() < 1e-3);
    assert!(eye > 5.0 * camera, "eye {eye} rad, camera {camera} rad");
    let default = asset.evaluate_draws(&art, SlotCameraRole::External);
    let disc = default.iter().find(|d| d.is_prop_disc).unwrap();
    assert_eq!(disc.shutter_blur_rad, eye);
}

#[test]
fn an_override_layer_deactivates_parts_and_rebinds_materials() {
    let layer = r#"#usda 1.0
(
    defaultPrim = "C172"
    subLayers = [@c172.usdc@]
)
over "C172"
{
    over "Airframe"
    {
        over "BeaconLight" (active = false) {}
        over "HighWingTrim"
        {
            rel material:binding = </C172/Materials/MatWhitePaint>
        }
    }
}
"#;
    let base = UsdAircraftAsset::embedded_c172().unwrap();
    let tail = UsdAircraftAsset::from_usd_layers(
        "tail.usda",
        [
            ("c172.usdc", EMBEDDED_C172_USDC.into()),
            ("tail.usda", layer.as_bytes().to_vec().into()),
        ],
    )
    .unwrap();
    assert_eq!(tail.parts().len() + 1, base.parts().len());
    let wing = |a: &UsdAircraftAsset| {
        a.parts()
            .iter()
            .find(|p| p.prim_path.ends_with("/HighWingTrim"))
            .map(|p| p.material)
    };
    assert_ne!(wing(&tail), wing(&base));
    assert_eq!(wing(&tail).map(|m| m.metallic), Some(0.08));
}

/// Lowest rest-pose point of the part at `suffix`, and its rest origin, in metres.
fn rest_extremes(asset: &UsdAircraftAsset, suffix: &str) -> (f64, DVec3) {
    let part = asset.parts().iter().find(|p| p.prim_path.ends_with(suffix));
    let part = part.unwrap_or_else(|| panic!("no part {suffix}"));
    let lowest = part
        .vertices
        .iter()
        .map(|v| {
            part.rest_to_aircraft
                .transform_point3(glam::Vec3::from(v.position).as_dvec3())
                .z
        })
        .fold(f64::INFINITY, f64::min);
    (lowest, part.rest_to_aircraft.transform_point3(DVec3::ZERO))
}

#[test]
fn n642sp_flies_bare_gear_on_the_poh_wheel_base_without_moving_the_contact_plane() {
    let base = UsdAircraftAsset::embedded_c172().unwrap();
    let tail = UsdAircraftAsset::embedded_n642sp().unwrap();
    assert!(
        tail.parts()
            .iter()
            .all(|p| !p.prim_path.contains("Fairing"))
    );
    assert!(
        base.parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/WheelFairingLeft"))
    );
    for wheel in ["/NoseWheel/Tire", "/WheelLeft/Tire", "/WheelRight/Tire"] {
        let (generic, _) = rest_extremes(&base, wheel);
        let (n642sp, _) = rest_extremes(&tail, wheel);
        assert!(
            (generic - n642sp).abs() < 2e-3,
            "{wheel}: {generic} vs {n642sp}"
        );
    }
    let (_, nose) = rest_extremes(&tail, "/NoseWheel/Tire");
    let (main_bottom, main) = rest_extremes(&tail, "/WheelLeft/Tire");
    let wheel_base_in = (nose.y - main.y) / 0.0254;
    assert!(
        (wheel_base_in - 65.0).abs() < 1.0,
        "wheel base {wheel_base_in} in"
    );
    // 6.00-6 main tyre: 17.5 in outside diameter.
    let main_radius_in = (main.z - main_bottom) / 0.0254;
    assert!(
        (main_radius_in - 8.75).abs() < 0.2,
        "main tyre radius {main_radius_in} in"
    );
    assert!(
        tail.parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/WheelLeft/BrakeDisc"))
    );
}

#[test]
fn n642sp_carries_its_antennas_lights_and_flight_control_details() {
    let tail = UsdAircraftAsset::embedded_n642sp().unwrap();
    let role_of = |suffix: &str| {
        tail.parts()
            .iter()
            .find(|p| p.prim_path.ends_with(suffix))
            .map(|p| p.role)
    };
    for antenna in [
        "/Com1Whip",
        "/Com2Whip",
        "/GpsAntenna",
        "/VorElementLeft",
        "/VorElementRight",
        "/EltAntennaBlade",
        "/TransponderAntenna",
        "/AdfAntenna",
        "/MarkerBeaconAntenna",
        "/AdsbAntenna",
        "/PitotTube",
        "/StallWarningInlet",
    ] {
        assert_eq!(role_of(antenna), Some(PartRole::CabinExterior), "{antenna}");
    }
    assert_eq!(role_of("/RoofAntennaLeft"), None);
    assert_eq!(role_of("/LandingLight"), Some(PartRole::LandingLight));
    assert_eq!(role_of("/StrobeLeft"), Some(PartRole::BeaconLight));
    assert_eq!(role_of("/Rudder/TailLight"), Some(PartRole::Rudder));
    assert_eq!(role_of("/Elevator/TrimTab"), Some(PartRole::Elevator));
    assert_eq!(
        role_of("/AileronLeft/StaticWickOuter"),
        Some(PartRole::AileronLeft)
    );
    let red = tail
        .parts()
        .iter()
        .find(|p| p.prim_path.ends_with("/NavLightLeft"))
        .map(|p| p.material.emissive_rgb);
    assert_eq!(red, Some([1.0, 0.06, 0.04]));
}

#[test]
fn n642sp_wears_its_swoosh_and_12_inch_registration_on_both_sides() {
    let tail = UsdAircraftAsset::embedded_n642sp().unwrap();
    let part = |suffix: &str| {
        tail.parts()
            .iter()
            .find(|p| p.prim_path.ends_with(suffix))
            .unwrap_or_else(|| panic!("no part {suffix}"))
    };
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
        // 12-inch marks, measured on the projected glyphs.
        assert!(((hi - lo) - 0.3048).abs() < 0.01, "{suffix}: {}", hi - lo);
        assert_eq!(reg.material, part("/LiveryRegistrationPort").material);
    }
    for stripe in ["/LiverySwooshRedPort", "/LiverySwooshNavyStarboard"] {
        assert!(part(stripe).indices.len() >= 3 * 50, "{stripe}");
    }
    let white = part("/CabinWhite").material;
    assert_eq!(part("/CabinStripe").material, white);
    assert_eq!(part("/NoseCowlingStripe").material, white);
}
