use glam::{DVec3, Vec3};
use sim_core::AirframeArticulation;

use crate::{PartRole, SlotCameraRole, UsdAircraftAsset, UsdMeshPart};

fn builtins() -> [(&'static str, UsdAircraftAsset); 4] {
    [
        ("x500", UsdAircraftAsset::x500_quadrotor().unwrap()),
        ("fixed_wing", UsdAircraftAsset::fixed_wing_uav().unwrap()),
        ("p38g", UsdAircraftAsset::p38g().unwrap()),
        ("lander", UsdAircraftAsset::lunar_lander().unwrap()),
    ]
}

fn count(asset: &UsdAircraftAsset, role: PartRole) -> usize {
    asset.parts().iter().filter(|p| p.role == role).count()
}

fn rest_bounds(asset: &UsdAircraftAsset) -> (DVec3, DVec3) {
    let mut min = DVec3::splat(f64::INFINITY);
    let mut max = DVec3::splat(f64::NEG_INFINITY);
    for part in asset.parts() {
        for v in &part.vertices {
            let p = part
                .rest_to_aircraft
                .transform_point3(Vec3::from_array(v.position).as_dvec3());
            min = min.min(p);
            max = max.max(p);
        }
    }
    (min, max)
}

#[test]
fn every_builtin_has_finite_non_empty_geometry_and_plausible_size() {
    for (name, asset) in builtins() {
        assert!(asset.parts().len() >= 8, "{name}: {}", asset.parts().len());
        assert!(
            asset.triangle_count() > 1_000,
            "{name}: {}",
            asset.triangle_count()
        );
        for part in asset.parts() {
            assert!(
                !part.indices.is_empty() && part.indices.len() % 3 == 0,
                "{}",
                part.prim_path
            );
            let n = part.vertices.len() as u32;
            assert!(part.indices.iter().all(|i| *i < n), "{}", part.prim_path);
        }
        let (min, max) = rest_bounds(&asset);
        assert!(min.is_finite() && max.is_finite(), "{name}");
        let span = (max - min).max_element();
        let expected = match name {
            "x500" => 0.5..1.0,
            "fixed_wing" => 3.0..3.6,
            "p38g" => 15.5..16.5,
            _ => 8.0..10.0,
        };
        assert!(expected.contains(&span), "{name}: span {span}");
    }
}

#[test]
fn prim_paths_are_unique_and_rooted_per_vehicle() {
    for (name, asset) in builtins() {
        let root = asset.parts()[0]
            .prim_path
            .split('/')
            .nth(1)
            .unwrap()
            .to_owned();
        let mut paths: Vec<&str> = asset.parts().iter().map(|p| p.prim_path.as_str()).collect();
        assert!(
            paths.iter().all(|p| p.starts_with(&format!("/{root}/"))),
            "{name}"
        );
        paths.sort_unstable();
        let before = paths.len();
        paths.dedup();
        assert_eq!(before, paths.len(), "{name}");
    }
}

#[test]
fn articulation_roles_are_tagged_on_each_builtin() {
    let [(_, x500), (_, uav), (_, p38), (_, lander)] = builtins();
    for role in [
        PartRole::PropellerSpinner,
        PartRole::PropellerBlades,
        PartRole::PropellerDisc,
    ] {
        assert_eq!(count(&x500, role), 4, "{role:?}");
        assert_eq!(count(&uav, role), 1, "{role:?}");
        assert!(count(&p38, role) >= 2, "{role:?}");
        assert_eq!(count(&lander, role), 0, "{role:?}");
    }
    assert_eq!(count(&x500, PartRole::CabinGlass), 1);
    assert_eq!(count(&x500, PartRole::BeaconLight), 1);
    for role in [
        PartRole::AileronLeft,
        PartRole::AileronRight,
        PartRole::FlapLeft,
        PartRole::FlapRight,
        PartRole::RuddervatorLeft,
        PartRole::RuddervatorRight,
        PartRole::CabinGlass,
    ] {
        assert_eq!(count(&uav, role), 1, "{role:?}");
    }
    assert_eq!(count(&p38, PartRole::LandingLight), 1);
    // Windscreen, hatch, a window either side of it, the rear glazing, and the
    // instruments' cover glass.
    assert_eq!(count(&p38, PartRole::CabinGlass), 6);
    // The P-38's ailerons, elevator and flaps are olive drab over a neutral-gray
    // underside part that deflects with them.
    for role in [
        PartRole::AileronLeft,
        PartRole::AileronRight,
        PartRole::Elevator,
        PartRole::Rudder,
        PartRole::NoseWheel,
        PartRole::WheelLeft,
        PartRole::WheelRight,
    ] {
        assert_eq!(count(&p38, role), 2, "{role:?}");
    }
    for role in [PartRole::FlapLeft, PartRole::FlapRight] {
        assert_eq!(count(&p38, role), 4, "{role:?}");
    }
    // Each of the three legs is a painted strut and a chromed piston; each of the three
    // bays has two doors.
    assert_eq!(count(&p38, PartRole::GearStruts), 6);
    let doors = p38
        .parts()
        .iter()
        .filter(|p| p.prim_path.contains("/GearDoor") && p.custom_hinge_axis.is_some())
        .count();
    assert_eq!(doors, 6);
    assert!(count(&lander, PartRole::GearStruts) >= 1);
    assert_eq!(count(&lander, PartRole::CabinGlass), 1);
}

#[test]
fn p38g_retracted_gear_stows_struts_wheels_and_landing_lights() {
    let p38 = UsdAircraftAsset::p38g().unwrap();
    let down = p38.evaluate_draws(&AirframeArticulation::default(), SlotCameraRole::External);
    let up = p38.evaluate_draws(
        &AirframeArticulation {
            gear_pos_norm: 0.0,
            ..AirframeArticulation::default()
        },
        SlotCameraRole::External,
    );
    assert!(
        down.len() > up.len(),
        "retracting gear must stow gear struts, wheels, and retractable landing lights: down={}, up={}",
        down.len(),
        up.len()
    );
    for draw in &up {
        let role = p38.parts()[draw.part_index].role;
        assert!(
            !matches!(
                role,
                PartRole::GearStruts
                    | PartRole::NoseWheel
                    | PartRole::WheelLeft
                    | PartRole::WheelRight
                    | PartRole::LandingLight
            ),
            "unexpected gear/light role {role:?} visible when gear_pos_norm == 0"
        );
    }
}

#[test]
fn single_sided_triangles_face_along_their_vertex_normals() {
    for (name, asset) in builtins() {
        for part in asset.parts().iter().filter(|p| !p.double_sided) {
            let pos = |i: u32| Vec3::from_array(part.vertices[i as usize].position);
            let nrm = |i: u32| Vec3::from_array(part.vertices[i as usize].normal);
            // Twisted blades fold a few sliver triangles at their thin trailing edges, where
            // the averaged vertex normal is no longer a reliable outward reference.
            let (mut good, mut total) = (0, 0);
            for &[a, b, c] in part.indices.as_chunks::<3>().0 {
                let face = (pos(b) - pos(a)).cross(pos(c) - pos(a));
                if face.length_squared() < 1e-14 {
                    continue;
                }
                total += 1;
                if face.dot(nrm(a) + nrm(b) + nrm(c)) > 0.0 {
                    good += 1;
                }
            }
            assert!(
                good * 100 >= total * 96,
                "{name} {}: {good}/{total}",
                part.prim_path
            );
        }
    }
}

/// Azimuth of the outermost blade vertex about aircraft `+Z` through the rotor hub.
fn blade_tip_azimuth(asset: &UsdAircraftAsset, path: &str, phase: f64) -> f64 {
    let art = AirframeArticulation {
        propeller_phase_rad: phase,
        ..AirframeArticulation::default()
    };
    let draws = asset.evaluate_draws(&art, SlotCameraRole::External);
    let (index, part) = asset
        .parts()
        .iter()
        .enumerate()
        .find(|(_, p)| p.prim_path == path)
        .unwrap();
    let draw = draws.iter().find(|d| d.part_index == index).unwrap();
    let tip = tip_vertex(part);
    let hub = draw.part_to_aircraft.transform_point3(DVec3::ZERO);
    let p = draw.part_to_aircraft.transform_point3(tip) - hub;
    p.y.atan2(p.x)
}

fn tip_vertex(part: &UsdMeshPart) -> DVec3 {
    part.vertices
        .iter()
        .map(|v| Vec3::from_array(v.position).as_dvec3())
        .max_by(|a, b| a.length().total_cmp(&b.length()))
        .unwrap()
}

#[test]
fn adjacent_quadrotor_rotors_counter_rotate() {
    let x500 = UsdAircraftAsset::x500_quadrotor().unwrap();
    let turn = |name: &str| {
        let path = format!("/X500/Rotors/{name}/Blades");
        let delta = blade_tip_azimuth(&x500, &path, 0.5) - blade_tip_azimuth(&x500, &path, 0.0);
        (delta + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
    };
    let front_right = turn("FrontRight");
    let front_left = turn("FrontLeft");
    assert!((front_right.abs() - 0.5).abs() < 1e-6, "{front_right}");
    assert!((front_left.abs() - 0.5).abs() < 1e-6, "{front_left}");
    assert!(front_right * front_left < 0.0, "{front_right} {front_left}");
    assert!(turn("RearLeft") * front_right > 0.0);
    assert!(turn("RearRight") * front_left > 0.0);
}

/// Trailing-edge displacement of `path` under `art` relative to the rest pose.
fn trailing_edge_shift(asset: &UsdAircraftAsset, path: &str, art: &AirframeArticulation) -> DVec3 {
    let index = asset
        .parts()
        .iter()
        .position(|p| p.prim_path == path)
        .unwrap();
    let part = &asset.parts()[index];
    let te = part
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position).as_dvec3())
        .min_by(|a, b| a.y.total_cmp(&b.y))
        .unwrap();
    let at = |a: &AirframeArticulation| {
        let draws = asset.evaluate_draws(a, SlotCameraRole::External);
        let draw = draws.iter().find(|d| d.part_index == index).unwrap();
        draw.part_to_aircraft.transform_point3(te)
    };
    at(art) - at(&AirframeArticulation::default())
}

#[test]
fn ruddervators_mix_elevator_together_and_rudder_differentially() {
    let uav = UsdAircraftAsset::fixed_wing_uav().unwrap();
    let left = "/FixedWingUav/Tail/RuddervatorLeft";
    let right = "/FixedWingUav/Tail/RuddervatorRight";
    let elevator = AirframeArticulation {
        elevator_rad: 0.25,
        ..AirframeArticulation::default()
    };
    let (l, r) = (
        trailing_edge_shift(&uav, left, &elevator),
        trailing_edge_shift(&uav, right, &elevator),
    );
    assert!(l.z < -0.005 && r.z < -0.005, "{l:?} {r:?}");
    assert!(
        (l.x + r.x).abs() < 1e-6,
        "pitch must be symmetric: {l:?} {r:?}"
    );
    let rudder = AirframeArticulation {
        rudder_rad: 0.25,
        ..AirframeArticulation::default()
    };
    let (l, r) = (
        trailing_edge_shift(&uav, left, &rudder),
        trailing_edge_shift(&uav, right, &rudder),
    );
    assert!(l.z * r.z < 0.0, "{l:?} {r:?}");
    assert!(
        l.x < -0.005 && r.x < -0.005,
        "both trailing edges swing the same way: {l:?} {r:?}"
    );
}

#[test]
fn ailerons_deflect_their_trailing_edges() {
    let uav = UsdAircraftAsset::fixed_wing_uav().unwrap();
    let art = AirframeArticulation {
        left_aileron_rad: 0.3,
        right_aileron_rad: -0.3,
        ..AirframeArticulation::default()
    };
    let l = trailing_edge_shift(&uav, "/FixedWingUav/Wing/AileronLeft", &art);
    let r = trailing_edge_shift(&uav, "/FixedWingUav/Wing/AileronRight", &art);
    assert!(l.z < -0.01 && r.z > 0.01, "{l:?} {r:?}");
}

#[test]
fn p38g_curtiss_electric_blades_feather_and_fowler_and_coolant_flaps_articulate() {
    let p38 = UsdAircraftAsset::p38g().unwrap();
    let feathered = AirframeArticulation {
        propeller_rpm: 0.0,
        propeller_blade_angle_deg: 85.0,
        propeller_feathered: [true, false],
        flap_deg: 8.0,
        coolant_flap_norm: [0.85, 0.85],
        ..AirframeArticulation::default()
    };
    let blade_shift = trailing_edge_shift(&p38, "/P38g/PropLeft/Blade0", &feathered);
    assert!(
        blade_shift.length() > 0.05,
        "feathering port Curtiss Electric blade to 85 deg must rotate blade chord about its radial axis: {blade_shift:?}"
    );
    let fowler_shift = trailing_edge_shift(&p38, "/P38g/Wing/FlapInboardLeft", &feathered);
    assert!(
        fowler_shift.y < -0.02 && fowler_shift.z < -0.02,
        "8 deg Maneuver Fowler flap must both translate aft (-Y) and deflect down (-Z): {fowler_shift:?}"
    );
    let coolant_shift = trailing_edge_shift(&p38, "/P38g/Cooling/CoolantFlapLeftOut", &feathered);
    assert!(
        coolant_shift.x.abs() > 0.02,
        "opening hydraulic glycol radiator exit flaps must swing trailing edge outward: {coolant_shift:?}"
    );
}

#[test]
fn p38g_instrument_faces_are_display_surfaces_the_pilot_sees() {
    let p38 = UsdAircraftAsset::p38g().unwrap();
    let faces: Vec<usize> = p38
        .parts()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.role == PartRole::DisplaySurface)
        .map(|(i, _)| i)
        .collect();
    // Twenty-seven instrument faces and every lettered placard.
    assert_eq!(faces.len(), 27 + sim_p38g::p38g_placards().len());
    assert!(
        p38.parts()
            .iter()
            .any(|p| p.prim_path == "/P38g/CockpitInterior/Instruments/flight_airspeed/airspeed")
    );
    for camera in [SlotCameraRole::PilotEye, SlotCameraRole::External] {
        let draws = p38.evaluate_draws(&AirframeArticulation::default(), camera);
        for face in &faces {
            assert!(draws.iter().any(|d| d.part_index == *face), "{camera:?}");
        }
    }
}

#[test]
fn p38_stressed_skin_is_painted_by_its_bake_and_the_c172_has_none() {
    let asset = crate::UsdAircraftAsset::p38g().unwrap();
    let skin = asset.skin().expect("the P-38G bakes a skin");
    assert_eq!(skin.albedo.size(), (4096, 4096));
    assert_eq!(skin.orm.size(), (2048, 2048));
    // The stated budget: albedo, normal and pack with mips under 160 MiB.
    assert!(skin.byte_len() < 160 << 20, "{} bytes", skin.byte_len());
    let part = |path: &str| asset.parts().iter().find(|p| p.prim_path == path).unwrap();
    for path in [
        "/P38g/Booms/UpperOlive",
        "/P38g/Wing/LowerGray",
        "/P38g/Gondola/UpperOlive",
        "/P38g/Empennage/VerticalFins",
    ] {
        let skinned = part(path);
        assert_eq!(skinned.surface, crate::PartSurface::Skin, "{path}");
        assert_eq!(skinned.detail, crate::SurfaceDetail::default(), "{path}");
        for v in &skinned.vertices {
            assert!(
                v.uv.iter().all(|c| (0.0..=1.0).contains(c)),
                "{path} {:?}",
                v.uv
            );
            let t = glam::Vec3::from_array([v.tangent[0], v.tangent[1], v.tangent[2]]);
            assert!((t.length() - 1.0).abs() < 1e-3, "{path} tangent {t}");
            assert!(t.dot(glam::Vec3::from(v.normal)).abs() < 1e-3, "{path}");
        }
    }
    assert_eq!(
        part("/P38g/Canopy/Glazing/Hatch").surface,
        crate::PartSurface::Material
    );
    assert!(
        crate::UsdAircraftAsset::embedded_c172()
            .unwrap()
            .skin()
            .is_none()
    );
}

#[test]
#[ignore = "writes the P-38G skin maps as PNGs into $SIMULATE_VERIFY_OUT"]
fn write_p38_skin_maps() {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(std::path::PathBuf::from) else {
        return;
    };
    std::fs::create_dir_all(&dir).unwrap();
    let asset = crate::UsdAircraftAsset::p38g().unwrap();
    let skin = asset.skin().unwrap();
    for (name, map) in [
        ("albedo", &skin.albedo),
        ("normal", &skin.normal),
        ("orm", &skin.orm),
    ] {
        let (w, h) = map.size();
        let bytes = map.levels().first().unwrap();
        let rgba: Vec<u8> = match map.format() {
            crate::usd_aircraft::SkinFormat::Normal8 => bytes
                .as_chunks::<2>()
                .0
                .iter()
                .flat_map(|c| [c[0], c[1], 255, 255])
                .collect(),
            _ => bytes.clone(),
        };
        image::RgbaImage::from_raw(w, h, rgba)
            .unwrap()
            .save(dir.join(format!("p38_skin_{name}.png")))
            .unwrap();
    }
}

/// The canopy opens as the P-38G's did: the top hatch swings up about its aft
/// frame, lifting its forward edge clear, and each side window winds down past
/// the sill, the other panes staying put.
#[test]
fn p38g_canopy_hatch_swings_up_and_side_windows_wind_down() {
    let p38 = UsdAircraftAsset::p38g().unwrap();
    let mut open = AirframeArticulation::default();
    open.cockpit.canopy = sim_core::CanopyOpening {
        hatch: 1.0,
        side_windows: [1.0, 0.0],
    };
    // Where the vertex of `path` furthest along `toward` goes as the canopy opens.
    let moved = |path: &str, toward: DVec3| {
        let index = p38
            .parts()
            .iter()
            .position(|p| p.prim_path == path)
            .unwrap();
        let vertex = p38.parts()[index]
            .vertices
            .iter()
            .map(|v| Vec3::from_array(v.position).as_dvec3())
            .max_by(|a, b| a.dot(toward).total_cmp(&b.dot(toward)))
            .unwrap();
        let at = |a: &AirframeArticulation| {
            let draws = p38.evaluate_draws(a, SlotCameraRole::External);
            let draw = draws.iter().find(|d| d.part_index == index).unwrap();
            draw.part_to_aircraft.transform_point3(vertex)
        };
        (at(&AirframeArticulation::default()), at(&open))
    };
    let (shut, raised) = moved("/P38g/Canopy/Glazing/Hatch", DVec3::Y);
    assert!(
        raised.z > shut.z + 0.4 && raised.y < shut.y,
        "{shut:?} -> {raised:?}"
    );
    let sill = f64::from(sim_p38g::canopy_sill_z(shut.y as f32));
    let (top, wound) = moved("/P38g/Canopy/Glazing/SideWindowLeft", DVec3::Z);
    assert!(
        top.z > sill + 0.1 && wound.z < sill + 0.01,
        "{top:?} -> {wound:?}"
    );
    // Its top edge ends out on the sill, not standing inside the cabin.
    assert!(wound.x < top.x - 0.05 && (wound.y - top.y).abs() < 1e-9);
    for fixed in ["SideWindowRight", "Windscreen", "Rear"] {
        let (a, b) = moved(&format!("/P38g/Canopy/Glazing/{fixed}"), DVec3::Z);
        assert!(a.abs_diff_eq(b, 1e-9), "{fixed}");
    }
}
