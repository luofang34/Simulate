use glam::{DVec3, Vec3};
use sim_core::AirframeArticulation;

use crate::{PartRole, SlotCameraRole, UsdAircraftAsset, UsdMeshPart};

fn builtins() -> [(&'static str, UsdAircraftAsset); 3] {
    [
        ("x500", UsdAircraftAsset::x500_quadrotor().unwrap()),
        ("fixed_wing", UsdAircraftAsset::fixed_wing_uav().unwrap()),
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
    let [(_, x500), (_, uav), (_, lander)] = builtins();
    for role in [
        PartRole::PropellerSpinner,
        PartRole::PropellerBlades,
        PartRole::PropellerDisc,
    ] {
        assert_eq!(count(&x500, role), 4, "{role:?}");
        assert_eq!(count(&uav, role), 1, "{role:?}");
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
    assert!(count(&lander, PartRole::GearStruts) >= 1);
    assert_eq!(count(&lander, PartRole::CabinGlass), 1);
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
