//! A drawn airframe's wheels stand where its flight model's gear touches the ground.
//!
//! Each model is placed at a fixed structural datum
//! ([`sim_maplibre::OwnshipVisualKind::structural_origin_in`]). With the aircraft parked
//! and settled, each gear leg's wheel bottom as drawn (the model's rest wheel raised by the
//! leg's compression, as the renderer articulates it), in the flight model's structural
//! frame, must meet that leg's BOGEY contact point raised by the same compression: where
//! the ground is under the leg.

use glam::{DVec3, Vec3};
use sim_core::ControlInputs;
use sim_geo::Geodetic;
use sim_maplibre::{PartRole, UsdAircraftAsset};

use crate::vehicles::{FreeFlightPlant, FreeFlightVehicle};
use crate::world_surface::WorldSurfaceHandle;

const METRES_PER_INCH: f64 = 0.0254;
/// How far a wheel bottom may sit from where the flight model puts the ground.
const TOLERANCE_M: f64 = 0.01;
const LEGS: [(&str, PartRole); 3] = [
    ("NOSE", PartRole::NoseWheel),
    ("LEFT_MAIN", PartRole::WheelLeft),
    ("RIGHT_MAIN", PartRole::WheelRight),
];

/// The BOGEY contact `name`'s structural location in an FDM file, inches.
fn bogey_in(fdm_xml: &str, name: &str) -> DVec3 {
    let start = fdm_xml
        .find(&format!("type=\"BOGEY\" name=\"{name}\""))
        .unwrap_or_else(|| panic!("no BOGEY {name}"));
    let contact = &fdm_xml[start..];
    let coord = |axis: &str| -> f64 {
        let open = format!("<{axis}>");
        let at = contact.find(&open).unwrap() + open.len();
        let end = at + contact[at..].find('<').unwrap();
        contact[at..end].trim().parse().unwrap()
    };
    DVec3::new(coord("x"), coord("y"), coord("z"))
}

/// The bottom of `role`'s wheel: the middle of its parts' footprint at their lowest
/// point, in model metres (`x` right, `y` forward, `z` up).
fn wheel_bottom(asset: &UsdAircraftAsset, role: PartRole) -> Vec3 {
    let (low, high) = asset
        .parts()
        .iter()
        .filter(|part| part.role == role)
        .flat_map(|part| {
            let rest = part.rest_to_aircraft.as_mat4();
            part.vertices
                .iter()
                .map(move |v| rest.transform_point3(Vec3::from_array(v.position)))
        })
        .fold(
            (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
            |(low, high), p| (low.min(p), high.max(p)),
        );
    assert!(low.is_finite(), "no {role:?} parts");
    Vec3::new((low.x + high.x) * 0.5, (low.y + high.y) * 0.5, low.z)
}

/// Per leg, the wheel bottom's offset from where the flight model puts the ground under
/// that leg, in metres along the structural axes (aft, right, up).
fn wheel_offsets(
    vehicle: FreeFlightVehicle,
    fdm_xml: &str,
    asset: &UsdAircraftAsset,
) -> (Vec<(&'static str, DVec3)>, [f64; 3]) {
    let visual = crate::map_rig::aircraft::ownship_visual(None, Some(vehicle));
    let origin = DVec3::from_array(visual.structural_origin_in().unwrap());
    let parked = Geodetic::from_degrees(40.5244, -74.5984, 30.0);
    let mut plant = FreeFlightPlant::new_on_ground(vehicle, parked, 0.0, true).unwrap();
    let world = WorldSurfaceHandle::new();
    for _ in 0..((4.0 / plant.period().as_seconds()) as usize) {
        plant.step(ControlInputs::default(), &world).unwrap();
    }
    let compression_m = plant.articulation(0.0).gear_compression_m;
    let offsets = LEGS
        .iter()
        .zip(compression_m)
        .map(|(&(name, role), compression)| {
            let ground_in = bogey_in(fdm_xml, name) + DVec3::Z * compression / METRES_PER_INCH;
            let v = wheel_bottom(asset, role).as_dvec3() / METRES_PER_INCH;
            let wheel_in =
                origin + DVec3::new(-v.y, v.x, v.z) + DVec3::Z * compression / METRES_PER_INCH;
            (name, (wheel_in - ground_in) * METRES_PER_INCH)
        })
        .collect();
    (offsets, compression_m)
}

fn assert_wheels_on_the_ground(
    label: &str,
    (offsets, compression_m): (&[(&str, DVec3)], [f64; 3]),
) {
    let report: Vec<String> = offsets
        .iter()
        .map(|(leg, d)| format!("{leg}: aft {:+.3} right {:+.3} up {:+.3} m", d.x, d.y, d.z))
        .chain(std::iter::once(format!(
            "(static compression {compression_m:?} m)"
        )))
        .collect();
    assert!(
        offsets.iter().all(|(_, d)| d.length() < TOLERANCE_M),
        "{label} wheels off the flight model's gear:\n{}",
        report.join("\n")
    );
}

/// Red until the C172 model's gear matches the flight model's: drawn from its structural
/// datum its nose wheel sits within 1.1 cm of `NOSE`, but its mains stand 14 cm aft of
/// `LEFT_MAIN`/`RIGHT_MAIN` (a 70.3 in wheelbase against 64.4 in), 1.4 cm outboard, and
/// all three wheels 1.0 to 1.5 cm above the ground.
#[test]
#[ignore = "red: C172 model mains 14 cm aft of the flight model's (wheelbase 70.3 vs 64.4 in)"]
fn c172_wheels_meet_the_fdm_gear_red_until_the_model_wheelbase_matches() {
    let asset = UsdAircraftAsset::embedded_c172().unwrap();
    let fdm = include_str!("../../../../../vehicles/c172/fdm/c172_class_reference.xml");
    let offsets = wheel_offsets(FreeFlightVehicle::JsbsimC172, fdm, &asset);
    assert_wheels_on_the_ground("C172", (&offsets.0, offsets.1));
}

/// Red until the P-38G's model gear and flight model gear agree: drawn from its structural
/// datum its wheels reach 22 cm below the flight model's contact points at static
/// compression, its nose wheel stands 35 cm and its mains 27.5 cm aft of them, and its
/// mains sit under the booms (93 in) while the flight model's stand at 98 in.
#[test]
#[ignore = "red: P-38G model wheels 22 cm below and 28-35 cm aft of the flight model's gear"]
fn p38g_wheels_meet_the_fdm_gear_red_until_the_gear_geometry_agrees() {
    let asset = UsdAircraftAsset::p38g().unwrap();
    let fdm = include_str!("../../../../../vehicles/p38g/fdm/p38g.xml");
    let offsets = wheel_offsets(FreeFlightVehicle::JsbsimP38g, fdm, &asset);
    assert_wheels_on_the_ground("P-38G", (&offsets.0, offsets.1));
}
