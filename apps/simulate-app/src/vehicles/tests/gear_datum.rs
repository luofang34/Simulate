//! A drawn airframe's wheels stand where its flight model's gear touches the ground.
//!
//! Each model is placed at a fixed structural datum
//! ([`crate::vehicles::ModelOrigins`]). With the aircraft parked
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
    let origin = crate::vehicles::ModelOrigins::default()
        .origin_in(visual)
        .unwrap();
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

/// Red while the drawn N642SP's gear, fitted with its propeller disc on the sourced
/// landmarks, stands 1.2 in aft of its POH stations (the disc 2.1 in forward): its nose
/// wheel 2.9 cm aft of the flight model's `NOSE` and its mains 4.5 cm aft of
/// `LEFT_MAIN`/`RIGHT_MAIN` (the flight model's mains at FS 57.6 against the POH's 58.2),
/// 0.8 cm outboard, and the wheels 1.0 to 1.5 cm above the ground.
#[test]
#[ignore = "red: C172 fitted gear 1.2 in aft of its POH stations (mesh propeller 2.1 in forward)"]
fn c172_wheels_meet_the_fdm_gear_red_while_the_fit_leaves_the_gear_1_in_aft() {
    let asset = UsdAircraftAsset::embedded_n642sp().unwrap();
    let fdm = include_str!("../../../../../vehicles/c172s/fdm/c172s.xml");
    let offsets = wheel_offsets(FreeFlightVehicle::JsbsimC172, fdm, &asset);
    assert_wheels_on_the_ground("C172", (&offsets.0, offsets.1));
}

/// The P-38G's wheels, placed by its AN 01-75F-2 landmarks, stand on its flight model's
/// gear: both take the drawing's axles and static tyre deflection.
#[test]
fn p38g_wheels_meet_the_fdm_gear() {
    let asset = UsdAircraftAsset::p38g().unwrap();
    let fdm = include_str!("../../../../../vehicles/p38g/fdm/p38g.xml");
    let offsets = wheel_offsets(FreeFlightVehicle::JsbsimP38g, fdm, &asset);
    assert_wheels_on_the_ground("P-38G", (&offsets.0, offsets.1));
}

/// The THRUSTER `n`'s structural location in an FDM file, inches.
fn thruster_in(fdm_xml: &str, n: usize) -> DVec3 {
    let start = fdm_xml.match_indices("<thruster>").nth(n).unwrap().0;
    let block = &fdm_xml[start..];
    let coord = |axis: &str| -> f64 {
        let open = format!("<{axis}>");
        let at = block.find(&open).unwrap() + open.len();
        let end = at + block[at..].find('<').unwrap();
        block[at..end].trim().parse().unwrap()
    };
    DVec3::new(coord("x"), coord("y"), coord("z"))
}

/// The P-38G's propeller discs, placed by its landmarks, sit on its flight model's
/// thrusters: both take the drawing's propeller plane, boom centreline and thrust line.
#[test]
fn p38g_propeller_discs_sit_on_the_fdm_thrusters() {
    let fdm = include_str!("../../../../../vehicles/p38g/fdm/p38g.xml");
    let visual =
        crate::map_rig::aircraft::ownship_visual(None, Some(FreeFlightVehicle::JsbsimP38g));
    let origin = crate::vehicles::ModelOrigins::default()
        .origin_in(visual)
        .unwrap();
    let forward_m = f64::from(sim_p38g::PROP_HUB_Y + sim_p38g::PROP_BLADE_OFFSET_M);
    for (n, side) in [(0, -1.0), (1, 1.0)] {
        let disc = DVec3::new(
            -forward_m,
            side * f64::from(sim_p38g::BOOM_X),
            f64::from(sim_p38g::PROP_HUB_Z),
        ) / METRES_PER_INCH
            + origin;
        let thruster = thruster_in(fdm, n);
        assert!(
            (disc - thruster).length() < 0.1,
            "thruster {n} at {thruster} in, drawn disc at {disc} in"
        );
    }
}

/// The parked height the P-38G spawns at is where its gear holds the loaded CG: the
/// flight model's CG sits [`crate::vehicles::P38G_LOADED_CG_ABOVE_FRL_IN`] above the
/// fuselage reference line.
#[test]
fn p38g_parked_height_matches_the_flight_models_loaded_cg() {
    let parked = Geodetic::from_degrees(40.5244, -74.5984, 30.0);
    let plant =
        FreeFlightPlant::new_on_ground(FreeFlightVehicle::JsbsimP38g, parked, 0.0, true).unwrap();
    let cg = plant.cg_structural_in().unwrap();
    assert!(
        (cg.z - crate::vehicles::P38G_LOADED_CG_ABOVE_FRL_IN).abs() < 0.05
            && (cg.x - crate::vehicles::P38G_LOADED_CG_FS_IN).abs() < 0.3,
        "flight model CG {cg} in"
    );
}

/// The parked P-38G stands on AN 01-75F-2's static ground line: nose-up by `5°33'46"`,
/// from the first step (it spawns at that attitude and height) to settled.
#[test]
fn p38g_stands_nose_up_on_the_drawn_static_ground_line() {
    let parked = Geodetic::from_degrees(40.5244, -74.5984, 30.0);
    let mut plant =
        FreeFlightPlant::new_on_ground(FreeFlightVehicle::JsbsimP38g, parked, 0.0, true).unwrap();
    let world = WorldSurfaceHandle::new();
    let pitch_deg = |plant: &FreeFlightPlant| {
        let state = plant.state();
        let up = sim_geo::LocalFrame::at(&sim_geo::ecef_to_geodetic(state.position_ecef)).up();
        (state.attitude * DVec3::X).dot(up).asin().to_degrees()
    };
    let drawn = f64::from(sim_p38g::drawing::STATIC_GROUND_ANGLE_DEG);
    let mut worst = 0.0_f64;
    for _ in 0..((6.0 / plant.period().as_seconds()) as usize) {
        plant.step(ControlInputs::default(), &world).unwrap();
        worst = worst.max((pitch_deg(&plant) - drawn).abs());
    }
    let settled = pitch_deg(&plant);
    assert!(
        (settled - drawn).abs() < 0.2,
        "settled at {settled:.2} deg, drawn {drawn:.2}"
    );
    assert!(
        worst < 0.6,
        "pitch swung {worst:.2} deg off the static line"
    );
}
