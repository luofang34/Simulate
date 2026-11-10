use glam::{DVec3, Vec2, Vec3};
use sim_core::{AirframeArticulation, InstrumentNeedles};

use super::atlas::AtlasBuilder;
use super::needle::ear_clip;
use super::{FACE_PX, face_geometry};
use crate::{PartRole, PartSurface, SlotCameraRole, UsdAircraftAsset, UsdMeshPart};

fn area([a, b, c]: [Vec2; 3]) -> f32 {
    0.5 * (b - a).perp_dot(c - a)
}

#[test]
fn ear_clipping_covers_a_concave_pointer_exactly() {
    // The altimeter's ten-thousands pointer: a triangle on a thin shaft.
    let outline = [
        Vec2::new(0.0, 10.0),
        Vec2::new(2.0, 8.0),
        Vec2::new(0.3, 8.0),
        Vec2::new(0.3, 0.0),
        Vec2::new(-0.3, 0.0),
        Vec2::new(-0.3, 8.0),
        Vec2::new(-2.0, 8.0),
    ];
    let triangles = ear_clip(&outline);
    assert_eq!(triangles.len(), outline.len() - 2);
    let total: f32 = triangles.iter().map(|&t| area(t)).sum();
    // Shaft 0.6 x 8 plus a triangle 4 wide and 2 tall.
    assert!((total - (0.6 * 8.0 + 4.0)).abs() < 1e-4, "{total}");
    assert!(triangles.iter().all(|&t| area(t) > 0.0));
}

#[test]
fn the_atlas_carries_a_full_mip_chain() {
    let mut builder = AtlasBuilder::new(5, 8);
    let rect = builder.place(4, [8, 8], &[255; 8 * 8 * 4]);
    assert_eq!(rect, [0.0, 0.5, 0.25, 0.5]);
    let atlas = builder.finish();
    assert_eq!(atlas.size(), (32, 16));
    let levels = atlas.levels();
    assert_eq!(levels.len(), 6);
    assert_eq!(levels.last().unwrap().len(), 4);
}

fn p38() -> UsdAircraftAsset {
    UsdAircraftAsset::p38g().unwrap()
}

fn needles(asset: &UsdAircraftAsset) -> Vec<(usize, &UsdMeshPart)> {
    asset
        .parts()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.role == PartRole::InstrumentNeedle)
        .collect()
}

#[test]
fn every_p38_face_prints_its_dial_and_carries_its_needles() {
    let asset = p38();
    assert!(asset.display_atlas().is_some());
    let faces: Vec<&UsdMeshPart> = asset
        .parts()
        .iter()
        .filter(|p| p.role == PartRole::DisplaySurface && p.prim_path.contains("/Instruments/"))
        .collect();
    // Sixteen pack dials, six of Indicate's own and five empty cases.
    assert_eq!(faces.len(), 27);
    let printed: Vec<_> = faces
        .iter()
        .filter(|f| !f.prim_path.ends_with("/empty"))
        .collect();
    assert_eq!(printed.len(), 22);
    let (atlas_px, _) = asset.display_atlas().unwrap().size();
    for face in printed {
        let PartSurface::Display { uv_rect, glow_rgb } = face.surface else {
            panic!("{} is not printed", face.prim_path);
        };
        assert!(uv_rect[2] > 0.0 && uv_rect[3] > 0.0);
        assert!(glow_rgb.iter().any(|c| *c > 0.1), "radium paint glows");
        // The quad spans the printed face: its share of the bake's cell is
        // the face's share of the case.
        let (_, radius_m) = face_geometry(face).unwrap();
        let cell = FACE_PX as f32 / atlas_px as f32;
        let share = uv_rect[2] / cell;
        let case_m = radius_m / share * 2.0;
        assert!(
            [0.079_375, 0.057_15, 0.076, 0.056, 0.095, 0.105, 0.085]
                .iter()
                .any(|case| (case_m - case).abs() < 2e-4),
            "{} case {case_m}",
            face.prim_path
        );
    }
    // One needle per single-pointer dial, three on the altimeter, each
    // engine gage unit and the clock, two on each fuel gauge and on the
    // turn and bank (its needle and ball), one bank index on the horizon.
    let needles = needles(&asset);
    assert_eq!(needles.len(), 11 + 3 + 2 * 3 + 2 * 2 + 3 + 2 + 1 + 1);
    let altimeter: Vec<_> = needles
        .iter()
        .filter(|(_, p)| p.prim_path.contains("/flight_altimeter/altimeter/"))
        .map(|(_, p)| p.needle_channel)
        .collect();
    let expected: Vec<_> = (0..3)
        .map(|hand| InstrumentNeedles::channel(1, hand))
        .collect();
    assert_eq!(altimeter, expected);
    assert_eq!(FACE_PX % 4, 0);
}

#[test]
fn p38_needles_stand_between_face_and_glass_and_turn_clockwise_from_the_seat() {
    let asset = p38();
    let (index, needle) = needles(&asset)
        .into_iter()
        .find(|(_, p)| p.prim_path.ends_with("/flight_airspeed/airspeed/Pointer0"))
        .unwrap();
    let face = asset
        .parts()
        .iter()
        .find(|p| p.prim_path == "/P38g/CockpitInterior/Instruments/flight_airspeed/airspeed")
        .unwrap();
    let (centre, _) = face_geometry(face).unwrap();
    let face_at = face.rest_to_aircraft.transform_point3(centre.as_dvec3());
    let pivot = needle.rest_to_aircraft.transform_point3(DVec3::ZERO);
    let out_of_face = face.rest_to_aircraft.transform_vector3(DVec3::NEG_Y);
    let standoff = (pivot - face_at).dot(out_of_face);
    assert!(
        standoff > 0.001 && standoff < 0.003,
        "{standoff} m off the face"
    );

    let tip_at = |angle: f64| {
        let mut art = AirframeArticulation::default();
        art.instrument_needles.set(0, 0, angle);
        let draws = asset.evaluate_draws(&art, SlotCameraRole::PilotEye);
        let draw = draws.iter().find(|d| d.part_index == index).unwrap();
        draw.part_to_aircraft.transform_vector3(DVec3::Z)
    };
    let (rest, quarter) = (tip_at(0.0), tip_at(std::f64::consts::FRAC_PI_2));
    assert!(rest.dot(quarter).abs() < 1e-6, "{rest} {quarter}");
    // Seen from the seat, looking into the panel, a positive angle turns
    // the needle clockwise.
    let into_panel = face.rest_to_aircraft.transform_vector3(DVec3::Y);
    assert!(
        rest.cross(quarter).dot(into_panel) > 0.9,
        "{rest} {quarter}"
    );

    let mut hidden = AirframeArticulation::default();
    hidden.instrument_needles.set_visible(0, 0, false);
    let draws = asset.evaluate_draws(&hidden, SlotCameraRole::PilotEye);
    assert!(draws.iter().all(|d| d.part_index != index));
}

#[test]
fn the_kollsman_drum_turns_its_next_digit_into_the_window() {
    let asset = p38();
    let face = asset
        .parts()
        .iter()
        .find(|p| p.prim_path == "/P38g/CockpitInterior/Instruments/flight_altimeter/altimeter")
        .unwrap();
    // The face is a square frame round the window: four quads.
    assert_eq!(face.indices.len(), 4 * 6);
    let toward_pilot = face.rest_to_aircraft.transform_vector3(DVec3::NEG_Y);
    let drums: Vec<(usize, &UsdMeshPart)> = asset
        .parts()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.role == PartRole::InstrumentDrum && p.prim_path.contains("/Drum"))
        .collect();
    assert_eq!(drums.len(), 4, "29.92 on four wheels");
    assert!(
        asset
            .parts()
            .iter()
            .any(|p| p.prim_path.ends_with("/DrumSeparator"))
    );
    let (index, wheel) = drums[0];
    // The rim's outward normal at digit 1's centre, v = 0.15.
    let digit_one = wheel
        .vertices
        .iter()
        .min_by(|a, b| (a.uv[1] - 0.15).abs().total_cmp(&(b.uv[1] - 0.15).abs()))
        .unwrap();
    let normal_at = |position: f64| {
        let mut art = AirframeArticulation::default();
        art.instrument_needles.set_wheel(1, 0, position);
        let draws = asset.evaluate_draws(&art, SlotCameraRole::PilotEye);
        let draw = draws.iter().find(|d| d.part_index == index).unwrap();
        draw.part_to_aircraft
            .transform_vector3(Vec3::from_array(digit_one.normal).as_dvec3())
            .normalize()
    };
    assert!(
        normal_at(0.0).dot(toward_pilot) < 0.85,
        "digit 1 is off centre at rest"
    );
    assert!(
        normal_at(1.0).dot(toward_pilot) > 0.99,
        "digit 1 faces the pilot at 1"
    );
    // A drum with no value is not drawn.
    let mut blank = AirframeArticulation::default();
    blank.instrument_needles.set_slot_state(1, false, false);
    let draws = asset.evaluate_draws(&blank, SlotCameraRole::PilotEye);
    assert!(draws.iter().all(|d| d.part_index != index));
}

#[test]
fn every_dial_has_a_failure_flag_that_shows_only_when_flagged() {
    let asset = p38();
    let flags: Vec<(usize, &UsdMeshPart)> = asset
        .parts()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.role == PartRole::InstrumentFlag)
        .collect();
    assert_eq!(flags.len(), 22);
    let shown = |art: &AirframeArticulation| {
        let draws = asset.evaluate_draws(art, SlotCameraRole::PilotEye);
        flags
            .iter()
            .filter(|(i, _)| draws.iter().any(|d| d.part_index == *i))
            .count()
    };
    assert_eq!(shown(&AirframeArticulation::default()), 0);
    let mut art = AirframeArticulation::default();
    art.instrument_needles.set_slot_state(0, true, true);
    assert_eq!(shown(&art), 1);
}

/// The gyro horizon's bar is a card that turns and slides on its card
/// channel; the directional gyro's and compass's heading cards are drums
/// turning on their slots' first wheel channel; the miniature aircraft, the
/// lubber lines and the turn indicator's tube are fixed in front.
#[test]
fn cards_tapes_and_overlays_stand_in_front_of_their_faces() {
    let asset = p38();
    let card = |path: &str| {
        asset
            .parts()
            .iter()
            .enumerate()
            .find(|(_, p)| p.prim_path.contains(path))
            .unwrap()
    };
    let (bar_index, bar) = card("/gyro_horizon/gyro_horizon/Card0/");
    assert_eq!(bar.role, PartRole::InstrumentCard);
    let channel = bar.needle_channel.unwrap();
    let (slot, _) = (
        channel / sim_core::CARDS_PER_SLOT,
        channel % sim_core::CARDS_PER_SLOT,
    );
    let tip = Vec3::from_array(
        bar.vertices
            .iter()
            .max_by(|a, b| a.position[0].total_cmp(&b.position[0]))
            .unwrap()
            .position,
    )
    .as_dvec3();
    let at = |turn: f64, travel: f64| {
        let mut art = AirframeArticulation::default();
        art.instrument_needles.set_card(slot, 0, turn, travel, true);
        let draws = asset.evaluate_draws(&art, SlotCameraRole::PilotEye);
        let draw = draws.iter().find(|d| d.part_index == bar_index).unwrap();
        draw.part_to_aircraft.transform_point3(tip)
    };
    let level = at(0.0, 0.0);
    // Turned clockwise as the pilot sees it, the bar's right end drops.
    assert!(at(0.3, 0.0).z < level.z - 0.003);
    // Travelling down the card lowers it.
    assert!(at(0.0, 0.005).z < level.z - 0.004);
    for path in [
        "/directional_gyro/directional_gyro/Tape",
        "/magnetic_compass/magnetic_compass/Tape",
    ] {
        let (_, tape) = card(path);
        assert_eq!(tape.role, PartRole::InstrumentDrum);
        assert!(matches!(tape.surface, PartSurface::Display { .. }));
        assert_eq!(
            tape.needle_channel
                .map(|c| c % sim_core::DRUM_WHEELS_PER_SLOT),
            Some(0)
        );
    }
    for path in [
        "/gyro_horizon/gyro_horizon/Overlay0/",
        "/turn_and_bank/turn_and_bank/Overlay0/",
    ] {
        assert_eq!(card(path).1.role, PartRole::CabinInterior);
    }
}

/// Every placard whose wording the photographs show is printed on its plate.
#[test]
fn sourced_placards_are_lettered_on_their_plates() {
    let asset = p38();
    let placards: Vec<&UsdMeshPart> = asset
        .parts()
        .iter()
        .filter(|p| p.prim_path.starts_with("/P38g/CockpitInterior/Placards/"))
        .collect();
    assert_eq!(placards.len(), sim_p38g::p38g_placards().len());
    for plate in placards {
        let PartSurface::Display { uv_rect, .. } = plate.surface else {
            panic!("{} is not lettered", plate.prim_path);
        };
        // A long plate keeps its proportions in the atlas.
        assert!(uv_rect[2] > uv_rect[3], "{} {uv_rect:?}", plate.prim_path);
    }
}
