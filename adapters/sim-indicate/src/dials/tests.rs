use std::f32::consts::TAU;

use indicate_instrument_analog::Lighting;
use indicate_instrument_descriptor::DesignFrame;
use indicate_instrument_glyphs::{FaceTable, Typeface};
use indicate_instrument_scene::{MAX_SCENE_BYTES, SceneWriter};
use syspack::InstrumentPackDocument;

use super::{
    DialSource, EngineGauges, GaugeIndications, Lane, PanelDials, PointerValues, SlotDial,
    bake_face,
};

const FIGHTER: &[u8] =
    include_bytes!("../../../../instruments/usaaf-1943-fighter/instruments.json");
const SIX_PACK: &[u8] =
    include_bytes!("../../../../instruments/ga-analog-six-pack/instruments.json");

const KT_TO_M_S: f64 = 1_852.0 / 3_600.0;
const MPH_TO_M_S: f64 = 0.447_04;
const FT_TO_M: f64 = 0.3048;
const INHG_TO_PA: f64 = 3_386.389;
const PSI_TO_PA: f64 = 6_894.757;
const US_GALLON_M3: f64 = 0.003_785_411_784;

fn dials(pack: &[u8]) -> PanelDials {
    let pack = InstrumentPackDocument::parse(pack).unwrap();
    let (dials, refused) = PanelDials::from_pack(
        &pack,
        &["front_left", "front_right", "rear_left", "rear_right"],
    );
    assert!(refused.is_empty(), "{refused:?}");
    dials
}

fn slot<'a>(dials: &'a PanelDials, name: &str) -> &'a SlotDial {
    dials.slots().iter().find(|s| s.name == name).unwrap()
}

/// Clockwise from twelve, folded into `[0, 360)`.
fn wrapped_deg(rad: f32) -> f32 {
    rad.rem_euclid(TAU).to_degrees()
}

/// `dial`'s pointer values for `shown`, with no direct lane fed.
fn values(dial: &DialSource, shown: &GaugeIndications) -> PointerValues {
    let state = shown.resolve();
    dial.values(&state.data, &state.beside(), |_| None)
}

#[test]
fn every_slot_of_both_packs_makes_a_dial() {
    assert_eq!(dials(FIGHTER).slots().len(), 22);
    assert_eq!(dials(SIX_PACK).slots().len(), 4);
}

#[test]
fn the_fighter_airspeed_needle_sits_on_the_packs_scale_point() {
    let dials = dials(FIGHTER);
    let asi = &slot(&dials, "flight_airspeed").dial;
    let shown = GaugeIndications {
        airspeed_m_s: Some(200.0 * MPH_TO_M_S),
        ..GaugeIndications::default()
    };
    let values = values(asi, &shown);
    assert!(
        (values.pointers[0].unwrap() - 200.0).abs() < 0.05,
        "{values:?}"
    );
    let pose = asi.pose(&values);
    // The pack engraves 200 mph at 206 degrees from twelve o'clock.
    assert!((wrapped_deg(pose.pointers[0].angle_rad) - 206.0).abs() < 0.1);
    assert!(pose.pointers[0].live && !pose.flag);
}

#[test]
fn the_altimeter_hands_turn_once_per_thousand_ten_thousand_and_hundred_thousand_feet() {
    let dials = dials(FIGHTER);
    let alt = &slot(&dials, "flight_altimeter").dial;
    let shown = GaugeIndications {
        altitude_m: Some(12_345.0 * FT_TO_M),
        altimeter_setting_hpa: Some(1_013.25),
        ..GaugeIndications::default()
    };
    let values = values(alt, &shown);
    assert!(
        (values.pointers[2].unwrap() - 12_345.0).abs() < 0.5,
        "{values:?}"
    );
    assert!((values.window.unwrap() - 29.92).abs() < 0.01, "{values:?}");
    let pose = alt.pose(&values);
    // Lowest first: ten-thousands pointer, thousands hand, hundreds hand.
    let [ten_thousands, thousands, hundreds, _] = pose.pointers.map(|p| wrapped_deg(p.angle_rad));
    assert!((hundreds - 0.345 * 360.0).abs() < 0.2, "{hundreds}");
    assert!((thousands - 0.2345 * 360.0).abs() < 0.2, "{thousands}");
    assert!(
        (ten_thousands - 0.12345 * 360.0).abs() < 0.2,
        "{ten_thousands}"
    );
    assert!(pose.windows[0].live);
}

#[test]
fn each_engine_gauge_reads_its_own_engine() {
    let dials = dials(FIGHTER);
    let gauges = |inhg: f64, rpm: f64| EngineGauges {
        manifold_pressure_pa: Some(inhg * INHG_TO_PA),
        rpm: Some(rpm),
        ..EngineGauges::default()
    };
    let shown = GaugeIndications {
        engines: vec![gauges(30.0, 2_000.0), gauges(45.0, 3_000.0)],
        ..GaugeIndications::default()
    };
    let read = |name: &str| values(&slot(&dials, name).dial, &shown).pointers[0].unwrap();
    assert!((read("engine_mp_left") - 30.0).abs() < 0.01);
    assert!((read("engine_mp_right") - 45.0).abs() < 0.01);
    assert!((read("engine_rpm_left") - 2_000.0).abs() < 0.5);
    assert!((read("engine_rpm_right") - 3_000.0).abs() < 0.5);
}

#[test]
fn an_engine_gage_unit_reads_three_engine_lanes_each_on_its_own_sub_dial() {
    let dials = dials(FIGHTER);
    let unit = &slot(&dials, "engine_gage_left").dial;
    assert!(unit.lanes().all(|lane| matches!(lane, Lane::State(_))));
    let shown = GaugeIndications {
        engines: vec![EngineGauges {
            oil_temperature_k: Some(273.15 + 50.0),
            oil_pressure_pa: Some(100.0 * PSI_TO_PA),
            fuel_pressure_pa: Some(12.5 * PSI_TO_PA),
            ..EngineGauges::default()
        }],
        ..GaugeIndications::default()
    };
    let values = values(unit, &shown);
    let pose = unit.pose(&values);
    // Each scale's midpoint: oil temperature at twelve on its upper arc, oil
    // pressure at nine o'clock on the left, fuel pressure at three on the right.
    let deg = pose.pointers.map(|p| wrapped_deg(p.angle_rad));
    assert!(
        deg[0].abs() < 0.1 || (deg[0] - 360.0).abs() < 0.1,
        "{deg:?}"
    );
    assert!((deg[1] - 270.0).abs() < 0.1, "{deg:?}");
    assert!((deg[2] - 90.0).abs() < 0.1, "{deg:?}");
    assert!(!pose.flag);
    let build = unit.physical_build().unwrap();
    let pivots: Vec<[f32; 2]> = build
        .pointers
        .iter()
        .flatten()
        .map(|p| p.pivot_mm)
        .collect();
    assert_eq!(pivots.len(), 3);
    assert!(pivots[1][0] < 0.0 && pivots[2][0] > 0.0 && pivots[0][1] < 0.0);
    // The engine on the other side of the panel is not this one's.
    let right = &slot(&dials, "engine_gage_right").dial;
    assert!(
        right
            .pose(&super::DialSource::values(
                right,
                &shown.resolve().data,
                &shown.resolve().beside(),
                |_| None
            ))
            .flag
    );
}

#[test]
fn a_dual_fuel_gauge_reads_its_two_tanks() {
    let dials = dials(FIGHTER);
    let fuel = &slot(&dials, "fuel_front").dial;
    let shown = GaugeIndications {
        tanks_m3: vec![
            Some(90.0 * US_GALLON_M3),
            Some(45.0 * US_GALLON_M3),
            None,
            None,
        ],
        ..GaugeIndications::default()
    };
    let pose = fuel.pose(&values(fuel, &shown));
    let [left, right, ..] = pose.pointers.map(|p| p.angle_rad.to_degrees());
    // Full is 240 degrees past the -120 degree reference, half 120 degrees.
    assert!(
        (left - 120.0).abs() < 0.2 && right.abs() < 0.2,
        "{left} {right}"
    );
    // The rear gauge's tanks report nothing, so its pointers park.
    let rear = &slot(&dials, "fuel_rear").dial;
    assert!(rear.pose(&values(rear, &shown)).flag);
}

#[test]
fn a_dial_without_input_parks_and_flags() {
    let dials = dials(SIX_PACK);
    let asi = &slot(&dials, "top_left").dial;
    let pose = asi.pose(&PointerValues::default());
    assert!(pose.flag && !pose.pointers[0].live);
    let kt = values(
        asi,
        &GaugeIndications {
            airspeed_m_s: Some(100.0 * KT_TO_M_S),
            ..GaugeIndications::default()
        },
    );
    assert!((kt.pointers[0].unwrap() - 100.0).abs() < 0.05);
}

#[test]
fn every_fighter_face_bakes_with_luminous_paint_marked_in_alpha() {
    let dials = dials(FIGHTER);
    let side = 256;
    for slot in dials.slots() {
        let face = bake_face(&slot.dial, side, &FaceTable::DEFAULT).unwrap();
        assert_eq!(face.rgba.len(), (side * side * 4) as usize);
        assert_eq!((face.width, face.height), (side, side));
        let glowing = face.rgba.chunks(4).filter(|px| px[3] > 128).count();
        // Indicate's heading-card dials carry their radium on the card and
        // lubber line, not the face.
        let on_the_face = !matches!(slot.name.as_str(), "directional_gyro" | "magnetic_compass");
        assert!(
            glowing > 100 || !on_the_face,
            "{}: {glowing} luminous pixels",
            slot.name
        );
    }
    let asi = bake_face(
        &slot(&dials, "flight_airspeed").dial,
        side,
        &FaceTable::DEFAULT,
    )
    .unwrap();
    // The centre of the airspeed face carries no paint at all.
    let centre = ((side / 2) * side + side / 2) as usize * 4;
    assert_eq!(asi.rgba[centre + 3], 0);
}

#[test]
fn the_same_dial_draws_whole_for_a_flat_panel() {
    let dials = dials(FIGHTER);
    let unit = &slot(&dials, "engine_gage_right").dial;
    let mut values = PointerValues::default();
    values.pointers[..3].copy_from_slice(&[Some(70.0), Some(65.0), Some(16.0)]);
    let pose = unit.pose(&values);
    let mut scene = vec![0_u8; MAX_SCENE_BYTES];
    let mut writer = SceneWriter::new(&mut scene).unwrap();
    let frame = DesignFrame {
        width: 240.0,
        height: 240.0,
    };
    unit.draw_dial(&pose, Lighting::Day, frame, &mut writer)
        .unwrap();
    assert!(writer.finish() > 0);
}

#[test]
fn the_kollsman_drum_strip_and_the_off_flag_bake() {
    let dials = dials(FIGHTER);
    let alt = &slot(&dials, "flight_altimeter").dial;
    let strip = super::bake_drum_strip(alt, 400, &FaceTable::DEFAULT)
        .unwrap()
        .unwrap();
    assert_eq!(strip.height, 400);
    assert!(strip.width > 10 && strip.width < 400, "{}", strip.width);
    // Luminous digits glow; the drum between them does not.
    assert!(strip.rgba.chunks(4).any(|px| px[3] > 128));
    assert!(strip.rgba.chunks(4).filter(|px| px[3] == 0).count() > strip.rgba.len() / 8);
    let asi = &slot(&dials, "flight_airspeed").dial;
    assert!(
        super::bake_drum_strip(asi, 400, &FaceTable::DEFAULT)
            .unwrap()
            .is_none()
    );
    let flag = super::bake_off_flag(120, 48).unwrap();
    let red = flag
        .rgba
        .chunks(4)
        .filter(|px| px[0] > 150 && px[1] < 80)
        .count();
    let white = flag
        .rgba
        .chunks(4)
        .filter(|px| px[0] > 200 && px[1] > 200)
        .count();
    assert!(red > 2000 && white > 100, "{red} {white}");
}

const US_MS33558: &[u8] = include_bytes!("../../../../typefaces/us-ms33558/us-ms33558.idtf");

#[test]
fn the_fighter_dials_are_lettered_in_the_faces_the_pack_pins() {
    let dials = dials(FIGHTER);
    let face = Typeface::parse(US_MS33558).unwrap();
    let faces = dials.lettering().faces(&[("us-ms33558", face)]).unwrap();
    // The reference face, then the Kollsman-1940 and regular styles.
    assert_eq!(faces.len(), 3);
    let table = FaceTable::new(&faces);
    let asi = &slot(&dials, "flight_airspeed").dial;
    let lettered = bake_face(asi, 256, &table).unwrap();
    let plain = bake_face(asi, 256, &FaceTable::DEFAULT).unwrap();
    let differing = lettered
        .rgba
        .chunks(4)
        .zip(plain.rgba.chunks(4))
        .filter(|(a, b)| a != b)
        .count();
    assert!(differing > 500, "{differing} pixels lettered differently");
    let strip = super::bake_drum_strip(&slot(&dials, "flight_altimeter").dial, 400, &table)
        .unwrap()
        .unwrap();
    assert!(strip.rgba.chunks(4).any(|px| px[3] > 128));
    // A face the pack names but the host does not supply is refused.
    assert!(dials.lettering().faces(&[]).is_err());
}
