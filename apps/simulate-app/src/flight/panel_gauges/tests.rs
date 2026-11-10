use mbd_airdata::{ISA_P0_PA, impact_pressure_from_cas_pa};
use sim_core::{FUEL_TANKS, InstrumentNeedles};
use sim_pack::builtin_p38g_package;

use super::sources::EngineInputs;
use super::{GaugeInputs, PanelGauges};

const MPH_TO_M_S: f64 = 0.447_04;
const IN_HG_PA: f64 = 3_386.389;
const PSI_PA: f64 = 6_894.757;
const US_GALLON_M3: f64 = 0.003_785_411_784;

/// Layout slots of `usaaf-1943-fighter`.
const AIRSPEED: usize = 0;
const ALTIMETER: usize = 1;
const RATE_OF_CLIMB: usize = 2;
const MP_LEFT: usize = 3;
const MP_RIGHT: usize = 4;
const RPM_LEFT: usize = 5;
const RPM_RIGHT: usize = 6;
const SUCTION: usize = 7;
const GAGE_LEFT: usize = 10;
const COOLANT_LEFT: usize = 12;
const FUEL_FRONT: usize = 14;
const GYRO_HORIZON: usize = 16;
const DIRECTIONAL_GYRO: usize = 17;
const TURN_AND_BANK: usize = 18;
const MAGNETIC_COMPASS: usize = 19;
const SYNCHROSCOPE: usize = 20;
const CLOCK: usize = 21;

fn p38_gauges() -> PanelGauges {
    let pkg = builtin_p38g_package().unwrap();
    PanelGauges::for_pack(pkg.instrument_pack())
}

fn engine(inhg: f64, rpm: f64, oil_c: f64, oil_psi: f64) -> EngineInputs {
    EngineInputs {
        manifold_pressure_pa: inhg * IN_HG_PA,
        rpm,
        oil_temperature_k: oil_c + 273.15,
        oil_pressure_pa: oil_psi * PSI_PA,
        shaft_angle_deg: None,
    }
}

fn cruise() -> GaugeInputs {
    let qc = impact_pressure_from_cas_pa(200.0 * MPH_TO_M_S);
    let mut fuel_tanks_m3 = [None; FUEL_TANKS];
    fuel_tanks_m3[..4].copy_from_slice(&[90.0, 45.0, 60.0, 30.0].map(|g| Some(g * US_GALLON_M3)));
    GaugeInputs {
        pitot_static_pa: Some([ISA_P0_PA + qc, ISA_P0_PA]),
        altimeter_setting_hpa: Some(1_013.25),
        engines: vec![
            engine(40.0, 3_000.0, 50.0, 100.0),
            engine(30.0, 1_750.0, 70.0, 60.0),
        ],
        fuel_tanks_m3,
        flight: None,
        clock_time_of_day_s: None,
    }
}

/// Face angle of `hand` on `slot`, degrees clockwise from twelve in
/// `(-180, 180]`.
fn hand_deg(needles: InstrumentNeedles, slot: usize, hand: usize) -> f64 {
    let deg = needles
        .angle_rad(InstrumentNeedles::channel(slot, hand).unwrap())
        .to_degrees()
        .rem_euclid(360.0);
    if deg > 180.0 { deg - 360.0 } else { deg }
}

fn angle_deg(needles: InstrumentNeedles, slot: usize) -> f64 {
    hand_deg(needles, slot, 0)
}

#[test]
fn every_p38_gauge_reads_its_own_source_on_its_dial_scale() {
    let mut gauges = p38_gauges();
    // Sixteen gauges and six direct instruments: two engine gage units of
    // three pointers and two dual fuel gauges add six pointers.
    assert_eq!(gauges.dials().slots().len(), 22);
    assert_eq!(gauges.gauges.len(), 22);
    gauges.settle(&cruise());
    let n = gauges.needles();
    // 200 mph is engraved at 206 deg; the pack's bench error reads 2 mph high
    // where the scale spaces 0.37 deg per mph.
    let asi = angle_deg(n, AIRSPEED) + 360.0 * f64::from(angle_deg(n, AIRSPEED) < 0.0);
    assert!((asi - 206.7).abs() < 0.3, "ASI {asi}");
    assert!(angle_deg(n, ALTIMETER).abs() < 1.0, "sea-level altimeter");
    assert!(
        (angle_deg(n, RATE_OF_CLIMB) + 90.0).abs() < 1.0,
        "level flight ROC"
    );
    // 10-50 inHg over 320 degrees from -160; 0-3,500 rpm over 290 from -150.
    let mp = |inhg: f64| (inhg - 10.0) / 40.0 * 320.0 - 160.0;
    assert!((angle_deg(n, MP_LEFT) - mp(40.0)).abs() < 0.5);
    assert!((angle_deg(n, MP_RIGHT) - mp(30.0)).abs() < 0.5);
    let rpm = |rpm: f64| rpm / 3_500.0 * 290.0 - 150.0;
    assert!((angle_deg(n, RPM_LEFT) - rpm(3_000.0)).abs() < 0.5);
    assert!((angle_deg(n, RPM_RIGHT) - rpm(1_750.0)).abs() < 0.5);
}

#[test]
fn the_engine_gage_unit_and_fuel_gauges_read_every_pointer() {
    let mut gauges = p38_gauges();
    gauges.settle(&cruise());
    let n = gauges.needles();
    // Left engine: 50 C at twelve, 100 psi at nine o'clock on its sub-dial.
    assert!(hand_deg(n, GAGE_LEFT, 0).abs() < 0.5);
    assert!((hand_deg(n, GAGE_LEFT, 1) + 90.0).abs() < 0.5);
    // The fuel pressure the aircraft does not publish rests at its stop.
    assert!((hand_deg(n, GAGE_LEFT, 2) - 20.0).abs() < 0.5);
    // Front tanks: 90 of 90 gal and 45 of 90, 240 degrees from -120.
    assert!((hand_deg(n, FUEL_FRONT, 0) - 120.0).abs() < 0.5);
    assert!(hand_deg(n, FUEL_FRONT, 1).abs() < 0.5);
    // Coolant temperature and suction are not published either.
    assert!((angle_deg(n, COOLANT_LEFT) + 135.0).abs() < 0.5);
    assert!((angle_deg(n, SUCTION) + 150.0).abs() < 0.5);
}

/// A needle is a damped second-order system: it lags a step input and settles.
#[test]
fn a_tachometer_needle_follows_a_step_with_its_pack_dynamics() {
    let mut gauges = p38_gauges();
    let mut inputs = cruise();
    inputs.engines[0].rpm = 0.0;
    gauges.settle(&inputs);
    inputs.engines[0].rpm = 3_000.0;
    gauges.step(1.0 / 60.0, &inputs);
    let early = angle_deg(gauges.needles(), RPM_LEFT);
    assert!(early > -150.0 && early < -50.0, "first step {early}");
    for _ in 0..600 {
        gauges.step(1.0 / 60.0, &inputs);
    }
    let settled = angle_deg(gauges.needles(), RPM_LEFT);
    assert!(
        (settled - (3_000.0 / 3_500.0 * 290.0 - 150.0)).abs() < 0.5,
        "settled {settled}"
    );
}

/// The clock shows the time of day it is handed and the synchroscope how far
/// the left shaft leads the right, with no mechanism between; with nothing
/// handed over the clock shows no hands.
#[test]
fn the_clock_and_synchroscope_read_the_state_directly() {
    let mut gauges = p38_gauges();
    let mut inputs = cruise();
    gauges.settle(&inputs);
    let n = gauges.needles();
    let hour = InstrumentNeedles::channel(CLOCK, 0).unwrap();
    assert!(!n.visible(hour), "a clock without a time hides its hands");
    inputs.clock_time_of_day_s = Some(10.0 * 3_600.0 + 10.0 * 60.0 + 30.0);
    inputs.engines[0].shaft_angle_deg = Some(100.0);
    inputs.engines[1].shaft_angle_deg = Some(10.0);
    gauges.settle(&inputs);
    let n = gauges.needles();
    assert!((hand_deg(n, CLOCK, 0) - (305.25 - 360.0)).abs() < 0.05);
    assert!((hand_deg(n, CLOCK, 1) - 63.0).abs() < 0.05);
    assert!((hand_deg(n, CLOCK, 2).abs() - 180.0).abs() < 0.05);
    assert!((hand_deg(n, SYNCHROSCOPE, 0) - 90.0).abs() < 0.05);
}

#[test]
fn the_clock_is_noon_under_the_sun_and_six_in_the_morning_a_quarter_turn_west() {
    use super::flight_state::solar_time_of_day_s;
    let sun = glam::DVec3::X;
    assert!((solar_time_of_day_s(glam::DVec3::X * 6.4e6, sun, 0.0) - 43_200.0).abs() < 1e-6);
    let west = glam::DVec3::new(0.0, -6.4e6, 0.0);
    assert!((solar_time_of_day_s(west, sun, 0.0) - 21_600.0).abs() < 1e-6);
    assert!((solar_time_of_day_s(west, sun, 90.0) - 21_690.0).abs() < 1e-6);
}

/// The aircraft over Newark heading 090 true, 20 degrees right wing down and
/// 5 nose up, turning right at 0.1 rad/s with 1 m/s² of lateral force.
fn banked_over_newark() -> GaugeInputs {
    use glam::{DQuat, DVec3, EulerRot};
    use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
    let at = Geodetic::from_degrees(40.69, -74.17, 300.0);
    let local = LocalFrame::at(&at);
    let attitude = DQuat::from_mat3(&local.ned_to_ecef())
        * DQuat::from_rotation_z(90_f64.to_radians())
        * DQuat::from_euler(EulerRot::ZYX, 0.0, 5_f64.to_radians(), 20_f64.to_radians());
    let vehicle = sim_core::VehicleState {
        position_ecef: BodyEllipsoid::WGS84.geodetic_to_fixed(&at),
        velocity_ecef: local.east() * 90.0,
        attitude: attitude.normalize(),
        angular_rate_body: DVec3::new(0.0, 0.0, 0.1),
    };
    cruise().with_flight(
        sim_core::SimInstant::START,
        vehicle,
        Some(DVec3::new(0.0, 1.0, 0.0)),
        sim_scenario::ScenarioWorld::EarthGlobe,
    )
}

/// The gyro horizon, directional gyro, compass and turn and bank read the
/// flight state the avionics feed builds, the compass through the World
/// Magnetic Model's variation (about 12.7 degrees west at Newark).
#[test]
fn the_gyro_and_compass_instruments_read_the_flight_state() {
    let mut gauges = p38_gauges();
    gauges.settle(&banked_over_newark());
    let n = gauges.needles();
    let bar = InstrumentNeedles::card_channel(GYRO_HORIZON, 0).unwrap();
    let (turn, travel) = n.card(bar);
    assert!(n.card_visible(bar));
    assert!(
        (turn.to_degrees() + 20.0).abs() < 0.1,
        "bar turned {}",
        turn.to_degrees()
    );
    assert!(travel > 0.0, "nose up lowers the bar: {travel}");
    let drum = |slot: usize| n.wheel(InstrumentNeedles::wheel_channel(slot, 0).unwrap()) * 36.0;
    let magnetic = 90.0 + 12.7;
    assert!(
        (drum(DIRECTIONAL_GYRO) - magnetic).abs() < 1.5,
        "DG {}",
        drum(DIRECTIONAL_GYRO)
    );
    assert!(
        (drum(MAGNETIC_COMPASS) + magnetic).abs() < 1.5,
        "compass {}",
        drum(MAGNETIC_COMPASS)
    );
    assert!(n.drum_live(DIRECTIONAL_GYRO) && n.drum_live(MAGNETIC_COMPASS));
    // The needle (hand 1) deflects right for a right turn; the ball shows.
    assert!(hand_deg(n, TURN_AND_BANK, 1) > 5.0);
    assert!(n.visible(InstrumentNeedles::channel(TURN_AND_BANK, 0).unwrap()));
}
