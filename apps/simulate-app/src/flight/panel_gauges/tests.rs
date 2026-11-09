use mbd_airdata::{ISA_P0_PA, impact_pressure_from_cas_pa};
use sim_core::InstrumentNeedles;
use sim_pack::builtin_p38g_package;

use super::{GaugeInputs, PanelGauges};

const MPH_TO_M_S: f64 = 0.447_04;
const IN_HG_PA: f64 = 3_386.389;

fn p38_gauges() -> PanelGauges {
    let pkg = builtin_p38g_package().unwrap();
    PanelGauges::for_pack(pkg.instrument_pack())
}

fn cruise() -> GaugeInputs {
    let qc = impact_pressure_from_cas_pa(200.0 * MPH_TO_M_S);
    GaugeInputs {
        pitot_static_pa: Some([ISA_P0_PA + qc, ISA_P0_PA]),
        manifold_pressure_pa: [40.0 * IN_HG_PA, 30.0 * IN_HG_PA],
        engine_rpm: [3_000.0, 2_250.0],
    }
}

fn angle_deg(needles: InstrumentNeedles, slot: usize) -> f64 {
    needles
        .angle_rad(InstrumentNeedles::channel(slot, 0).unwrap())
        .to_degrees()
}

/// Layout order of `usaaf-1943-fighter`: airspeed, altimeter, rate of climb,
/// manifold pressure left/right, tachometer left/right.
#[test]
fn every_p38_gauge_reads_its_own_source_on_its_dial_scale() {
    let mut gauges = p38_gauges();
    assert_eq!(gauges.gauges.len(), 7);
    let inputs = cruise();
    gauges.settle(&inputs);
    let n = gauges.needles();
    // 200 mph is engraved at 120 deg and the dial spaces 0.6 deg/mph there; the
    // pack's bench instrument error reads 2 mph high.
    assert!(
        (angle_deg(n, 0) - 121.2).abs() < 0.2,
        "ASI {}",
        angle_deg(n, 0)
    );
    assert!(angle_deg(n, 1).abs() < 1.0, "sea-level altimeter");
    assert!(angle_deg(n, 2).abs() < 1.0, "level flight VSI");
    let mp = |inhg: f64| (inhg - 10.0) / 65.0 * 300.0;
    assert!((angle_deg(n, 3) - mp(40.0)).abs() < 0.5);
    assert!((angle_deg(n, 4) - mp(30.0)).abs() < 0.5);
    assert!((angle_deg(n, 5) - 200.0).abs() < 0.5);
    assert!((angle_deg(n, 6) - 150.0).abs() < 0.5);
}

/// A needle is a damped second-order system: it lags a step input and settles.
#[test]
fn a_tachometer_needle_follows_a_step_with_its_pack_dynamics() {
    let mut gauges = p38_gauges();
    let mut inputs = cruise();
    inputs.engine_rpm = [0.0, 0.0];
    gauges.settle(&inputs);
    inputs.engine_rpm = [3_000.0, 0.0];
    gauges.step(1.0 / 60.0, &inputs);
    let early = angle_deg(gauges.needles(), 5);
    assert!(early > 0.0 && early < 100.0, "first step {early}");
    for _ in 0..600 {
        gauges.step(1.0 / 60.0, &inputs);
    }
    let settled = angle_deg(gauges.needles(), 5);
    assert!((settled - 200.0).abs() < 0.5, "settled {settled}");
}
