use mbd_airdata::{ISA_P0_PA, impact_pressure_from_cas_pa};
use syspack::{
    InstrumentMechanism, MemoryPackageSource, PackageSource as _, SyspackError, SyspackManifest,
};

use crate::{PackError, builtin_c172_package, builtin_p38g_package, builtin_p38g_source};

const MPH_TO_M_S: f64 = 0.447_04;
const KT_TO_M_S: f64 = 0.514_444;

#[test]
fn each_aircraft_gets_the_instruments_of_the_pack_it_pins() {
    let c172 = builtin_c172_package().unwrap();
    let p38 = builtin_p38g_package().unwrap();
    assert_ne!(c172.instrument_pack(), p38.instrument_pack());
    let pack = p38
        .instrument_pack()
        .expect("the P-38G pins the 1943 fighter pack");
    assert!(pack.instrument("manifold_pressure_1").is_some());
    assert!(p38.pitot_static_device("pitot_static").is_some());
}

/// FS-M-19-1492-A, flaps and gear up: 115 mph indicated is 113 mph on the
/// water column (instrument error +2) and 127 mph calibrated (installation error
/// -14). Flying 127 mph CAS through the P-38G's position-error table, its
/// pitot-static plumbing and the pack's airspeed indicator must read 115.
#[test]
fn the_p38g_airspeed_indicator_reproduces_the_aaf_installation_test() {
    let p38 = builtin_p38g_package().unwrap();
    let pec = p38.pitot_static_calibration().unwrap();
    let cas_m_s = 127.0 * MPH_TO_M_S;
    let water_column_m_s = pec.kcas_to_kias(0.0, cas_m_s / KT_TO_M_S) * KT_TO_M_S;
    let qc_true = impact_pressure_from_cas_pa(cas_m_s);
    // The total head reads true; the static holes sense the position error.
    let total_pa = ISA_P0_PA + qc_true;
    let static_pa = total_pa - impact_pressure_from_cas_pa(water_column_m_s);

    let spec = p38
        .instrument_pack()
        .unwrap()
        .instrument("airspeed")
        .unwrap();
    let InstrumentMechanism::Airspeed(mut asi) = spec.mechanism().unwrap() else {
        panic!("the airspeed instrument is not an airspeed indicator");
    };
    let mut reading = asi.step(1.0 / 60.0, total_pa, static_pa);
    for _ in 0..240 {
        reading = asi.step(1.0 / 60.0, total_pa, static_pa);
    }
    assert!(
        (reading.indicated - 115.0).abs() < 0.6,
        "indicated {} mph",
        reading.indicated
    );
}

#[test]
fn a_pin_on_another_revision_of_the_pack_refuses_the_vehicle() {
    let source = builtin_p38g_source();
    let mut text = String::from_utf8(source.read_manifest_bytes().unwrap()).unwrap();
    let pinned = text.find("\"manifest_sha256\": \"").unwrap() + 20;
    text.replace_range(pinned..pinned + 64, &"0".repeat(64));
    let manifest: SyspackManifest = serde_json::from_str(&text).unwrap();
    let assets = manifest
        .assets
        .keys()
        .map(|path| (path.clone(), source.read_asset_bytes(path).unwrap()))
        .collect();
    let tampered = MemoryPackageSource::new(text.into_bytes(), assets);
    assert!(matches!(
        crate::LoadedSyspack::load(&tampered),
        Err(PackError::Syspack(SyspackError::PinMismatch { .. }))
    ));
}
