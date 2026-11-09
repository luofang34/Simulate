//! Closed-loop flight test of `vehicles/g4m1` against the Navy's test of G4M1
//! Model 11 aircraft No. 1: the shipped `performance/poh_benchmarks.json` is the
//! test card, flown through the same calibration harness every package uses.

use jsbsim::{Fdm, LoadOptions};
use sim_pack::{
    LoadedSyspack, builtin_g4m1_kasei15_package, builtin_g4m1_package, evaluate_package,
};

fn new_fdm(package: &LoadedSyspack) -> Result<Fdm, Box<dyn std::error::Error>> {
    let files = package.build_jsbsim_resolver()?;
    let (aircraft_dir, model_name) = package.jsbsim_model()?;
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true))?;
    fdm.set_dt(1.0 / 120.0);
    fdm.load_model(&files, aircraft_dir, model_name)?;
    Ok(fdm)
}

/// The package's declared masses are the masses JSBSim flies.
#[test]
fn the_manifest_agrees_with_the_mass_jsbsim_flies() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_g4m1_package()?;
    assert_eq!(package.manifest().id, "g4m1");
    assert!(package.instrument_pack().is_none());
    assert!(package.pitot_static_device("pitot_static").is_some());
    let pounds = package.tree().mass_properties()?.total_mass_kg / 0.453_592_37;
    let mut fdm = new_fdm(&package)?;
    fdm.run_ic()?;
    let flown = fdm.get_property("inertia/weight-lbs")?;
    assert!((pounds - flown).abs() < 1e-2, "{pounds} vs {flown}");
    Ok(())
}

/// The whole Navy test card passes within the card's tolerances.
#[test]
fn g4m1_matches_the_navy_test_of_aircraft_no_1() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_g4m1_package()?;
    let report = evaluate_package(&package)?;
    assert!(
        report.all_passed,
        "G4M1 failed its Navy test card:\n{}",
        report.to_markdown()
    );
    Ok(())
}

/// A card's rows as `(name, passed)` and the card as Markdown.
type CardRows = (Vec<(String, bool)>, String);

/// Every row of a variant's card as `(name, passed)`: level points as
/// `level <alt> ft`, climb-rate points as `climb rate <alt> ft`, timed climbs by
/// their card label.
fn card_rows(package: &LoadedSyspack) -> Result<CardRows, Box<dyn std::error::Error>> {
    let report = evaluate_package(package)?;
    let mut rows: Vec<(String, bool)> = report
        .cruises
        .iter()
        .map(|c| (format!("level {:.0} ft", c.pressure_alt_ft), c.passed))
        .collect();
    rows.extend(
        report
            .climb_times
            .iter()
            .map(|c| (c.label.clone(), c.passed)),
    );
    rows.extend(
        report
            .climbs
            .iter()
            .map(|c| (format!("climb rate {:.0} ft", c.pressure_alt_ft), c.passed)),
    );
    Ok((rows, report.to_markdown()))
}

/// A variant's card passes exactly the rows its validation record lists as
/// passing and fails exactly those it lists as failing, so a fix (a failing row
/// starts passing) or a regression (a passing row fails) both show here.
fn matches_its_validation_record(
    package: &LoadedSyspack,
) -> Result<(), Box<dyn std::error::Error>> {
    let path = package
        .manifest()
        .assets
        .keys()
        .find(|p| p.ends_with("/validation.json"))
        .ok_or("the variant declares a validation record")?;
    let record: serde_json::Value =
        serde_json::from_slice(package.asset(path).ok_or("validation record")?)?;
    let listed = |key: &str| -> Vec<String> {
        record
            .get(key)
            .and_then(serde_json::Value::as_array)
            .map(|names| {
                names
                    .iter()
                    .filter_map(|n| n.as_str().map(str::to_owned))
                    .collect()
            })
            .unwrap_or_default()
    };
    let (rows, card) = card_rows(package)?;
    let mut passing: Vec<String> = rows.iter().filter(|r| r.1).map(|r| r.0.clone()).collect();
    let mut failing: Vec<String> = rows.iter().filter(|r| !r.1).map(|r| r.0.clone()).collect();
    let (mut want_pass, mut want_fail) = (listed("passing"), listed("failing"));
    for list in [&mut passing, &mut failing, &mut want_pass, &mut want_fail] {
        list.sort();
    }
    assert_eq!(passing, want_pass, "{path}: passing rows\n{card}");
    assert_eq!(failing, want_fail, "{path}: failing rows\n{card}");
    let status = record
        .get("status")
        .and_then(serde_json::Value::as_str)
        .unwrap_or_default();
    assert_eq!(
        status == "calibrated",
        failing.is_empty(),
        "{path}: status {status}"
    );
    Ok(())
}

/// The Kasei 11 reference passes its whole card, as its validation record says.
#[test]
fn g4m1_kasei11_matches_its_validation_record() -> Result<(), Box<dyn std::error::Error>> {
    matches_its_validation_record(&builtin_g4m1_package()?)
}

/// The Kasei 15 variant is the Kasei 11 airframe with the other engine part,
/// nothing refitted, flown against the whole Navy test of aircraft No. 241: its
/// level speeds pass out of sample, and exactly the climb and ceiling rows its
/// validation record names fail.
#[test]
fn g4m1_kasei15_matches_its_validation_record_against_aircraft_no_241()
-> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_g4m1_kasei15_package()?;
    assert_eq!(package.manifest().id, "g4m1-kasei15");
    assert_eq!(
        package.part("engine0").map(|p| p.manifest().id.as_str()),
        Some("mitsubishi-kasei-15")
    );
    matches_its_validation_record(&package)
}

/// The variants share one airframe directory; each declares only its own card,
/// and a manifest declaring two cards is refused rather than graded on either.
#[test]
fn each_variant_flies_its_own_card_and_two_cards_are_refused()
-> Result<(), Box<dyn std::error::Error>> {
    let k11 = builtin_g4m1_package()?;
    let k15 = builtin_g4m1_kasei15_package()?;
    let designator = |p: &LoadedSyspack| p.calibration_suite().map(|c| c.type_designator.clone());
    assert_eq!(designator(&k11).as_deref(), Some("G4M1"));
    assert_eq!(designator(&k15).as_deref(), Some("G4M1-K15"));

    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../vehicles/g4m1");
    let mut manifest: serde_json::Value =
        serde_json::from_slice(&std::fs::read(dir.join("syspack.json"))?)?;
    let second = "performance/kasei15/poh_benchmarks.json";
    let bytes = std::fs::read(dir.join(second))?;
    manifest["assets"][second] = serde_json::json!({
        "sha256": syspack::sha256_hex(&bytes),
        "size_bytes": bytes.len(),
    });
    let mut assets = std::collections::BTreeMap::new();
    for path in manifest["assets"].as_object().ok_or("assets")?.keys() {
        assets.insert(path.clone(), std::fs::read(dir.join(path))?);
    }
    let source = sim_pack::MemoryPackageSource::new(serde_json::to_vec(&manifest)?, assets);
    assert!(matches!(
        LoadedSyspack::load(&source),
        Err(sim_pack::PackError::InvalidCalibration { .. })
    ));
    Ok(())
}

/// Each engine is its own JSBSim engine on its own controls and tanks: cutting
/// the right engine's mixture leaves the left at power, the right windmilling,
/// and the asymmetry yaws the aircraft toward the dead engine.
#[test]
fn either_engine_can_fail_and_the_aircraft_yaws_toward_it() -> Result<(), Box<dyn std::error::Error>>
{
    let package = builtin_g4m1_package()?;
    let mut fdm = new_fdm(&package)?;
    for (prop, val) in [
        ("ic/h-sl-ft", 5_000.0),
        ("ic/vt-fps", 150.0 * 1.687_81),
        ("ic/alpha-deg", 2.0),
        ("ic/theta-deg", 2.0),
        ("propulsion/magneto_cmd", 3.0),
        ("propulsion/set-running", -1.0),
    ] {
        fdm.set_property(prop, val)?;
    }
    for n in 0..2 {
        fdm.set_property(&format!("fcs/throttle-cmd-norm[{n}]"), 0.8)?;
        fdm.set_property(&format!("fcs/mixture-cmd-norm[{n}]"), 1.0)?;
        fdm.set_property(&format!("fcs/advance-cmd-norm[{n}]"), 0.9)?;
    }
    fdm.run_ic()?;
    fdm.set_property("fcs/mixture-cmd-norm[1]", 0.0)?;
    for _ in 0..600 {
        fdm.run()?;
    }
    let left = fdm.get_property("propulsion/engine[0]/thrust-lbs")?;
    let right = fdm.get_property("propulsion/engine[1]/thrust-lbs")?;
    let yaw_rate = fdm.get_property("velocities/r-rad_sec")?;
    assert!(fdm.get_property("propulsion/engine[0]/power-hp")? > 800.0);
    assert!(
        left > 1_000.0 && right < 200.0,
        "left {left} lb, right {right} lb"
    );
    assert!(
        yaw_rate > 0.01,
        "yaw rate {yaw_rate} rad/s toward the dead right engine"
    );
    let left_fuel = fdm.get_property("propulsion/tank[0]/contents-lbs")?;
    let right_fuel = fdm.get_property("propulsion/tank[1]/contents-lbs")?;
    assert!(
        left_fuel < right_fuel,
        "only the live engine draws: {left_fuel} vs {right_fuel}"
    );
    Ok(())
}
