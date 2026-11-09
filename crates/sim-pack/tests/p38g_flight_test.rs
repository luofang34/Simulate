//! Closed-loop flight test of `vehicles/p38g` against the P-38G's own AAF test
//! reports: the shipped `performance/poh_benchmarks.json` is the test card, flown
//! through the same calibration harness every package uses.

use std::io::Write as _;

use jsbsim::{Fdm, LoadOptions};
use sim_pack::{LoadedSyspack, builtin_p38g_package, evaluate_package};

fn new_fdm(package: &LoadedSyspack) -> Result<Fdm, Box<dyn std::error::Error>> {
    let files = package.build_jsbsim_resolver()?;
    let (aircraft_dir, model_name) = package.jsbsim_model()?;
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true))?;
    fdm.set_dt(1.0 / 120.0);
    fdm.load_model(&files, aircraft_dir, model_name)?;
    Ok(fdm)
}

/// Engine operating point at a fixed altitude, speed and throttle, printed for
/// calibration work (`cargo test -p sim-pack --test p38g_flight_test -- --ignored --nocapture`).
#[test]
#[ignore = "diagnostic probe, prints engine operating points"]
fn engine_operating_points() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_p38g_package()?;
    for &(alt_ft, ktas, throttle) in &[
        (0.0, 135.0, 0.9),
        (0.0, 135.0, 1.0),
        (5_000.0, 300.0, 0.9),
        (14_700.0, 330.0, 0.9),
        (22_600.0, 350.0, 0.97),
        (30_000.0, 345.0, 0.97),
        (30_000.0, 214.0, 1.0),
        (0.0, 185.0, 0.0),
        (0.0, 0.0, 0.0),
        (10_000.0, 250.0, 0.6),
    ] {
        let mut fdm = new_fdm(&package)?;
        for (prop, val) in [
            ("ic/h-sl-ft", alt_ft),
            ("ic/vt-fps", ktas * 1.687_81),
            ("ic/alpha-deg", 2.0),
            ("ic/theta-deg", 2.0),
            ("propulsion/magneto_cmd", 3.0),
            ("propulsion/set-running", -1.0),
        ] {
            fdm.set_property(prop, val)?;
        }
        for n in 0..2 {
            fdm.set_property(&format!("fcs/throttle-cmd-norm[{n}]"), throttle)?;
            fdm.set_property(&format!("fcs/mixture-cmd-norm[{n}]"), 1.0)?;
            fdm.set_property(&format!("fcs/advance-cmd-norm[{n}]"), 1.0)?;
        }
        fdm.run_ic()?;
        for _ in 0..600 {
            fdm.set_property("ic/vt-fps", ktas * 1.687_81)?;
            if !fdm.run()? {
                break;
            }
        }
        let mut line = format!("alt {alt_ft:>6} ktas {ktas:>5} thr {throttle:.2}:");
        for prop in [
            "propulsion/engine[0]/map-inhg",
            "propulsion/engine[0]/engine-rpm",
            "propulsion/engine[0]/power-hp",
            "propulsion/engine[0]/blade-angle",
            "propulsion/engine[0]/thrust-lbs",
            "propulsion/engine[1]/power-hp",
            "propulsion/engine[0]/fuel-flow-rate-gph",
            "fcs/mixture-pos-norm[0]",
            "propulsion/engine[0]/set-running",
        ] {
            let short = prop.rsplit('/').next().unwrap_or(prop);
            line.push_str(&format!(" {short}={:.1}", fdm.get_property(prop)?));
        }
        writeln!(std::io::stdout(), "{line}")?;
    }
    Ok(())
}

/// The whole AAF test card passes within the suite's tolerances.
#[test]
fn p38g_matches_aaf_flight_test_reports() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_p38g_package()?;
    let report = evaluate_package(&package)?;
    assert!(
        report.all_passed,
        "P-38G failed its AAF test card:\n{}",
        report.to_markdown()
    );
    Ok(())
}
