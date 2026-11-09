//! Diagnostic probes for `vehicles/g4m1`, for calibration work
//! (`cargo test --release -p sim-pack --test g4m1_probes -- --ignored --nocapture`).

use std::io::Write as _;

use jsbsim::{Fdm, LoadOptions};
use sim_pack::{
    CalibrationSuite, LoadedSyspack, builtin_g4m1_kasei15_package, builtin_g4m1_package,
    evaluate_package_against,
};

fn new_fdm(package: &LoadedSyspack) -> Result<Fdm, Box<dyn std::error::Error>> {
    let files = package.build_jsbsim_resolver()?;
    let (aircraft_dir, model_name) = package.jsbsim_model()?;
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true))?;
    fdm.set_dt(1.0 / 120.0);
    fdm.load_model(&files, aircraft_dir, model_name)?;
    Ok(fdm)
}

/// Propeller lever for a governed crankshaft speed (the FDM maps 0..1 to 1,600..2,450 rpm).
fn lever_for(rpm: f64) -> f64 {
    (rpm - 1_600.0) / 850.0
}

/// Engine 0's output at each engine part's rating points (altitude, rpm and
/// manifold pressure from the part's engine.json), with the brake-specific fuel
/// consumption the ace-rs catalog entry carries.
#[test]
#[ignore = "diagnostic probe, prints engine operating points"]
fn engine_operating_points() -> Result<(), Box<dyn std::error::Error>> {
    // (altitude ft, KTAS, rpm, MAP inHg, rating hp)
    let kasei11 = [
        (0.0, 120.0, 2_450.0, 39.76, 1_505.0),
        (0.0, 120.0, 2_350.0, 37.0, 1_340.0),
        (7_218.0, 120.0, 2_350.0, 37.0, 1_460.0),
        (13_451.0, 120.0, 2_350.0, 37.0, 1_360.0),
    ];
    let kasei15 = [
        (0.0, 120.0, 2_450.0, 39.76, 1_440.0),
        (8_530.0, 120.0, 2_350.0, 37.0, 1_400.0),
        (19_685.0, 120.0, 2_350.0, 37.0, 1_280.0),
    ];
    for (package, rows) in [
        (builtin_g4m1_package()?, &kasei11[..]),
        (builtin_g4m1_kasei15_package()?, &kasei15[..]),
    ] {
        writeln!(std::io::stdout(), "{}", package.manifest().name)?;
        for &row in rows {
            writeln!(std::io::stdout(), "{}", probe_engine(&package, row)?)?;
        }
    }
    Ok(())
}

/// Engine 0's rated-rpm power from sea level to 9,000 m at 37.0 inHg or full
/// throttle above the full-throttle height, with the supercharger speed the
/// engine part selected: the change altitude is where second speed overtakes
/// first (regenerate the part with the change altitude at 0 m and at 99,999 m to
/// see each speed alone).
#[test]
#[ignore = "diagnostic probe, prints power against altitude"]
fn rated_power_by_altitude() -> Result<(), Box<dyn std::error::Error>> {
    for package in [builtin_g4m1_package()?, builtin_g4m1_kasei15_package()?] {
        writeln!(std::io::stdout(), "{}", package.manifest().name)?;
        for metres in (0..=36).map(|k| f64::from(k) * 250.0) {
            let line = probe_engine(&package, (metres / 0.3048, 140.0, 2_350.0, 37.0, 1.0))?;
            writeln!(std::io::stdout(), "{metres:>6.0} m {line}")?;
        }
    }
    Ok(())
}

/// Both variants side by side at the card's weight and power: level speed at
/// rated power and steady climb rate on the card's climb schedule, by altitude.
/// The Kasei 11 column is the calibrated reference; the Kasei 15 column is the
/// same airframe with the other engine part, a model prediction checked against
/// aircraft No. 241 only where the Kasei 15 card has points.
#[test]
#[ignore = "diagnostic probe, prints both variants' performance"]
fn variant_predictions() -> Result<(), Box<dyn std::error::Error>> {
    let reference = builtin_g4m1_package()?;
    let card = reference
        .calibration_suite()
        .ok_or("the G4M1 ships a card")?;
    let (Some(cruise), Some(climb), Some(timed)) = (
        card.cruise_points.first().copied(),
        card.climb_points.first().copied(),
        card.climb_time_points.first().cloned(),
    ) else {
        return Err("the G4M1 card has cruise, climb and timed points".into());
    };
    let mut suite: CalibrationSuite = card.clone();
    suite.climb_time_points.clear();
    suite.cruise_points.clear();
    suite.climb_points.clear();
    let altitudes: Vec<f64> = (0..=9).map(|km| f64::from(km) * 1_000.0 / 0.3048).collect();
    for &alt_ft in &altitudes {
        let mut level = cruise;
        level.pressure_alt_ft = alt_ft;
        level.target_ktas = 230.0;
        suite.cruise_points.push(level);
        let mut rate = climb;
        rate.pressure_alt_ft = alt_ft;
        rate.climb_kcas = (timed.climb_kcas + timed.climb_kcas_per_1000ft * alt_ft / 1_000.0)
            .max(climb.climb_kcas);
        suite.climb_points.push(rate);
    }
    let mut out = std::io::stdout().lock();
    writeln!(
        out,
        "| Alt (m) | K11 KTAS | K15 KTAS | K11 ROC (fpm) | K15 ROC (fpm) |"
    )?;
    writeln!(out, "|---:|---:|---:|---:|---:|")?;
    let k11 = evaluate_package_against(&reference, &suite)?;
    let k15 = evaluate_package_against(&builtin_g4m1_kasei15_package()?, &suite)?;
    for (i, &alt_ft) in altitudes.iter().enumerate() {
        let (Some(a), Some(b), Some(c), Some(d)) = (
            k11.cruises.get(i),
            k15.cruises.get(i),
            k11.climbs.get(i),
            k15.climbs.get(i),
        ) else {
            break;
        };
        writeln!(
            out,
            "| {:.0} | {:.1} | {:.1} | {:.0} | {:.0} |",
            alt_ft * 0.3048,
            a.sim_ktas,
            b.sim_ktas,
            c.sim_fpm,
            d.sim_fpm
        )?;
    }
    Ok(())
}

fn probe_engine(
    package: &LoadedSyspack,
    (alt_ft, ktas, rpm, map, rating): (f64, f64, f64, f64, f64),
) -> Result<String, Box<dyn std::error::Error>> {
    let mut fdm = new_fdm(package)?;
    for (prop, val) in [
        ("ic/h-sl-ft", alt_ft),
        ("ic/terrain-elevation-ft", -50.0_f64),
        ("ic/vt-fps", ktas * 1.687_81),
        ("ic/alpha-deg", 2.0),
        ("ic/theta-deg", 2.0),
        ("propulsion/magneto_cmd", 3.0),
        ("propulsion/set-running", -1.0),
    ] {
        fdm.set_property(prop, val)?;
    }
    let mut throttle = 0.8;
    for n in 0..2 {
        fdm.set_property(&format!("fcs/throttle-cmd-norm[{n}]"), throttle)?;
        fdm.set_property(&format!("fcs/mixture-cmd-norm[{n}]"), 1.0)?;
        fdm.set_property(&format!("fcs/advance-cmd-norm[{n}]"), lever_for(rpm))?;
    }
    fdm.run_ic()?;
    for _ in 0..1_200 {
        let err = map - fdm.get_property("propulsion/engine[0]/map-inhg")?;
        throttle = (throttle + 0.0005 * err).clamp(0.0, 1.0);
        for n in 0..2 {
            fdm.set_property(&format!("fcs/throttle-cmd-norm[{n}]"), throttle)?;
        }
        if !fdm.run()? {
            break;
        }
    }
    let hp = fdm.get_property("propulsion/engine[0]/power-hp")?;
    let pps = fdm.get_property("propulsion/engine[0]/fuel-flow-rate-pps")?;
    let bsfc = pps * 3_600.0 * 0.453_592 / (hp * 0.745_7);
    let mut line = format!(
        "alt {alt_ft:>6} rpm {rpm:.0} map {map:.2}: {hp:.0} hp vs {rating:.0} ({:+.1}%), bsfc {bsfc:.3} kg/kWh",
        100.0 * (hp - rating) / rating
    );
    for prop in [
        "propulsion/engine[0]/map-inhg",
        "propulsion/engine[0]/engine-rpm",
        "propulsion/engine[0]/boost-speed",
        "propulsion/engine[0]/boostloss-hp",
        "propulsion/engine[0]/carb-air-temp-degC",
    ] {
        let short = prop.rsplit('/').next().unwrap_or(prop);
        line.push_str(&format!(" {short}={:.2}", fdm.get_property(prop)?));
    }
    Ok(line)
}

/// Rate of climb across speed and altitude, and time to climb across constant
/// speeds, at the card's climb power and weight: the Navy report gives no climb
/// speed, so this is what the card's fitted climb schedule is chosen against.
#[test]
#[ignore = "diagnostic probe, sweeps climb KCAS"]
fn best_climb_speed_sweep() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_g4m1_package()?;
    let Some(card) = package.calibration_suite() else {
        return Err("the G4M1 ships no test card".into());
    };
    let Some(template) = card.climb_points.first().copied() else {
        return Err("the G4M1 card has no climb point".into());
    };
    let mut suite: CalibrationSuite = card.clone();
    suite.cruise_points.clear();
    suite.climb_time_points.clear();
    suite.climb_points.clear();
    for alt_ft in [0.0, 9_843.0, 18_000.0, 26_247.0, 31_234.0] {
        for kcas in [85.0, 95.0, 105.0, 115.0, 125.0, 135.0] {
            let mut point = template;
            point.pressure_alt_ft = alt_ft;
            point.climb_kcas = kcas;
            suite.climb_points.push(point);
        }
    }
    let Some(timed) = card.climb_time_points.first().cloned() else {
        return Err("the G4M1 card has no time-to-climb point".into());
    };
    for (start_alt_ft, end_alt_ft) in [(0.0, 9_843.0), (0.0, 26_247.0), (26_247.0, 31_234.0)] {
        for kcas in [95.0, 100.0, 105.0, 110.0, 115.0, 120.0] {
            let mut point = timed.clone();
            point.label = format!("{start_alt_ft:.0}-{end_alt_ft:.0} ft at {kcas:.0} KCAS");
            point.start_alt_ft = start_alt_ft;
            point.end_alt_ft = end_alt_ft;
            point.climb_kcas = kcas;
            suite.climb_time_points.push(point);
        }
    }
    let report = evaluate_package_against(&package, &suite)?;
    for c in &report.climbs {
        writeln!(
            std::io::stdout(),
            "alt {:>6.0} kcas {:>5.1} rpm {:>5.0} roc {:>6.0}",
            c.pressure_alt_ft,
            c.sim_kcas,
            c.sim_rpm,
            c.sim_fpm
        )?;
    }
    for c in &report.climb_times {
        writeln!(std::io::stdout(), "{}: {:.2} min", c.label, c.sim_min)?;
    }
    Ok(())
}
