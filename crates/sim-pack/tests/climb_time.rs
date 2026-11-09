//! Time-to-climb and rated-RPM climbs through the shared calibration harness,
//! flown on the P-38G: a constant-speed twin whose card states climb rates, so
//! the timed ladder can be checked against the steady climbs it is built from.

use sim_pack::{
    BrakeReleaseStart, CalibrationSuite, ClimbTimeBenchmark, builtin_p38g_package,
    evaluate_package_against,
};

const FPS_PER_KT: f64 = 1.687_809_857_1;
const G_FPS2: f64 = 32.174_049;

fn suite_with(card: &CalibrationSuite, climb_times: Vec<ClimbTimeBenchmark>) -> CalibrationSuite {
    let mut suite = card.clone();
    suite.stalls.clear();
    suite.cruise_points.clear();
    suite.climb_time_points = climb_times;
    suite
}

fn timed(card: &CalibrationSuite, end_alt_ft: f64) -> Result<ClimbTimeBenchmark, String> {
    let first = card
        .climb_points
        .first()
        .ok_or("the P-38G card has climbs")?;
    Ok(ClimbTimeBenchmark {
        label: format!("0 to {end_alt_ft} ft"),
        gross_mass_kg: first.gross_mass_kg,
        start_alt_ft: 0.0,
        end_alt_ft,
        climb_kcas: first.climb_kcas,
        climb_kcas_per_1000ft: 0.0,
        target_map_inhg: first.target_map_inhg,
        target_rpm: None,
        target_min: 1.0,
        from_brake_release: None,
    })
}

/// A ladder over one 10,000 ft band takes the energy climbed over the mean of
/// the band-end climb rates (the trapezoid on 1/rate with rates this close),
/// using the same steady climbs the rate card flies.
#[test]
fn a_timed_climb_agrees_with_the_steady_climbs_it_spans() -> Result<(), Box<dyn std::error::Error>>
{
    let package = builtin_p38g_package()?;
    let card = package
        .calibration_suite()
        .ok_or("the P-38G ships a card")?;
    let band = timed(card, 10_000.0)?;
    let mut suite = suite_with(card, vec![band.clone()]);
    suite.climb_points.truncate(2);
    let report = evaluate_package_against(&package, &suite)?;
    let [low, high] = report.climbs.as_slice() else {
        return Err("two rate points".into());
    };
    let [time] = report.climb_times.as_slice() else {
        return Err("one timed point".into());
    };
    let energy = |kcas: f64, alt_ft: f64| {
        let sigma = (1.0 - 6.875_585_6e-6 * alt_ft).powf(4.255_879_7);
        let v_fps = kcas / sigma.sqrt() * FPS_PER_KT;
        alt_ft + v_fps * v_fps / (2.0 * G_FPS2)
    };
    let gained = energy(band.climb_kcas, 10_000.0) - energy(band.climb_kcas, 0.0);
    let bracket = gained * 0.5 * (1.0 / low.sim_fpm + 1.0 / high.sim_fpm);
    assert!(
        (time.sim_min - bracket).abs() < 0.05 * bracket,
        "{} min against {bracket} min from the band-end rates",
        time.sim_min
    );
    Ok(())
}

/// Starting the clock at brake release adds the take-off, and holding a lower
/// governed RPM costs climb.
#[test]
fn brake_release_and_rated_rpm_both_slow_the_climb() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_p38g_package()?;
    let card = package
        .calibration_suite()
        .ok_or("the P-38G ships a card")?;
    let airborne = timed(card, 5_000.0)?;
    let from_brakes = ClimbTimeBenchmark {
        from_brake_release: Some(BrakeReleaseStart {
            flap_norm: 1.0 / 3.0,
            lift_off_kcas: 100.0,
            obstacle_kcas: 110.0,
        }),
        ..airborne.clone()
    };
    let rated = ClimbTimeBenchmark {
        target_rpm: Some(2_600.0),
        ..airborne.clone()
    };
    let mut suite = suite_with(card, vec![airborne, from_brakes, rated]);
    suite.climb_points.clear();
    let report = evaluate_package_against(&package, &suite)?;
    let [airborne, from_brakes, rated] = report.climb_times.as_slice() else {
        return Err("three timed points".into());
    };
    let take_off_min = from_brakes.sim_min - airborne.sim_min;
    assert!(
        (0.1..1.5).contains(&take_off_min),
        "take-off adds {take_off_min} min"
    );
    assert!(
        rated.sim_min > airborne.sim_min * 1.03,
        "{rated:?} vs {airborne:?}"
    );
    Ok(())
}
