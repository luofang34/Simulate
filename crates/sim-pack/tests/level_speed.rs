//! A level-speed point flies until its speed settles, so the result is the
//! aircraft's, not the speed the harness started it at.

use sim_pack::{builtin_p38g_package, evaluate_package_against};

#[test]
fn a_level_speed_does_not_depend_on_where_it_started() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_p38g_package()?;
    let card = package
        .calibration_suite()
        .ok_or("the P-38G ships a card")?;
    let point = *card
        .cruise_points
        .first()
        .ok_or("the P-38G card has cruises")?;
    let mut suite = card.clone();
    suite.stalls.clear();
    suite.climb_points.clear();
    suite.cruise_points = [-40.0, 0.0, 40.0]
        .iter()
        .map(|offset| {
            let mut seeded = point;
            seeded.target_ktas += offset;
            seeded
        })
        .collect();
    let report = evaluate_package_against(&package, &suite)?;
    let speeds: Vec<f64> = report.cruises.iter().map(|c| c.sim_ktas).collect();
    let spread = speeds.iter().copied().fold(f64::MIN, f64::max)
        - speeds.iter().copied().fold(f64::MAX, f64::min);
    assert!(spread < 1.0, "{speeds:?}");
    Ok(())
}
