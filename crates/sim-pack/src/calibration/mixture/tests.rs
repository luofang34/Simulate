use super::{MixtureProcedure, Sample, best_power, rich_of_peak};

fn curve() -> Vec<Sample> {
    // Power peaks at 0.80 and holds to 0.78; EGT peaks at 0.70.
    [
        (1.00, 170.0, 1250.0),
        (0.90, 176.0, 1300.0),
        (0.80, 180.0, 1360.0),
        (0.78, 180.0, 1375.0),
        (0.75, 179.0, 1400.0),
        (0.70, 174.0, 1420.0),
        (0.65, 165.0, 1405.0),
    ]
    .into_iter()
    .map(|(lever, power_hp, egt_degf)| Sample {
        lever,
        power_hp,
        egt_degf,
    })
    .collect()
}

#[test]
fn best_power_stops_at_the_richest_peak() {
    assert!((best_power(&curve()) - 0.80).abs() < 1e-9);
}

#[test]
fn rich_of_peak_takes_the_leanest_setting_that_is_cool_enough() {
    // Peak 1420°F at 0.70; 50°F rich means EGT <= 1370°F on the rich side.
    assert!((rich_of_peak(&curve(), 50.0) - 0.80).abs() < 1e-9);
}

#[test]
fn a_flat_sweep_keeps_full_rich() {
    let flat: Vec<Sample> = (0..5)
        .map(|i| Sample {
            lever: 1.0 - 0.1 * f64::from(i),
            power_hp: 100.0,
            egt_degf: 1300.0,
        })
        .collect();
    assert!((best_power(&flat) - 1.0).abs() < 1e-9);
    assert!((rich_of_peak(&flat, 50.0) - 1.0).abs() < 1e-9);
}

#[test]
fn the_poh_climb_rule_leans_above_3000_ft() {
    assert_eq!(
        MixtureProcedure::climb(2_000.0, false),
        MixtureProcedure::FullRich
    );
    assert_eq!(
        MixtureProcedure::climb(4_000.0, false),
        MixtureProcedure::BestPower
    );
    assert_eq!(
        MixtureProcedure::climb(0.0, true),
        MixtureProcedure::BestPower
    );
}
