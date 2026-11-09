use super::*;

#[test]
fn a_constant_rate_climb_takes_band_over_rate() {
    let rungs = [0.0, 1_000.0, 2_000.0].map(|energy_ft| Rung {
        energy_ft,
        rate_fpm: 500.0,
    });
    assert!((integrate_minutes(&rungs) - 4.0).abs() < 1e-12);
}

#[test]
fn a_rate_falling_linearly_integrates_to_the_harmonic_mean() {
    // 1,000 fpm falling to 500 fpm over 1,000 ft: the trapezoid on 1/rate.
    let rungs = [
        Rung {
            energy_ft: 0.0,
            rate_fpm: 1_000.0,
        },
        Rung {
            energy_ft: 1_000.0,
            rate_fpm: 500.0,
        },
    ];
    assert!((integrate_minutes(&rungs) - 1.5).abs() < 1e-12);
}

#[test]
fn a_rung_that_cannot_climb_never_arrives() {
    let rungs = [
        Rung {
            energy_ft: 0.0,
            rate_fpm: 300.0,
        },
        Rung {
            energy_ft: 1_000.0,
            rate_fpm: -20.0,
        },
    ];
    assert!(integrate_minutes(&rungs).is_infinite());
}
