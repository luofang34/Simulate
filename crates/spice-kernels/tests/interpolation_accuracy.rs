//! How closely type 9 and type 13 segments reproduce an analytic Kepler
//! orbit between samples, measured through a written and re-read kernel.

#[path = "support/kepler.rs"]
mod kepler;

use spice_kernels::{Error, FRAME_J2000, SampledStates, SpkFile, SpkSegment};

const SAMPLES: usize = 195;

/// Largest position (km) and velocity (km/s) error of the kernel's segment
/// over its coverage, probed at 37 points in every sampling interval.
fn worst_error(
    orbit: &kepler::Orbit,
    steps: &[f64],
    build: impl Fn(SampledStates) -> Result<SpkSegment, Error>,
    window: usize,
) -> Result<(f64, f64), Error> {
    // Two revolutions at the 60 s mean step: the period is 5829 s.
    let (epochs, states) = orbit.sample(orbit.epoch + 123.0, steps, SAMPLES);
    let mut file = SpkFile::new("accuracy")?;
    file.push(build(SampledStates::new(epochs.clone(), states, window)?)?);
    let read = SpkFile::parse(&file.to_bytes()?)?;

    let mut worst = (0.0_f64, 0.0_f64);
    for pair in epochs.windows(2) {
        let (Some(&from), Some(&to)) = (pair.first(), pair.last()) else {
            continue;
        };
        for k in 0..37 {
            let et = from + (to - from) * f64::from(k) / 37.0;
            let (position, velocity) =
                kepler::difference(&read.state(-5, 399, et)?, &orbit.state(et));
            worst = (worst.0.max(position), worst.1.max(velocity));
        }
    }
    Ok(worst)
}

fn lagrange(samples: SampledStates) -> Result<SpkSegment, Error> {
    SpkSegment::lagrange("lagrange", -5, 399, FRAME_J2000, samples)
}

fn hermite(samples: SampledStates) -> Result<SpkSegment, Error> {
    SpkSegment::hermite("hermite", -5, 399, FRAME_J2000, samples)
}

/// Steps averaging 60 s but unequal, which is what types 9 and 13 are for.
const UNEQUAL_MINUTE: [f64; 3] = [45.0, 60.0, 75.0];
const UNEQUAL_HALF_MINUTE: [f64; 3] = [22.5, 30.0, 37.5];

#[test]
fn hermite_degree_seven_holds_a_low_orbit_to_two_micrometres() -> Result<(), Error> {
    let orbit = kepler::low_earth_orbit();
    let (position, velocity) = worst_error(&orbit, &UNEQUAL_MINUTE, hermite, 4)?;
    // Measured: 7.0e-10 km and 3.0e-11 km/s.
    assert!(position < 2.0e-9, "position error {position} km");
    assert!(velocity < 1.0e-10, "velocity error {velocity} km/s");
    Ok(())
}

#[test]
fn lagrange_degree_seven_holds_a_low_orbit_to_a_millimetre() -> Result<(), Error> {
    let orbit = kepler::low_earth_orbit();
    let (position, velocity) = worst_error(&orbit, &UNEQUAL_MINUTE, lagrange, 8)?;
    // Measured: 3.4e-7 km and 1.0e-9 km/s, both in the end intervals where
    // the window cannot be centred.
    assert!(position < 1.0e-6, "position error {position} km");
    assert!(velocity < 3.0e-9, "velocity error {velocity} km/s");
    Ok(())
}

#[test]
fn halving_the_step_shrinks_the_error_at_the_expected_order() -> Result<(), Error> {
    let orbit = kepler::low_earth_orbit();
    // Low orders, so that truncation error stays far above rounding error at
    // both step sizes: a cubic (order four) for each type.
    let (coarse_l, _) = worst_error(&orbit, &UNEQUAL_MINUTE, lagrange, 4)?;
    let (fine_l, _) = worst_error(&orbit, &UNEQUAL_HALF_MINUTE, lagrange, 4)?;
    let (coarse_h, _) = worst_error(&orbit, &UNEQUAL_MINUTE, hermite, 2)?;
    let (fine_h, _) = worst_error(&orbit, &UNEQUAL_HALF_MINUTE, hermite, 2)?;
    // Measured ratios: 16.0 for both.
    for (coarse, fine) in [(coarse_l, fine_l), (coarse_h, fine_h)] {
        let ratio = coarse / fine;
        assert!((12.0..20.0).contains(&ratio), "order-four ratio {ratio}");
    }
    Ok(())
}
