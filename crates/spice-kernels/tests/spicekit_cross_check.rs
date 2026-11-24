//! An independent reader, `spicekit`, opens the kernels this crate writes
//! and evaluates the same states as the crate's own evaluator.

#[path = "support/kepler.rs"]
mod kepler;

use spice_kernels::{FRAME_J2000, SampledStates, SpkFile, SpkSegment};

type TestResult<T = ()> = Result<T, Box<dyn std::error::Error>>;

/// Agreement required between the two evaluators, relative to the length of
/// the position and of the velocity.
const RELATIVE_TOLERANCE: f64 = 1.0e-12;

const TARGET: i32 = -42;
const CENTER: i32 = 399;

#[derive(Clone, Copy)]
enum Kind {
    Lagrange,
    Hermite,
}

fn kernel(kind: Kind, window: usize, count: usize) -> TestResult<(SpkFile, Vec<f64>)> {
    let orbit = kepler::low_earth_orbit();
    let (epochs, states) = orbit.sample(orbit.epoch + 7.0, &[40.0, 65.0, 55.0], count);
    let samples = SampledStates::new(epochs.clone(), states, window)?;
    let segment = match kind {
        Kind::Lagrange => {
            SpkSegment::lagrange("cross-check", TARGET, CENTER, FRAME_J2000, samples)?
        }
        Kind::Hermite => SpkSegment::hermite("cross-check", TARGET, CENTER, FRAME_J2000, samples)?,
    };
    let mut file = SpkFile::new("spicekit cross-check")?;
    file.push(segment);
    Ok((file, epochs))
}

/// Probe epochs: each of `fractions` of the way through every sampling
/// interval, and the samples themselves when `on_samples`.
fn probes(epochs: &[f64], fractions: &[f64], on_samples: bool) -> Vec<f64> {
    let mut probes = if on_samples {
        epochs.to_vec()
    } else {
        Vec::new()
    };
    for pair in epochs.windows(2) {
        if let [from, to] = *pair {
            probes.extend(
                fractions
                    .iter()
                    .map(|fraction| from + (to - from) * fraction),
            );
        }
    }
    probes
}

/// Position and velocity disagreement of two states, each relative to the
/// length of the first state's position or velocity.
fn relative_difference(ours: &[f64; 6], theirs: &[f64; 6]) -> (f64, f64) {
    let (position, velocity) = kepler::difference(ours, theirs);
    let (radius, speed) = kepler::difference(ours, &[0.0; 6]);
    (position / radius, velocity / speed)
}

/// Writes `file`, opens it with spicekit, and returns the worst relative
/// position and velocity disagreement over `probes`.
fn worst_disagreement(file: &SpkFile, probes: &[f64]) -> TestResult<(f64, f64)> {
    let directory = tempfile::tempdir()?;
    let path = directory.path().join("cross_check.bsp");
    file.write_file_blocking(&path)?;
    let other = spicekit::SpkFile::open(&path)?;
    let mut worst = (0.0_f64, 0.0_f64);
    for &et in probes {
        let ours = file.state(TARGET, CENTER, et)?;
        let theirs = other.state(TARGET, CENTER, et)?;
        let (position, velocity) = relative_difference(&ours, &theirs);
        worst = (worst.0.max(position), worst.1.max(velocity));
    }
    Ok(worst)
}

fn assert_agreement(case: &str, (position, velocity): (f64, f64)) {
    assert!(
        position <= RELATIVE_TOLERANCE,
        "{case}: position {position:e}"
    );
    assert!(
        velocity <= RELATIVE_TOLERANCE,
        "{case}: velocity {velocity:e}"
    );
}

const WHOLE_INTERVAL: [f64; 6] = [0.07, 0.25, 0.49, 0.51, 0.75, 0.93];
/// Where an odd window's nearest sample is the earlier one.
const FIRST_HALF: [f64; 3] = [0.07, 0.25, 0.49];

#[test]
fn spicekit_agrees_on_type_9_with_even_windows() -> TestResult {
    for window in [2, 4, 8, 12] {
        let (file, epochs) = kernel(Kind::Lagrange, window, 260)?;
        let worst = worst_disagreement(&file, &probes(&epochs, &WHOLE_INTERVAL, true))?;
        assert_agreement(&format!("type 9 window {window}"), worst);
    }
    Ok(())
}

#[test]
fn spicekit_agrees_on_type_13_with_even_windows() -> TestResult {
    for window in [2, 4, 6] {
        let (file, epochs) = kernel(Kind::Hermite, window, 260)?;
        let worst = worst_disagreement(&file, &probes(&epochs, &WHOLE_INTERVAL, true))?;
        assert_agreement(&format!("type 13 window {window}"), worst);
    }
    Ok(())
}

/// spicekit centres an odd window on the earlier bracketing sample; SPICE,
/// and this crate, centre it on the nearest. The two pick the same samples
/// only in the first half of an interval, so that is where they are compared;
/// a probe on a sample is left out because there the nearest sample is the
/// later of the bracketing pair.
#[test]
fn spicekit_agrees_on_odd_windows_where_both_pick_the_same_samples() -> TestResult {
    let cases = [
        (Kind::Lagrange, 3),
        (Kind::Lagrange, 7),
        (Kind::Hermite, 3),
        (Kind::Hermite, 5),
    ];
    for (kind, window) in cases {
        let (file, epochs) = kernel(kind, window, 120)?;
        let worst = worst_disagreement(&file, &probes(&epochs, &FIRST_HALF, false))?;
        assert_agreement(&format!("odd window {window}"), worst);
    }
    Ok(())
}

#[test]
fn spicekit_reads_every_segment_of_a_kernel_with_chained_summary_records() -> TestResult {
    let orbit = kepler::low_earth_orbit();
    let mut file = SpkFile::new("thirty segments")?;
    file.set_comment_lines(vec!["Two summary records.".into()])?;
    for index in 0..30_i32 {
        let count = 30 + 9 * usize::try_from(index)?;
        let (epochs, states) = orbit.sample(orbit.epoch, &[50.0, 70.0], count);
        let samples = SampledStates::new(epochs, states, 4)?;
        let (name, target) = (format!("seg {index}"), -200 - index);
        file.push(if index % 2 == 0 {
            SpkSegment::lagrange(&name, target, CENTER, FRAME_J2000, samples)?
        } else {
            SpkSegment::hermite(&name, target, CENTER, FRAME_J2000, samples)?
        });
    }
    let directory = tempfile::tempdir()?;
    let path = directory.path().join("thirty.bsp");
    file.write_file_blocking(&path)?;
    let other = spicekit::SpkFile::open(&path)?;

    assert_eq!(other.segments().len(), 30);
    for (ours, theirs) in file.segments().iter().zip(other.segments()) {
        assert_eq!(ours.name(), theirs.name);
        assert_eq!(
            (ours.target(), ours.center(), ours.frame()),
            (theirs.target, theirs.center, theirs.frame)
        );
        assert_eq!(ours.data().data_type(), theirs.data_type);
        assert_eq!(ours.coverage(), [theirs.start_et, theirs.end_et]);
        let et = theirs.start_et + 0.37 * (theirs.end_et - theirs.start_et);
        let theirs_state = other.state(ours.target(), CENTER, et)?;
        assert_agreement(
            ours.name(),
            relative_difference(&ours.state(et)?, &theirs_state),
        );
    }
    Ok(())
}
