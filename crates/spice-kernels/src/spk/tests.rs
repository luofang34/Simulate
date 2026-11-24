use alloc::vec;
use alloc::vec::Vec;

use super::interpolation::{hermite, lagrange};
use super::{FRAME_J2000, SampledStates, SpkData, SpkFile, SpkSegment};
use crate::error::Error;

/// A degree-seven polynomial and its derivative.
fn septic(x: f64) -> (f64, f64) {
    let coefficients = [3.0, -1.5, 0.25, 2.0, -0.75, 0.5, 0.125, -0.0625];
    let mut value = 0.0;
    let mut derivative = 0.0;
    for &c in coefficients.iter().rev() {
        derivative = derivative * x + value;
        value = value * x + c;
    }
    (value, derivative)
}

const NODES: [f64; 8] = [-1.7, -1.1, -0.6, -0.2, 0.3, 0.9, 1.4, 1.8];

#[test]
fn lagrange_reproduces_a_polynomial_of_its_degree() {
    for x in [-1.7, -1.0, 0.0, 0.31, 1.25, 1.8] {
        let mut values: Vec<f64> = NODES.iter().map(|&node| septic(node).0).collect();
        let got = lagrange(&NODES, &mut values, x).unwrap();
        assert!((got - septic(x).0).abs() < 1e-12, "x={x} got={got}");
    }
    assert_eq!(lagrange(&NODES, &mut [0.0; 3], 0.0), None);
}

#[test]
fn hermite_reproduces_a_polynomial_of_its_degree_with_its_derivative() {
    let nodes = [-1.7, -0.6, 0.3, 1.8];
    let values: Vec<f64> = nodes.iter().map(|&node| septic(node).0).collect();
    let slopes: Vec<f64> = nodes.iter().map(|&node| septic(node).1).collect();
    for x in [-1.7, -1.0, 0.0, 0.3, 1.25, 1.8] {
        let (value, derivative) = hermite(&nodes, &values, &slopes, x).unwrap();
        let (want_value, want_derivative) = septic(x);
        assert!((value - want_value).abs() < 1e-12, "x={x} value={value}");
        assert!(
            (derivative - want_derivative).abs() < 1e-11,
            "x={x} derivative={derivative}"
        );
    }
    assert_eq!(hermite(&nodes, &values, &slopes[..2], 0.0), None);
}

fn samples(count: usize, window: usize) -> SampledStates {
    let epochs: Vec<f64> = (0..count).map(|k| 10.0 * k as f64).collect();
    let states = epochs
        .iter()
        .map(|&t| [t, 2.0 * t, -t, 1.0, 2.0, -1.0])
        .collect();
    SampledStates::new(epochs, states, window).unwrap()
}

#[test]
fn even_window_brackets_the_epoch() {
    let s = samples(10, 4);
    assert_eq!(s.window_range(45.0), 3..7);
    assert_eq!(s.window_range(41.0), 3..7);
    assert_eq!(s.window_range(49.0), 3..7);
    // On a sample the bracketing interval is the one that ends there.
    assert_eq!(s.window_range(50.0), 3..7);
    // Near either end the window is shifted inward.
    assert_eq!(s.window_range(0.0), 0..4);
    assert_eq!(s.window_range(5.0), 0..4);
    assert_eq!(s.window_range(90.0), 6..10);
    assert_eq!(s.window_range(89.0), 6..10);
}

#[test]
fn odd_window_centres_on_the_nearest_sample() {
    let s = samples(10, 3);
    assert_eq!(s.window_range(44.0), 3..6);
    assert_eq!(s.window_range(46.0), 4..7);
    // A tie goes to the later sample.
    assert_eq!(s.window_range(45.0), 4..7);
    assert_eq!(s.window_range(50.0), 4..7);
    assert_eq!(s.window_range(0.0), 0..3);
    assert_eq!(s.window_range(90.0), 7..10);
    assert_eq!(samples(10, 1).window_range(46.0), 5..6);
}

#[test]
fn stored_words_follow_the_segment_layout() {
    let s = samples(250, 6);
    let words = s.to_words();
    assert_eq!(words.len(), 250 * 6 + 250 + 2 + 2);
    assert_eq!(words[6..12], [10.0, 20.0, -10.0, 1.0, 2.0, -1.0]);
    assert_eq!(words[1500], 0.0);
    assert_eq!(words[1749], 2490.0);
    // The directory holds the hundredth and two-hundredth epochs.
    assert_eq!(words[1750..1752], [990.0, 1990.0]);
    assert_eq!(words[1752..], [5.0, 250.0]);
    assert_eq!(SampledStates::from_words(&words).unwrap().unwrap(), s);
    // Exactly a multiple of the directory spacing adds no further entry.
    assert_eq!(samples(200, 2).to_words().len(), 200 * 7 + 1 + 2);
    assert_eq!(samples(201, 2).to_words().len(), 201 * 7 + 2 + 2);
}

#[test]
fn malformed_words_are_reported_with_the_segment() {
    let mut words = samples(5, 2).to_words();
    let stored = |words: Vec<f64>| SpkSegment::from_stored("s", 1, 0, 1, [0.0, 40.0], 13, words);
    assert!(stored(words.clone()).is_ok());
    let last = words.len() - 1;
    words[last] = 6.0;
    assert!(matches!(
        stored(words.clone()),
        Err(Error::MalformedSegment { data_type: 13, .. })
    ));
    words[last] = 5.5;
    assert!(matches!(
        stored(words.clone()),
        Err(Error::MalformedSegment { .. })
    ));
    words[last] = 5.0;
    words[last - 1] = -1.0;
    assert!(matches!(
        stored(words.clone()),
        Err(Error::MalformedSegment { .. })
    ));
    words[last - 1] = 1.0;
    words[31] = words[30];
    assert!(matches!(
        stored(words),
        Err(Error::EpochsOutOfOrder { index: 1, .. })
    ));
    assert!(matches!(
        stored(vec![1.0]),
        Err(Error::MalformedSegment { .. })
    ));
}

#[test]
fn sample_and_segment_validation_mirrors_the_spice_writers() {
    let state = [0.0; 6];
    let new = SampledStates::new;
    assert!(matches!(
        new(vec![0.0, 1.0], vec![state], 1),
        Err(Error::LengthMismatch { .. })
    ));
    assert!(matches!(
        new(vec![0.0], vec![state], 2),
        Err(Error::TooFewStates { .. })
    ));
    assert!(matches!(
        new(vec![0.0], vec![state], 0),
        Err(Error::TooFewStates { .. })
    ));
    assert!(matches!(
        new(vec![0.0, 0.0], vec![state; 2], 2),
        Err(Error::EpochsOutOfOrder { index: 1, .. })
    ));
    assert!(matches!(
        new(vec![0.0, f64::NAN], vec![state; 2], 2),
        Err(Error::NonFinite {
            what: "epoch",
            index: 1
        })
    ));
    let mut bad = [state; 2];
    bad[1][4] = f64::INFINITY;
    assert!(matches!(
        new(vec![0.0, 1.0], bad.to_vec(), 2),
        Err(Error::NonFinite {
            what: "state",
            index: 1
        })
    ));

    let lagrange = |window| SpkSegment::lagrange("s", 1, 0, FRAME_J2000, samples(40, window));
    let hermite = |window| SpkSegment::hermite("s", 1, 0, FRAME_J2000, samples(40, window));
    assert!(matches!(
        lagrange(1),
        Err(Error::InvalidWindow { data_type: 9, .. })
    ));
    assert!(lagrange(28).is_ok());
    assert!(matches!(lagrange(29), Err(Error::InvalidWindow { .. })));
    assert!(hermite(1).is_ok());
    assert!(hermite(14).is_ok());
    assert!(matches!(
        hermite(15),
        Err(Error::InvalidWindow { data_type: 13, .. })
    ));
    // One sample cannot span a non-empty coverage interval.
    assert!(matches!(
        SpkSegment::hermite("s", 1, 0, FRAME_J2000, samples(1, 1)),
        Err(Error::InvalidCoverage { .. })
    ));
    assert!(matches!(
        SpkSegment::hermite(&"n".repeat(41), 1, 0, FRAME_J2000, samples(4, 2)),
        Err(Error::TextTooLong { max: 40, .. })
    ));
    for coverage in [
        [-1.0, 20.0],
        [10.0, 400.0],
        [20.0, 20.0],
        [30.0, 20.0],
        [f64::NAN, 20.0],
    ] {
        assert!(matches!(
            hermite(2).unwrap().with_coverage(coverage),
            Err(Error::InvalidCoverage { .. })
        ));
    }
}

#[test]
fn evaluation_respects_coverage_and_precedence() {
    let early = SpkSegment::hermite("early", 5, 0, FRAME_J2000, samples(10, 2)).unwrap();
    let narrowed = early.clone().with_coverage([20.0, 60.0]).unwrap();
    assert!(matches!(
        narrowed.state(10.0),
        Err(Error::OutOfCoverage { .. })
    ));
    assert!(matches!(
        narrowed.state(f64::NAN),
        Err(Error::OutOfCoverage { .. })
    ));
    assert_eq!(
        narrowed.state(20.0).unwrap(),
        [20.0, 40.0, -20.0, 1.0, 2.0, -1.0]
    );

    // A later segment for the same pair, offset so the two can be told apart.
    let epochs: Vec<f64> = (3..7).map(|k| 10.0 * f64::from(k)).collect();
    let states = epochs
        .iter()
        .map(|&t| [t + 1000.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        .collect();
    let late = SampledStates::new(epochs, states, 2).unwrap();
    let late = SpkSegment::lagrange("late", 5, 0, FRAME_J2000, late).unwrap();

    let mut file = SpkFile::new("precedence").unwrap();
    file.push(early);
    file.push(late);
    assert_eq!(file.state(5, 0, 25.0).unwrap()[0], 25.0);
    assert!((file.state(5, 0, 45.0).unwrap()[0] - 1045.0).abs() < 1e-12);
    assert_eq!(file.segment_for(5, 0, 45.0).unwrap().name(), "late");
    assert!(matches!(
        file.state(5, 3, 45.0),
        Err(Error::NoCoverage {
            target: 5,
            center: 3,
            ..
        })
    ));
    assert!(matches!(
        file.state(5, 0, 500.0),
        Err(Error::NoCoverage { .. })
    ));
}

#[test]
fn unmodelled_types_are_kept_verbatim_and_not_evaluated() {
    let words = vec![1.0, 2.0, 3.0, 4.0];
    let raw = SpkSegment::from_stored("raw", 3, 0, 1, [0.0, 1.0], 2, words.clone()).unwrap();
    assert_eq!(
        raw.data(),
        &SpkData::Raw {
            data_type: 2,
            words
        }
    );
    assert_eq!(raw.data().data_type(), 2);
    assert!(matches!(
        raw.state(0.5),
        Err(Error::UnsupportedDataType { data_type: 2 })
    ));
    assert!(matches!(
        SpkSegment::from_stored("raw", 3, 0, 1, [0.0, 1.0], 2, vec![]),
        Err(Error::MalformedSegment { .. })
    ));
}

#[test]
fn only_spk_shaped_dafs_are_accepted() {
    let pck = crate::Daf::new("PCK", 2, 5, "").unwrap();
    assert!(matches!(
        SpkFile::from_daf(&pck),
        Err(Error::WrongKernelType { .. })
    ));
    let odd = crate::Daf::new("SPK", 3, 6, "").unwrap();
    assert!(matches!(
        SpkFile::from_daf(&odd),
        Err(Error::InvalidSummaryFormat { nd: 3, ni: 6 })
    ));
}
