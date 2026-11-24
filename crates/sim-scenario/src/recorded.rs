//! Comparison of recorded values that reports every one that left its
//! tolerance.

/// Fails with every `(name, now, recorded, tolerance)` row that left its
/// tolerance, each printed in full.
pub(crate) fn assert_rows(rows: &[(&str, f64, f64, f64)]) {
    let off: Vec<String> = rows
        .iter()
        .filter(|(_, now, recorded, tolerance)| {
            let error = (now - recorded).abs();
            error.is_nan() || error > *tolerance
        })
        .map(|(name, now, recorded, tolerance)| {
            format!("{name}: {now:?} against {recorded:?} (tolerance {tolerance:e})")
        })
        .collect();
    assert!(off.is_empty(), "\n{}", off.join("\n"));
}
