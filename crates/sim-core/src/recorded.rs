//! Recorded values of fixed scenarios and the comparison that reports every
//! one that left its tolerance.

use glam::DVec3;

/// One recorded quantity: name, value now, recorded value, tolerance.
pub(crate) type Row = (&'static str, f64, f64, f64);

/// Fails with every row that left its tolerance, each printed in full.
pub(crate) fn assert_rows(rows: &[Row]) {
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

/// Three rows for the components of `now`.
pub(crate) fn vec_rows(
    name: [&'static str; 3],
    now: DVec3,
    recorded: [f64; 3],
    tol: f64,
) -> [Row; 3] {
    [
        (name[0], now.x, recorded[0], tol),
        (name[1], now.y, recorded[1], tol),
        (name[2], now.z, recorded[2], tol),
    ]
}
