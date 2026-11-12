//! The C172S's validation record, `vehicles/c172s/performance/validation.json`:
//! which card rows and which named checks are known to fail, and why.

/// The record, embedded so the tests grade the same file the package ships.
pub const RECORD: &str = include_str!("../../../../vehicles/c172s/performance/validation.json");

/// The names listed under `key`, sorted.
#[cfg(test)]
pub fn listed(key: &str) -> Vec<String> {
    let record: serde_json::Value = serde_json::from_str(RECORD).unwrap_or_default();
    let mut names: Vec<String> = record
        .get(key)
        .and_then(serde_json::Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|n| n.as_str().map(str::to_owned))
        .collect();
    names.sort();
    names
}

/// Grades a named check against the record: it must pass unless the record
/// lists it under `failing_checks`, and must fail if it does, so a fix or a
/// regression both show.
#[cfg(test)]
pub fn graded(check: &str, passed: bool, detail: &str) {
    let known = listed("failing_checks").iter().any(|c| c == check);
    assert_eq!(
        passed, !known,
        "{check} (recorded failing: {known}): {detail}"
    );
}
