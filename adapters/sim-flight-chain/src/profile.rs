//! Temporary wall-time probe for profiling one route second in the page.

use std::cell::RefCell;
use std::collections::BTreeMap;

thread_local! {
    static TOTALS: RefCell<BTreeMap<String, (f64, u64)>> = const { RefCell::new(BTreeMap::new()) };
}

/// A start mark.
pub fn start() -> web_time::Instant {
    web_time::Instant::now()
}

/// Adds the time since `from` under `label`.
pub fn stop(label: &str, from: web_time::Instant) {
    let seconds = from.elapsed().as_secs_f64();
    TOTALS.with(|totals| {
        let mut totals = totals.borrow_mut();
        let entry = totals.entry(label.to_owned()).or_insert((0.0, 0));
        entry.0 += seconds;
        entry.1 = entry.1.wrapping_add(1);
    });
}

/// Takes the totals: label, seconds, calls.
pub fn take() -> Vec<(String, f64, u64)> {
    TOTALS.with(|totals| {
        std::mem::take(&mut *totals.borrow_mut())
            .into_iter()
            .map(|(label, (seconds, calls))| (label, seconds, calls))
            .collect()
    })
}
