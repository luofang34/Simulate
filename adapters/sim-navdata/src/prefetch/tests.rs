use navigate_data::{GroundHeight, MemoryStore, TileKey};
use pollster::block_on;

use crate::fixtures::{CIRCUIT, FakeSource, camera};
use crate::{
    NavdataError, PackagePlan, PrefetchOutcome, PrefetchProgress, SourceSet, index_path,
    plan_package, prefetch,
};

struct Flat;

impl GroundHeight for Flat {
    fn ground_m(&self, _point: [f64; 2]) -> Option<f64> {
        Some(300.0)
    }
}

fn plan() -> (PackagePlan, Vec<TileKey>) {
    let sources = SourceSet::default();
    let plan = plan_package(&CIRCUIT, camera(), &sources, &Flat).unwrap();
    let keys = plan.keys(&sources);
    (plan, keys)
}

fn run(
    keys: &[TileKey],
    budget: u64,
    source: &FakeSource,
    store: &mut MemoryStore,
) -> (Result<PrefetchOutcome, NavdataError>, Vec<PrefetchProgress>) {
    let mut heard = Vec::new();
    let outcome = block_on(prefetch(
        keys,
        &SourceSet::default(),
        budget,
        source,
        store,
        &mut |progress| heard.push(*progress),
    ));
    (outcome, heard)
}

#[test]
fn every_planned_tile_is_fetched_once_and_stored() {
    let (_, keys) = plan();
    let source = FakeSource::new(300.0);
    let mut store = MemoryStore::default();
    let (outcome, heard) = run(&keys, u64::MAX, &source, &mut store);
    let outcome = outcome.unwrap();
    assert_eq!(source.asked(), keys.len());
    assert_eq!(
        (
            outcome.progress.done,
            outcome.progress.fetched,
            outcome.progress.reused
        ),
        (keys.len(), keys.len(), 0)
    );
    assert!(outcome.absent.is_empty());
    assert!(keys.iter().all(|key| store.get(&index_path(key)).is_some()));

    assert_eq!(
        heard.first().map(|p| (p.done, p.total)),
        Some((0, keys.len()))
    );
    assert_eq!(heard.len(), keys.len() + 1);
    assert!(
        heard
            .windows(2)
            .all(|pair| pair[1].done == pair[0].done + 1)
    );
    assert!(heard.windows(2).all(|pair| pair[1].bytes >= pair[0].bytes));
    assert_eq!(heard.last(), Some(&outcome.progress));
}

#[test]
fn a_second_prefetch_reuses_the_store_and_fetches_nothing() {
    let (_, keys) = plan();
    let source = FakeSource::new(300.0);
    let mut store = MemoryStore::default();
    let first = run(&keys, u64::MAX, &source, &mut store).0.unwrap();
    let again = run(&keys, u64::MAX, &source, &mut store).0.unwrap();
    assert_eq!(source.asked(), keys.len());
    assert_eq!(
        (again.progress.reused, again.progress.fetched),
        (keys.len(), 0)
    );
    assert_eq!(again.progress.bytes, first.progress.bytes);
}

#[test]
fn tiles_the_source_lacks_are_gaps_not_failures() {
    let (_, keys) = plan();
    let mut source = FakeSource::new(300.0);
    source.statuses.insert("/terrarium/12/", 404);
    let mut store = MemoryStore::default();
    let outcome = run(&keys, u64::MAX, &source, &mut store).0.unwrap();
    let elevation = keys
        .iter()
        .filter(|key| index_path(key).contains(".elevation."))
        .count();
    assert!(elevation > 0);
    assert_eq!(outcome.progress.gaps, elevation);
    assert_eq!(outcome.absent.len(), elevation);
    assert_eq!(outcome.progress.done, keys.len());
    assert_eq!(outcome.progress.fetched, keys.len() - elevation);
}

#[test]
fn a_failing_source_stops_the_prefetch_and_the_next_attempt_resumes() {
    let (_, keys) = plan();
    let mut source = FakeSource::new(300.0);
    source.statuses.insert("/terrarium/12/", 503);
    let mut store = MemoryStore::default();
    let error = run(&keys, u64::MAX, &source, &mut store).0.unwrap_err();
    assert!(
        matches!(&error, NavdataError::Transport { url, message }
            if url.contains("/terrarium/12/") && message == "HTTP status 503"),
        "{error}"
    );
    let stored = keys
        .iter()
        .filter(|key| store.get(&index_path(key)).is_some())
        .count();
    assert!(stored > 0 && stored < keys.len());

    source.statuses.clear();
    let asked_before = source.asked();
    let outcome = run(&keys, u64::MAX, &source, &mut store).0.unwrap();
    assert_eq!(outcome.progress.reused, stored);
    assert_eq!(source.asked() - asked_before, keys.len() - stored);
}

#[test]
fn a_source_that_cannot_be_reached_is_named() {
    let (_, keys) = plan();
    let mut source = FakeSource::new(300.0);
    source.statuses.insert("World_Imagery", 0);
    let error = run(&keys, u64::MAX, &source, &mut MemoryStore::default())
        .0
        .unwrap_err();
    assert!(
        matches!(&error, NavdataError::Transport { url, message }
            if url.contains("World_Imagery") && message == "connection refused"),
        "{error}"
    );
}

#[test]
fn a_dropped_connection_or_a_busy_server_is_asked_again() {
    let (_, keys) = plan();
    let source = FakeSource::new(300.0);
    source.drops.set(2);
    let outcome = run(&keys, u64::MAX, &source, &mut MemoryStore::default())
        .0
        .unwrap();
    assert_eq!(outcome.progress.fetched, keys.len());
    assert_eq!(source.asked(), keys.len() + 2);

    // A tile that keeps failing is asked for three times, then named.
    let mut busy = FakeSource::new(300.0);
    busy.statuses.insert("/terrarium/12/", 503);
    let one = [*keys
        .iter()
        .find(|key| index_path(key).contains(".elevation."))
        .unwrap()];
    let error = run(&one, u64::MAX, &busy, &mut MemoryStore::default())
        .0
        .unwrap_err();
    assert!(
        matches!(error, NavdataError::Transport { message, .. } if message == "HTTP status 503")
    );
    assert_eq!(busy.asked(), 3);
}

#[test]
fn the_prefetch_stops_at_the_budget() {
    let (_, keys) = plan();
    let source = FakeSource::new(300.0);
    let mut store = MemoryStore::default();
    let (outcome, heard) = run(&keys, 100_000, &source, &mut store);
    let error = outcome.unwrap_err();
    let NavdataError::BudgetReached {
        budget_bytes,
        bytes,
        done,
        total,
    } = error
    else {
        panic!("{error}");
    };
    assert_eq!((budget_bytes, total), (100_000, keys.len()));
    assert!(bytes > 100_000 && done < total);
    assert_eq!(heard.last().map(|p| (p.bytes, p.done)), Some((bytes, done)));
    assert!(store.paths().all(|path| !path.starts_with("packages/")));

    // The tiles held count against the budget when they are reused, too.
    let again = run(&keys, 100_000, &source, &mut store).0.unwrap_err();
    assert!(matches!(again, NavdataError::BudgetReached { .. }));
}
