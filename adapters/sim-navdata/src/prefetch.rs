//! Filling the tile cache with a plan's tiles, side by side, with progress
//! and a stop at the storage budget.

use futures_util::stream::{self, StreamExt};
use navigate_data::{Fetched, TileKey, blob_path, read_all};
use std::collections::BTreeSet;

use crate::cache::{RepairPort, lookup, record};
use crate::source::tile_url;
use crate::transport::fetched;
use crate::{NavdataError, SourceSet, TileTransport};

#[cfg(test)]
mod tests;

/// Tiles fetched at once: what a browser opens to one host without queueing.
pub const FETCHES_AT_ONCE: usize = 6;
/// Times one tile is asked for before its failure stops the prefetch: a
/// plan of thousands of tiles meets the odd dropped connection or busy
/// server.
pub const FETCH_ATTEMPTS: u32 = 3;

/// How far a prefetch has come.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PrefetchProgress {
    /// Tiles planned.
    pub total: usize,
    /// Tiles settled: held, or known absent.
    pub done: usize,
    /// Of those, tiles the store already held.
    pub reused: usize,
    /// Of those, tiles fetched now.
    pub fetched: usize,
    /// Of those, tiles the source does not have.
    pub gaps: usize,
    /// Bytes of the tiles held, reused and fetched.
    pub bytes: u64,
}

/// A finished prefetch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PrefetchOutcome {
    /// The final count.
    pub progress: PrefetchProgress,
    /// The tiles the source does not have: gaps of the package.
    pub absent: BTreeSet<TileKey>,
}

/// The tile `key`: from the cache in `store`, or fetched through
/// `transport` and stored. The flag says whether it was fetched.
pub(crate) async fn obtain<T, S>(
    key: &TileKey,
    sources: &SourceSet,
    transport: &T,
    store: &mut S,
) -> Result<(Fetched, bool), NavdataError>
where
    T: TileTransport,
    S: RepairPort,
{
    if let Some(tile) = lookup(store, key).await? {
        let bytes = read_all(store, &blob_path(tile.digest))
            .await
            .map_err(NavdataError::data("reading the tile cache"))?;
        if let Some(bytes) = bytes {
            return Ok((Fetched::Tile(bytes), false));
        }
    }
    let answer = download(key, sources, transport).await?;
    if let Fetched::Tile(bytes) = &answer {
        record(store, key, bytes).await?;
    }
    Ok((answer, true))
}

async fn download<T: TileTransport>(
    key: &TileKey,
    sources: &SourceSet,
    transport: &T,
) -> Result<Fetched, NavdataError> {
    let source = sources.of(key.layer);
    let url = tile_url(source, key);
    let mut message = String::new();
    for attempt in 1..=FETCH_ATTEMPTS {
        match transport.get(&url).await {
            // A server error may pass; any other status is the answer.
            Ok(answer) if answer.status >= 500 && attempt < FETCH_ATTEMPTS => {}
            Ok(answer) => return fetched(source, &url, answer),
            Err(refusal) => message = refusal,
        }
    }
    Err(NavdataError::Transport { url, message })
}

/// Brings every tile of `keys` into the cache in `store`: those it holds
/// are counted, the rest are fetched [`FETCHES_AT_ONCE`] at a time through
/// `transport`. `progress` hears each change. The bytes of the tiles held
/// are counted against `budget_bytes`; reaching it stops the prefetch, so
/// what was planned is never sealed short.
///
/// # Errors
///
/// Returns [`NavdataError::BudgetReached`] at the budget, and a failed
/// fetch or store. Tiles stored before an error stay in the cache, so the
/// next attempt resumes.
pub async fn prefetch<T, S>(
    keys: &[TileKey],
    sources: &SourceSet,
    budget_bytes: u64,
    transport: &T,
    store: &mut S,
    progress: &mut dyn FnMut(&PrefetchProgress),
) -> Result<PrefetchOutcome, NavdataError>
where
    T: TileTransport,
    S: RepairPort,
{
    let mut count = PrefetchProgress {
        total: keys.len(),
        ..PrefetchProgress::default()
    };
    let over = |count: &PrefetchProgress| {
        (count.bytes > budget_bytes).then_some(NavdataError::BudgetReached {
            budget_bytes,
            bytes: count.bytes,
            done: count.done,
            total: count.total,
        })
    };
    progress(&count);
    let mut missing = Vec::new();
    for key in keys {
        match lookup(store, key).await? {
            Some(tile) => {
                count.done = count.done.saturating_add(1);
                count.reused = count.reused.saturating_add(1);
                count.bytes = count.bytes.saturating_add(tile.bytes);
                progress(&count);
                if let Some(error) = over(&count) {
                    return Err(error);
                }
            }
            None => missing.push(*key),
        }
    }
    let mut absent = BTreeSet::new();
    let mut answers = stream::iter(missing)
        .map(|key| async move { (key, download(&key, sources, transport).await) })
        .buffer_unordered(FETCHES_AT_ONCE);
    while let Some((key, answer)) = answers.next().await {
        match answer? {
            Fetched::Tile(bytes) => {
                let tile = record(store, &key, &bytes).await?;
                count.fetched = count.fetched.saturating_add(1);
                count.bytes = count.bytes.saturating_add(tile.bytes);
            }
            Fetched::Absent => {
                absent.insert(key);
                count.gaps = count.gaps.saturating_add(1);
            }
        }
        count.done = count.done.saturating_add(1);
        progress(&count);
        if let Some(error) = over(&count) {
            return Err(error);
        }
    }
    Ok(PrefetchOutcome {
        progress: count,
        absent,
    })
}
