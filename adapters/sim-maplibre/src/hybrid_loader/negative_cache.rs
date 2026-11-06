//! Bounded memory of remote tile URLs the server answered with "no such tile".
//!
//! Imagery servers routinely lack the deepest zoom over rural ground; without this memory
//! every frame that still covers such a tile re-issues the same HTTPS request.

use std::{
    collections::{HashSet, VecDeque},
    future::Future,
    sync::{Mutex, PoisonError},
};

use maplibre::io::source_client::SourceFetchError;

/// Remote URLs remembered as missing before the oldest is forgotten.
pub(crate) const NEGATIVE_CACHE_CAPACITY: usize = 2048;

/// FIFO-bounded set of URLs known to have no tile.
#[derive(Debug)]
pub(crate) struct NegativeTileCache {
    capacity: usize,
    missing: HashSet<String>,
    order: VecDeque<String>,
}

impl NegativeTileCache {
    pub(crate) fn new(capacity: usize) -> Self {
        Self {
            capacity,
            missing: HashSet::new(),
            order: VecDeque::new(),
        }
    }

    pub(crate) fn contains(&self, url: &str) -> bool {
        self.missing.contains(url)
    }

    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.missing.len()
    }

    /// Remembers `url`, forgetting the oldest entries once over capacity.
    pub(crate) fn insert(&mut self, url: &str) {
        if self.capacity == 0 || !self.missing.insert(url.to_owned()) {
            return;
        }
        self.order.push_back(url.to_owned());
        while self.order.len() > self.capacity {
            if let Some(oldest) = self.order.pop_front() {
                self.missing.remove(&oldest);
            }
        }
    }
}

/// Answers `url` from `cache` when it is known missing, otherwise runs `fetch` and remembers
/// a "not found" answer. A poisoned lock still holds a consistent set, so it is reused.
pub(crate) async fn fetch_negative_cached<F, Fut>(
    cache: &Mutex<NegativeTileCache>,
    url: &str,
    fetch: F,
) -> Result<Vec<u8>, SourceFetchError>
where
    F: FnOnce() -> Fut,
    Fut: Future<Output = Result<Vec<u8>, SourceFetchError>>,
{
    let known_missing = cache
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .contains(url);
    if known_missing {
        return Err(SourceFetchError::not_found(url));
    }
    let result = fetch().await;
    if let Err(error) = &result
        && error.is_not_found()
    {
        cache
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(url);
    }
    result
}
