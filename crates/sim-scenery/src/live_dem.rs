//! Hand-off of decoded Terrarium tiles from tile-loader threads to the
//! simulation thread's [`AuthoritativeWorldSurface`].

use std::collections::{BTreeSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError, TryLockError};

use crate::catalog::require_valid;
use crate::{AuthoritativeWorldSurface, SceneryError, TerrariumTile, TileId};

#[cfg(test)]
mod tests;

/// Tiles decoded and waiting for the simulation thread.
type Pending = Vec<(TileId, Arc<TerrariumTile>)>;

/// Bounded deduplication window kept comfortably below `DEFAULT_LIVE_TILE_CAPACITY` (`1,024`)
/// so a tile still in `RecentTiles` has not yet been evicted from the default `LiveDem`.
const RECENT_INGEST_CAPACITY: usize = 768;

#[derive(Debug, Default)]
struct RecentTiles {
    order: VecDeque<TileId>,
    present: BTreeSet<TileId>,
}

impl RecentTiles {
    fn contains(&self, id: TileId) -> bool {
        self.present.contains(&id)
    }

    /// Records `id`, returning `true` if it was newly inserted.
    fn insert(&mut self, id: TileId) -> bool {
        if !self.present.insert(id) {
            return false;
        }
        if self.order.len() >= RECENT_INGEST_CAPACITY
            && let Some(evicted) = self.order.pop_front()
        {
            self.present.remove(&evicted);
        }
        self.order.push_back(id);
        true
    }

    /// Forgets `id`, so the next fetch of it is ingested again.
    fn remove(&mut self, id: TileId) {
        if self.present.remove(&id) {
            self.order.retain(|&queued| queued != id);
        }
    }
}

/// A cheap-to-clone queue between any number of tile loaders and the one
/// thread that owns the world surface.
///
/// Loaders decode on their own thread and hold the lock only to push; the
/// simulation thread drains with a non-blocking `try_lock`, so a busy loader
/// delays a tile by one drain and never stalls a physics step.
#[derive(Clone, Debug, Default)]
pub struct LiveDemBridge {
    pending: Arc<Mutex<Pending>>,
    recent: Arc<Mutex<RecentTiles>>,
    generation: Arc<AtomicU64>,
}

impl LiveDemBridge {
    /// An empty bridge.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Decodes Terrarium PNG bytes fetched for `id` and queues the tile.
    /// Decoding happens on the calling thread, before the lock is taken.
    pub fn ingest_terrarium_png(&self, id: TileId, png_bytes: &[u8]) -> Result<(), SceneryError> {
        require_valid(id)?;
        let tile = TerrariumTile::from_png_bytes(png_bytes)?;
        let _inserted = self
            .recent
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(id);
        self.push(id, Arc::new(tile));
        Ok(())
    }

    /// Decodes and queues immutable Terrarium PNG bytes for `id` only when `id` is not already
    /// in the bridge's recent-tile window, avoiding duplicate PNG decompression when OTW, SVS,
    /// and EFB maps independently fetch the same cached tile.
    pub fn ingest_terrarium_png_if_new(
        &self,
        id: TileId,
        png_bytes: &[u8],
    ) -> Result<bool, SceneryError> {
        require_valid(id)?;
        if self
            .recent
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .contains(id)
        {
            return Ok(false);
        }
        let tile = TerrariumTile::from_png_bytes(png_bytes)?;
        let newly_added = self
            .recent
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(id);
        if newly_added {
            self.push(id, Arc::new(tile));
        }
        Ok(newly_added)
    }

    /// Queues an already decoded tile for `id`.
    pub fn ingest_terrarium_tile(
        &self,
        id: TileId,
        tile: TerrariumTile,
    ) -> Result<(), SceneryError> {
        require_valid(id)?;
        let _inserted = self
            .recent
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(id);
        self.push(id, Arc::new(tile));
        Ok(())
    }

    /// Counts every tile ever queued; a changed value means new ground is on
    /// its way.
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// Tiles queued and not yet drained; `0` while a loader holds the lock.
    #[must_use]
    pub fn pending_len_may_be_stale(&self) -> usize {
        match self.pending.try_lock() {
            Ok(pending) => pending.len(),
            Err(TryLockError::Poisoned(poisoned)) => poisoned.into_inner().len(),
            Err(TryLockError::WouldBlock) => 0,
        }
    }

    /// Moves every queued tile into `surface` and returns how many became
    /// resident. Returns `0` without waiting when a loader holds the lock;
    /// the tiles stay queued for the next drain.
    pub fn drain_into(&self, surface: &mut AuthoritativeWorldSurface) -> usize {
        let drained: Pending = match self.pending.try_lock() {
            Ok(mut pending) => std::mem::take(&mut *pending),
            Err(TryLockError::Poisoned(poisoned)) => std::mem::take(&mut *poisoned.into_inner()),
            Err(TryLockError::WouldBlock) => return 0,
        };
        let resident = drained
            .into_iter()
            .filter(|(id, tile)| {
                surface
                    .insert_shared_terrarium_tile(*id, Arc::clone(tile))
                    .is_ok()
            })
            .count();
        self.forget_evicted(surface);
        resident
    }

    /// Forgets the tiles `surface` has evicted, so a loader fetching one again hands it
    /// over again instead of dropping it as already seen.
    pub fn forget_evicted(&self, surface: &mut AuthoritativeWorldSurface) {
        let evicted = surface.take_evicted_live_tiles();
        if evicted.is_empty() {
            return;
        }
        let mut recent = self.recent.lock().unwrap_or_else(PoisonError::into_inner);
        for id in evicted {
            recent.remove(id);
        }
    }

    fn push(&self, id: TileId, tile: Arc<TerrariumTile>) {
        // A loader that panicked mid-push left a complete Vec behind, so its
        // contents are still sound.
        self.pending
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push((id, tile));
        self.generation.fetch_add(1, Ordering::AcqRel);
    }
}
