//! Hand-off of decoded Terrarium tiles from tile-loader threads to the
//! simulation thread's [`AuthoritativeWorldSurface`].

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError, TryLockError};

use crate::catalog::require_valid;
use crate::{AuthoritativeWorldSurface, SceneryError, TerrariumTile, TileId};

#[cfg(test)]
mod tests;

/// Tiles decoded and waiting for the simulation thread.
type Pending = Vec<(TileId, Arc<TerrariumTile>)>;

/// A cheap-to-clone queue between any number of tile loaders and the one
/// thread that owns the world surface.
///
/// Loaders decode on their own thread and hold the lock only to push; the
/// simulation thread drains with a non-blocking `try_lock`, so a busy loader
/// delays a tile by one drain and never stalls a physics step.
#[derive(Clone, Debug, Default)]
pub struct LiveDemBridge {
    pending: Arc<Mutex<Pending>>,
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
        self.push(id, Arc::new(tile));
        Ok(())
    }

    /// Queues an already decoded tile for `id`.
    pub fn ingest_terrarium_tile(
        &self,
        id: TileId,
        tile: TerrariumTile,
    ) -> Result<(), SceneryError> {
        require_valid(id)?;
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
        drained
            .into_iter()
            .filter(|(id, tile)| {
                surface
                    .insert_shared_terrarium_tile(*id, Arc::clone(tile))
                    .is_ok()
            })
            .count()
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
