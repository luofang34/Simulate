//! Sealing a package with Navigate's own `prepare`, over the tile cache.

use std::collections::BTreeSet;

use navigate_data::{
    FetchPort, Fetched, MemoryStore, Package, PrepareSummary, ReadPort, TileKey, WritePort,
    blob_path, prepare, read_all,
};

use crate::cache::lookup;
use crate::{NavdataError, PackagePlan, SourceSet};

#[cfg(test)]
mod tests;

/// A sealed package.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SealedPackage {
    /// Its identity and counts, as Navigate's `prepare` reports them.
    pub summary: PrepareSummary,
    /// The whole package in memory: manifest, descriptor, tile indexes and
    /// every tile, opened and checked. A navigator reads it without
    /// waiting on a store.
    pub memory: MemoryStore,
    /// Bytes of the files in memory.
    pub memory_bytes: u64,
    /// Tiles whose bytes equal another tile's: open water, or a no-data
    /// picture the source does not declare.
    pub repeated_tiles: usize,
}

/// A fetch port that answers from the tile cache alone. `prepare` asks for
/// every planned tile in turn; each was prefetched or found absent.
struct CacheFetch<'a, S> {
    store: &'a S,
    absent: &'a BTreeSet<TileKey>,
}

impl<S: ReadPort + WritePort> FetchPort for CacheFetch<'_, S> {
    async fn fetch(&self, key: &TileKey) -> Result<Fetched, String> {
        if self.absent.contains(key) {
            return Ok(Fetched::Absent);
        }
        let not_prefetched = || NavdataError::NotPrefetched {
            tile: key.to_string(),
        };
        let tile = lookup(self.store, key)
            .await
            .map_err(|error| error.to_string())?
            .ok_or_else(|| not_prefetched().to_string())?;
        read_all(self.store, &blob_path(tile.digest))
            .await
            .map_err(|error| error.to_string())?
            .map(Fetched::Tile)
            .ok_or_else(|| not_prefetched().to_string())
    }
}

/// Seals the package of `plan` from the tiles a prefetch left in `store`,
/// with `absent` the tiles the sources do not have. Navigate's `prepare`
/// writes the package into memory; what the store lacks of it (the tile
/// indexes, the manifest, the descriptor) is then written there, so the
/// store holds the package too.
///
/// # Errors
///
/// Returns Navigate's refusal, a planned tile that was not prefetched, and
/// a failed store.
pub async fn seal_package<S: ReadPort + WritePort>(
    plan: &PackagePlan,
    sources: &SourceSet,
    release_id: &str,
    absent: &BTreeSet<TileKey>,
    store: &mut S,
) -> Result<SealedPackage, NavdataError> {
    let mut memory = MemoryStore::default();
    let summary = {
        let fetch = CacheFetch {
            store: &*store,
            absent,
        };
        prepare(
            &plan.tiles,
            &sources.layer_specs(),
            &sources.descriptor(release_id),
            plan.tile_count(),
            &fetch,
            &mut memory,
        )
        .await
        .map_err(NavdataError::data("sealing the package"))?
    };
    let opened = Package::open(&memory, summary.identity)
        .await
        .map_err(NavdataError::data("opening the sealed package"))?;
    let distinct_tiles = opened
        .verify_all(&memory)
        .await
        .map_err(NavdataError::data("checking the sealed package"))?;
    let mut memory_bytes = 0_u64;
    let paths: Vec<String> = memory.paths().map(str::to_owned).collect();
    for path in &paths {
        let Some(bytes) = memory.get(path) else {
            continue;
        };
        memory_bytes = memory_bytes.saturating_add(bytes.len() as u64);
        let failed = NavdataError::data("writing the package to the store");
        if !store.contains(path).await.map_err(&failed)? {
            store.write(path, bytes).await.map_err(&failed)?;
        }
    }
    Ok(SealedPackage {
        repeated_tiles: summary.tiles.saturating_sub(distinct_tiles),
        summary,
        memory,
        memory_bytes,
    })
}
