//! The persistent tile cache: each tile's bytes once under their digest,
//! and an index entry by tile key saying which digest that is.
//!
//! A store addresses chunks by content, so it cannot say whether it holds
//! the tile of an address. The index entry at
//! `tiles/<source>.<revision>.<layer>.<z>.<x>.<y>` holds the digest and the
//! length. The chunk is written before the entry, so an entry never names
//! bytes the store lacks.
//!
//! A store keeps the first write of a path, and a browser creates a file
//! before the write into it commits. A write cut off there (the page left,
//! the tab was closed, the browser refused it) leaves an empty or short
//! file that would stand for the tile forever. So a tile is held only when
//! its entry reads and its chunk's bytes hash to the chunk's address, and a
//! record reads back what it wrote and replaces what a cut-off write left.

use std::future::Future;

use navigate_data::{
    DataError, LayerKind, MemoryStore, ReadPort, Revision, Sha256Digest, TileKey, WritePort,
    blob_path, read_all,
};

use crate::NavdataError;

#[cfg(test)]
mod tests;

/// A tile the cache holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CachedTile {
    /// The digest of its bytes.
    pub digest: Sha256Digest,
    /// Its length, bytes.
    pub bytes: u64,
}

/// What the cache holds for one tile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CacheState {
    /// The tile, whole.
    Held(CachedTile),
    /// No index entry: the tile was never stored.
    NoEntry,
    /// An index entry that does not read as `<sha256> <length>`: what it
    /// holds, in words.
    UnreadableEntry(String),
    /// An entry whose chunk the store does not hold.
    NoChunk(CachedTile),
    /// A chunk whose bytes do not hash to its address.
    WrongChunk {
        /// The tile the entry names.
        tile: CachedTile,
        /// Bytes the chunk holds.
        kept_bytes: u64,
    },
}

impl CacheState {
    /// The tile, when the cache holds it whole.
    #[must_use]
    pub const fn held(&self) -> Option<CachedTile> {
        match self {
            Self::Held(tile) => Some(*tile),
            _ => None,
        }
    }

    /// Why the cache does not hold the tile, in words; `None` when it does.
    #[must_use]
    pub fn why_not_held(&self) -> Option<String> {
        match self {
            Self::Held(_) => None,
            Self::NoEntry => Some("it has no index entry".to_owned()),
            Self::UnreadableEntry(what) => Some(format!("its index entry holds {what}")),
            Self::NoChunk(tile) => Some(format!("its chunk {} is missing", tile.digest)),
            Self::WrongChunk { tile, kept_bytes } => Some(format!(
                "its chunk {} holds {kept_bytes} bytes that are not the {} its address names",
                tile.digest, tile.bytes
            )),
        }
    }
}

/// A store that can replace what a cut-off write left at a path. The data
/// contract keeps a path's first write, which a cut-off write would
/// otherwise hold for good.
pub trait RepairPort: ReadPort + WritePort {
    /// Replaces whatever the store holds at `path` with `bytes`.
    fn replace(&mut self, path: &str, bytes: &[u8]) -> impl Future<Output = Result<(), DataError>>;
}

impl RepairPort for MemoryStore {
    async fn replace(&mut self, path: &str, bytes: &[u8]) -> Result<(), DataError> {
        self.remove(path);
        self.write(path, bytes).await
    }
}

/// The logical path of the index entry of `key`.
pub fn index_path(key: &TileKey) -> String {
    let revision = match key.revision {
        Revision::Unversioned => "unversioned".to_owned(),
        Revision::Release(release) => format!("release-{release}"),
        Revision::Collected(day) => format!("collected-{}", day.0),
    };
    let layer = match key.layer {
        LayerKind::Imagery => "imagery",
        LayerKind::Elevation => "elevation",
        LayerKind::Unsupported => "unsupported",
    };
    let address = key.address;
    format!(
        "tiles/{}.{revision}.{layer}.{}.{}.{}",
        key.source, address.zoom, address.x, address.y
    )
}

/// What the cache in `store` holds for `key`.
///
/// # Errors
///
/// Returns the store's failure.
pub async fn inspect<S: ReadPort>(store: &S, key: &TileKey) -> Result<CacheState, NavdataError> {
    let failed = NavdataError::data("reading the tile cache");
    let Some(entry) = read_all(store, &index_path(key)).await.map_err(&failed)? else {
        return Ok(CacheState::NoEntry);
    };
    let Some(tile) = parse_entry(&entry) else {
        return Ok(CacheState::UnreadableEntry(contents_words(&entry)));
    };
    let Some(chunk) = read_all(store, &blob_path(tile.digest))
        .await
        .map_err(&failed)?
    else {
        return Ok(CacheState::NoChunk(tile));
    };
    if Sha256Digest::of(&chunk) == tile.digest && chunk.len() as u64 == tile.bytes {
        Ok(CacheState::Held(tile))
    } else {
        Ok(CacheState::WrongChunk {
            tile,
            kept_bytes: chunk.len() as u64,
        })
    }
}

/// The cached tile of `key`, when the store holds it whole.
///
/// # Errors
///
/// Returns the store's failure.
pub async fn lookup<S: ReadPort>(
    store: &S,
    key: &TileKey,
) -> Result<Option<CachedTile>, NavdataError> {
    Ok(inspect(store, key).await?.held())
}

/// Stores the bytes of the tile `key` and its index entry, each read back
/// and replaced when the store kept what a cut-off write left there.
///
/// # Errors
///
/// Returns the store's failure, and [`NavdataError::StoreKept`] when the
/// store does not keep what it was given even after replacing it.
pub async fn record<S: RepairPort>(
    store: &mut S,
    key: &TileKey,
    bytes: &[u8],
) -> Result<CachedTile, NavdataError> {
    let tile = CachedTile {
        digest: Sha256Digest::of(bytes),
        bytes: bytes.len() as u64,
    };
    write_kept(store, &blob_path(tile.digest), bytes).await?;
    let entry = format!("{} {}", tile.digest, tile.bytes);
    write_kept(store, &index_path(key), entry.as_bytes()).await?;
    Ok(tile)
}

/// Writes `bytes` at `path` so that the store holds them: replaced when the
/// first write there was cut off.
async fn write_kept<S: RepairPort>(
    store: &mut S,
    path: &str,
    bytes: &[u8],
) -> Result<(), NavdataError> {
    let failed = NavdataError::data("writing the tile cache");
    store.write(path, bytes).await.map_err(&failed)?;
    if kept(store, path).await?.as_deref() == Some(bytes) {
        return Ok(());
    }
    store.replace(path, bytes).await.map_err(&failed)?;
    match kept(store, path).await? {
        Some(read) if read == bytes => Ok(()),
        read => Err(NavdataError::StoreKept {
            path: path.to_owned(),
            detail: format!(
                "{} where {} bytes were written",
                read.as_deref().map_or_else(
                    || "no file".to_owned(),
                    |read| format!("{} other bytes", read.len())
                ),
                bytes.len()
            ),
        }),
    }
}

async fn kept<S: ReadPort>(store: &S, path: &str) -> Result<Option<Vec<u8>>, NavdataError> {
    read_all(store, path)
        .await
        .map_err(NavdataError::data("reading back the tile cache"))
}

/// A file's contents in words, for saying what a cut-off write left.
fn contents_words(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        "nothing (an empty file)".to_owned()
    } else {
        format!("{} bytes that do not read", bytes.len())
    }
}

/// An entry is `<sha256 hex> <length>`; anything else is no entry.
fn parse_entry(entry: &[u8]) -> Option<CachedTile> {
    let text = std::str::from_utf8(entry).ok()?;
    let (digest, bytes) = text.split_once(' ')?;
    Some(CachedTile {
        digest: digest.parse().ok()?,
        bytes: bytes.parse().ok()?,
    })
}
