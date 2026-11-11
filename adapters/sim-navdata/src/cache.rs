//! The persistent tile cache: each tile's bytes once under their digest,
//! and an index entry by tile key saying which digest that is.
//!
//! A store addresses chunks by content, so it cannot say whether it holds
//! the tile of an address. The index entry at
//! `tiles/<source>.<revision>.<layer>.<z>.<x>.<y>` holds the digest and the
//! length. The chunk is written before the entry, so an entry never names
//! bytes the store lacks.

use navigate_data::{
    LayerKind, ReadPort, Revision, Sha256Digest, TileKey, WritePort, blob_path, read_all,
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

/// The cached tile of `key`, when the store holds both its entry and its
/// bytes.
///
/// # Errors
///
/// Returns the store's failure.
pub async fn lookup<S: ReadPort + WritePort>(
    store: &S,
    key: &TileKey,
) -> Result<Option<CachedTile>, NavdataError> {
    let failed = NavdataError::data("reading the tile cache");
    let Some(entry) = read_all(store, &index_path(key)).await.map_err(&failed)? else {
        return Ok(None);
    };
    let Some(tile) = parse_entry(&entry) else {
        return Ok(None);
    };
    let held = store
        .contains(&blob_path(tile.digest))
        .await
        .map_err(&failed)?;
    Ok(held.then_some(tile))
}

/// Stores the bytes of the tile `key` and its index entry.
///
/// # Errors
///
/// Returns the store's failure.
pub async fn record<S: WritePort>(
    store: &mut S,
    key: &TileKey,
    bytes: &[u8],
) -> Result<CachedTile, NavdataError> {
    let failed = NavdataError::data("writing the tile cache");
    let tile = CachedTile {
        digest: Sha256Digest::of(bytes),
        bytes: bytes.len() as u64,
    };
    store
        .write(&blob_path(tile.digest), bytes)
        .await
        .map_err(&failed)?;
    let entry = format!("{} {}", tile.digest, tile.bytes);
    store
        .write(&index_path(key), entry.as_bytes())
        .await
        .map_err(&failed)?;
    Ok(tile)
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
