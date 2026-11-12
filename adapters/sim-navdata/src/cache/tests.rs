use navigate_data::{
    DataError, LayerKind, MemoryStore, ReadPort, Sha256Digest, TileAddress, TileKey, WritePort,
    blob_path,
};
use pollster::block_on;

use crate::{
    CacheState, CachedTile, NavdataError, RepairPort, SourceSet, index_path, inspect, lookup,
    record,
};

fn key(kind: LayerKind) -> TileKey {
    SourceSet::default().tile_key(kind, TileAddress::new(15, 16_288, 12_159).unwrap())
}

#[test]
fn an_entry_is_named_by_source_revision_layer_and_address() {
    assert_eq!(
        index_path(&key(LayerKind::Imagery)),
        "tiles/esri_world_imagery.unversioned.imagery.15.16288.12159"
    );
    assert_eq!(
        index_path(&key(LayerKind::Elevation)),
        "tiles/mapzen_terrarium.unversioned.elevation.15.16288.12159"
    );
}

#[test]
fn a_recorded_tile_is_found_with_its_digest_and_length() {
    let mut store = MemoryStore::default();
    let key = key(LayerKind::Imagery);
    assert_eq!(block_on(lookup(&store, &key)), Ok(None));
    let tile = block_on(record(&mut store, &key, b"tile bytes")).unwrap();
    assert_eq!(
        tile,
        CachedTile {
            digest: Sha256Digest::of(b"tile bytes"),
            bytes: 10
        }
    );
    assert_eq!(block_on(lookup(&store, &key)), Ok(Some(tile)));
    assert_eq!(store.get(&blob_path(tile.digest)), Some(&b"tile bytes"[..]));
    assert_eq!(
        block_on(lookup(&store, &self::key(LayerKind::Elevation))),
        Ok(None)
    );
}

#[test]
fn an_entry_without_its_bytes_or_that_does_not_read_is_no_tile() {
    let key = key(LayerKind::Imagery);
    let mut store = MemoryStore::default();
    let tile = block_on(record(&mut store, &key, b"tile bytes")).unwrap();
    assert!(store.remove(&blob_path(tile.digest)));
    assert_eq!(block_on(lookup(&store, &key)), Ok(None));

    for entry in [&b"not an entry"[..], b"", b"abcd 10", &[0xff, 0xfe]] {
        let mut store = MemoryStore::default();
        block_on(store.write(&index_path(&key), entry)).unwrap();
        assert_eq!(block_on(lookup(&store, &key)), Ok(None), "{entry:?}");
    }
}

/// What a cut-off write leaves, with the cause the cache names for it.
#[test]
fn a_cut_off_write_is_no_tile_and_the_cache_says_why() {
    let key = key(LayerKind::Imagery);
    let digest = Sha256Digest::of(b"tile bytes");
    let entry = format!("{digest} 10");
    let states = [
        (
            vec![(index_path(&key), Vec::new())],
            "its index entry holds nothing (an empty file)",
        ),
        (
            vec![(index_path(&key), entry.clone().into_bytes())],
            "is missing",
        ),
        (
            vec![
                (index_path(&key), entry.clone().into_bytes()),
                (blob_path(digest), b"tile".to_vec()),
            ],
            "holds 4 bytes that are not the 10",
        ),
        (
            vec![
                (index_path(&key), entry.into_bytes()),
                (blob_path(digest), b"TILE BYTES".to_vec()),
            ],
            "holds 10 bytes that are not the 10",
        ),
    ];
    for (files, why) in states {
        let mut store = MemoryStore::default();
        for (path, bytes) in &files {
            block_on(store.write(path, bytes)).unwrap();
        }
        let state = block_on(inspect(&store, &key)).unwrap();
        assert_eq!(state.held(), None, "{state:?}");
        let words = state.why_not_held().unwrap();
        assert!(words.contains(why), "{words}");
        // Recording the tile again replaces what the cut-off write left.
        let tile = block_on(record(&mut store, &key, b"tile bytes")).unwrap();
        assert_eq!(block_on(inspect(&store, &key)), Ok(CacheState::Held(tile)));
    }
}

/// A store whose writes never hold.
struct Forgetful(MemoryStore);

impl ReadPort for Forgetful {
    type File = <MemoryStore as ReadPort>::File;

    async fn open(&self, path: &str) -> Result<Option<Self::File>, DataError> {
        self.0.open(path).await
    }
}

impl WritePort for Forgetful {
    async fn write(&mut self, _path: &str, _bytes: &[u8]) -> Result<(), DataError> {
        Ok(())
    }

    async fn contains(&self, path: &str) -> Result<bool, DataError> {
        self.0.contains(path).await
    }
}

impl RepairPort for Forgetful {
    async fn replace(&mut self, _path: &str, _bytes: &[u8]) -> Result<(), DataError> {
        Ok(())
    }
}

#[test]
fn a_store_that_keeps_no_write_is_named_as_the_fault() {
    let mut store = Forgetful(MemoryStore::default());
    let error = block_on(record(&mut store, &key(LayerKind::Imagery), b"tile bytes")).unwrap_err();
    assert!(error.store_fault(), "{error}");
    let NavdataError::StoreKept { path, detail } = &error else {
        panic!("{error:?}");
    };
    assert_eq!(path, &blob_path(Sha256Digest::of(b"tile bytes")));
    assert_eq!(detail, "no file where 10 bytes were written");
    assert!(
        !NavdataError::NotPrefetched {
            tile: "t".into(),
            cause: "c".into()
        }
        .store_fault()
    );
}
