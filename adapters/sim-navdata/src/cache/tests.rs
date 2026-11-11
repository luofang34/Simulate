use navigate_data::{
    LayerKind, MemoryStore, Sha256Digest, TileAddress, TileKey, WritePort, blob_path,
};
use pollster::block_on;

use crate::{CachedTile, SourceSet, index_path, lookup, record};

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
