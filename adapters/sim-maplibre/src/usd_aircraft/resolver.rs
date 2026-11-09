//! In-memory OpenUSD asset resolution for embedded stages.

use std::{
    borrow::Cow,
    collections::BTreeMap,
    io::{self, Cursor},
    time::SystemTime,
};

use openusd::ar::{Asset, AssetBuffer, ResolvedPath, Resolver};

/// In-memory `openusd::ar::Resolver` serving embedded `.usdc` / `.usda` buffers on both
/// native and `wasm32` targets without filesystem access.
#[derive(Clone, Debug, Default)]
pub(crate) struct EmbeddedUsdResolver {
    assets: BTreeMap<String, AssetBuffer>,
}

impl EmbeddedUsdResolver {
    pub(crate) fn single(name: &str, bytes: AssetBuffer) -> Self {
        Self {
            assets: BTreeMap::from([(normalize_asset_key(name), bytes)]),
        }
    }

    pub(crate) fn insert(&mut self, name: &str, bytes: AssetBuffer) {
        self.assets.insert(normalize_asset_key(name), bytes);
    }
}

impl Resolver for EmbeddedUsdResolver {
    fn create_identifier(&self, asset_path: &str, _anchor: Option<&ResolvedPath>) -> String {
        normalize_asset_key(asset_path)
    }

    fn resolve(&self, asset_path: &str) -> Option<ResolvedPath> {
        let key = normalize_asset_key(asset_path);
        self.assets
            .contains_key(&key)
            .then(|| ResolvedPath::new(key))
    }

    fn resolve_for_new_asset(&self, asset_path: &str) -> Option<ResolvedPath> {
        Some(ResolvedPath::new(normalize_asset_key(asset_path)))
    }

    fn open_asset(&self, resolved_path: &ResolvedPath) -> io::Result<Box<dyn Asset>> {
        let key = normalize_asset_key(&resolved_path.to_string_lossy());
        let buffer = self
            .assets
            .get(&key)
            .cloned()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, key))?;
        Ok(Box::new(Cursor::new(buffer)))
    }

    fn in_memory_identifier<'a>(&self, asset_path: &'a str) -> Cow<'a, str> {
        Cow::Owned(normalize_asset_key(asset_path))
    }

    fn get_modification_timestamp(
        &self,
        _path: &str,
        _resolved: &ResolvedPath,
    ) -> Option<SystemTime> {
        None
    }

    fn identity(&self) -> String {
        "sim-embedded-usd".to_owned()
    }
}

fn normalize_asset_key(path: &str) -> String {
    path.trim_start_matches("./")
        .trim_start_matches('/')
        .to_owned()
}
