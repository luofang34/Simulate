//! The stores map packages are read from, as the host gives them.
//!
//! A request names a package by its store and its identity; the store is a
//! name the host resolves. On a desktop it is a directory; in a browser it
//! is whatever the page keeps packages in, memory or OPFS. Any
//! `navigate_data::ReadPort` serves: [`MapStore`] erases its type, so the
//! trial and the navigator read every kind the same way.

use std::collections::BTreeMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;

use navigate_data::{DataError, DirectoryStore, RandomAccess, ReadPort};

use super::TrialError;
use super::maps::PackageRefusal;

type Pending<'a, T> = Pin<Box<dyn Future<Output = T> + 'a>>;

trait AnyFile: Send + Sync {
    fn file_len(&self) -> u64;
    fn read_any(&self, offset: u64, length: usize) -> Pending<'_, Result<Vec<u8>, DataError>>;
}

impl<F: RandomAccess + Send + Sync> AnyFile for F {
    fn file_len(&self) -> u64 {
        self.len()
    }

    fn read_any(&self, offset: u64, length: usize) -> Pending<'_, Result<Vec<u8>, DataError>> {
        Box::pin(self.read_at(offset, length))
    }
}

trait AnyStore: Send + Sync {
    fn open_any(&self, path: String) -> Pending<'_, Result<Option<MapFile>, DataError>>;
}

impl<S> AnyStore for S
where
    S: ReadPort + Send + Sync,
    S::File: Send + Sync + 'static,
{
    fn open_any(&self, path: String) -> Pending<'_, Result<Option<MapFile>, DataError>> {
        Box::pin(async move {
            let file = self.open(&path).await?;
            Ok(file.map(|file| MapFile(Arc::new(file))))
        })
    }
}

/// An open file of a [`MapStore`].
#[derive(Clone)]
pub struct MapFile(Arc<dyn AnyFile>);

impl RandomAccess for MapFile {
    fn len(&self) -> u64 {
        self.0.file_len()
    }

    fn read_at(
        &self,
        offset: u64,
        length: usize,
    ) -> impl Future<Output = Result<Vec<u8>, DataError>> {
        self.0.read_any(offset, length)
    }
}

/// A store of map packages, whatever kind the host keeps them in.
#[derive(Clone)]
pub struct MapStore(Arc<dyn AnyStore>);

impl std::fmt::Debug for MapStore {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("MapStore")
    }
}

impl MapStore {
    /// A store over `port`.
    pub fn new<S>(port: S) -> Self
    where
        S: ReadPort + Send + Sync + 'static,
        S::File: Send + Sync + 'static,
    {
        Self(Arc::new(port))
    }

    /// The store in directory `root`.
    pub fn directory(root: &str) -> Self {
        Self::new(DirectoryStore::new(root))
    }
}

impl ReadPort for MapStore {
    type File = MapFile;

    fn open(&self, path: &str) -> impl Future<Output = Result<Option<MapFile>, DataError>> {
        self.0.open_any(path.to_owned())
    }
}

/// How the host turns a request's store name into a store.
pub trait MapStores {
    /// The store named `name`.
    fn store(&self, name: &str) -> Result<MapStore, TrialError>;
}

/// Each store name is a directory: the desktop host.
#[derive(Debug, Clone, Copy, Default)]
pub struct DirectoryStores;

impl MapStores for DirectoryStores {
    fn store(&self, name: &str) -> Result<MapStore, TrialError> {
        Ok(MapStore::directory(name))
    }
}

/// Stores the host registered by name, such as a browser page's memory
/// or OPFS stores.
#[derive(Debug, Clone, Default)]
pub struct NamedStores {
    stores: BTreeMap<String, MapStore>,
}

impl NamedStores {
    /// No stores.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers `store` under `name`, replacing any store of that name.
    pub fn insert(&mut self, name: impl Into<String>, store: MapStore) {
        self.stores.insert(name.into(), store);
    }
}

impl MapStores for NamedStores {
    fn store(&self, name: &str) -> Result<MapStore, TrialError> {
        self.stores
            .get(name)
            .cloned()
            .ok_or_else(|| TrialError::Package {
                field: "maps",
                reason: PackageRefusal::Missing,
                detail: format!("no store named `{name}`"),
            })
    }
}
