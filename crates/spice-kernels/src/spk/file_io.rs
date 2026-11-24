//! File-system entry points; everything else works on byte slices.

use std::path::Path;

use super::SpkFile;
use crate::error::Error;

impl SpkFile {
    /// Reads and parses the kernel at `path`.
    pub fn read_file_blocking(path: impl AsRef<Path>) -> Result<Self, Error> {
        let path = path.as_ref();
        let bytes = std::fs::read(path).map_err(|source| Error::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Self::parse(&bytes)
    }

    /// Encodes the kernel and writes it to `path`, replacing any file there.
    pub fn write_file_blocking(&self, path: impl AsRef<Path>) -> Result<(), Error> {
        let path = path.as_ref();
        let bytes = self.to_bytes()?;
        std::fs::write(path, bytes).map_err(|source| Error::Io {
            path: path.to_path_buf(),
            source,
        })
    }
}
