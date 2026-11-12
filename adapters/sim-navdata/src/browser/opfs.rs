//! A `navigate-data` store on the origin private file system.
//!
//! Logical paths map one to one onto files under one directory of the
//! origin's private file system: `chunks/<sha256>`, `packages/<id>.json`,
//! `tiles/<key>`. A file appears whole or not at all: the browser commits a
//! writable stream when it closes. The store lasts across reloads and
//! counts against the origin's storage quota.
//!
//! A writable stream left open keeps a swap file the browser does not clean
//! up, so the store writes one file at a time and aborts the one in hand
//! when the page goes away mid-write.
//!
//! The browser creates a file when its handle is asked for, before the
//! stream commits, so a write cut off leaves an empty or short file. The
//! store keeps a path's first write as the data contract asks, and
//! [`RepairPort::replace`] rewrites one the cache found cut off.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;
use std::sync::Arc;

use navigate_data::{DataError, RandomAccess, ReadPort, WritePort, checked_path};
use wasm_bindgen::closure::Closure;
use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use web_sys::{
    DomException, File, FileSystemDirectoryHandle, FileSystemFileHandle,
    FileSystemGetDirectoryOptions, FileSystemGetFileOptions, FileSystemWritableFileStream,
};

use super::js_text;
use crate::RepairPort;

/// The writable stream a write has open, shared with the page's `pagehide`
/// listener that aborts it.
type OpenStream = Rc<RefCell<Option<FileSystemWritableFileStream>>>;

/// A store under one directory of the origin private file system.
pub struct OpfsStore {
    root: FileSystemDirectoryHandle,
    /// The directories opened so far, by logical path.
    directories: RefCell<BTreeMap<String, FileSystemDirectoryHandle>>,
    /// The stream of the write in progress, if one is.
    open: OpenStream,
    /// Aborts [`Self::open`] when the page is hidden for good; removed with
    /// the store.
    on_leave: Option<Closure<dyn FnMut()>>,
}

/// The `pagehide` listener that aborts `open`'s stream, registered on the
/// page's window; `None` without a window.
fn abort_on_leave(open: &OpenStream) -> Option<Closure<dyn FnMut()>> {
    let window = web_sys::window()?;
    let open = Rc::clone(open);
    let listener: Closure<dyn FnMut()> = Closure::new(move || {
        if let Some(stream) = open.borrow_mut().take() {
            // The page is going; the abort's promise settles without it.
            drop(stream.abort());
        }
    });
    window
        .add_event_listener_with_callback("pagehide", listener.as_ref().unchecked_ref())
        .ok()?;
    Some(listener)
}

impl Drop for OpfsStore {
    fn drop(&mut self) {
        if let (Some(window), Some(listener)) = (web_sys::window(), &self.on_leave) {
            window
                .remove_event_listener_with_callback("pagehide", listener.as_ref().unchecked_ref())
                .ok();
        }
    }
}

/// A file of an [`OpfsStore`], read whole when it was opened.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OpfsFile(Arc<[u8]>);

impl RandomAccess for OpfsFile {
    fn len(&self) -> u64 {
        self.0.len() as u64
    }

    async fn read_at(&self, offset: u64, length: usize) -> Result<Vec<u8>, DataError> {
        usize::try_from(offset)
            .ok()
            .and_then(|start| self.0.get(start..start.checked_add(length)?))
            .map(<[u8]>::to_vec)
            .ok_or_else(|| DataError::Store {
                path: "<origin private file system>".to_owned(),
                message: format!("range {offset}+{length} past {} bytes", self.0.len()),
            })
    }
}

fn store_error(path: &str) -> impl Fn(JsValue) -> DataError + '_ {
    move |error| DataError::Store {
        path: path.to_owned(),
        message: js_text(error),
    }
}

/// Whether the browser says the file or directory is not there.
fn not_found(error: &JsValue) -> bool {
    error
        .dyn_ref::<DomException>()
        .is_some_and(|exception| exception.name() == "NotFoundError")
}

impl OpfsStore {
    /// Opens, creating it if need be, the store in the directory `name` of
    /// this origin's private file system.
    ///
    /// # Errors
    ///
    /// Returns the browser's refusal as text: a browser without the origin
    /// private file system, or a context that may not use it.
    pub async fn open(name: &str) -> Result<Self, String> {
        let window = web_sys::window().ok_or("the page has no window")?;
        let origin = JsFuture::from(window.navigator().storage().get_directory())
            .await
            .map_err(js_text)?;
        let origin: FileSystemDirectoryHandle = origin
            .dyn_into()
            .map_err(|_| "the origin private file system is not a directory".to_owned())?;
        let options = FileSystemGetDirectoryOptions::new();
        options.set_create(true);
        let root = JsFuture::from(origin.get_directory_handle_with_options(name, &options))
            .await
            .map_err(js_text)?
            .dyn_into()
            .map_err(|_| format!("{name} is not a directory"))?;
        let open = OpenStream::default();
        Ok(Self {
            root,
            directories: RefCell::new(BTreeMap::new()),
            on_leave: abort_on_leave(&open),
            open,
        })
    }

    /// The directory holding `path` and the file's name in it; `None` when
    /// the directory is not there and `create` is off.
    async fn directory(
        &self,
        path: &str,
        create: bool,
    ) -> Result<Option<(FileSystemDirectoryHandle, String)>, DataError> {
        checked_path(path)?;
        let (parent, name) = path.rsplit_once('/').unwrap_or(("", path));
        if parent.is_empty() {
            return Ok(Some((self.root.clone(), name.to_owned())));
        }
        if let Some(found) = self.directories.borrow().get(parent) {
            return Ok(Some((found.clone(), name.to_owned())));
        }
        let options = FileSystemGetDirectoryOptions::new();
        options.set_create(create);
        let mut directory = self.root.clone();
        for part in parent.split('/') {
            let next = JsFuture::from(directory.get_directory_handle_with_options(part, &options));
            directory = match next.await {
                Ok(handle) => handle.unchecked_into(),
                Err(error) if not_found(&error) => return Ok(None),
                Err(error) => return Err(store_error(path)(error)),
            };
        }
        self.directories
            .borrow_mut()
            .insert(parent.to_owned(), directory.clone());
        Ok(Some((directory, name.to_owned())))
    }

    /// The handle of the file at `path`, when the store holds it.
    async fn file(&self, path: &str) -> Result<Option<FileSystemFileHandle>, DataError> {
        let Some((directory, name)) = self.directory(path, false).await? else {
            return Ok(None);
        };
        match JsFuture::from(directory.get_file_handle(&name)).await {
            Ok(handle) => Ok(Some(handle.unchecked_into())),
            Err(error) if not_found(&error) => Ok(None),
            Err(error) => Err(store_error(path)(error)),
        }
    }
}

impl ReadPort for OpfsStore {
    type File = OpfsFile;

    async fn open(&self, path: &str) -> Result<Option<OpfsFile>, DataError> {
        let failed = store_error(path);
        let Some(handle) = self.file(path).await? else {
            return Ok(None);
        };
        let file: File = JsFuture::from(handle.get_file())
            .await
            .map_err(&failed)?
            .unchecked_into();
        let buffer = JsFuture::from(file.array_buffer()).await.map_err(&failed)?;
        let bytes = js_sys::Uint8Array::new(&buffer).to_vec();
        Ok(Some(OpfsFile(Arc::from(bytes))))
    }
}

/// Writes `bytes` through `stream` and commits them. The bytes go over in
/// a buffer of their own rather than as a view into the module's memory,
/// so nothing the browser keeps for the write refers to that memory.
async fn write_and_close(
    stream: &FileSystemWritableFileStream,
    bytes: &[u8],
) -> Result<(), JsValue> {
    let copy = js_sys::Uint8Array::from(bytes);
    JsFuture::from(stream.write_with_buffer_source(&copy)?).await?;
    JsFuture::from(stream.close()).await?;
    Ok(())
}

impl OpfsStore {
    /// Writes `bytes` to the file at `path`, created or truncated.
    async fn write_file(&mut self, path: &str, bytes: &[u8]) -> Result<(), DataError> {
        let failed = store_error(path);
        let Some((directory, name)) = self.directory(path, true).await? else {
            return Err(DataError::Missing {
                path: path.to_owned(),
            });
        };
        let options = FileSystemGetFileOptions::new();
        options.set_create(true);
        let handle: FileSystemFileHandle =
            JsFuture::from(directory.get_file_handle_with_options(&name, &options))
                .await
                .map_err(&failed)?
                .unchecked_into();
        let stream: FileSystemWritableFileStream = JsFuture::from(handle.create_writable())
            .await
            .map_err(&failed)?
            .unchecked_into();
        *self.open.borrow_mut() = Some(stream.clone());
        let written = write_and_close(&stream, bytes).await;
        // Taken by the page's leaving when it went first: nothing left to end.
        if self.open.borrow_mut().take().is_none() {
            return Err(failed(JsValue::from_str("the page left during the write")));
        }
        match written {
            Ok(()) => Ok(()),
            Err(error) => {
                // An open stream keeps the browser's swap file for it; only
                // an abort lets it go.
                JsFuture::from(stream.abort()).await.ok();
                Err(failed(error))
            }
        }
    }
}

impl RepairPort for OpfsStore {
    async fn replace(&mut self, path: &str, bytes: &[u8]) -> Result<(), DataError> {
        self.write_file(path, bytes).await
    }
}

impl WritePort for OpfsStore {
    async fn write(&mut self, path: &str, bytes: &[u8]) -> Result<(), DataError> {
        // A path holds content that never changes: the first write stands.
        if self.file(path).await?.is_some() {
            return Ok(());
        }
        self.write_file(path, bytes).await
    }

    async fn contains(&self, path: &str) -> Result<bool, DataError> {
        Ok(self.file(path).await?.is_some())
    }
}
