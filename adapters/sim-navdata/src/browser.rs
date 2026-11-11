//! The browser's ports: fetch for the tile sources, the origin private file
//! system for the store, and the storage quota.

mod fetch;
mod opfs;
mod quota;

pub use fetch::BrowserTransport;
pub use opfs::{OpfsFile, OpfsStore};
pub use quota::{StorageQuota, request_persistence, storage_quota};

use wasm_bindgen::{JsCast, JsValue};

/// A JavaScript error as text: its message when it has one.
fn js_text(value: JsValue) -> String {
    match value.dyn_ref::<js_sys::Error>() {
        Some(error) => format!("{}: {}", error.name(), error.message()),
        None => format!("{value:?}"),
    }
}
