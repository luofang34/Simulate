//! What the browser lets this origin store.

use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;
use web_sys::{StorageEstimate, StorageManager};

/// The origin's storage as the browser estimates it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageQuota {
    /// The most this origin may store, bytes, when the browser says.
    pub quota_bytes: Option<u64>,
    /// What it stores now, bytes, when the browser says.
    pub usage_bytes: Option<u64>,
    /// Whether the browser has promised not to evict this origin's storage
    /// under pressure.
    pub persistent: bool,
}

impl StorageQuota {
    /// Bytes still free under the quota, when the browser gives both
    /// numbers.
    pub fn free_bytes(&self) -> Option<u64> {
        Some(self.quota_bytes?.saturating_sub(self.usage_bytes?))
    }
}

fn manager() -> Option<StorageManager> {
    Some(web_sys::window()?.navigator().storage())
}

async fn flag(promise: Result<js_sys::Promise, wasm_bindgen::JsValue>) -> bool {
    match promise {
        Ok(promise) => JsFuture::from(promise)
            .await
            .ok()
            .and_then(|value| value.as_bool())
            .unwrap_or(false),
        Err(_) => false,
    }
}

/// The origin's quota and usage; `None` in a browser that gives no
/// estimate.
pub async fn storage_quota() -> Option<StorageQuota> {
    let manager = manager()?;
    let estimate: StorageEstimate = JsFuture::from(manager.estimate().ok()?)
        .await
        .ok()?
        .unchecked_into();
    let bytes = |value: Option<f64>| {
        value
            .filter(|bytes| *bytes >= 0.0)
            .map(|bytes| bytes as u64)
    };
    Some(StorageQuota {
        quota_bytes: bytes(estimate.get_quota()),
        usage_bytes: bytes(estimate.get_usage()),
        persistent: flag(manager.persisted()).await,
    })
}

/// Asks the browser to keep this origin's storage under pressure. Returns
/// whether it agreed; a browser may ask the visitor, or decide by itself.
pub async fn request_persistence() -> bool {
    match manager() {
        Some(manager) => flag(manager.persist()).await,
        None => false,
    }
}
