//! Tile sources over the browser's `fetch`.

use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;
use web_sys::Response;

use super::js_text;
use crate::{HttpAnswer, TileTransport};

/// HTTP through the page's `fetch`. The sources must allow the page's
/// origin to read their answers; the tile servers in the registry do.
#[derive(Debug, Clone, Copy, Default)]
pub struct BrowserTransport;

impl TileTransport for BrowserTransport {
    async fn get(&self, url: &str) -> Result<HttpAnswer, String> {
        let window = web_sys::window().ok_or("the page has no window")?;
        let response = JsFuture::from(window.fetch_with_str(url))
            .await
            .map_err(js_text)?;
        let response: Response = response
            .dyn_into()
            .map_err(|_| "fetch did not answer with a response".to_owned())?;
        let buffer = JsFuture::from(response.array_buffer().map_err(js_text)?)
            .await
            .map_err(js_text)?;
        Ok(HttpAnswer {
            status: response.status(),
            body: js_sys::Uint8Array::new(&buffer).to_vec(),
        })
    }
}
