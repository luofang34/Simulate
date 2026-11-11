//! The two ways the app starts and obtains its renderer: blocking on
//! desktop, asynchronously in the browser (which cannot block on GPU setup).
//! Both hand the finished renderer back through the event loop, so the app
//! has a single code path for "renderer arrived".

#[cfg(not(target_arch = "wasm32"))]
mod native;
#[cfg(any(target_arch = "wasm32", test))]
mod surface_ids;
#[cfg(target_arch = "wasm32")]
mod web;

#[cfg(not(target_arch = "wasm32"))]
pub use native::{report_failure, run, set_title, spawn_renderer, startup_done, window_attributes};
#[cfg(target_arch = "wasm32")]
pub use web::{report_failure, run, set_title, spawn_renderer, startup_done, window_attributes};
