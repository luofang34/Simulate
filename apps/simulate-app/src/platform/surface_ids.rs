//! Ids of the page's progress surface, shared by `index.html`, `boot.mjs`
//! and the browser platform code.

/// The surface; its `data-state` is `loading`, `failed` or `done`.
pub const ROOT: &str = "boot";
/// The line saying what is happening.
pub const PHASE: &str = "boot-phase";
/// The bar; `data-unknown` on it means the work has no measure.
pub const BAR: &str = "boot-bar";
/// The filled part of the bar.
pub const FILL: &str = "boot-fill";
/// The line of numbers or of an error's text.
pub const DETAIL: &str = "boot-detail";

#[cfg(test)]
mod tests;
