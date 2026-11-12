//! Ids of the page's progress surface, shared by `index.html`, `boot.mjs`
//! and the browser platform code, and the scripts the page ships beside
//! the app.

/// The surface; its `data-state` is `loading`, `failed`, `done`, `plan` or
/// `refused`.
pub const ROOT: &str = "boot";
/// The column the surface's content sits in; the app adds sections to it.
pub const CARD: &str = "boot-card";
/// The line saying what is happening.
pub const PHASE: &str = "boot-phase";
/// The bar; `data-unknown` on it means the work has no measure.
pub const BAR: &str = "boot-bar";
/// The filled part of the bar.
pub const FILL: &str = "boot-fill";
/// The line of numbers or of an error's text.
pub const DETAIL: &str = "boot-detail";

/// The list of labelled lines: a link's plan, or why it is refused.
pub const LINES: &str = "boot-lines";

/// The worker script, beside the page, that writes the map package's store
/// through sync access handles.
pub const WRITER_SCRIPT: &str = "opfs-writer.js";

#[cfg(test)]
mod tests;
