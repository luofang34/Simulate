//! The map package's status as a page shows it: a fixed number of short
//! lines in every stage, so the block keeps its height from planning to
//! sealing, and the plan's lines under it do not move.

use sim_navdata::PrefetchProgress;

use super::megabytes;

#[cfg(test)]
mod tests;

/// Lines in the map package's status block, in every stage.
pub const PACKAGE_LINES: usize = 2;

/// Where the map package is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PackageStage<'a> {
    /// No set of the link uses the terrain camera.
    NotNeeded,
    /// Reading the terrain height the plan needs.
    ReadingGround,
    /// The plan stands and waits for the visitor to start the fetch.
    PlanReady,
    /// Fetching the plan's tiles.
    Fetching(&'a PrefetchProgress),
    /// Navigate checks every tile and writes the manifest.
    Sealing,
    /// Sealed and loaded; `stored` when the browser's storage keeps it.
    Sealed {
        /// Whether the browser's storage keeps the package for a reload.
        stored: bool,
    },
    /// Not built; the reason stands in the section's lines.
    Refused,
}

/// The status block of `stage`.
#[must_use]
pub fn package_lines(stage: PackageStage<'_>) -> [String; PACKAGE_LINES] {
    let pair = |first: &str, second: &str| [first.to_owned(), second.to_owned()];
    match stage {
        PackageStage::NotNeeded => pair("No map package", "No set uses the terrain camera"),
        PackageStage::ReadingGround => pair("Planning the package", "Reading terrain on the route"),
        PackageStage::PlanReady => pair("Plan above", "Nothing fetched yet"),
        PackageStage::Fetching(progress) => [
            format!(
                "Fetching {} / {} tiles · {}",
                progress.done,
                progress.total,
                megabytes(progress.bytes)
            ),
            format!(
                "Reused {} · not at source {}",
                progress.reused, progress.gaps
            ),
        ],
        PackageStage::Sealing => pair("Sealing the package", "Navigate checks every tile"),
        PackageStage::Sealed { stored: true } => pair(
            "Sealed · loaded for the navigator",
            "Kept in this browser's storage",
        ),
        PackageStage::Sealed { stored: false } => pair(
            "Sealed · loaded for the navigator",
            "In memory only · refetched on reload",
        ),
        PackageStage::Refused => pair("Not built", "Reason below"),
    }
}
