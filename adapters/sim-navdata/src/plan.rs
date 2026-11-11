//! The plan of a map package: Navigate's resource model run for a route and
//! a camera, the tiles it selects, and their size against a budget.

use std::collections::BTreeSet;

use navigate_data::{
    GroundHeight, PlannedCamera, PlannedLeg, PlannedTiles, ResourcePlan, ResourceRequest,
    SizeEstimate, TileKey, estimate_size, plan_resources, plan_tiles,
};

use crate::{NavdataError, SourceSet};

#[cfg(test)]
mod tests;

/// Largest search radius about the navigation estimate, metres: the value
/// the desktop runs plan with.
pub const SEARCH_RADIUS_M: f64 = 500.0;
/// Allowed deviation from the planned track, metres: the value the desktop
/// runs plan with.
pub const DEVIATION_M: f64 = 500.0;
/// Farthest slant range the navigator renders or matches, metres: the value
/// the desktop runs plan with.
pub const MAX_RANGE_M: f64 = 4_000.0;
/// Zoom levels between the imagery and the elevation of one band: the value
/// of Navigate's package tool.
pub const ELEVATION_OFFSET: u32 = 3;

/// A camera as the resource model sees it, from its image size, horizontal
/// field of view and depression, the way Navigate's package tool derives
/// the focal length.
pub fn planned_camera(
    width_px: u32,
    height_px: u32,
    horizontal_fov_deg: f64,
    depression_deg: f64,
) -> PlannedCamera {
    PlannedCamera {
        focal_px: f64::from(width_px) / 2.0 / (horizontal_fov_deg.to_radians() / 2.0).tan(),
        width_px,
        height_px,
        depression_rad: depression_deg.to_radians(),
    }
}

/// What a route and a camera need of the sources.
#[derive(Debug, Clone, PartialEq)]
pub struct PackagePlan {
    /// The inputs of the resource model.
    pub request: ResourceRequest,
    /// What each leg needs: its height above ground, zooms and strips.
    pub resources: ResourcePlan,
    /// The tiles selected, by layer.
    pub tiles: PlannedTiles,
    /// Their count by layer and zoom, with the estimate and the upper
    /// bound in bytes.
    pub size: SizeEstimate,
}

impl PackagePlan {
    /// Tiles selected.
    pub fn tile_count(&self) -> usize {
        self.tiles.values().map(BTreeSet::len).sum()
    }

    /// The key of every selected tile at `sources`, in layer then address
    /// order.
    pub fn keys(&self, sources: &SourceSet) -> Vec<TileKey> {
        self.tiles
            .iter()
            .flat_map(|(kind, addresses)| {
                addresses
                    .iter()
                    .map(|address| sources.tile_key(*kind, *address))
            })
            .collect()
    }

    /// Judges the plan against a storage budget on its upper bound, before
    /// any tile of it is fetched.
    ///
    /// # Errors
    ///
    /// Returns [`NavdataError::OverBudget`] with what the plan needs.
    pub fn within(&self, budget_bytes: u64) -> Result<(), NavdataError> {
        if self.size.fits(budget_bytes) {
            return Ok(());
        }
        Err(NavdataError::OverBudget {
            needed_bytes: self.size.upper_bytes,
            budget_bytes,
            tiles: self.tile_count(),
        })
    }
}

/// Plans the package for the route through `points`, each `[lat, lon,
/// height]` in degrees and metres above mean sea level, the reference of
/// `ground`, seen by `camera`.
///
/// # Errors
///
/// Returns Navigate's refusal: fewer than two points, ground unknown on a
/// leg, a leg below the ground, a corridor past the Web Mercator latitudes.
pub fn plan_package<G: GroundHeight + ?Sized>(
    points: &[[f64; 3]],
    camera: PlannedCamera,
    sources: &SourceSet,
    ground: &G,
) -> Result<PackagePlan, NavdataError> {
    let legs: Vec<PlannedLeg> = points
        .windows(2)
        .filter_map(|pair| match pair {
            [from, to] => Some(PlannedLeg {
                from: *from,
                to: *to,
            }),
            _ => None,
        })
        .collect();
    let request = ResourceRequest {
        legs,
        cameras: vec![camera],
        search_radius_m: SEARCH_RADIUS_M,
        deviation_m: DEVIATION_M,
        max_range_m: MAX_RANGE_M,
        imagery: sources.imagery.limits,
        elevation: sources.elevation.limits,
        elevation_offset: ELEVATION_OFFSET,
    };
    let resources = plan_resources(&request, ground)
        .map_err(NavdataError::data("planning what each leg needs"))?;
    let tiles = plan_tiles(&request.legs, &resources)
        .map_err(NavdataError::data("selecting the tiles of the plan"))?;
    let size = estimate_size(&request, &tiles);
    Ok(PackagePlan {
        request,
        resources,
        tiles,
        size,
    })
}
