use std::f64::consts::TAU;

use sim_geo::Geodetic;

use crate::noise::{lattice_unit, value_noise};
use crate::{
    CatalogScenery, CatalogVerticalDatum, ImageryRaster, SceneryError, TerrariumTile, TileId,
    encode_terrarium, project_to_tile, tile_point_to_geodetic,
};

#[cfg(test)]
mod tests;

/// Field-pattern cell size in imagery pixels: small enough that a mounted
/// camera always has several distinct fields in view.
const FIELD_CELL_PX: f64 = 14.0;
/// Road spacing in imagery pixels.
const ROAD_SPACING_PX: i64 = 96;
/// Road width in imagery pixels.
const ROAD_WIDTH_PX: i64 = 2;
/// Field surface colours; adjacent fields contrast for feature matching.
const FIELD_RGB: [[f64; 3]; 5] = [
    [58.0, 94.0, 44.0],
    [122.0, 140.0, 62.0],
    [166.0, 148.0, 96.0],
    [40.0, 66.0, 40.0],
    [190.0, 178.0, 132.0],
];

/// Lays out a block of synthetic corridor tiles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SyntheticCorridorSpec {
    /// A point in the south-west tile.
    pub south_west: Geodetic,
    /// Zoom of both elevation and imagery tiles.
    pub zoom: u8,
    /// Tiles east of the south-west tile, inclusive.
    pub tiles_east: u32,
    /// Tiles north of the south-west tile, inclusive.
    pub tiles_north: u32,
    /// Pixels along a tile edge.
    pub tile_px: u32,
    /// Selects the terrain and imagery pattern.
    pub seed: u64,
}

impl SyntheticCorridorSpec {
    /// A corridor over central Delaware, about 20 × 10 km.
    pub fn delaware() -> Self {
        Self {
            south_west: Geodetic::from_degrees(39.05, -75.70, 0.0),
            zoom: 12,
            tiles_east: 3,
            tiles_north: 2,
            tile_px: 256,
            seed: 0xDE1A_3A4E,
        }
    }

    /// Eastward extension of the Delaware corridor covering Dover AFB (`KDOV`).
    pub fn dover_kdov() -> Self {
        Self {
            south_west: Geodetic::from_degrees(39.05, -75.49, 0.0),
            zoom: 12,
            tiles_east: 2,
            tiles_north: 2,
            tile_px: 256,
            seed: 0xDE1A_3A4E,
        }
    }

    /// Central Jersey Regional Airport (`47N`, Manville, NJ — uncontrolled CTAF) and pattern airspace.
    pub fn central_jersey_47n() -> Self {
        Self {
            south_west: Geodetic::from_degrees(40.48, -74.68, 0.0),
            zoom: 12,
            tiles_east: 3,
            tiles_north: 2,
            tile_px: 256,
            seed: 0x0047_4E4A,
        }
    }

    /// Newark Liberty International Airport (`KEWR` — Class B towered) and Newark Bay pattern airspace.
    pub fn newark_kewr() -> Self {
        Self {
            south_west: Geodetic::from_degrees(40.62, -74.24, 0.0),
            zoom: 12,
            tiles_east: 3,
            tiles_north: 3,
            tile_px: 256,
            seed: 0x4B45_5752,
        }
    }
}

/// Graded flat aerodrome plateau blended smoothly into surrounding synthetic relief.
#[derive(Debug, Clone, Copy)]
struct AerodromePlateau {
    centre: Geodetic,
    elevation_m: f64,
    flat_radius_m: f64,
    blend_collar_m: f64,
}

impl CatalogScenery {
    /// A fully resident block of synthetic tiles: rolling ridge relief and
    /// non-repeating high-contrast imagery, the same on every run. Heights go
    /// through Terrarium encoding, as a downloaded tile's would. Synthetic
    /// heights, here and in the airport catalogs, are ellipsoidal.
    pub fn synthetic_corridor(spec: &SyntheticCorridorSpec) -> Result<Self, SceneryError> {
        let invalid = |detail: &str| SceneryError::InvalidCatalog {
            detail: detail.to_owned(),
        };
        let origin = project_to_tile(&spec.south_west, spec.zoom)
            .ok_or_else(|| invalid("corridor origin has no tile"))?
            .tile;
        if spec.tile_px == 0 || spec.tiles_east == 0 || spec.tiles_north == 0 {
            return Err(invalid("corridor has no pixels"));
        }
        let mut catalog = Self::new().with_vertical_datum(CatalogVerticalDatum::EllipsoidWgs84);
        for north in 0..spec.tiles_north {
            let y = origin
                .y
                .checked_sub(north)
                .ok_or_else(|| invalid("corridor runs off the top of the map"))?;
            for east in 0..spec.tiles_east {
                let tile = TileId {
                    zoom: spec.zoom,
                    x: origin.x + east,
                    y,
                };
                let (elevation, imagery) = synthetic_tile(tile, spec, &[])?;
                catalog.insert_elevation(tile, elevation)?;
                catalog.insert_imagery(tile, imagery)?;
            }
        }
        Ok(catalog)
    }

    /// Builds a multi-airport regional catalog covering the Delaware corridor, Dover (`KDOV`),
    /// Central Jersey Regional (`47N` uncontrolled), and Newark Liberty (`KEWR` towered),
    /// with flat-graded airport plateaus so runways and taxiways drape cleanly.
    pub fn synthetic_regional_network() -> Result<Self, SceneryError> {
        let plateaus = [
            AerodromePlateau {
                centre: Geodetic::from_degrees(39.129_5, -75.466_0, 0.0),
                elevation_m: crate::AerodromeId::DoverKdov.field_elevation_msl_m(),
                flat_radius_m: 2_600.0,
                blend_collar_m: 1_800.0,
            },
            AerodromePlateau {
                centre: Geodetic::from_degrees(40.524_5, -74.598_4, 0.0),
                elevation_m: crate::AerodromeId::CentralJersey47N.field_elevation_msl_m(),
                flat_radius_m: 1_500.0,
                blend_collar_m: 1_500.0,
            },
            AerodromePlateau {
                centre: Geodetic::from_degrees(40.692_5, -74.168_7, 0.0),
                elevation_m: crate::AerodromeId::NewarkKewr.field_elevation_msl_m(),
                flat_radius_m: 3_400.0,
                blend_collar_m: 2_000.0,
            },
        ];
        let mut catalog = Self::new().with_vertical_datum(CatalogVerticalDatum::EllipsoidWgs84);
        for spec in [
            SyntheticCorridorSpec::delaware(),
            SyntheticCorridorSpec::dover_kdov(),
            SyntheticCorridorSpec::central_jersey_47n(),
            SyntheticCorridorSpec::newark_kewr(),
        ] {
            append_corridor_into(&mut catalog, &spec, &plateaus)?;
        }
        Ok(catalog)
    }

    /// Builds a single flat-graded airport tile block for `aerodrome`.
    pub fn synthetic_aerodrome(aerodrome: crate::AerodromeId) -> Result<Self, SceneryError> {
        let (spec, plateau) = match aerodrome {
            crate::AerodromeId::DoverKdov => (
                SyntheticCorridorSpec::dover_kdov(),
                AerodromePlateau {
                    centre: Geodetic::from_degrees(39.129_5, -75.466_0, 0.0),
                    elevation_m: crate::AerodromeId::DoverKdov.field_elevation_msl_m(),
                    flat_radius_m: 2_600.0,
                    blend_collar_m: 1_800.0,
                },
            ),
            crate::AerodromeId::CentralJersey47N => (
                SyntheticCorridorSpec::central_jersey_47n(),
                AerodromePlateau {
                    centre: Geodetic::from_degrees(40.524_5, -74.598_4, 0.0),
                    elevation_m: crate::AerodromeId::CentralJersey47N.field_elevation_msl_m(),
                    flat_radius_m: 1_500.0,
                    blend_collar_m: 1_500.0,
                },
            ),
            crate::AerodromeId::NewarkKewr => (
                SyntheticCorridorSpec::newark_kewr(),
                AerodromePlateau {
                    centre: Geodetic::from_degrees(40.692_5, -74.168_7, 0.0),
                    elevation_m: crate::AerodromeId::NewarkKewr.field_elevation_msl_m(),
                    flat_radius_m: 3_400.0,
                    blend_collar_m: 2_000.0,
                },
            ),
        };
        let mut catalog = Self::new().with_vertical_datum(CatalogVerticalDatum::EllipsoidWgs84);
        append_corridor_into(&mut catalog, &spec, &[plateau])?;
        Ok(catalog)
    }
}

fn append_corridor_into(
    catalog: &mut CatalogScenery,
    spec: &SyntheticCorridorSpec,
    plateaus: &[AerodromePlateau],
) -> Result<(), SceneryError> {
    let invalid = |detail: &str| SceneryError::InvalidCatalog {
        detail: detail.to_owned(),
    };
    let origin = project_to_tile(&spec.south_west, spec.zoom)
        .ok_or_else(|| invalid("corridor origin has no tile"))?
        .tile;
    for north in 0..spec.tiles_north {
        let y = origin
            .y
            .checked_sub(north)
            .ok_or_else(|| invalid("corridor runs off the top of the map"))?;
        for east in 0..spec.tiles_east {
            let tile = TileId {
                zoom: spec.zoom,
                x: origin.x + east,
                y,
            };
            let (elevation, imagery) = synthetic_tile(tile, spec, plateaus)?;
            catalog.insert_elevation(tile, elevation)?;
            catalog.insert_imagery(tile, imagery)?;
        }
    }
    Ok(())
}

fn synthetic_tile(
    tile: TileId,
    spec: &SyntheticCorridorSpec,
    plateaus: &[AerodromePlateau],
) -> Result<(TerrariumTile, ImageryRaster), SceneryError> {
    let size = spec.tile_px;
    let pixels = size as usize * size as usize;
    let mut terrarium = Vec::with_capacity(pixels * 3);
    let mut rgba8 = Vec::with_capacity(pixels * 4);
    for y in 0..size {
        for x in 0..size {
            let u = (f64::from(x) + 0.5) / f64::from(size);
            let v = (f64::from(y) + 0.5) / f64::from(size);
            let position =
                tile_point_to_geodetic(tile, u, v).ok_or(SceneryError::TileOutOfRange { tile })?;
            let raw_h = ridge_height_m(&position, spec.seed);
            let graded_h = apply_plateaus(&position, raw_h, plateaus);
            terrarium.extend_from_slice(&encode_terrarium(graded_h));
            let global_x = i64::from(tile.x) * i64::from(size) + i64::from(x);
            let global_y = i64::from(tile.y) * i64::from(size) + i64::from(y);
            rgba8.extend_from_slice(&ground_colour(global_x, global_y, spec.seed));
        }
    }
    Ok((
        TerrariumTile::from_rgb(size, size, &terrarium)?,
        ImageryRaster::new(size, size, rgba8)?,
    ))
}

fn apply_plateaus(position: &Geodetic, raw_height_m: f64, plateaus: &[AerodromePlateau]) -> f64 {
    let mut h = raw_height_m;
    for p in plateaus {
        let d_north_m = (position.lat_rad - p.centre.lat_rad) * 6_378_137.0;
        let d_east_m = (position.lon_rad - p.centre.lon_rad) * 6_378_137.0 * p.centre.lat_rad.cos();
        let dist_m = d_north_m.hypot(d_east_m);
        if dist_m <= p.flat_radius_m {
            return p.elevation_m;
        }
        let outer_m = p.flat_radius_m + p.blend_collar_m;
        if dist_m < outer_m {
            let frac = (dist_m - p.flat_radius_m) / p.blend_collar_m.max(1.0);
            let smooth = 0.5 - 0.5 * (std::f64::consts::PI * frac).cos();
            h = p.elevation_m * (1.0 - smooth) + h * smooth;
        }
    }
    h
}

/// Rolling ridges running roughly north-east, tens of metres high, over
/// broader swells. Pure in position, so tiles agree on their shared edges.
fn ridge_height_m(position: &Geodetic, seed: u64) -> f64 {
    // Radians to kilometres, near enough for a pattern that only has to look
    // like terrain.
    let east_km = position.lon_rad * 6_378.0 * position.lat_rad.cos();
    let north_km = position.lat_rad * 6_378.0;
    let across = (east_km - 0.6 * north_km) / 3.0;
    let ridge = (TAU * across).sin() * 0.5 + 0.5;
    let swell = value_noise(seed, east_km / 8.0, north_km / 8.0);
    let detail = value_noise(seed ^ 1, east_km / 0.9, north_km / 0.9);
    25.0 + 45.0 * ridge * ridge + 40.0 * swell + 6.0 * detail
}

/// One imagery pixel, from its position on the world pixel grid: fields in a
/// warped patchwork, crossed by a road grid, with per-pixel speckle.
fn ground_colour(x: i64, y: i64, seed: u64) -> [u8; 4] {
    if x.rem_euclid(ROAD_SPACING_PX) < ROAD_WIDTH_PX
        || y.rem_euclid(ROAD_SPACING_PX) < ROAD_WIDTH_PX
    {
        return [214, 212, 206, 255];
    }
    let (fx, fy) = (x as f64, y as f64);
    let warp_x = value_noise(seed ^ 2, fx / 40.0, fy / 40.0) * 2.0 * FIELD_CELL_PX;
    let warp_y = value_noise(seed ^ 3, fx / 40.0, fy / 40.0) * 2.0 * FIELD_CELL_PX;
    let cell_x = ((fx + warp_x) / FIELD_CELL_PX).floor() as i64;
    let cell_y = ((fy + warp_y) / FIELD_CELL_PX).floor() as i64;
    let palette = lattice_unit(seed ^ 4, cell_x, cell_y) * FIELD_RGB.len() as f64;
    let [r, g, b] = FIELD_RGB
        .get(palette as usize)
        .copied()
        .unwrap_or([0.0, 0.0, 0.0]);
    let speckle = 0.82 + 0.36 * lattice_unit(seed ^ 5, x, y);
    let channel = |value: f64| (value * speckle).clamp(0.0, 255.0) as u8;
    [channel(r), channel(g), channel(b), 255]
}
