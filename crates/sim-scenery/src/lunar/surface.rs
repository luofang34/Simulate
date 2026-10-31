use super::{LITTLE_WEST_CRATER, LunarCrater, WEST_CRATER, rolling_m};
use crate::noise::{lattice_hash, lattice_unit};

/// Mature mare regolith.
const REGOLITH_ALBEDO: f32 = 0.08;
const REGOLITH_EMISSIVITY: f32 = 0.95;
/// Freshly excavated ejecta, brighter than the weathered plain.
const EJECTA_ALBEDO: f32 = 0.13;
/// Bare rock.
pub(super) const ROCK_ALBEDO: f32 = 0.16;
const ROCK_EMISSIVITY: f32 = 0.90;

/// Depth of a simple crater's floor below its rim, as a fraction of diameter.
const DEPTH_TO_DIAMETER: f64 = 0.2;
/// Height of the raised rim, as a fraction of diameter.
const RIM_TO_DIAMETER: f64 = 0.04;

/// Boulders sit on a jittered grid so a query only inspects nearby cells.
const BOULDER_CELL_M: f64 = 4.0;
const BOULDER_SEED: u64 = 0xB01D_E125;
/// Boulder radii, metres. The largest must stay under one grid cell, the
/// reach of a query.
const MAX_BOULDER_RADIUS_M: f64 = 3.0;
const MIN_BOULDER_RADIUS_M: f64 = 0.6;
/// Share of grid cells holding a boulder on the open plain.
const PLAIN_BOULDER_DENSITY: f64 = 0.002;
/// Share of grid cells holding a boulder on the crater's ejecta blanket.
const EJECTA_BOULDER_DENSITY: f64 = 0.7;
/// The ejecta blanket reaches this many crater radii from the centre.
const EJECTA_REACH_RADII: f64 = 2.6;
/// The clear ground around the Lunar Module's touchdown point, metres.
const CLEAR_PAD_RADIUS_M: f64 = 45.0;

/// What is on the ground at one point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Ground {
    pub(super) height_m: f64,
    pub(super) albedo: f32,
    pub(super) emissivity: f32,
}

/// Height of a crater's contribution at `distance` from its centre: a
/// parabolic bowl inside the rim, an ejecta apron falling away outside.
fn crater_relief_m(crater: &LunarCrater, distance: f64) -> f64 {
    let radius = crater.diameter_m / 2.0;
    let depth = crater.diameter_m * DEPTH_TO_DIAMETER;
    let rim = crater.diameter_m * RIM_TO_DIAMETER;
    let rho = distance / radius;
    if rho < 1.0 {
        -depth + (depth + rim) * rho * rho
    } else {
        rim * rho.powi(-3)
    }
}

fn distance_to(crater: &LunarCrater, east: f64, north: f64) -> f64 {
    (east - crater.east_m).hypot(north - crater.north_m)
}

/// Bare terrain without boulders: the part a low Sun's long shadows depend on.
pub(super) fn terrain_height_m(east: f64, north: f64) -> f64 {
    rolling_m(east, north)
        + [WEST_CRATER, LITTLE_WEST_CRATER]
            .iter()
            .map(|crater| crater_relief_m(crater, distance_to(crater, east, north)))
            .sum::<f64>()
}

/// Fraction of grid cells that hold a boulder around a cell centre.
fn boulder_density(east: f64, north: f64) -> f64 {
    if east.hypot(north) < CLEAR_PAD_RADIUS_M {
        return 0.0;
    }
    let rho = 2.0 * distance_to(&WEST_CRATER, east, north) / WEST_CRATER.diameter_m;
    if (0.9..EJECTA_REACH_RADII).contains(&rho) {
        // Blocks thin out with distance from the rim, as ballistic ejecta do.
        let thinning = (EJECTA_REACH_RADII - rho) / (EJECTA_REACH_RADII - 0.9);
        return PLAIN_BOULDER_DENSITY + EJECTA_BOULDER_DENSITY * thinning;
    }
    PLAIN_BOULDER_DENSITY
}

/// A boulder's height above the terrain at a point, or `None` if the point is
/// not on one. Each boulder is a spheroidal dome, decided only by its own
/// cell's hash so queries in any order agree.
fn boulder_height_m(east: f64, north: f64) -> Option<f64> {
    let cell_x = (east / BOULDER_CELL_M).floor() as i64;
    let cell_y = (north / BOULDER_CELL_M).floor() as i64;
    (cell_x - 1..=cell_x + 1)
        .flat_map(|x| (cell_y - 1..=cell_y + 1).map(move |y| (x, y)))
        .filter_map(|(x, y)| boulder_dome_m(x, y, east, north))
        .reduce(f64::max)
}

fn boulder_dome_m(cell_x: i64, cell_y: i64, east: f64, north: f64) -> Option<f64> {
    let hash = lattice_hash(BOULDER_SEED, cell_x, cell_y);
    let jitter = |shift: u32| f64::from((hash >> shift) as u16) / 65_536.0;
    let centre_east = (cell_x as f64 + jitter(0)) * BOULDER_CELL_M;
    let centre_north = (cell_y as f64 + jitter(16)) * BOULDER_CELL_M;
    if lattice_unit(BOULDER_SEED ^ 1, cell_x, cell_y) >= boulder_density(centre_east, centre_north)
    {
        return None;
    }
    let size = lattice_unit(BOULDER_SEED ^ 2, cell_x, cell_y);
    // Cubing skews the population toward small rocks, as blocks fragment.
    let radius =
        MIN_BOULDER_RADIUS_M + (MAX_BOULDER_RADIUS_M - MIN_BOULDER_RADIUS_M) * size * size * size;
    let along = (east - centre_east).hypot(north - centre_north) / radius;
    (along < 1.0).then(|| 0.8 * radius * (1.0 - along * along).sqrt())
}

/// The surface at a local point, with its materials.
pub(super) fn ground_at(east: f64, north: f64) -> Ground {
    let terrain = terrain_height_m(east, north);
    if let Some(boulder) = boulder_height_m(east, north) {
        return Ground {
            height_m: terrain + boulder,
            albedo: ROCK_ALBEDO,
            emissivity: ROCK_EMISSIVITY,
        };
    }
    // Excavated material lightens the plain gradually, thinning with its
    // distance from the rim.
    let rho = 2.0 * distance_to(&WEST_CRATER, east, north) / WEST_CRATER.diameter_m;
    let ejecta = ((EJECTA_REACH_RADII - rho) / (EJECTA_REACH_RADII - 1.0)).clamp(0.0, 1.0) as f32;
    Ground {
        height_m: terrain,
        albedo: REGOLITH_ALBEDO + (EJECTA_ALBEDO - REGOLITH_ALBEDO) * ejecta,
        emissivity: REGOLITH_EMISSIVITY,
    }
}
