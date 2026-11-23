//! Synthetic 3D terrain mesh generator, HTAWS relative-elevation shading, and 3D runway /
//! Highway-in-the-Sky approach overlays (`sim-scenery` + `sim-geo`) on an explicitly declared
//! [`PlanetaryReference`].

/// Global `AirportDatabase` destination/approach runway resolver.
pub mod runways;
/// Surveyed airport runway GeoJSON `FeatureCollection` builder.
pub mod runways_geojson;

use crate::error::FlightDeckError;
use crate::planet::{PlanetaryBodyId, PlanetaryReference};
use crate::telemetry::{FEET_TO_METRES, OwnshipState};
use glam::DVec3;
use indicate_instrument_hud::Runway as HudRunway;
use runways::{ApproachRunwaySelection, RunwayEyeAnchor, resolve_approach_and_overlays};
use sim_geo::Geodetic;
use sim_scenery::{CatalogScenery, ElevationSource, SyntheticCorridorSpec};

const GRID_HALF_STEPS: i32 = 16; // 33 x 33 vertex grid (2,048 triangles)
const GRID_STEP_M: f64 = 550.0; // ~17.6 km x 17.6 km 3D synthetic terrain carpet

/// 3D charted runway strip and extended approach gates in ownship-centered East-North-Up metres.
#[derive(Clone, Debug, PartialEq)]
pub struct RunwayOverlay3D {
    /// Aerodrome and runway label (`"47N RWY 07"` or `"KEWR RWY 22L"`).
    pub label: String,
    /// Four corners of the paved/usable runway strip in ENU metres (`[east, north, up]`).
    pub corners_enu_m: [[f32; 3]; 4],
    /// Extended centerline line segment (`[threshold_enu, approach_fix_enu]`).
    pub centerline_enu_m: [[f32; 3]; 2],
    /// Highway-in-the-Sky (`HITS`) `3.0°` glideslope gate rectangles in ENU metres.
    pub glideslope_gates_enu_m: Vec<[[f32; 3]; 4]>,
}

/// Ownship-centered 3D synthetic terrain patch with HTAWS relative-elevation vertex colors.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TerrainMesh {
    /// Vertex positions in local East-North-Up metres (`[east_m, north_m, up_m]`).
    pub positions_enu_m: Vec<[f32; 3]>,
    /// Unit surface normals in local East-North-Up (`[nx, ny, nz]`).
    pub normals_enu: Vec<[f32; 3]>,
    /// Per-vertex HTAWS + hypsometric RGBA8 colors (`[r, g, b, a]`).
    pub colors_rgba: Vec<[u8; 4]>,
    /// Counter-clockwise triangle indices (`3 * tri_count`).
    pub indices: Vec<u32>,
    /// Ground elevation directly beneath ownship in metres MSL.
    pub ground_elevation_msl_m: f64,
    /// Height above ground level (`AGL`) in feet when ownship altitude is valid.
    pub agl_ft: Option<f64>,
    /// 3D runway strips and approach gates within range.
    pub runways: Vec<RunwayOverlay3D>,
}

/// Terrain & runway generator backed by `sim-scenery`'s `CatalogScenery` DEMs and global `AirportDatabase`.
#[derive(Clone, Debug)]
pub struct SyntheticTerrainGenerator {
    scenery_47n: CatalogScenery,
    scenery_kewr: CatalogScenery,
}

impl SyntheticTerrainGenerator {
    /// Initializes resident `47N` and `KEWR` `CatalogScenery` DEMs.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Terrain`] if corridor generation fails.
    pub fn builtin_ny_nj() -> Result<Self, FlightDeckError> {
        let scenery_47n =
            CatalogScenery::synthetic_corridor(&SyntheticCorridorSpec::central_jersey_47n())
                .map_err(|e| FlightDeckError::Terrain {
                    reason: e.to_string(),
                })?;
        let scenery_kewr =
            CatalogScenery::synthetic_corridor(&SyntheticCorridorSpec::newark_kewr()).map_err(
                |e| FlightDeckError::Terrain {
                    reason: e.to_string(),
                },
            )?;
        Ok(Self {
            scenery_47n,
            scenery_kewr,
        })
    }

    /// Queries ground elevation in metres above `planet`'s declared vertical datum at
    /// `(lat_deg, lon_deg)`. Reads resident `CatalogScenery` Terrarium tiles on `WGS-84` Earth,
    /// and deterministic regolith/crater relief on non-Earth bodies (`Moon`, `Mars`).
    #[must_use]
    pub fn ground_elevation_datum_m(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        planet: &PlanetaryReference,
    ) -> f64 {
        if !planet.is_earth_wgs84() {
            return analytic_planetary_relief_m(lat_deg, lon_deg, planet.id);
        }
        let geo = Geodetic::from_degrees(lat_deg, lon_deg, 0.0);
        if let Ok(Some(h)) = self.scenery_47n.ground_height_m(&geo) {
            return h.max(0.0);
        }
        if let Ok(Some(h)) = self.scenery_kewr.ground_height_m(&geo) {
            return h.max(0.0);
        }
        analytic_nj_corridor_relief_m(lat_deg, lon_deg)
    }

    /// Generates the ownship-centered 3D ENU [`TerrainMesh`] and nearest conformal [`HudRunway`]
    /// on the explicitly declared [`PlanetaryReference`].
    #[must_use]
    pub fn build_mesh(
        &self,
        ownship: &OwnshipState,
        planet: &PlanetaryReference,
    ) -> (TerrainMesh, Option<HudRunway>) {
        let (mesh, approach) = self.build_mesh_with_eye(ownship, planet, None, None);
        (mesh, approach.map(|a| a.hud_runway))
    }

    /// Generates the ownship-centered [`TerrainMesh`] and active [`ApproachRunwaySelection`]
    /// projected from `eye_override` (when provided) or ownship's cockpit anchor.
    #[must_use]
    pub fn build_mesh_with_eye(
        &self,
        ownship: &OwnshipState,
        planet: &PlanetaryReference,
        eye_override: Option<RunwayEyeAnchor>,
        route_label: Option<&str>,
    ) -> (TerrainMesh, Option<ApproachRunwaySelection>) {
        let Some(own_geo) = ownship.valid_body_geodetic(planet) else {
            return (TerrainMesh::default(), None);
        };
        let own_lat_deg = own_geo.lat_rad.to_degrees();
        let own_lon_deg = own_geo.lon_rad.to_degrees();
        let own_alt_datum_m = ownship
            .best_altitude_m()
            .unwrap_or_else(|| planet.height_above_datum_m(&own_geo));
        let center_ground_m = eye_override
            .and_then(|e| e.onboard_dem_datum_m)
            .filter(|v| v.is_finite())
            .unwrap_or_else(|| self.ground_elevation_datum_m(own_lat_deg, own_lon_deg, planet));
        let agl_ft = Some((own_alt_datum_m - center_ground_m) / FEET_TO_METRES);

        let flat_origin = planet.geodetic_from_datum(own_lat_deg, own_lon_deg, 0.0);

        let side = (GRID_HALF_STEPS * 2 + 1) as usize;
        let vert_count = side * side;
        let mut heights_datum = Vec::with_capacity(vert_count);
        let mut positions = Vec::with_capacity(vert_count);

        for iy in -GRID_HALF_STEPS..=GRID_HALF_STEPS {
            let north_m = f64::from(iy) * GRID_STEP_M;
            for ix in -GRID_HALF_STEPS..=GRID_HALF_STEPS {
                let east_m = f64::from(ix) * GRID_STEP_M;
                let sample_geo =
                    planet.offset_geodetic_enu(&flat_origin, DVec3::new(east_m, north_m, 0.0));
                let h_datum = self.ground_elevation_datum_m(
                    sample_geo.lat_rad.to_degrees(),
                    sample_geo.lon_rad.to_degrees(),
                    planet,
                );
                heights_datum.push(h_datum);
                positions.push([
                    east_m as f32,
                    north_m as f32,
                    (h_datum - own_alt_datum_m) as f32,
                ]);
            }
        }

        let normals = compute_grid_normals(&heights_datum, side, GRID_STEP_M as f32);
        let colors = compute_htaws_colors(&heights_datum, &normals, own_alt_datum_m, planet);
        let indices = build_grid_indices(side);
        let eye = eye_override.unwrap_or(RunwayEyeAnchor {
            latitude_deg: own_lat_deg,
            longitude_deg: own_lon_deg,
            altitude_datum_m: own_alt_datum_m,
            onboard_dem_datum_m: None,
        });
        let (runways, approach) = resolve_approach_and_overlays(
            ownship,
            planet,
            &flat_origin,
            own_alt_datum_m,
            eye,
            route_label,
        );

        (
            TerrainMesh {
                positions_enu_m: positions,
                normals_enu: normals,
                colors_rgba: colors,
                indices,
                ground_elevation_msl_m: center_ground_m,
                agl_ft,
                runways,
            },
            approach,
        )
    }
}

pub(crate) fn build_hits_gates(
    thr_enu: [f32; 3],
    dir_en: [f32; 2],
    right_en: [f32; 2],
) -> Vec<[[f32; 3]; 4]> {
    let [tx, ty, tz] = thr_enu;
    let [dx, dy] = dir_en;
    let [rx, ry] = right_en;
    let slope_tan = libm::tanf(3.0_f32.to_radians());
    let half_w = 95.0f32;
    let half_h = 38.0f32;
    let mut gates = Vec::with_capacity(4);
    for dist_m in [1_400.0_f32, 2_800.0, 4_500.0, 6_400.0] {
        let cx = tx - dx * dist_m;
        let cy = ty - dy * dist_m;
        let cz = tz + 15.0 + dist_m * slope_tan;
        gates.push([
            [cx - rx * half_w, cy - ry * half_w, cz - half_h],
            [cx + rx * half_w, cy + ry * half_w, cz - half_h],
            [cx + rx * half_w, cy + ry * half_w, cz + half_h],
            [cx - rx * half_w, cy - ry * half_w, cz + half_h],
        ]);
    }
    gates
}

fn compute_grid_normals(heights: &[f64], side: usize, step_m: f32) -> Vec<[f32; 3]> {
    let mut out = Vec::with_capacity(heights.len());
    for iy in 0..side {
        for ix in 0..side {
            let x_prev = ix.saturating_sub(1);
            let x_next = (ix + 1).min(side.saturating_sub(1));
            let y_prev = iy.saturating_sub(1);
            let y_next = (iy + 1).min(side.saturating_sub(1));
            let hx0 = *heights.get(iy * side + x_prev).unwrap_or(&0.0) as f32;
            let hx1 = *heights.get(iy * side + x_next).unwrap_or(&0.0) as f32;
            let hy0 = *heights.get(y_prev * side + ix).unwrap_or(&0.0) as f32;
            let hy1 = *heights.get(y_next * side + ix).unwrap_or(&0.0) as f32;
            let dx = ((x_next - x_prev).max(1) as f32) * step_m;
            let dy = ((y_next - y_prev).max(1) as f32) * step_m;
            let nx = -(hx1 - hx0) / dx;
            let ny = -(hy1 - hy0) / dy;
            let nz = 1.0_f32;
            let inv = 1.0 / libm::sqrtf(nx * nx + ny * ny + nz * nz).max(1e-6);
            out.push([nx * inv, ny * inv, nz * inv]);
        }
    }
    out
}

fn compute_htaws_colors(
    heights_datum: &[f64],
    normals: &[[f32; 3]],
    own_alt_datum_m: f64,
    planet: &PlanetaryReference,
) -> Vec<[u8; 4]> {
    let [sx, sy, sz] = [-0.45_f32, 0.35, 0.82];
    let mut colors = Vec::with_capacity(heights_datum.len());
    for (idx, &h_datum) in heights_datum.iter().enumerate() {
        let [nx, ny, nz] = normals.get(idx).copied().unwrap_or([0.0, 0.0, 1.0]);
        let shade = (nx * sx + ny * sy + nz * sz).clamp(0.35, 1.08);
        let clearance_m = own_alt_datum_m - h_datum;
        let [br, bg, bb]: [f32; 3] = if planet.is_earth_wgs84() && h_datum <= 1.2 {
            [24.0, 58.0, 96.0]
        } else if clearance_m <= 30.48 {
            [232.0, 48.0, 42.0]
        } else if clearance_m <= 152.4 {
            [236.0, 172.0, 34.0]
        } else {
            hypsometric_rgb(h_datum, planet.id)
        };
        colors.push([
            (br * shade).clamp(0.0, 255.0) as u8,
            (bg * shade).clamp(0.0, 255.0) as u8,
            (bb * shade).clamp(0.0, 255.0) as u8,
            255,
        ]);
    }
    colors
}

fn hypsometric_rgb(h_datum: f64, body: PlanetaryBodyId) -> [f32; 3] {
    match body {
        PlanetaryBodyId::MoonIauSphere => {
            let band = (((h_datum + 400.0) / 1_200.0).clamp(0.0, 1.0)) as f32;
            let lum = 78.0 + 74.0 * band;
            [lum, lum * 0.99, lum * 1.02]
        }
        PlanetaryBodyId::MarsIauSpheroid => {
            let band = (((h_datum + 600.0) / 1_800.0).clamp(0.0, 1.0)) as f32;
            [138.0 + 62.0 * band, 82.0 + 44.0 * band, 58.0 + 28.0 * band]
        }
        PlanetaryBodyId::EarthWgs84Egm96 | PlanetaryBodyId::EarthWgs84Ellipsoid => {
            let band = ((h_datum / 220.0).clamp(0.0, 1.0)) as f32;
            [
                46.0 + 78.0 * band,
                84.0 + 48.0 * (1.0 - 0.4 * band),
                44.0 + 26.0 * band,
            ]
        }
    }
}

fn build_grid_indices(side: usize) -> Vec<u32> {
    let quads = side.saturating_sub(1);
    let mut indices = Vec::with_capacity(quads * quads * 6);
    for iy in 0..quads {
        for ix in 0..quads {
            let i00 = (iy * side + ix) as u32;
            let i10 = i00 + 1;
            let i01 = ((iy + 1) * side + ix) as u32;
            let i11 = i01 + 1;
            indices.extend_from_slice(&[i00, i10, i11, i00, i11, i01]);
        }
    }
    indices
}

fn analytic_nj_corridor_relief_m(lat_deg: f64, lon_deg: f64) -> f64 {
    if lon_deg > -74.145 && lat_deg < 40.72 {
        // Newark Bay / Upper NY Bay tidal water
        return 0.0;
    }
    // First & Second Watchung Mountain ridges running SW-NE between 47N and KEWR
    let ridge_axis = (lat_deg - 40.61) * 0.78 - (lon_deg + 74.48) * 0.62;
    let watchung = 148.0 * libm::exp(-(ridge_axis * ridge_axis) / 0.0018);
    let rolling = 24.0
        + 16.0 * libm::sin(lat_deg * 42.0) * libm::cos(lon_deg * 37.0)
        + 9.0 * libm::sin((lat_deg + lon_deg) * 95.0);
    (rolling + watchung).max(2.0)
}

fn analytic_planetary_relief_m(lat_deg: f64, lon_deg: f64, body: PlanetaryBodyId) -> f64 {
    let undulation = 140.0 * libm::sin(lat_deg * 18.0) * libm::cos(lon_deg * 16.0)
        + 55.0 * libm::cos((lat_deg - lon_deg) * 44.0);
    let d_crater = libm::hypot((lat_deg - 0.68) * 30.0, (lon_deg - 23.48) * 30.0);
    let crater_ring = 220.0 * libm::exp(-((d_crater - 1.8) * (d_crater - 1.8)) / 0.35)
        - 180.0 * libm::exp(-(d_crater * d_crater) / 0.9);
    match body {
        PlanetaryBodyId::MoonIauSphere => undulation + crater_ring,
        PlanetaryBodyId::MarsIauSpheroid => 1.6 * undulation + 0.8 * crater_ring,
        PlanetaryBodyId::EarthWgs84Egm96 | PlanetaryBodyId::EarthWgs84Ellipsoid => 0.0,
    }
}
