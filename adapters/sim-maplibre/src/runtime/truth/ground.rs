//! The terrain height the physical scene draws, so hosts can check that what stands on
//! the ground stands on the ground they see.

use maplibre::coords::LatLon;

use super::TruthSceneMap;

impl TruthSceneMap {
    /// Height above mean sea level of the terrain the last frame drew at a place; `None`
    /// outside the drawn tiles or before their DEM has loaded.
    #[must_use]
    pub fn drawn_ground_msl(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        self.map
            .context()
            .drawn_ground_cached(LatLon::new(lat_deg, lon_deg))
            .ok()
            .flatten()
    }

    /// Height above mean sea level at a place from the finest DEM tile loaded so far, drawn
    /// or not; `None` before any tile covering it has arrived.
    #[must_use]
    pub fn loaded_ground_msl(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        self.map
            .context()
            .loaded_ground_cached(LatLon::new(lat_deg, lon_deg))
    }
}
