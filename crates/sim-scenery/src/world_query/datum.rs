//! The surface's vertical datum, runways registered at run time, and whether
//! resident live terrain is the ground as drawn.

use sim_geo::{Geodetic, egm96_undulation_m};

use super::{AuthoritativeWorldSurface, Body, Earth, RunwayStrip};
use crate::CatalogVerticalDatum;

/// Web Mercator (`EPSG:3857`) coverage limit in radians (`85.05112878°`).
const MERCATOR_MAX_LAT_RAD: f64 = 1.484_422_229_745_332_4;

impl AuthoritativeWorldSurface {
    /// The live-tile Earth: orthometric, as published Terrarium tiles are,
    /// with [`Self::with_unflattened_visual_dem`] on because the renderer
    /// drapes those tiles as they arrive.
    #[must_use]
    pub fn earth_live_dem() -> Self {
        Self::earth(CatalogVerticalDatum::OrthometricMslEgm96).with_unflattened_visual_dem(true)
    }

    /// Whether the renderer draws the resident DEM exactly as it was made resident,
    /// runway datum and all ([`crate::RunwayDatum::stamp_terrarium`]). When on, a
    /// runway over resident DEM of a zoom the datum stamps
    /// ([`crate::MIN_STAMP_ZOOM`] and finer) reads that DEM, so gear neither sinks
    /// below nor floats above the drawn mesh; over coarser DEM, or before any tile
    /// streams in, it reads the charted runway. Ignored on the Moon.
    #[must_use]
    pub fn with_unflattened_visual_dem(mut self, unflattened: bool) -> Self {
        if let Body::Earth(earth) = &mut self.body {
            earth.unflattened_visual_dem = unflattened;
            self.bump();
        }
        self
    }

    /// See [`Self::with_unflattened_visual_dem`]; `false` on the Moon.
    #[must_use]
    pub fn unflattened_visual_dem(&self) -> bool {
        match &self.body {
            Body::Earth(earth) => earth.unflattened_visual_dem,
            Body::Moon(_) => false,
        }
    }

    /// Metres from the ellipsoid up to the surface's height reference at a
    /// latitude and longitude: the EGM96 undulation when Earth heights are
    /// read as MSL, zero when they are read as ellipsoidal and on the Moon.
    #[must_use]
    pub fn geoid_undulation_m(&self, lat_rad: f64, lon_rad: f64) -> f64 {
        match self.vertical_datum() {
            Some(CatalogVerticalDatum::OrthometricMslEgm96) => egm96_undulation_m(lat_rad, lon_rad),
            Some(CatalogVerticalDatum::EllipsoidWgs84) | None => 0.0,
        }
    }

    /// The ellipsoidal height of a point `height_msl_m` above the surface's
    /// height reference, by [`Self::geoid_undulation_m`].
    #[must_use]
    pub fn msl_to_ellipsoidal_m(&self, lat_rad: f64, lon_rad: f64, height_msl_m: f64) -> f64 {
        height_msl_m + self.geoid_undulation_m(lat_rad, lon_rad)
    }

    /// Replaces the runways registered at run time — say the strips of the
    /// airport a user picked from the airport database — keeping those added
    /// by [`Self::with_aerodrome`], and changes the world version. Strip
    /// heights are read in [`Self::vertical_datum`]. Ignored on the Moon.
    pub fn set_extra_runways(&mut self, strips: Vec<RunwayStrip>) {
        if let Body::Earth(earth) = &mut self.body {
            earth.extra_runways = strips;
            self.bump();
        }
    }

    /// How many runway strips [`Self::set_extra_runways`] registered.
    #[must_use]
    pub fn extra_runway_len(&self) -> usize {
        match &self.body {
            Body::Earth(earth) => earth.extra_runways.len(),
            Body::Moon(_) => 0,
        }
    }

    /// Sets the polar cap heights (`[north_m, south_m]`, in [`Self::vertical_datum`])
    /// poleward of Web Mercator's `±85.0511°` limit where no DEM or catalog tile
    /// exists, matching the GPU terrain polar caps. Ignored on the Moon.
    #[must_use]
    pub fn with_polar_elevations_m(mut self, north_m: f64, south_m: f64) -> Self {
        if let Body::Earth(earth) = &mut self.body
            && north_m.is_finite()
            && south_m.is_finite()
        {
            earth.polar_elevations_m = Some([north_m, south_m]);
            self.bump();
        }
        self
    }
}

impl Earth {
    pub(super) fn polar_height_m(&self, position: &Geodetic) -> Option<f64> {
        let [north_m, south_m] = self.polar_elevations_m?;
        (position.lat_rad.abs() > MERCATOR_MAX_LAT_RAD).then_some(if position.lat_rad >= 0.0 {
            north_m
        } else {
            south_m
        })
    }
}
