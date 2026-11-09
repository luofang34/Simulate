//! How an Earth surface composes its ground: resident terrain, then the runways laid
//! over it.

use sim_geo::Geodetic;

use super::{AuthoritativeWorldSurface, Body, Earth, SurfaceSampleSource, runway};
use crate::{MIN_STAMP_ZOOM, RunwayDatum};

impl AuthoritativeWorldSurface {
    /// Lays every runway of `datum` over the terrain, at the charted heights the drawn
    /// terrain carries once its tiles are stamped with the same datum
    /// ([`RunwayDatum::stamp_terrarium`]). Ignored on the Moon.
    #[must_use]
    pub fn with_runway_datum(mut self, datum: RunwayDatum) -> Self {
        if let Body::Earth(earth) = &mut self.body {
            earth.runway_datum = Some(datum);
            self.bump();
        }
        self
    }
}

impl Earth {
    /// Height in [`Self::datum`]: the live overlay, the catalog DEM, the polar
    /// cap datum beyond Web Mercator's `±85.0511°` limit, or the fallback,
    /// blended toward the runway strip covering the point most strongly.
    pub(super) fn raw_height(&self, position: &Geodetic) -> Option<(f64, SurfaceSampleSource)> {
        let base = self
            .live
            .height_in(position, self.datum)
            .or_else(|| {
                self.catalog
                    .as_ref()
                    .and_then(|catalog| catalog.raw_height_and_zoom(position))
            })
            .map(|(height, zoom)| (height, SurfaceSampleSource::ResidentDem { zoom }))
            .or_else(|| {
                self.polar_height_m(position)
                    .map(|height| (height, SurfaceSampleSource::FallbackElevation))
            })
            .or_else(|| {
                self.fallback_m
                    .map(|height| (height, SurfaceSampleSource::FallbackElevation))
            });
        let registered =
            runway::strongest(self.runways.iter().chain(&self.extra_runways), position);
        let charted = self
            .runway_datum
            .as_ref()
            .and_then(|datum| datum.height_and_weight(position));
        let runway = registered
            .into_iter()
            .chain(charted)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        match (base, runway) {
            (base, None) => base,
            (None, Some((height, _))) => Some((height, SurfaceSampleSource::RunwaySurface)),
            (Some((terrain, source)), Some((height, weight))) => {
                // Gear must stand on the terrain the renderer draws: a live DEM it
                // draws as served, which carries the runway datum at every zoom the
                // datum stamps. A coarser tile only stands in until finer ones arrive,
                // so the runway itself is the better ground until then.
                let as_drawn = self.unflattened_visual_dem
                    && matches!(source, SurfaceSampleSource::ResidentDem { zoom } if zoom >= MIN_STAMP_ZOOM);
                let target = if as_drawn { terrain } else { height };
                Some((
                    terrain + (target - terrain) * weight,
                    SurfaceSampleSource::RunwaySurface,
                ))
            }
        }
    }
}
