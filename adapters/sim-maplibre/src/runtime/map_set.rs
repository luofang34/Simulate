//! Zero or more named [`InstrumentMapSurface`]s backing one avionics preset.

use glam::DVec3;
use sim_scenery::CatalogScenery;

use super::{InstrumentMapSpec, InstrumentMapSurface, SharedMapRuntime};
use crate::MapAdapterError;

/// The instrument map surfaces of one avionics preset, keyed by id.
///
/// A preset may own no maps (round gauges), one (a moving map), or several with
/// unrelated styles (a PFD synthetic vision view beside an MFD chart). Every surface
/// shares the same [`SharedMapRuntime`]; none knows about the others.
#[derive(Default)]
pub struct AvionicsMapSet {
    surfaces: Vec<InstrumentMapSurface>,
}

impl AvionicsMapSet {
    /// Builds one surface per spec, refusing duplicate ids.
    pub fn new(
        runtime: &SharedMapRuntime,
        specs: Vec<InstrumentMapSpec>,
    ) -> Result<Self, MapAdapterError> {
        let mut surfaces: Vec<InstrumentMapSurface> = Vec::with_capacity(specs.len());
        for spec in specs {
            if surfaces.iter().any(|s| s.id() == spec.id) {
                return Err(MapAdapterError::DuplicateInstrumentMap { id: spec.id });
            }
            surfaces.push(InstrumentMapSurface::new(runtime, spec)?);
        }
        Ok(Self { surfaces })
    }

    /// Number of surfaces.
    #[must_use]
    pub fn len(&self) -> usize {
        self.surfaces.len()
    }

    /// Whether the preset has no map surfaces.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.surfaces.is_empty()
    }

    /// Surface ids in declaration order.
    pub fn ids(&self) -> impl Iterator<Item = &str> {
        self.surfaces.iter().map(InstrumentMapSurface::id)
    }

    /// The surface with `id`.
    #[must_use]
    pub fn get(&self, id: &str) -> Option<&InstrumentMapSurface> {
        self.surfaces.iter().find(|s| s.id() == id)
    }

    /// The surface with `id`, mutably.
    pub fn get_mut(&mut self, id: &str) -> Option<&mut InstrumentMapSurface> {
        self.surfaces.iter_mut().find(|s| s.id() == id)
    }

    /// Every surface in declaration order.
    pub fn iter(&self) -> impl Iterator<Item = &InstrumentMapSurface> {
        self.surfaces.iter()
    }

    /// Every surface in declaration order, mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut InstrumentMapSurface> {
        self.surfaces.iter_mut()
    }

    /// Updates the solar direction on every surface.
    pub fn set_sun_direction_ecef(&mut self, sun_ecef: DVec3) -> Result<(), MapAdapterError> {
        self.surfaces
            .iter_mut()
            .try_for_each(|s| s.set_sun_direction_ecef(sun_ecef))
    }

    /// Forwards the display pixel ratio to every surface.
    pub fn set_pixel_ratio(&mut self, ratio: f64) {
        self.surfaces
            .iter_mut()
            .for_each(|s| s.set_pixel_ratio(ratio));
    }

    /// Whether any 2D chart surface's redraw interval has elapsed at `at`.
    #[must_use]
    pub fn any_chart_due(&self, at: std::time::Duration) -> bool {
        self.surfaces
            .iter()
            .any(|s| !s.is_conformal() && s.is_due(at))
    }

    /// Synchronously supplies catalog DEM tiles to every surface; returns the total supplied.
    pub fn supply_dem_synchronously(
        &mut self,
        scenery: &CatalogScenery,
    ) -> Result<usize, MapAdapterError> {
        self.surfaces.iter_mut().try_fold(0_usize, |total, s| {
            Ok(total.saturating_add(s.supply_dem_synchronously(scenery)?))
        })
    }

    /// Marks every chart surface dirty.
    pub fn invalidate_charts(&mut self) {
        self.surfaces
            .iter_mut()
            .for_each(InstrumentMapSurface::invalidate);
    }

    /// Whether any surface has pending tile arrivals to draw.
    #[must_use]
    pub fn needs_redraw(&self) -> bool {
        self.surfaces.iter().any(InstrumentMapSurface::needs_redraw)
    }
}
