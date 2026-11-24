//! A surveyed local anomaly added to a body's global field.
//!
//! A global harmonic model stops at wavelengths of hundreds of kilometres;
//! what a gravimeter or magnetometer navigates by is shorter. An overlay
//! carries that as north-east-down vectors on a latitude-longitude grid on
//! the body's ellipsoid, in the unit of the field it is added to.

use std::sync::Arc;

use glam::{DMat3, DVec3};
use sim_geo::{BodyEllipsoid, GravityField, MagneticField, MagneticFieldError};
use sim_spacetime::DomainHasher;
use world_core::{DecimalYear, Ellipsoid};
use world_potential::{
    AnomalyGrid, CanonicalContent, Derivatives, FieldError, GridGeometry, OutsideCoverage,
    VectorField,
};

use crate::FieldBuildError;
use crate::scratch::with_workspace;

#[cfg(test)]
mod tests;

/// North-east-down anomaly vectors on a regular geodetic grid, interpolated
/// bilinearly and applied at every height.
#[derive(Clone, PartialEq)]
pub struct AnomalyOverlay {
    grid: AnomalyGrid<Arc<[f64]>>,
    values_ned: Arc<[f64]>,
    refuses_outside: bool,
}

impl core::fmt::Debug for AnomalyOverlay {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AnomalyOverlay")
            .field("geometry", &self.grid.geometry())
            .finish()
    }
}

impl AnomalyOverlay {
    /// The overlay of `values_ned` (three per node, rows from the south,
    /// west to east within a row) on `geometry`, with latitudes on `body`.
    ///
    /// # Errors
    ///
    /// [`FieldBuildError::Model`] for a malformed grid or a value count
    /// other than `3 × rows × columns`.
    pub fn new(
        geometry: GridGeometry,
        values_ned: Vec<f64>,
        body: &BodyEllipsoid,
        outside: OutsideCoverage,
    ) -> Result<Self, FieldBuildError> {
        let shape = Ellipsoid {
            semi_major_m: body.semi_major_m,
            flattening: body.flattening,
        };
        let values_ned: Arc<[f64]> = Arc::from(values_ned);
        let grid = AnomalyGrid::new(geometry, Arc::clone(&values_ned), shape, outside)
            .map_err(FieldBuildError::model("anomaly grid"))?;
        Ok(Self {
            grid,
            values_ned,
            refuses_outside: outside == OutsideCoverage::Refuse,
        })
    }

    /// Writes the grid, every node's anomaly and what a point off the grid gets.
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        let g = self.grid.geometry();
        for value in [
            g.south_deg,
            g.west_deg,
            g.latitude_step_deg,
            g.longitude_step_deg,
        ] {
            h.write_f64_canonical(value);
        }
        h.write_u64(g.rows as u64);
        h.write_u64(g.columns as u64);
        h.write_u8(u8::from(self.refuses_outside));
        for &value in self.values_ned.iter() {
            h.write_f64_canonical(value);
        }
    }

    /// The anomaly at `fixed`, body-fixed axes, in the grid's unit.
    ///
    /// # Errors
    ///
    /// [`FieldError::OutsideCoverage`] off a grid that refuses to
    /// extrapolate, [`FieldError::InvalidPosition`] for a non-finite point.
    pub fn anomaly_fixed(&self, fixed: DVec3) -> Result<DVec3, FieldError> {
        // A grid ignores the date and needs no harmonic scratch.
        let sample = with_workspace(0, |workspace| {
            self.grid.sample(
                workspace,
                fixed.to_array(),
                DecimalYear(0.0),
                Derivatives::Gradient,
            )
        })?;
        Ok(DVec3::from_array(sample.vector))
    }
}

/// A gravity field with local anomalies, m/s², added to it. A bilinear grid
/// has no meaningful gradient, so the sum states none.
#[derive(Debug, Clone)]
pub struct GravityWithAnomaly {
    /// The global field.
    pub base: Arc<dyn GravityField>,
    /// The anomalies added to it, m/s².
    pub overlays: Vec<AnomalyOverlay>,
}

impl GravityField for GravityWithAnomaly {
    /// Off a grid that refuses to extrapolate the global field stands
    /// alone, as where nothing was surveyed.
    fn attraction_fixed(&self, fixed: DVec3) -> DVec3 {
        self.overlays
            .iter()
            .filter_map(|overlay| overlay.anomaly_fixed(fixed).ok())
            .fold(self.base.attraction_fixed(fixed), |sum, anomaly| {
                sum + anomaly
            })
    }

    fn attraction_gradient_fixed(&self, fixed: DVec3) -> Option<DMat3> {
        match self.overlays.is_empty() {
            true => self.base.attraction_gradient_fixed(fixed),
            false => None,
        }
    }
}

/// Leads the content of a field with overlays; no `world-potential` model
/// kind takes it.
const OVERLAID_KIND: u8 = 0xA0;

/// A magnetic field with local anomalies, nT, added to it.
#[derive(Debug, Clone)]
pub struct MagnetismWithAnomaly {
    /// The global field.
    pub base: Arc<dyn MagneticField>,
    /// The crustal and local anomalies added to it, nT.
    pub overlays: Vec<AnomalyOverlay>,
}

impl MagneticField for MagnetismWithAnomaly {
    /// The global field's content, then each grid's in the order they add.
    fn visit_content(&self, f: &mut dyn FnMut(&[u8])) {
        f(&[OVERLAID_KIND]);
        self.base.visit_content(f);
        f(&(self.overlays.len() as u64).to_le_bytes());
        for overlay in &self.overlays {
            overlay.grid.visit_content(f);
        }
    }

    fn field_fixed_nt(&self, fixed: DVec3, decimal_year: f64) -> Result<DVec3, MagneticFieldError> {
        let mut field = self.base.field_fixed_nt(fixed, decimal_year)?;
        for overlay in &self.overlays {
            field += overlay
                .anomaly_fixed(fixed)
                .map_err(|_| MagneticFieldError::OutsideDomain { fixed_m: fixed })?;
        }
        Ok(field)
    }

    fn field_gradient_fixed_nt_per_m(
        &self,
        fixed: DVec3,
        decimal_year: f64,
    ) -> Result<Option<DMat3>, MagneticFieldError> {
        match self.overlays.is_empty() {
            true => self.base.field_gradient_fixed_nt_per_m(fixed, decimal_year),
            false => self.field_fixed_nt(fixed, decimal_year).map(|_| None),
        }
    }
}
