//! Terrain-profile propagation models (`DeltaBullington` and NTIA/ITS `Itm`).

use world_propagation::itm::{Climate, Itm, ItmLoss, ItmPath, VariabilityMode};
use world_propagation::p526::{DeltaBullington, DeltaBullingtonLoss, ProfilePath};
use world_propagation::{Ground, Loss, PathLoss, Polarisation};

use super::super::path::{
    ExceedancePercentages, PathGeometry, PropagationLoss, PropagationLossError,
};
use super::{require_terrain_profile, resolve_heights_above_ground, signal_from_path};

/// ITU-R P.526-16 delta-Bullington terrain diffraction loss adapter behind [`PropagationLoss`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeltaBullingtonPropagationLoss {
    inner: DeltaBullington,
    default_polarisation: Polarisation,
    default_tx_height_m: Option<f64>,
    default_rx_height_m: Option<f64>,
}

impl DeltaBullingtonPropagationLoss {
    /// Builds a delta-Bullington terrain diffraction model.
    ///
    /// # Errors
    ///
    /// Returns `PropagationError` if `effective_earth_radius_m` or `ground` constants are invalid.
    pub fn new(
        effective_earth_radius_m: f64,
        ground: Ground,
    ) -> Result<Self, world_propagation::PropagationError> {
        Ok(Self {
            inner: DeltaBullington::new(effective_earth_radius_m, ground)?,
            default_polarisation: Polarisation::Vertical,
            default_tx_height_m: None,
            default_rx_height_m: None,
        })
    }

    /// Overrides the fallback polarisation used when `PathGeometry::polarisation` is `None`.
    #[must_use]
    pub const fn with_default_polarisation(mut self, polarisation: Polarisation) -> Self {
        self.default_polarisation = polarisation;
        self
    }

    /// Sets fallback antenna structural heights above profile endpoints when geodetic sites are omitted.
    #[must_use]
    pub const fn with_default_antenna_heights_m(
        mut self,
        tx_height_m: f64,
        rx_height_m: f64,
    ) -> Self {
        self.default_tx_height_m = Some(tx_height_m);
        self.default_rx_height_m = Some(rx_height_m);
        self
    }

    /// Evaluates delta-Bullington loss and component diagnostics.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationLossError`] if `terrain_profile` or terminal heights are missing or invalid.
    pub fn evaluate(
        &self,
        path: &PathGeometry,
    ) -> Result<DeltaBullingtonLoss, PropagationLossError> {
        let name = self.name();
        let signal = signal_from_path(&name, path, self.default_polarisation)?;
        let (profile, first, last) = require_terrain_profile(&name, path)?;
        let (tx_antenna_height_m, rx_antenna_height_m) = resolve_heights_above_ground(
            &name,
            path,
            first.height_m,
            last.height_m,
            self.default_tx_height_m,
            self.default_rx_height_m,
        )?;
        let geom = ProfilePath {
            profile,
            tx_antenna_height_m,
            rx_antenna_height_m,
        };
        self.inner
            .path_loss(&geom, &signal)
            .map_err(|source| PropagationLossError::Model {
                model: name,
                source,
            })
    }
}

impl PropagationLoss for DeltaBullingtonPropagationLoss {
    fn name(&self) -> String {
        "itu-p526-delta-bullington".to_owned()
    }

    fn loss_db(&self, path: &PathGeometry) -> Result<f64, PropagationLossError> {
        self.evaluate(path).map(|out| out.loss_db())
    }
}

/// NTIA/ITS Irregular Terrain Model (Longley-Rice point-to-point) adapter behind [`PropagationLoss`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ItmPropagationLoss {
    inner: Itm,
    default_polarisation: Polarisation,
    default_exceedance: Option<ExceedancePercentages>,
    default_tx_height_m: Option<f64>,
    default_rx_height_m: Option<f64>,
}

impl ItmPropagationLoss {
    /// Builds an ITM Longley-Rice point-to-point loss model.
    ///
    /// # Errors
    ///
    /// Returns `PropagationError` if refractivity (`250..=400`) or ground constants are invalid.
    pub fn new(
        climate: Climate,
        refractivity_n: f64,
        ground: Ground,
        mode: VariabilityMode,
    ) -> Result<Self, world_propagation::PropagationError> {
        Ok(Self {
            inner: Itm::new(climate, refractivity_n, ground, mode)?,
            default_polarisation: Polarisation::Vertical,
            default_exceedance: None,
            default_tx_height_m: None,
            default_rx_height_m: None,
        })
    }

    /// Drops location variability when receiver site is known.
    #[must_use]
    pub fn without_location_variability(mut self) -> Self {
        self.inner = self.inner.without_location_variability();
        self
    }

    /// Drops direct situation variability as used in interference studies.
    #[must_use]
    pub fn without_situation_variability(mut self) -> Self {
        self.inner = self.inner.without_situation_variability();
        self
    }

    /// Overrides the fallback polarisation used when `PathGeometry::polarisation` is `None`.
    #[must_use]
    pub const fn with_default_polarisation(mut self, polarisation: Polarisation) -> Self {
        self.default_polarisation = polarisation;
        self
    }

    /// Sets fallback exceedance quantiles `(time, location, situation)` when `PathGeometry::exceedance` is `None`.
    #[must_use]
    pub const fn with_default_exceedance(mut self, exceedance: ExceedancePercentages) -> Self {
        self.default_exceedance = Some(exceedance);
        self
    }

    /// Sets fallback structural antenna heights (`0.5..=3000.0 m`) when geodetic sites are omitted.
    #[must_use]
    pub const fn with_default_antenna_heights_m(
        mut self,
        tx_height_m: f64,
        rx_height_m: f64,
    ) -> Self {
        self.default_tx_height_m = Some(tx_height_m);
        self.default_rx_height_m = Some(rx_height_m);
        self
    }

    /// Evaluates full ITM basic transmission loss, propagation mode, and diagnostics.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationLossError`] if required profile/quantiles/sites are absent or invalid.
    pub fn evaluate(&self, path: &PathGeometry) -> Result<ItmLoss, PropagationLossError> {
        let name = self.name();
        let signal = signal_from_path(&name, path, self.default_polarisation)?;
        let (profile, first, last) = require_terrain_profile(&name, path)?;
        let exceedance = path.exceedance.or(self.default_exceedance).ok_or_else(|| {
            PropagationLossError::MissingPathData {
                model: name.clone(),
                field: "exceedance",
            }
        })?;
        let (tx_height_m, rx_height_m) = resolve_heights_above_ground(
            &name,
            path,
            first.height_m,
            last.height_m,
            self.default_tx_height_m,
            self.default_rx_height_m,
        )?;
        let geom = ItmPath {
            profile,
            tx_height_m,
            rx_height_m,
            time: exceedance.time,
            location: exceedance.location,
            situation: exceedance.situation,
        };
        self.inner
            .path_loss(&geom, &signal)
            .map_err(|source| PropagationLossError::Model {
                model: name,
                source,
            })
    }
}

impl PropagationLoss for ItmPropagationLoss {
    fn name(&self) -> String {
        "ntia-itm-longley-rice".to_owned()
    }

    fn loss_db(&self, path: &PathGeometry) -> Result<f64, PropagationLossError> {
        self.evaluate(path).map(|out| out.loss_db())
    }
}
