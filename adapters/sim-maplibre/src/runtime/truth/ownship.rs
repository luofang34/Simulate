//! Which airframe and per-tail livery the truth scene draws for ownship.

use std::sync::Arc;

use super::TruthSceneMap;
use crate::{MapAdapterError, OwnshipVisualKind, TrafficModelId, UsdAircraftAsset};

impl TruthSceneMap {
    /// Selects the 3D airframe drawn for ownship; traffic keeps the scene's asset.
    ///
    /// A procedural airframe is built and uploaded the first time it is selected and kept
    /// for later switches. Fails, keeping the previous airframe, when the airframe cannot be
    /// built.
    pub fn set_ownship_visual(&mut self, kind: OwnshipVisualKind) -> Result<(), MapAdapterError> {
        let mut library =
            self.aircraft_library
                .try_borrow_mut()
                .map_err(|_| MapAdapterError::OpenUsd {
                    detail: "ownship airframe library is in use by a frame".to_owned(),
                })?;
        library.select_ownship(kind)
    }

    /// Installs an explicit per-tail [`UsdAircraftAsset`] (such as `N642SP`, `N53295`, or a
    /// runtime-imported external livery/variant) for ownship while traffic keeps the generic
    /// scene airframe.
    pub fn set_ownship_livery_asset(
        &mut self,
        asset: Arc<UsdAircraftAsset>,
    ) -> Result<(), MapAdapterError> {
        let mut library =
            self.aircraft_library
                .try_borrow_mut()
                .map_err(|_| MapAdapterError::OpenUsd {
                    detail: "ownship airframe library is in use by a frame".to_owned(),
                })?;
        library.set_custom_ownship(asset);
        Ok(())
    }

    /// Registers a non-ownship OpenUSD airframe for traffic or ground NPCs, returning its handle.
    pub fn register_traffic_model(
        &mut self,
        asset: Arc<UsdAircraftAsset>,
    ) -> Result<TrafficModelId, MapAdapterError> {
        let mut library =
            self.aircraft_library
                .try_borrow_mut()
                .map_err(|_| MapAdapterError::OpenUsd {
                    detail: "ownship airframe library is in use by a frame".to_owned(),
                })?;
        Ok(library.register_traffic_model(asset))
    }

    /// Active ownship [`UsdAircraftAsset`] currently rendered in external and cabin views.
    #[must_use]
    pub fn active_ownship_asset(&self) -> Option<Arc<UsdAircraftAsset>> {
        self.aircraft_library
            .try_borrow()
            .ok()
            .and_then(|library| library.active_ownship_asset())
    }

    /// The 3D airframe drawn for ownship.
    #[must_use]
    pub fn ownship_visual(&self) -> OwnshipVisualKind {
        self.aircraft_library
            .try_borrow()
            .map_or(OwnshipVisualKind::default(), |library| {
                library.ownship_kind()
            })
    }

    /// Sets whether ownship 3D cockpit controls (yoke/stick/column) are hidden in interior views.
    pub fn set_hide_cockpit_controls(&mut self, hide: bool) {
        self.hide_cockpit_controls = hide;
    }

    /// Whether ownship 3D cockpit controls (yoke/stick/column) are hidden in interior views.
    #[must_use]
    pub const fn hides_cockpit_controls(&self) -> bool {
        self.hide_cockpit_controls
    }

    /// Sets the exposure time of the camera the scene is drawn for, which a turning
    /// propeller smears over; the eye's by default.
    pub fn set_shutter_exposure_s(&mut self, shutter_s: f64) {
        self.shutter_s = shutter_s;
    }
}
