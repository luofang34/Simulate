//! Which airframe the truth scene draws for ownship.

use super::TruthSceneMap;
use crate::{MapAdapterError, OwnshipVisualKind};

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

    /// The 3D airframe drawn for ownship.
    #[must_use]
    pub fn ownship_visual(&self) -> OwnshipVisualKind {
        self.aircraft_library
            .try_borrow()
            .map_or(OwnshipVisualKind::default(), |library| {
                library.ownship_kind()
            })
    }
}
