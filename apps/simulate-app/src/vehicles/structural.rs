//! Where each free-flight vehicle's drawn model sits in its flight model's structural
//! frame: fitted once per model to the sourced landmarks its package ships, measured on
//! the mesh the renderer draws.

use std::sync::OnceLock;

use glam::DVec3;
use sim_maplibre::{OwnshipVisualKind, StructuralDatum, UsdAircraftAsset};
use sim_pack::{LoadedSyspack, builtin_c172p_package, builtin_c172s_package, builtin_p38g_package};
use tracing::{info, warn};

/// The fitted origins of the models that have a structural frame, each fitted on first
/// use: a model load and a fit take a tenth of a second.
#[derive(Debug, Default)]
pub struct ModelOrigins {
    c172: OnceLock<Option<DVec3>>,
    c172p: OnceLock<Option<DVec3>>,
    p38g: OnceLock<Option<DVec3>>,
}

impl ModelOrigins {
    /// `visual`'s origin in its flight model's structural frame (inches, `x` aft, `y`
    /// right, `z` up); `None` for a model without a structural frame or whose fit failed.
    pub fn origin_in(&self, visual: OwnshipVisualKind) -> Option<DVec3> {
        let cell = match visual {
            OwnshipVisualKind::C172 => &self.c172,
            OwnshipVisualKind::C172p => &self.c172p,
            OwnshipVisualKind::P38g => &self.p38g,
            OwnshipVisualKind::Hidden
            | OwnshipVisualKind::FixedWingUav
            | OwnshipVisualKind::QuadrotorX500
            | OwnshipVisualKind::LunarLander
            | OwnshipVisualKind::Raketa => return None,
        };
        *cell.get_or_init(|| fit(visual).map(|datum| datum.origin_in()))
    }
}

/// Fits `visual`'s drawn model to its package's landmarks.
fn fit(visual: OwnshipVisualKind) -> Option<StructuralDatum> {
    let loaded: Result<(LoadedSyspack, UsdAircraftAsset), String> = match visual {
        OwnshipVisualKind::C172 => builtin_c172s_package()
            .map_err(|error| error.to_string())
            .and_then(|p| {
                Ok((
                    p,
                    UsdAircraftAsset::embedded_n642sp().map_err(|e| e.to_string())?,
                ))
            }),
        OwnshipVisualKind::C172p => builtin_c172p_package()
            .map_err(|error| error.to_string())
            .and_then(|p| {
                Ok((
                    p,
                    UsdAircraftAsset::embedded_c172p_d_ecjb().map_err(|e| e.to_string())?,
                ))
            }),
        OwnshipVisualKind::P38g => builtin_p38g_package()
            .map_err(|error| error.to_string())
            .and_then(|p| Ok((p, UsdAircraftAsset::p38g().map_err(|e| e.to_string())?))),
        _ => return None,
    };
    let (package, asset) = match loaded {
        Ok(loaded) => loaded,
        Err(error) => {
            warn!(?visual, %error, "no structural datum: model or package unavailable");
            return None;
        }
    };
    let landmarks = package.structural_landmarks()?;
    match StructuralDatum::fit(landmarks, &asset) {
        Ok(datum) => {
            info!(
                ?visual,
                origin_in = ?datum.origin_in(),
                residuals_aft_in = ?datum.residuals_aft_in(),
                "drawn model placed on its structural landmarks"
            );
            Some(datum)
        }
        Err(error) => {
            warn!(?visual, %error, "no structural datum");
            None
        }
    }
}
