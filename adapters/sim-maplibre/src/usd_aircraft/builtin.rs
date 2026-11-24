//! The airframes embedded in the binary: their OpenUSD layers, the ambient occlusion baked
//! for them ahead of time, and the one composed copy of each that a process shares.

use std::sync::{Arc, Mutex, PoisonError};

use openusd::ar::AssetBuffer;
// The Cessna layers are the packages' own bytes, so the binary carries each once.
use sim_pack::builtin_usd_layers as layer;

use super::{EmbeddedUsdResolver, UsdAircraftAsset, loader};
use crate::MapAdapterError;

#[cfg(test)]
mod tests;

/// The generic Cessna 172 airframe layer, under the name this module's users compose it by.
pub(super) use sim_pack::builtin_usd_layers::C172_USDC as EMBEDDED_C172_USDC;

/// Embedded `Raketa` single-stage test rocket OpenUSD stage (`retro_rocket_v4.usd`).
const EMBEDDED_RAKETA_USD: &[u8] =
    include_bytes!("../../../../vehicles/raketa/assets/retro_rocket_v4.usd");

/// Ambient occlusion of [`BuiltinAirframe::C172`], one value per vertex in bake order.
const C172_OCCLUSION: &[u8] = include_bytes!("builtin/data/c172.ao");
/// Ambient occlusion of [`BuiltinAirframe::N642sp`], laid out like [`C172_OCCLUSION`].
const N642SP_OCCLUSION: &[u8] = include_bytes!("builtin/data/n642sp.ao");
/// Ambient occlusion of [`BuiltinAirframe::DEcjb`], laid out like [`C172_OCCLUSION`].
const D_ECJB_OCCLUSION: &[u8] = include_bytes!("builtin/data/d_ecjb.ao");

/// An airframe the app draws straight from the OpenUSD stage embedded in the binary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BuiltinAirframe {
    /// The generic BSD-3-Clause Cessna 172 (`c172.usdc`) every traffic aircraft draws.
    C172,
    /// `N642SP`, the default Cessna 172S Skyhawk SP ownship tail.
    N642sp,
    /// `D-ECJB`, Mathias Rust's 1987 Reims F172P, the default Cessna 172P ownship tail.
    DEcjb,
    /// The `Raketa` single-stage test rocket (`retro_rocket_v4.usd`).
    Raketa,
}

/// One built-in airframe as the process shares it.
struct Shared {
    asset: Option<Arc<UsdAircraftAsset>>,
    /// Times the stage was composed; more than one means the sharing is broken.
    compositions: u32,
}

/// A composed built-in is a pure function of the embedded layers, and composing one holds
/// a browser's only thread, so every consumer in the process (ownship, traffic, the
/// structural fit, a parked NPC) draws from one copy.
static SHARED: [Mutex<Shared>; BuiltinAirframe::ALL.len()] = [const {
    Mutex::new(Shared {
        asset: None,
        compositions: 0,
    })
}; BuiltinAirframe::ALL.len()];

impl BuiltinAirframe {
    /// Every built-in airframe, in declaration order.
    pub const ALL: [Self; 4] = [Self::C172, Self::N642sp, Self::DEcjb, Self::Raketa];

    /// The stage's root layer and every layer it composes, by resolver name.
    fn layers(self) -> (&'static str, Vec<(&'static str, &'static [u8])>) {
        match self {
            Self::C172 => ("c172.usdc", vec![("c172.usdc", EMBEDDED_C172_USDC)]),
            Self::N642sp => (
                "n642sp.usda",
                vec![
                    ("c172.usdc", EMBEDDED_C172_USDC),
                    ("c172s_airframe.usda", layer::C172S_AIRFRAME_USDA),
                    ("n642sp.usda", layer::N642SP_USDA),
                    ("n642sp_livery.usda", layer::N642SP_LIVERY_USDA),
                ],
            ),
            Self::DEcjb => (
                "d_ecjb.usda",
                vec![
                    ("c172.usdc", EMBEDDED_C172_USDC),
                    ("c172p_airframe.usda", layer::C172P_AIRFRAME_USDA),
                    ("d_ecjb.usda", layer::D_ECJB_USDA),
                    ("d_ecjb_livery.usda", layer::D_ECJB_LIVERY_USDA),
                ],
            ),
            Self::Raketa => (
                "retro_rocket_v4.usd",
                vec![("retro_rocket_v4.usd", EMBEDDED_RAKETA_USD)],
            ),
        }
    }

    /// The occlusion baked ahead of time, sparing a browser the bake on its only thread.
    /// The rocket's table would outweigh the three Cessnas' together, so it bakes when
    /// chosen.
    const fn baked_occlusion(self) -> Option<&'static [u8]> {
        match self {
            Self::C172 => Some(C172_OCCLUSION),
            Self::N642sp => Some(N642SP_OCCLUSION),
            Self::DEcjb => Some(D_ECJB_OCCLUSION),
            Self::Raketa => None,
        }
    }

    /// Composes the stage, taking its occlusion from `baked` or, without one, baking it.
    fn compose(self, baked: Option<&[u8]>) -> Result<UsdAircraftAsset, MapAdapterError> {
        let (root, layers) = self.layers();
        let mut resolver = EmbeddedUsdResolver::default();
        for (name, bytes) in layers {
            resolver.insert(name, AssetBuffer::from(bytes));
        }
        loader::load_aircraft(root, resolver, baked)
    }

    /// Times this process composed the airframe's stage for [`UsdAircraftAsset::builtin`].
    #[cfg(test)]
    fn compositions(self) -> u32 {
        SHARED.get(self as usize).map_or(0, |shared| {
            shared
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .compositions
        })
    }
}

impl UsdAircraftAsset {
    /// The process's one composed copy of `airframe`, composed on first use.
    pub fn builtin(airframe: BuiltinAirframe) -> Result<Arc<Self>, MapAdapterError> {
        let Some(shared) = SHARED.get(airframe as usize) else {
            return airframe.compose(airframe.baked_occlusion()).map(Arc::new);
        };
        // Held across the composition so that a second caller waits for the first one's
        // result instead of composing its own.
        let mut shared = shared.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some(asset) = &shared.asset {
            return Ok(Arc::clone(asset));
        }
        shared.compositions = shared.compositions.wrapping_add(1);
        let asset = Arc::new(airframe.compose(airframe.baked_occlusion())?);
        shared.asset = Some(Arc::clone(&asset));
        Ok(asset)
    }

    /// A copy of the generic BSD-3-Clause Cessna 172 (`c172.usdc`) traffic stage.
    pub fn embedded_c172() -> Result<Self, MapAdapterError> {
        Self::builtin_copy(BuiltinAirframe::C172)
    }

    /// A copy of `N642SP`, the default Cessna 172S Skyhawk SP ownship stage.
    pub fn embedded_n642sp() -> Result<Self, MapAdapterError> {
        Self::builtin_copy(BuiltinAirframe::N642sp)
    }

    /// Composes `N172SP`, the selectable Cessna 172S Skyhawk SP factory demonstrator stage.
    pub fn embedded_c172s_n172sp() -> Result<Self, MapAdapterError> {
        Self::from_usd_layers(
            "n172sp.usda",
            [
                ("c172.usdc", EMBEDDED_C172_USDC.into()),
                ("c172s_airframe.usda", layer::C172S_AIRFRAME_USDA.into()),
                ("n172sp.usda", layer::N172SP_USDA.into()),
                ("n172sp_livery.usda", layer::N172SP_LIVERY_USDA.into()),
            ],
        )
    }

    /// A copy of `D-ECJB`, Mathias Rust's 1987 Reims F172P default Cessna 172P ownship stage.
    pub fn embedded_c172p_d_ecjb() -> Result<Self, MapAdapterError> {
        Self::builtin_copy(BuiltinAirframe::DEcjb)
    }

    /// Composes `N53295`, the selectable 1982 Cessna 172P Skyhawk II per-tail stage.
    pub fn embedded_c172p_n53295() -> Result<Self, MapAdapterError> {
        Self::from_usd_layers(
            "n53295.usda",
            [
                ("c172.usdc", EMBEDDED_C172_USDC.into()),
                ("c172p_airframe.usda", layer::C172P_AIRFRAME_USDA.into()),
                ("n53295.usda", layer::N53295_USDA.into()),
                ("n53295_livery.usda", layer::N53295_LIVERY_USDA.into()),
            ],
        )
    }

    /// A copy of the embedded `Raketa` (`retro_rocket_v4.usd`) OpenUSD stage.
    pub fn embedded_raketa() -> Result<Self, MapAdapterError> {
        Self::builtin_copy(BuiltinAirframe::Raketa)
    }

    /// An owned copy of the shared `airframe`, for a caller that goes on to change it.
    fn builtin_copy(airframe: BuiltinAirframe) -> Result<Self, MapAdapterError> {
        Self::builtin(airframe).map(|shared| shared.as_ref().clone())
    }
}
