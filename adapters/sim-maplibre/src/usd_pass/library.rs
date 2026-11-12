//! The airframes the aircraft pass can draw: the scene's own asset, which every traffic
//! aircraft uses, and the ownship airframe (or custom livery stage) chosen for the vehicle
//! being flown.
//!
//! Procedural airframes are built and uploaded once, on first selection, so switching
//! vehicles back and forth never rebuilds or re-uploads a mesh.

use std::{cell::RefCell, collections::BTreeMap, rc::Rc, sync::Arc};

use crate::{MapAdapterError, UsdAircraftAsset};

mod structural;

pub use structural::StructuralDatum;

/// Visual 3D airframe model rendered for ownship in external and cabin views.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum OwnshipVisualKind {
    /// No 3D ownship airframe mesh (e.g. reentry capsule without external mesh).
    Hidden,
    /// Cessna 172S Skyhawk SP (`UsdAircraftAsset::embedded_n642sp` by default).
    #[default]
    C172,
    /// Cessna 172P Skyhawk II / Reims F172P (`UsdAircraftAsset::embedded_c172p_d_ecjb` by default).
    C172p,
    /// Procedural Lockheed P-38G Lightning twin-boom fighter (`UsdAircraftAsset::p38g`).
    P38g,
    /// Procedural fixed-wing trainer / corridor UAV (`UsdAircraftAsset::fixed_wing_uav`).
    FixedWingUav,
    /// Procedural Holybro X500 quadrotor (`UsdAircraftAsset::x500_quadrotor`).
    QuadrotorX500,
    /// Procedural Apollo Lunar Module (`UsdAircraftAsset::lunar_lander`).
    LunarLander,
    /// Single-stage multi-body test rocket `Raketa` (`UsdAircraftAsset::embedded_raketa`).
    Raketa,
}

/// Which uploaded asset a draw's part index refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum AssetKey {
    /// The asset the scene was built with (used for traffic).
    Scene,
    /// [`UsdAircraftAsset::embedded_n642sp`]: default C-172S ownship tail (`N642SP`).
    C172Tail,
    /// [`UsdAircraftAsset::embedded_c172p_d_ecjb`]: default C-172P ownship tail (`D-ECJB`).
    C172pTail,
    /// Explicitly selected or runtime-imported ownship livery stage.
    CustomOwnship(u32),
    /// [`UsdAircraftAsset::p38g`].
    P38g,
    /// [`UsdAircraftAsset::fixed_wing_uav`].
    FixedWingUav,
    /// [`UsdAircraftAsset::x500_quadrotor`].
    QuadrotorX500,
    /// [`UsdAircraftAsset::lunar_lander`].
    LunarLander,
    /// [`UsdAircraftAsset::embedded_raketa`].
    Raketa,
    /// Groomed polar snow skiway mesh keyed by quantized runway dimensions.
    PolarSkiway(super::skiway::PolarSkiwayKey),
}

impl OwnshipVisualKind {
    /// The asset this airframe draws from, or `None` when ownship is hidden.
    const fn asset_key(self) -> Option<AssetKey> {
        match self {
            Self::Hidden => None,
            Self::C172 => Some(AssetKey::C172Tail),
            Self::C172p => Some(AssetKey::C172pTail),
            Self::P38g => Some(AssetKey::P38g),
            Self::FixedWingUav => Some(AssetKey::FixedWingUav),
            Self::QuadrotorX500 => Some(AssetKey::QuadrotorX500),
            Self::LunarLander => Some(AssetKey::LunarLander),
            Self::Raketa => Some(AssetKey::Raketa),
        }
    }
}

/// Every airframe built so far and the current ownship selection.
pub(crate) struct AircraftLibrary {
    scene: Arc<UsdAircraftAsset>,
    procedural: BTreeMap<AssetKey, Arc<UsdAircraftAsset>>,
    ownship: OwnshipVisualKind,
    custom_ownship_key: Option<AssetKey>,
    next_custom_gen: u32,
}

/// The library shared between a scene and its render-graph prepare system.
pub(crate) type SharedAircraftLibrary = Rc<RefCell<AircraftLibrary>>;

impl AircraftLibrary {
    /// A library drawing `scene` for traffic and, by default, ownship.
    pub(crate) fn new(scene: Arc<UsdAircraftAsset>) -> Self {
        Self {
            scene,
            procedural: BTreeMap::new(),
            ownship: OwnshipVisualKind::default(),
            custom_ownship_key: None,
            next_custom_gen: 0,
        }
    }

    /// Current ownship selection.
    pub(crate) const fn ownship_kind(&self) -> OwnshipVisualKind {
        self.ownship
    }

    /// Selects ownship's airframe, building a procedural one (or default C-172 tail) the first
    /// time it is chosen and clearing any custom livery override.
    pub(crate) fn select_ownship(
        &mut self,
        kind: OwnshipVisualKind,
    ) -> Result<(), MapAdapterError> {
        let build: Option<fn() -> Result<UsdAircraftAsset, MapAdapterError>> = match kind {
            OwnshipVisualKind::P38g => Some(UsdAircraftAsset::p38g),
            OwnshipVisualKind::FixedWingUav => Some(UsdAircraftAsset::fixed_wing_uav),
            OwnshipVisualKind::QuadrotorX500 => Some(UsdAircraftAsset::x500_quadrotor),
            OwnshipVisualKind::LunarLander => Some(UsdAircraftAsset::lunar_lander),
            OwnshipVisualKind::Raketa => Some(UsdAircraftAsset::embedded_raketa),
            OwnshipVisualKind::C172 => Some(UsdAircraftAsset::embedded_n642sp),
            OwnshipVisualKind::C172p => Some(UsdAircraftAsset::embedded_c172p_d_ecjb),
            OwnshipVisualKind::Hidden => None,
        };
        if let (Some(key), Some(build)) = (kind.asset_key(), build)
            && !self.procedural.contains_key(&key)
        {
            self.procedural.insert(key, Arc::new(build()?));
        }
        self.ownship = kind;
        self.custom_ownship_key = None;
        Ok(())
    }

    /// Installs an explicit ownship [`UsdAircraftAsset`] (built-in non-default livery or
    /// runtime-imported external livery/variant) while traffic keeps `scene`.
    pub(crate) fn set_custom_ownship(&mut self, asset: Arc<UsdAircraftAsset>) {
        if let Some(prev) = self.custom_ownship_key.take() {
            self.procedural.remove(&prev);
        }
        self.next_custom_gen = self.next_custom_gen.wrapping_add(1);
        let key = AssetKey::CustomOwnship(self.next_custom_gen);
        self.procedural.insert(key, asset);
        self.custom_ownship_key = Some(key);
    }

    /// Active ownship [`UsdAircraftAsset`] reference, if visible.
    pub(crate) fn active_ownship_asset(&self) -> Option<Arc<UsdAircraftAsset>> {
        let (key, _) = self.asset_for(true)?;
        match key {
            AssetKey::Scene => Some(Arc::clone(&self.scene)),
            other => self.procedural.get(&other).cloned(),
        }
    }

    /// The asset an aircraft draws from: ownship follows the selection, traffic the scene.
    pub(crate) fn asset_for(&self, ownship: bool) -> Option<(AssetKey, &UsdAircraftAsset)> {
        if ownship && self.ownship == OwnshipVisualKind::Hidden {
            return None;
        }
        if ownship
            && let Some(custom_key) = self.custom_ownship_key
            && let Some(asset) = self.get(custom_key)
        {
            return Some((custom_key, asset));
        }
        let mut key = if ownship {
            self.ownship.asset_key()?
        } else {
            AssetKey::Scene
        };
        if matches!(key, AssetKey::C172Tail | AssetKey::C172pTail)
            && !self.procedural.contains_key(&key)
        {
            key = AssetKey::Scene;
        }
        self.get(key).map(|asset| (key, asset))
    }

    /// Ensures a groomed polar skiway mesh for `spec` is built and resident.
    pub(crate) fn ensure_polar_skiway(&mut self, spec: super::skiway::PolarSkiwayKey) -> AssetKey {
        let key = AssetKey::PolarSkiway(spec);
        self.procedural
            .entry(key)
            .or_insert_with(|| Arc::new(super::skiway::build_polar_skiway(spec)));
        key
    }

    /// The asset uploaded under `key`, once built.
    pub(crate) fn get(&self, key: AssetKey) -> Option<&UsdAircraftAsset> {
        match key {
            AssetKey::Scene => Some(&self.scene),
            other => self.procedural.get(&other).map(Arc::as_ref),
        }
    }

    /// Every asset built so far, for lazy GPU upload.
    pub(crate) fn resident(&self) -> impl Iterator<Item = (AssetKey, &UsdAircraftAsset)> {
        std::iter::once((AssetKey::Scene, self.scene.as_ref())).chain(
            self.procedural
                .iter()
                .map(|(key, asset)| (*key, asset.as_ref())),
        )
    }
}
