//! Per-session aircraft variant registry, selectable tail liveries, and runtime import of
//! external livery bundles and external `syspack.json` variant directories.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

use sim_maplibre::UsdAircraftAsset;
use sim_pack::{
    ExternalLiveryBundle, PackError, VariantDescriptor, VariantRegistry, VehicleLivery,
    resolve_variant_alias,
};
#[cfg(not(target_arch = "wasm32"))]
use tracing::{info, warn};

use crate::error::AppSimError;

/// Environment variable pointing to an external livery JSON file or directory imported at startup.
#[cfg(not(target_arch = "wasm32"))]
pub const ENV_IMPORT_LIVERY: &str = "SIMULATE_IMPORT_LIVERY";

/// Environment variable pointing to an external `syspack.json` aircraft variant directory.
#[cfg(not(target_arch = "wasm32"))]
pub const ENV_IMPORT_VARIANT: &str = "SIMULATE_IMPORT_VARIANT";

/// Lightweight summary of one selectable livery for Hangar UI chips.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LiverySummary {
    /// Stable catalog identifier (e.g. `"n642sp"`, `"d-ecjb"`).
    pub id: String,
    /// Tail registration mark (e.g. `"N642SP"`, `"D-ECJB"`).
    pub tail_code: String,
    /// Human-readable livery name.
    pub name: String,
    /// Whether this livery was imported externally at runtime.
    pub external: bool,
}

/// Borrowed reference to a cached per-tail OpenUSD stage (`((aircraft_id, livery_id, rev), asset)`).
pub type OwnshipLiveryRef<'a> = ((&'a str, &'a str, u32), &'a Arc<UsdAircraftAsset>);

/// Live variant & livery catalog with cached composed [`UsdAircraftAsset`] stages.
pub struct LiverySession {
    registry: VariantRegistry,
    selected: BTreeMap<String, String>,
    assets: HashMap<(String, String), Arc<UsdAircraftAsset>>,
    revision: u32,
}

impl LiverySession {
    /// Loads the built-in aircraft family/variant registry (`c172s`, `c172p`, `p38g`) and
    /// applies any desktop startup imports from environment variables.
    pub fn new() -> Result<Self, AppSimError> {
        let registry = VariantRegistry::builtin()?;
        let mut selected = BTreeMap::new();
        for variant in registry.variants() {
            if let Some(default_id) = variant.default_livery_id() {
                selected.insert(variant.id.clone(), default_id.to_owned());
            }
        }
        #[cfg_attr(target_arch = "wasm32", allow(unused_mut))]
        let mut session = Self {
            registry,
            selected,
            assets: HashMap::new(),
            revision: 0,
        };
        #[cfg(not(target_arch = "wasm32"))]
        session.apply_environment_imports();
        Ok(session)
    }

    /// Monotonic generation bumped on every livery selection or import.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub const fn revision(&self) -> u32 {
        self.revision
    }

    /// Borrows the underlying [`VariantRegistry`].
    #[cfg(test)]
    #[must_use]
    pub const fn registry(&self) -> &VariantRegistry {
        &self.registry
    }

    /// Lists selectable liveries for `aircraft_id` in catalog order.
    #[must_use]
    pub fn liveries_for(&self, aircraft_id: &str) -> Vec<LiverySummary> {
        let Some(entry) = self.registry.variant(aircraft_id) else {
            return Vec::new();
        };
        let Some(cat) = entry.livery_catalog() else {
            return Vec::new();
        };
        cat.liveries
            .iter()
            .map(|liv| LiverySummary {
                id: liv.id.clone(),
                tail_code: liv.tail_code.clone(),
                name: liv.name.clone(),
                external: entry.external_livery(&liv.id).is_some(),
            })
            .collect()
    }

    /// Returns the active livery id for `aircraft_id`.
    #[must_use]
    pub fn active_livery_id(&self, aircraft_id: &str) -> Option<&str> {
        let entry = self.registry.variant(aircraft_id)?;
        self.selected
            .get(&entry.id)
            .map(String::as_str)
            .or_else(|| entry.default_livery_id())
    }

    /// Returns the active [`VehicleLivery`] descriptor and its index for `aircraft_id`.
    #[must_use]
    pub fn active_livery(&self, aircraft_id: &str) -> Option<(usize, &VehicleLivery)> {
        let entry = self.registry.variant(aircraft_id)?;
        let cat = entry.livery_catalog()?;
        let active_id = self.active_livery_id(aircraft_id)?;
        cat.liveries.iter().enumerate().find(|(_, liv)| {
            liv.id == active_id
                || liv.id.eq_ignore_ascii_case(active_id)
                || liv.tail_code.eq_ignore_ascii_case(active_id)
        })
    }

    /// Selects `livery_id` (or `tail_code`) on `aircraft_id`, composing and caching its
    /// OpenUSD stage when backed by USD layers or an external livery spec.
    pub fn select_livery(
        &mut self,
        aircraft_id: &str,
        livery_id: &str,
    ) -> Result<VehicleLivery, AppSimError> {
        let Some(variant) = self.registry.variant(aircraft_id) else {
            return Err(PackError::LiveryImport {
                detail: format!("unknown aircraft variant {aircraft_id:?}"),
            }
            .into());
        };
        let Some(livery) = variant
            .livery_catalog()
            .and_then(|cat| cat.livery(livery_id))
            .cloned()
        else {
            return Err(PackError::LiveryImport {
                detail: format!("unknown livery {livery_id:?} on {aircraft_id:?}"),
            }
            .into());
        };
        let canonical_aircraft = variant.id.clone();
        let key = (canonical_aircraft.clone(), livery.id.clone());
        if uses_usd_livery(variant, &livery) && !self.assets.contains_key(&key) {
            let composed = compose_variant_asset(variant, &livery.id)?;
            if let std::collections::hash_map::Entry::Vacant(slot) = self.assets.entry(key) {
                slot.insert(Arc::new(composed));
            }
        }
        self.selected.insert(canonical_aircraft, livery.id.clone());
        self.revision = self.revision.wrapping_add(1);
        Ok(livery)
    }

    /// Selects livery `index` in `aircraft_id`'s catalog.
    pub fn select_livery_index(
        &mut self,
        aircraft_id: &str,
        index: usize,
    ) -> Result<VehicleLivery, AppSimError> {
        let id = self
            .registry
            .variant(aircraft_id)
            .and_then(VariantDescriptor::livery_catalog)
            .and_then(|cat| cat.liveries.get(index))
            .map(|l| l.id.clone())
            .ok_or_else(|| PackError::LiveryImport {
                detail: format!("livery index {index} out of range for {aircraft_id:?}"),
            })?;
        self.select_livery(aircraft_id, &id)
    }

    /// Cycles to the next or previous livery on `aircraft_id`.
    #[cfg(test)]
    pub fn cycle_livery(
        &mut self,
        aircraft_id: &str,
        forward: bool,
    ) -> Result<Option<VehicleLivery>, AppSimError> {
        let Some(count) = self
            .registry
            .variant(aircraft_id)
            .and_then(VariantDescriptor::livery_catalog)
            .map(|cat| cat.liveries.len())
        else {
            return Ok(None);
        };
        if count == 0 {
            return Ok(None);
        }
        let cur = self.active_livery(aircraft_id).map_or(0, |(idx, _)| idx);
        let next = if forward {
            cur.wrapping_add(1) % count
        } else {
            cur.wrapping_add(count.saturating_sub(1)) % count
        };
        self.select_livery_index(aircraft_id, next).map(Some)
    }

    /// Returns the cached OpenUSD asset override for `aircraft_id`'s currently selected livery,
    /// when a non-default or explicitly selected USD livery is active.
    #[must_use]
    pub fn active_usd_override(&self, aircraft_id: &str) -> Option<OwnshipLiveryRef<'_>> {
        let entry = self.registry.variant(aircraft_id)?;
        let livery_id = self.selected.get(&entry.id)?.as_str();
        let asset = self.assets.get(&(entry.id.clone(), livery_id.to_owned()))?;
        Some(((entry.id.as_str(), livery_id, self.revision), asset))
    }

    /// Imports an external livery from raw declarative JSON spec bytes (plus optional
    /// pre-authored `.usda` overlay bytes), registers it into the target variant, composes its
    /// 3D OpenUSD stage, and selects it immediately.
    #[cfg(any(test, target_arch = "wasm32"))]
    pub fn import_livery_bytes(
        &mut self,
        target_aircraft_id: Option<&str>,
        json_bytes: &[u8],
        usda_bytes: Option<&[u8]>,
    ) -> Result<(String, VehicleLivery), AppSimError> {
        let bundle = match usda_bytes {
            Some(usda) => ExternalLiveryBundle::from_spec_and_usda_bytes(json_bytes, usda)?,
            None => ExternalLiveryBundle::from_spec_bytes(json_bytes)?,
        };
        self.finish_bundle_import(bundle, target_aircraft_id)
    }

    /// Imports an external livery from a filesystem directory (`livery.json` + optional
    /// `livery.usda`) or a standalone `*_livery.json` file.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn import_livery_path(
        &mut self,
        path: &std::path::Path,
        target_aircraft_id: Option<&str>,
    ) -> Result<(String, VehicleLivery), AppSimError> {
        let bundle = ExternalLiveryBundle::from_path(path)?;
        self.finish_bundle_import(bundle, target_aircraft_id)
    }

    /// Imports an external SHA-256-verified `syspack.json` aircraft variant directory into the
    /// session registry and caches its default OpenUSD livery when present.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn import_variant_dir(&mut self, path: &std::path::Path) -> Result<String, AppSimError> {
        let aircraft_id = self.registry.import_variant_dir(path)?;
        if let Some(variant) = self.registry.variant(&aircraft_id)
            && let Some(default_id) = variant.default_livery_id().map(str::to_owned)
        {
            if let Some(default_entry) = variant
                .livery_catalog()
                .and_then(|cat| cat.livery(&default_id))
                && uses_usd_livery(variant, default_entry)
            {
                let composed = compose_variant_asset(variant, &default_id)?;
                self.assets.insert(
                    (aircraft_id.clone(), default_id.clone()),
                    Arc::new(composed),
                );
            }
            self.selected.insert(aircraft_id.clone(), default_id);
        }
        self.revision = self.revision.wrapping_add(1);
        Ok(aircraft_id)
    }

    /// Applies optional `SIMULATE_IMPORT_VARIANT` and `SIMULATE_IMPORT_LIVERY` paths from the
    /// process environment.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn apply_environment_imports(&mut self) {
        if let Some(var_dir) = std::env::var_os(ENV_IMPORT_VARIANT) {
            let path = std::path::PathBuf::from(var_dir);
            match self.import_variant_dir(&path) {
                Ok(id) => {
                    info!(aircraft_id = %id, path = %path.display(), "imported external variant pack");
                }
                Err(err) => {
                    warn!(error = %err, path = %path.display(), "external variant import failed");
                }
            }
        }
        if let Some(liv_path) = std::env::var_os(ENV_IMPORT_LIVERY) {
            let path = std::path::PathBuf::from(liv_path);
            match self.import_livery_path(&path, None) {
                Ok((id, liv)) => info!(
                    aircraft_id = %id,
                    tail = %liv.tail_code,
                    path = %path.display(),
                    "imported external aircraft livery"
                ),
                Err(err) => {
                    warn!(error = %err, path = %path.display(), "external livery import failed");
                }
            }
        }
    }

    fn finish_bundle_import(
        &mut self,
        bundle: ExternalLiveryBundle,
        explicit_target: Option<&str>,
    ) -> Result<(String, VehicleLivery), AppSimError> {
        let target = explicit_target
            .map(str::to_owned)
            .or_else(|| bundle.target_aircraft_id.clone())
            .unwrap_or_else(|| "c172s".to_owned());
        let resolved_id = resolve_variant_alias(&target).to_owned();
        let entry = self.registry.import_external_livery(&resolved_id, bundle)?;
        if let Some(variant) = self.registry.variant(&resolved_id)
            && uses_usd_livery(variant, &entry)
        {
            let composed = compose_variant_asset(variant, &entry.id)?;
            self.assets
                .insert((resolved_id.clone(), entry.id.clone()), Arc::new(composed));
        }
        self.selected.insert(resolved_id.clone(), entry.id.clone());
        self.revision = self.revision.wrapping_add(1);
        Ok((resolved_id, entry))
    }
}

fn uses_usd_livery(variant: &VariantDescriptor, livery: &VehicleLivery) -> bool {
    livery.usd_root_layer.is_some()
        || livery.livery_spec_path.is_some()
        || variant.external_livery(&livery.id).is_some()
}

fn compose_variant_asset(
    variant: &VariantDescriptor,
    livery_id: &str,
) -> Result<UsdAircraftAsset, AppSimError> {
    Ok(UsdAircraftAsset::from_variant_livery(variant, livery_id)?)
}
