//! Per-session aircraft variant registry, selectable tail liveries, and runtime import of
//! external livery bundles and external `syspack.json` variant directories.

mod external_meta;
#[cfg(not(target_arch = "wasm32"))]
mod hangar_scan;

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

pub use external_meta::ExternalVehicleMeta;
use sim_maplibre::UsdAircraftAsset;
use sim_pack::{
    ExternalLiveryBundle, PackError, VariantDescriptor, VariantRegistry, VehicleLivery,
    resolve_variant_alias,
};

use crate::error::AppSimError;

/// Environment variable pointing to an external livery JSON file or directory imported at startup.
#[cfg(not(target_arch = "wasm32"))]
pub const ENV_IMPORT_LIVERY: &str = "SIMULATE_IMPORT_LIVERY";

/// Environment variable pointing to an external `syspack.json` aircraft variant directory.
#[cfg(not(target_arch = "wasm32"))]
pub const ENV_IMPORT_VARIANT: &str = "SIMULATE_IMPORT_VARIANT";

/// Environment variable pointing to one or more hangar directories of external aircraft packages.
#[cfg(not(target_arch = "wasm32"))]
pub const ENV_HANGAR_DIR: &str = "SIMULATE_HANGAR_DIR";

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
    externals: Vec<ExternalVehicleMeta>,
    revision: u32,
}

impl LiverySession {
    /// Loads the built-in aircraft family/variant registry (`c172s`, `c172p`, `p38g`) and
    /// applies any desktop startup imports from environment variables and `$HOME/hangar`.
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
            externals: Vec::new(),
            revision: 0,
        };
        #[cfg(not(target_arch = "wasm32"))]
        session.apply_environment_imports();
        Ok(session)
    }

    /// Imported external aircraft metadata in slot registration order.
    #[must_use]
    pub fn externals(&self) -> &[ExternalVehicleMeta] {
        &self.externals
    }

    /// External aircraft metadata for free-flight slot `slot`, if loaded.
    #[must_use]
    pub fn external_meta(&self, slot: u16) -> Option<&ExternalVehicleMeta> {
        self.externals.get(usize::from(slot))
    }

    /// Slot index assigned to external variant `aircraft_id`, if any.
    #[must_use]
    pub fn external_slot_by_id(&self, aircraft_id: &str) -> Option<u16> {
        self.externals
            .iter()
            .find(|m| m.aircraft_id == aircraft_id)
            .map(|m| m.slot)
    }

    /// Borrows a registered variant descriptor by id or alias.
    #[must_use]
    pub fn variant(&self, aircraft_id: &str) -> Option<&VariantDescriptor> {
        self.registry.variant(aircraft_id)
    }

    /// Composes (and caches) an OpenUSD asset for `(aircraft_id, livery_id)` for previews or NPCs.
    pub fn usd_asset_for(
        &mut self,
        aircraft_id: &str,
        livery_id: Option<&str>,
    ) -> Result<Option<Arc<UsdAircraftAsset>>, AppSimError> {
        let Some(variant) = self.registry.variant(aircraft_id) else {
            return Ok(None);
        };
        let resolved_id = variant.id.clone();
        let lid = livery_id
            .or_else(|| self.selected.get(&resolved_id).map(String::as_str))
            .or_else(|| variant.default_livery_id())
            .unwrap_or_default()
            .to_owned();
        if lid.is_empty() {
            return Ok(None);
        }
        let key = (resolved_id.clone(), lid.clone());
        if let Some(cached) = self.assets.get(&key) {
            return Ok(Some(Arc::clone(cached)));
        }
        let Some(entry) = variant.livery_catalog().and_then(|c| c.livery(&lid)) else {
            return Ok(None);
        };
        if !uses_usd_livery(variant, entry) {
            return Ok(None);
        }
        let composed = Arc::new(compose_variant_asset(variant, &entry.id)?);
        self.assets
            .insert((resolved_id, entry.id.clone()), Arc::clone(&composed));
        Ok(Some(composed))
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
        let default_id = self
            .registry
            .variant(&aircraft_id)
            .and_then(|v| v.default_livery_id().map(str::to_owned));
        if let Some(default_id) = default_id.as_deref()
            && let Some(variant) = self.registry.variant(&aircraft_id)
        {
            if let Some(default_entry) = variant
                .livery_catalog()
                .and_then(|cat| cat.livery(default_id))
                && uses_usd_livery(variant, default_entry)
            {
                let composed = compose_variant_asset(variant, default_id)?;
                self.assets.insert(
                    (aircraft_id.clone(), default_id.to_owned()),
                    Arc::new(composed),
                );
            }
            self.selected
                .insert(aircraft_id.clone(), default_id.to_owned());
        }
        self.register_external_meta(&aircraft_id, default_id.as_deref());
        self.revision = self.revision.wrapping_add(1);
        Ok(aircraft_id)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn register_external_meta(&mut self, aircraft_id: &str, default_id: Option<&str>) {
        if matches!(aircraft_id, "c172s" | "c172p" | "p38g") {
            return;
        }
        let Some(variant) = self.registry.variant(aircraft_id) else {
            return;
        };
        let slot = self
            .external_slot_by_id(aircraft_id)
            .unwrap_or(self.externals.len() as u16);
        let default_asset = default_id
            .and_then(|lid| self.assets.get(&(aircraft_id.to_owned(), lid.to_owned())))
            .map(Arc::as_ref);
        let family_title = (!variant.family_name.is_empty() && variant.family_name != variant.name)
            .then(|| variant.family_name.clone());
        let meta = ExternalVehicleMeta::from_package(
            slot,
            aircraft_id,
            &variant.name,
            family_title,
            &variant.package,
            default_asset,
        );
        if let Some(existing) = self.externals.get_mut(usize::from(slot)) {
            *existing = meta;
        } else {
            self.externals.push(meta);
        }
    }

    /// Applies optional `SIMULATE_HANGAR_DIR`, `$HOME/hangar`, `SIMULATE_IMPORT_VARIANT`, and
    /// `SIMULATE_IMPORT_LIVERY` paths from the process environment.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn apply_environment_imports(&mut self) {
        self.import_environment_paths();
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
