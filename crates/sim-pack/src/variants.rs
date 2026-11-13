//! Aircraft family grouping (`family: "c172"` -> `c172s`, `c172p`) and runtime
//! registration of external variant `syspack` packages and per-tail livery bundles.

use std::collections::BTreeMap;

use syspack::PackageSource;

use crate::{
    ExternalLiveryBundle, LiveryCatalog, LoadedSyspack, PackError, VehicleLivery,
    builtin_c172p_package, builtin_c172s_package, builtin_p38g_package,
};

/// Canonical compatibility alias resolution for legacy `"c172"` / `"c172_class_reference"` ids.
#[must_use]
pub fn resolve_variant_alias(id: &str) -> &str {
    match id.trim() {
        "c172" | "c172_class_reference" => "c172s",
        other => other,
    }
}

/// Summary of an aircraft family and its registered variant ids in display order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VariantFamilySummary {
    /// Stable family identifier (e.g. `"c172"`, `"p38"`).
    pub family: String,
    /// Human-readable family title (e.g. `"Cessna 172 Skyhawk"`).
    pub family_name: String,
    /// Registered variant package ids belonging to this family (e.g. `["c172s", "c172p"]`).
    pub variant_ids: Vec<String>,
}

/// One loaded aircraft variant package together with its live livery catalog and any
/// externally imported [`ExternalLiveryBundle`] overlays.
#[derive(Debug, Clone, PartialEq)]
pub struct VariantDescriptor {
    /// Stable variant package id (`"c172s"`, `"c172p"`, `"p38g"`, or imported id).
    pub id: String,
    /// Human-readable variant display name (`"Cessna 172S Skyhawk SP"`).
    pub name: String,
    /// Family slug grouping related variants (`"c172"`).
    pub family: String,
    /// Human-readable family display title (`"Cessna 172 Skyhawk"`).
    pub family_name: String,
    /// `true` when imported at runtime from an external `syspack` source/directory.
    pub is_external: bool,
    /// Verified system package for this variant.
    pub package: LoadedSyspack,
    /// External livery bundles registered onto this variant at runtime, keyed by livery `id`.
    pub external_liveries: BTreeMap<String, ExternalLiveryBundle>,
}

impl VariantDescriptor {
    /// Constructs a [`VariantDescriptor`] from a verified [`LoadedSyspack`].
    #[must_use]
    pub fn from_package(package: LoadedSyspack, is_external: bool) -> Self {
        let id = package.manifest().id.clone();
        let name = package.manifest().name.clone();
        let (default_family, default_family_name) = default_family_for(&id, &name);
        let family = package
            .livery_catalog()
            .and_then(|cat| cat.family.clone())
            .unwrap_or(default_family);
        let family_name = package
            .livery_catalog()
            .and_then(|cat| cat.family_name.clone())
            .unwrap_or(default_family_name);
        Self {
            id,
            name,
            family,
            family_name,
            is_external,
            package,
            external_liveries: BTreeMap::new(),
        }
    }

    /// Returns this variant's [`LiveryCatalog`], including any imported external liveries.
    #[must_use]
    pub const fn livery_catalog(&self) -> Option<&LiveryCatalog> {
        self.package.livery_catalog()
    }

    /// Default livery id for this variant when a catalog is present.
    #[must_use]
    pub fn default_livery_id(&self) -> Option<&str> {
        self.package
            .livery_catalog()
            .map(|cat| cat.default_livery.as_str())
    }

    /// Looks up a runtime-imported [`ExternalLiveryBundle`] by `id` or case-insensitive `tail`.
    #[must_use]
    pub fn external_livery(&self, id: &str) -> Option<&ExternalLiveryBundle> {
        self.external_liveries.get(id).or_else(|| {
            self.external_liveries
                .values()
                .find(|bundle| bundle.spec.tail.eq_ignore_ascii_case(id))
        })
    }
}

fn default_family_for(id: &str, name: &str) -> (String, String) {
    if id.starts_with("c172") {
        ("c172".to_owned(), "Cessna 172 Skyhawk".to_owned())
    } else if id.starts_with("p38") {
        ("p38".to_owned(), "Lockheed P-38 Lightning".to_owned())
    } else {
        (id.to_owned(), name.to_owned())
    }
}

/// Registry of built-in and externally imported aircraft variants grouped by family.
#[derive(Debug, Clone, PartialEq)]
pub struct VariantRegistry {
    order: Vec<String>,
    variants: BTreeMap<String, VariantDescriptor>,
}

impl VariantRegistry {
    /// Builds the registry seeded with all built-in fixed-wing variants (`c172s`, `c172p`, `p38g`).
    pub fn builtin() -> Result<Self, PackError> {
        let mut registry = Self {
            order: Vec::new(),
            variants: BTreeMap::new(),
        };
        for package in [
            builtin_c172s_package()?,
            builtin_c172p_package()?,
            builtin_p38g_package()?,
        ] {
            registry.insert_descriptor(VariantDescriptor::from_package(package, false));
        }
        Ok(registry)
    }

    fn insert_descriptor(&mut self, descriptor: VariantDescriptor) {
        let id = descriptor.id.clone();
        if !self.variants.contains_key(&id) {
            self.order.push(id.clone());
        }
        self.variants.insert(id, descriptor);
    }

    /// Ordered slice of registered variant ids.
    #[must_use]
    pub fn variant_ids(&self) -> &[String] {
        &self.order
    }

    /// Iterates all registered [`VariantDescriptor`]s in registration order.
    pub fn variants(&self) -> impl Iterator<Item = &VariantDescriptor> {
        self.order.iter().filter_map(|id| self.variants.get(id))
    }

    /// Looks up a variant by exact `id` or legacy compatibility alias (`"c172"` -> `"c172s"`).
    #[must_use]
    pub fn variant(&self, id_or_alias: &str) -> Option<&VariantDescriptor> {
        let resolved = resolve_variant_alias(id_or_alias);
        self.variants.get(resolved)
    }

    /// Returns aircraft families in registration order alongside their member variant ids.
    #[must_use]
    pub fn families(&self) -> Vec<VariantFamilySummary> {
        let mut out: Vec<VariantFamilySummary> = Vec::new();
        for desc in self.variants() {
            if let Some(existing) = out.iter_mut().find(|fam| fam.family == desc.family) {
                existing.variant_ids.push(desc.id.clone());
            } else {
                out.push(VariantFamilySummary {
                    family: desc.family.clone(),
                    family_name: desc.family_name.clone(),
                    variant_ids: vec![desc.id.clone()],
                });
            }
        }
        out
    }

    /// Returns all variants belonging to `family` (e.g. `"c172"` -> `[c172s, c172p]`).
    #[must_use]
    pub fn variants_in_family(&self, family: &str) -> Vec<&VariantDescriptor> {
        self.variants()
            .filter(|desc| desc.family.eq_ignore_ascii_case(family))
            .collect()
    }

    /// Loads, SHA-256-verifies, and registers an external variant `syspack` from `source`
    /// (supported on both native and `wasm32`). Returns the registered variant `id`.
    pub fn import_variant_source(
        &mut self,
        source: &dyn PackageSource,
    ) -> Result<String, PackError> {
        let package = LoadedSyspack::load(source)?;
        let id = package.manifest().id.clone();
        if self
            .variants
            .get(&id)
            .is_some_and(|existing| !existing.is_external)
        {
            return Err(PackError::LiveryImport {
                detail: format!("external variant package {id:?} collides with built-in variant"),
            });
        }
        self.insert_descriptor(VariantDescriptor::from_package(package, true));
        Ok(id)
    }

    /// Loads, SHA-256-verifies, and registers an external `syspack.json` vehicle directory.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn import_variant_dir(&mut self, dir: &std::path::Path) -> Result<String, PackError> {
        let source = syspack::DirectoryPackageSource::new(dir);
        self.import_variant_source(&source)
    }

    /// Validates and registers an [`ExternalLiveryBundle`] onto `variant_id_or_alias`
    /// (e.g. `"c172s"`, `"c172p"`, or `"c172"`).
    pub fn import_external_livery(
        &mut self,
        variant_id_or_alias: &str,
        bundle: ExternalLiveryBundle,
    ) -> Result<VehicleLivery, PackError> {
        let resolved = resolve_variant_alias(variant_id_or_alias).to_owned();
        let Some(desc) = self.variants.get_mut(&resolved) else {
            return Err(PackError::LiveryImport {
                detail: format!("unknown target variant {variant_id_or_alias:?}"),
            });
        };
        let entry = desc.package.register_external_livery(&bundle)?;
        desc.external_liveries.insert(entry.id.clone(), bundle);
        Ok(entry)
    }
}
