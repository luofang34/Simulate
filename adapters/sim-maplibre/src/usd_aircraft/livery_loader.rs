//! Runtime OpenUSD composition of built-in `syspack` liveries and externally imported
//! [`sim_pack::ExternalLiveryBundle`] payloads (`native` + `wasm32`).

use openusd::ar::AssetBuffer;

use super::{EMBEDDED_C172_USDC, UsdAircraftAsset, livery_compose};
use crate::MapAdapterError;

impl UsdAircraftAsset {
    /// Composes a built-in livery `livery_id` declared in `pack`'s `Liveries/catalog.json`.
    pub fn from_syspack_livery(
        pack: &sim_pack::LoadedSyspack,
        livery_id: &str,
    ) -> Result<Self, MapAdapterError> {
        let catalog = pack
            .livery_catalog()
            .ok_or_else(|| MapAdapterError::OpenUsd {
                detail: format!(
                    "syspack '{}' ships no Liveries/catalog.json",
                    pack.manifest().id
                ),
            })?;
        let entry = catalog
            .livery(livery_id)
            .ok_or_else(|| MapAdapterError::OpenUsd {
                detail: format!("unknown livery '{livery_id}' in '{}'", pack.manifest().id),
            })?;
        let Some(root) = entry.usd_root_layer.as_deref() else {
            if pack.manifest().id == "p38g" {
                return Self::p38g();
            }
            return Err(MapAdapterError::OpenUsd {
                detail: format!("livery '{livery_id}' declares no usd_root_layer"),
            });
        };
        let layers = collect_usd_layers(pack);
        Self::from_usd_layers_owned(root, layers)
    }

    /// Composes an [`sim_pack::ExternalLiveryBundle`] (either pre-projected `.usda` text or a
    /// runtime-raycast single-file JSON [`sim_pack::LiverySpec`]) onto `pack`'s airframe.
    pub fn from_external_livery(
        pack: &sim_pack::LoadedSyspack,
        bundle: &sim_pack::ExternalLiveryBundle,
    ) -> Result<Self, MapAdapterError> {
        let airframe_key = pack
            .manifest()
            .assets
            .keys()
            .find(|k| k.ends_with("_airframe.usda"))
            .map_or("c172s_airframe.usda", |k| k.trim_start_matches("usd/"));
        let livery_usda = match &bundle.usda_overlay {
            Some(text) => text.clone(),
            None => {
                let base = Self::embedded_c172()?;
                livery_compose::compose_c172_livery_usda(&base, &bundle.spec)?
            }
        };
        let livery_file = format!("{}_ext_livery.usda", bundle.id);
        let root_file = format!("{}_ext_root.usda", bundle.id);
        let root_usda =
            livery_compose::compose_c172_root_usda(&bundle.spec, airframe_key, &livery_file);
        let mut layers = collect_usd_layers(pack);
        if !layers.iter().any(|(k, _)| k.ends_with("c172.usdc")) {
            layers.push(("c172.usdc".to_owned(), EMBEDDED_C172_USDC.into()));
        }
        layers.push((livery_file, AssetBuffer::from(livery_usda.into_bytes())));
        layers.push((root_file.clone(), AssetBuffer::from(root_usda.into_bytes())));
        Self::from_usd_layers_owned(&root_file, layers)
    }

    /// Composes either a built-in or runtime-imported livery `livery_id` on `variant`.
    pub fn from_variant_livery(
        variant: &sim_pack::VariantDescriptor,
        livery_id: &str,
    ) -> Result<Self, MapAdapterError> {
        if let Some(ext) = variant.external_livery(livery_id) {
            return Self::from_external_livery(&variant.package, ext);
        }
        Self::from_syspack_livery(&variant.package, livery_id)
    }
}

fn collect_usd_layers(pack: &sim_pack::LoadedSyspack) -> Vec<(String, AssetBuffer)> {
    let mut layers = Vec::new();
    for path in pack.manifest().assets.keys() {
        if path.starts_with("usd/")
            && let Some(bytes) = pack.asset(path)
        {
            layers.push((path.clone(), AssetBuffer::from(bytes.to_vec())));
        }
    }
    layers
}
