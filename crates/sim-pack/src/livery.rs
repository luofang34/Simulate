//! FlightGear-compatible (`<PropertyList><sim><model><livery>`) and OpenUSD-layered
//! aircraft livery catalog (`Liveries/catalog.json`).

mod spec;

use serde::{Deserialize, Serialize};
use syspack::VerifiedSyspack;

use crate::PackError;

pub use spec::{
    ExternalLiveryBundle, LiveryGlyphSpec, LiveryMaterialSpec, LiveryPhotoAnchor, LiveryPhotoSpec,
    LiveryRegistrationSpec, LiverySpec, LiveryStripeSpec,
};

/// Canonical asset path for a package's multi-livery & DLC manifest.
pub const LIVERY_CATALOG_ASSET: &str = "Liveries/catalog.json";

/// One historical, factory, or externally imported aircraft livery entry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleLivery {
    /// Stable identifier matching the FlightGear `Models/Liveries/<id>.xml` stem.
    pub id: String,
    /// Human-readable display title.
    pub name: String,
    /// Relative path to the FlightGear `<PropertyList>` XML descriptor inside the package.
    pub flightgear_xml: String,
    /// Aircraft sub-variant represented by this livery (e.g. `"Cessna 172S Skyhawk SP"`).
    pub variant: String,
    /// Historical military or civil airframe serial number (e.g. `"F17202087"`).
    pub serial_number: String,
    /// Vertical fin / tail registration code (e.g. `"D-ECJB"`, `"N642SP"`).
    pub tail_code: String,
    /// Nose or fuselage tactical / short registration number.
    pub tactical_number: String,
    /// Squadron / aero-club / operator attribution.
    pub unit: String,
    /// Historical pilot or fleet attribution.
    pub pilot: String,
    /// Upper-surface paint standard (e.g. `"Matterhorn White"`).
    pub upper_surface_ana: String,
    /// Lower-surface / cheatline paint standard.
    pub lower_surface_ana: String,
    /// Linear sRGB diffuse reflectance for upper surfaces `[r, g, b]`.
    pub upper_rgb: [f32; 3],
    /// Linear sRGB diffuse reflectance for lower surfaces `[r, g, b]`.
    pub lower_rgb: [f32; 3],
    /// Linear sRGB accent color `[r, g, b]`.
    pub accent_rgb: [f32; 3],
    /// Exterior skin metallic factor in `[0, 1]`.
    pub metallic: f32,
    /// Exterior skin perceptual roughness in `[0, 1]`.
    pub roughness: f32,
    /// Optional DLC / external pack identifier (`None` for built-in liveries).
    pub dlc_pack: Option<String>,
    /// Optional per-tail OpenUSD root layer inside the package (e.g. `"usd/n642sp.usda"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub usd_root_layer: Option<String>,
    /// Optional ordered OpenUSD sublayers inside the package required to compose `usd_root_layer`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub usd_sublayers: Vec<String>,
    /// Optional declarative livery JSON spec asset inside the package (e.g. `"usd/n642sp_livery.json"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub livery_spec_path: Option<String>,
}

/// Multi-livery manifest pairing FlightGear `<PropertyList>` compatibility and OpenUSD
/// per-tail layer stacks with SHA-256-verified base and external livery descriptors.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryCatalog {
    /// Schema version (`1`).
    pub schema_version: u32,
    /// Package identifier (`"c172s"`, `"c172p"`, `"p38g"`).
    pub aircraft_id: String,
    /// Optional aircraft family identifier grouping related sub-variants (e.g. `"c172"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub family: Option<String>,
    /// Optional human-readable aircraft family display title (e.g. `"Cessna 172 Skyhawk"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub family_name: Option<String>,
    /// FlightGear `-set.xml` entry point (`"c172s-set.xml"`).
    pub flightgear_set_file: String,
    /// Default livery id (`"n642sp"`, `"d-ecjb"`, `"miss-virginia"`).
    pub default_livery: String,
    /// Virtual mount point for external/DLC livery packs.
    pub dlc_mount_point: String,
    /// Ordered livery descriptors.
    pub liveries: Vec<VehicleLivery>,
}

impl LiveryCatalog {
    /// Parses and validates `Liveries/catalog.json` and verifies that every referenced
    /// FlightGear XML and OpenUSD / livery JSON asset is present inside `verified`.
    pub fn from_verified(verified: &VerifiedSyspack, bytes: &[u8]) -> Result<Self, PackError> {
        let catalog: Self =
            serde_json::from_slice(bytes).map_err(|source| PackError::CalibrationParse {
                path: LIVERY_CATALOG_ASSET.to_owned(),
                source,
            })?;
        catalog.validate(verified)?;
        Ok(catalog)
    }

    /// Returns the default [`VehicleLivery`] declared by [`Self::default_livery`].
    #[must_use]
    pub fn default_entry(&self) -> Option<&VehicleLivery> {
        self.livery(&self.default_livery)
    }

    /// Looks up a livery by its stable `id` (case-insensitive fallback on `id` and `tail_code`).
    #[must_use]
    pub fn livery(&self, id: &str) -> Option<&VehicleLivery> {
        self.liveries.iter().find(|entry| {
            entry.id == id
                || entry.id.eq_ignore_ascii_case(id)
                || entry.tail_code.eq_ignore_ascii_case(id)
        })
    }

    /// Registers an externally imported [`ExternalLiveryBundle`] into this catalog,
    /// checking aircraft/family compatibility and replacing any previous external entry
    /// with the same `id`.
    pub fn register_external(
        &mut self,
        bundle: &ExternalLiveryBundle,
    ) -> Result<VehicleLivery, PackError> {
        if let Some(target) = bundle.target_aircraft_id.as_deref() {
            let matches_aircraft = target.eq_ignore_ascii_case(&self.aircraft_id);
            let matches_family = self
                .family
                .as_deref()
                .is_some_and(|fam| target.eq_ignore_ascii_case(fam));
            if !matches_aircraft && !matches_family {
                return Err(PackError::LiveryImport {
                    detail: format!(
                        "external livery {:?} targets {:?}, incompatible with aircraft {:?}",
                        bundle.id, target, self.aircraft_id
                    ),
                });
            }
        }
        let mut entry = bundle.entry.clone();
        entry.variant = self.aircraft_id.clone();
        if let Some(existing) = self.liveries.iter_mut().find(|item| item.id == entry.id) {
            if existing.dlc_pack.is_none() {
                return Err(PackError::LiveryImport {
                    detail: format!(
                        "external livery {:?} collides with built-in livery id",
                        entry.id
                    ),
                });
            }
            *existing = entry.clone();
        } else {
            self.liveries.push(entry.clone());
        }
        Ok(entry)
    }

    fn validate(&self, verified: &VerifiedSyspack) -> Result<(), PackError> {
        let bad = |detail: String| PackError::InvalidCalibration {
            path: LIVERY_CATALOG_ASSET.to_owned(),
            detail,
        };
        if self.schema_version != 1 {
            return Err(bad(format!(
                "unsupported livery catalog schema_version {}",
                self.schema_version
            )));
        }
        if self.liveries.is_empty() {
            return Err(bad("livery catalog must declare at least one livery".into()));
        }
        if self.default_entry().is_none() {
            return Err(bad(format!(
                "default_livery {:?} is not in liveries",
                self.default_livery
            )));
        }
        let Some(set_bytes) = verified.asset(&self.flightgear_set_file) else {
            return Err(bad(format!(
                "missing FlightGear set file {:?}",
                self.flightgear_set_file
            )));
        };
        let set_xml = std::str::from_utf8(set_bytes).map_err(|e| {
            bad(format!(
                "invalid UTF-8 in {}: {e}",
                self.flightgear_set_file
            ))
        })?;
        if !set_xml.contains("<PropertyList>") || !set_xml.contains("<livery>") {
            return Err(bad(format!(
                "FlightGear set file {:?} is missing <PropertyList> or <livery>",
                self.flightgear_set_file
            )));
        }
        for entry in &self.liveries {
            validate_livery_entry(verified, entry)?;
        }
        Ok(())
    }
}

fn validate_livery_entry(
    verified: &VerifiedSyspack,
    entry: &VehicleLivery,
) -> Result<(), PackError> {
    let bad = |detail: String| PackError::InvalidCalibration {
        path: LIVERY_CATALOG_ASSET.to_owned(),
        detail,
    };
    let Some(xml_bytes) = verified.asset(&entry.flightgear_xml) else {
        return Err(bad(format!(
            "livery {:?} references missing FlightGear XML {:?}",
            entry.id, entry.flightgear_xml
        )));
    };
    let xml = std::str::from_utf8(xml_bytes)
        .map_err(|e| bad(format!("invalid UTF-8 in {}: {e}", entry.flightgear_xml)))?;
    if !xml.contains("<PropertyList>")
        || !xml.contains("<livery>")
        || !xml.contains(&format!("<name type=\"string\">{}</name>", entry.name))
    {
        return Err(bad(format!(
            "FlightGear livery XML {:?} does not match catalog entry {:?}",
            entry.flightgear_xml, entry.id
        )));
    }
    if let Some(root) = &entry.usd_root_layer
        && verified.asset(root).is_none()
    {
        return Err(bad(format!(
            "livery {:?} references missing usd_root_layer {:?}",
            entry.id, root
        )));
    }
    for sub in &entry.usd_sublayers {
        if verified.asset(sub).is_none() {
            return Err(bad(format!(
                "livery {:?} references missing usd_sublayer {:?}",
                entry.id, sub
            )));
        }
    }
    if let Some(spec_path) = &entry.livery_spec_path {
        let Some(spec_bytes) = verified.asset(spec_path) else {
            return Err(bad(format!(
                "livery {:?} references missing livery_spec_path {:?}",
                entry.id, spec_path
            )));
        };
        LiverySpec::from_json_bytes(spec_bytes)?;
    }
    Ok(())
}
