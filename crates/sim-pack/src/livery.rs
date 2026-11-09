//! FlightGear-compatible (`<PropertyList><sim><model><livery>`) and DLC-extensible
//! aircraft livery catalog (`Liveries/catalog.json`).

use serde::{Deserialize, Serialize};
use syspack::VerifiedSyspack;

use crate::PackError;

/// Canonical asset path for a package's multi-livery & DLC manifest.
pub const LIVERY_CATALOG_ASSET: &str = "Liveries/catalog.json";

/// One historical, factory, or DLC aircraft livery entry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleLivery {
    /// Stable identifier matching the FlightGear `Models/Liveries/<id>.xml` stem.
    pub id: String,
    /// Human-readable display title.
    pub name: String,
    /// Relative path to the FlightGear `<PropertyList>` XML descriptor inside the package.
    pub flightgear_xml: String,
    /// Aircraft sub-variant represented by this livery (e.g. `"P-38G-13-LO"`).
    pub variant: String,
    /// Historical USAAF/military or civil serial number (e.g. `"43-2264"`).
    pub serial_number: String,
    /// Vertical fin / tail serial code (e.g. `"32264"`).
    pub tail_code: String,
    /// Nose or fuselage tactical number (e.g. `"147"`).
    pub tactical_number: String,
    /// Squadron / group / operator attribution.
    pub unit: String,
    /// Historical pilot or test crew attribution.
    pub pilot: String,
    /// Upper-surface paint standard (e.g. `"ANA 613 Olive Drab No. 41"`).
    pub upper_surface_ana: String,
    /// Lower-surface paint standard (e.g. `"ANA 603 Neutral Gray No. 43"`).
    pub lower_surface_ana: String,
    /// Linear sRGB diffuse reflectance for upper surfaces `[r, g, b]`.
    pub upper_rgb: [f32; 3],
    /// Linear sRGB diffuse reflectance for lower surfaces `[r, g, b]`.
    pub lower_rgb: [f32; 3],
    /// Linear sRGB accent color (spinner bands, nose trim, squadron stripes) `[r, g, b]`.
    pub accent_rgb: [f32; 3],
    /// Exterior skin metallic factor in `[0, 1]`.
    pub metallic: f32,
    /// Exterior skin perceptual roughness in `[0, 1]`.
    pub roughness: f32,
    /// Optional DLC pack identifier (`None` for base-package liveries).
    pub dlc_pack: Option<String>,
}

/// Multi-livery manifest pairing FlightGear `<PropertyList>` compatibility with
/// SHA-256-verified base and DLC livery descriptors.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryCatalog {
    /// Schema version (`1`).
    pub schema_version: u32,
    /// Package identifier (`"p38g"`).
    pub aircraft_id: String,
    /// FlightGear `-set.xml` entry point (`"p38g-set.xml"`).
    pub flightgear_set_file: String,
    /// Default livery id (`"miss-virginia"`).
    pub default_livery: String,
    /// Virtual mount point for external/DLC livery packs.
    pub dlc_mount_point: String,
    /// Ordered livery descriptors.
    pub liveries: Vec<VehicleLivery>,
}

impl LiveryCatalog {
    /// Parses and validates `Liveries/catalog.json` and verifies that every referenced
    /// FlightGear XML asset (`p38g-set.xml` and `Models/Liveries/*.xml`) is present and
    /// well-formed inside `verified`.
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

    /// Looks up a livery by its stable `id` (e.g. `"miss-virginia"`).
    #[must_use]
    pub fn livery(&self, id: &str) -> Option<&VehicleLivery> {
        self.liveries.iter().find(|entry| entry.id == id)
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
        }
        Ok(())
    }
}
