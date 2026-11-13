//! Validated per-tail C172/general-aviation livery specifications (`*_livery.json`)
//! and runtime-importable external livery bundles (JSON spec + optional `.usda` overlay).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{PackError, VehicleLivery};

const DEFAULT_RGB_WHITE: [f32; 3] = [0.90, 0.905, 0.91];
const DEFAULT_RGB_ACCENT: [f32; 3] = [0.05, 0.16, 0.44];

/// One `UsdPreviewSurface` material entry inside a [`LiverySpec`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryMaterialSpec {
    /// Linear sRGB diffuse reflectance `[r, g, b]`.
    pub diffuse: [f32; 3],
    /// Metallic factor in `[0, 1]`.
    #[serde(default)]
    pub metallic: f32,
    /// Perceptual roughness in `[0, 1]`.
    #[serde(default = "default_roughness")]
    pub roughness: f32,
}

const fn default_roughness() -> f32 {
    0.30
}

/// One pixel-to-model registration anchor for photo-traced livery specs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryPhotoAnchor {
    /// Landmark description (e.g. `"spinner tip"`).
    #[serde(default)]
    pub what: String,
    /// Image pixel coordinates `[u, v]`.
    pub px: [f64; 2],
    /// Model side-view coordinates `[y_m, z_m]`.
    pub model_yz: [f64; 2],
}

/// Photo-registration metadata mapping image pixel coordinates into model `(y, z)` metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryPhotoSpec {
    /// Attribution / provenance note for the reference photograph.
    #[serde(default)]
    pub source: String,
    /// Two or more similarity-transform anchor pairs.
    pub anchors_px_to_model_m: Vec<LiveryPhotoAnchor>,
}

/// One projected fuselage or fin paint band / tricolor stripe.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryStripeSpec {
    /// USD mesh stem (`Livery<name>Port` / `Livery<name>Starboard`).
    pub name: String,
    /// Human-readable description of the stripe or band.
    #[serde(default)]
    pub what: String,
    /// Material prim name under `/C172/Materials/<material>`.
    pub material: String,
    /// Optional crop origin & scale `[u0, v0, scale]` when `upper`/`lower` are in crop pixels.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crop_px: Option<[f64; 3]>,
    /// Upper boundary in crop pixels (used with `crop_px` and `photo`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub upper: Vec<[f64; 2]>,
    /// Lower boundary in crop pixels (used with `crop_px` and `photo`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lower: Vec<[f64; 2]>,
    /// Direct upper boundary in model `(y_m, z_m)` coordinates.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub upper_yz: Vec<[f64; 2]>,
    /// Direct lower boundary in model `(y_m, z_m)` coordinates.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lower_yz: Vec<[f64; 2]>,
}

/// Fuselage / tailcone registration marking specification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryRegistrationSpec {
    /// Registration characters (`A-Z`, `0-9`, `'-'`), e.g. `"N642SP"` or `"D-ECJB"`.
    pub text: String,
    /// Material prim name under `/C172/Materials/<material>`.
    pub material: String,
    /// Optional bounding box `[u0, v0, u1, v1]` in reference-photo pixels.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub box_px: Option<[f64; 4]>,
    /// Direct centre `[y_m, z_m]` in model metres when not using `box_px`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center_yz: Option<[f64; 2]>,
    /// Regulatory citation (e.g. 14 CFR 45.29 or ICAO Annex 7 / LuftVZO).
    #[serde(default)]
    pub regulation: String,
    /// Character cap height in metres (e.g. `0.3048` for 12 in or `0.30` for 30 cm).
    pub cap_height_m: f64,
    /// Character width as a multiple of `cap_height_m` (typically `0.6667`).
    pub glyph_width_h: f64,
    /// Stroke width as a multiple of `cap_height_m` (typically `0.1667`).
    pub stroke_h: f64,
    /// Inter-character gap as a multiple of `cap_height_m`.
    pub gap_h: f64,
    /// Horizontal italic shear per cap height (`0.0` upright, `0.57` slanted).
    #[serde(default)]
    pub slant: f64,
}

/// Custom stroke-centreline glyph override inside [`LiverySpec::glyphs`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveryGlyphSpec {
    /// Corner fillet radius in cap-height units.
    #[serde(default)]
    pub radius: f64,
    /// Single-polyline stroke centreline (`[[x_w, y_h], ...]`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stroke: Vec<[f64; 2]>,
    /// Multi-polyline stroke centrelines.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub strokes: Vec<Vec<[f64; 2]>>,
}

/// Declarative per-tail livery specification (`*_livery.json`), usable both offline by
/// `vehicles/tools/c172_livery.py` and at runtime for external livery import.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiverySpec {
    /// Optional explicit catalog slug (defaults to lowercased `tail`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    /// Tail registration string (e.g. `"N642SP"`, `"D-ECJB"`).
    pub tail: String,
    /// Optional display title for Hangar UI.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Optional target variant or family identifier (e.g. `"c172s"`, `"c172p"`, or `"c172"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub aircraft_id: Option<String>,
    /// Optional airframe serial number metadata.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub serial_number: Option<String>,
    /// Optional operator / aero-club metadata.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    /// Optional pilot attribution metadata.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pilot: Option<String>,
    /// Optional overall base-coat linear RGB `[r, g, b]` (defaults to Matterhorn White).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub base_coat_rgb: Option<[f32; 3]>,
    /// Optional Material definitions under `/C172/Materials`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub materials: BTreeMap<String, LiveryMaterialSpec>,
    /// Optional prim-to-material overrides on `/C172/Airframe`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub skin_overrides: BTreeMap<String, String>,
    /// Optional photo-anchor similarity fit when stripes or registration use pixel coordinates.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub photo: Option<LiveryPhotoSpec>,
    /// Fuselage and fin cheatlines / swooshes.
    #[serde(default)]
    pub stripes: Vec<LiveryStripeSpec>,
    /// Optional vertical-stabilizer national flag or fin flash bands.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fin_flash: Vec<LiveryStripeSpec>,
    /// Tailcone / fuselage registration marking specification.
    pub registration: LiveryRegistrationSpec,
    /// Optional custom glyph centrelines (`serde_json::Value` allows metadata string keys).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub glyphs: BTreeMap<String, serde_json::Value>,
}

impl LiverySpec {
    /// Parses and validates a [`LiverySpec`] JSON payload fail-closed.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, PackError> {
        let spec: Self = serde_json::from_slice(bytes).map_err(|err| PackError::LiveryImport {
            detail: format!("invalid livery JSON: {err}"),
        })?;
        spec.validate()?;
        Ok(spec)
    }

    /// Canonical lowercase catalog slug derived from `id` or `tail`.
    #[must_use]
    pub fn canonical_id(&self) -> String {
        let raw = self.id.as_deref().unwrap_or(&self.tail).trim();
        raw.chars()
            .map(|ch| {
                if ch.is_ascii_alphanumeric() {
                    ch.to_ascii_lowercase()
                } else {
                    '-'
                }
            })
            .collect::<String>()
            .trim_matches('-')
            .to_owned()
    }

    /// Extracts validated custom [`LiveryGlyphSpec`] entries from [`Self::glyphs`].
    #[must_use]
    pub fn custom_glyphs(&self) -> BTreeMap<char, LiveryGlyphSpec> {
        let mut out = BTreeMap::new();
        for (key, value) in &self.glyphs {
            let mut chars = key.chars();
            if let (Some(ch), None) = (chars.next(), chars.next())
                && let Ok(parsed) = serde_json::from_value::<LiveryGlyphSpec>(value.clone())
                && (!parsed.stroke.is_empty() || !parsed.strokes.is_empty())
            {
                out.insert(ch, parsed);
            }
        }
        out
    }

    /// Validates registration characters, geometry bounds, and coordinate references.
    pub fn validate(&self) -> Result<(), PackError> {
        let bad = |detail: &str| PackError::LiveryImport {
            detail: detail.to_owned(),
        };
        if self.tail.trim().is_empty() || self.canonical_id().is_empty() {
            return Err(bad("tail registration must not be empty"));
        }
        let reg = &self.registration;
        if reg.text.trim().is_empty() {
            return Err(bad("registration.text must not be empty"));
        }
        if !reg
            .text
            .chars()
            .all(|ch| ch.is_ascii_uppercase() || ch.is_ascii_digit() || ch == '-')
        {
            return Err(bad(
                "registration.text may only contain A-Z, 0-9, and '-' characters",
            ));
        }
        if !(0.08..=0.80).contains(&reg.cap_height_m)
            || !(0.2..=1.2).contains(&reg.glyph_width_h)
            || !(0.05..=0.4).contains(&reg.stroke_h)
            || !(0.02..=0.8).contains(&reg.gap_h)
            || !(-1.2..=1.2).contains(&reg.slant)
        {
            return Err(bad("registration dimensions are outside physical bounds"));
        }
        let has_photo = self
            .photo
            .as_ref()
            .is_some_and(|p| p.anchors_px_to_model_m.len() >= 2);
        if reg.center_yz.is_none() && !(reg.box_px.is_some() && has_photo) {
            return Err(bad(
                "registration requires either center_yz or box_px with >= 2 photo anchors",
            ));
        }
        for band in self.stripes.iter().chain(self.fin_flash.iter()) {
            validate_band(band, has_photo)?;
        }
        Ok(())
    }

    /// Synthesizes a [`VehicleLivery`] catalog entry for this external spec.
    #[must_use]
    pub fn to_catalog_entry(&self) -> VehicleLivery {
        let id = self.canonical_id();
        let upper_rgb = self.base_coat_rgb.unwrap_or(DEFAULT_RGB_WHITE);
        let accent_rgb = self
            .materials
            .get(&self.registration.material)
            .or_else(|| self.materials.values().next())
            .map_or(DEFAULT_RGB_ACCENT, |m| m.diffuse);
        VehicleLivery {
            id: id.clone(),
            name: self
                .name
                .clone()
                .unwrap_or_else(|| format!("{} — Custom Livery", self.tail)),
            flightgear_xml: format!("Models/Liveries/{id}.xml"),
            variant: self
                .aircraft_id
                .clone()
                .unwrap_or_else(|| "C172 Custom".to_owned()),
            serial_number: self
                .serial_number
                .clone()
                .unwrap_or_else(|| "CUSTOM".to_owned()),
            tail_code: self.tail.clone(),
            tactical_number: self.tail.clone(),
            unit: self
                .unit
                .clone()
                .unwrap_or_else(|| "External Livery".to_owned()),
            pilot: self.pilot.clone().unwrap_or_else(|| "Custom".to_owned()),
            upper_surface_ana: "Custom Base Coat".to_owned(),
            lower_surface_ana: "Custom Trim".to_owned(),
            upper_rgb,
            lower_rgb: upper_rgb,
            accent_rgb,
            metallic: 0.08,
            roughness: 0.30,
            dlc_pack: Some("external".to_owned()),
            usd_root_layer: None,
            usd_sublayers: Vec::new(),
            livery_spec_path: None,
        }
    }
}

fn validate_band(band: &LiveryStripeSpec, has_photo: bool) -> Result<(), PackError> {
    if band.name.trim().is_empty()
        || !band
            .name
            .chars()
            .all(|ch| ch.is_ascii_alphanumeric() || ch == '_')
    {
        return Err(PackError::LiveryImport {
            detail: format!("invalid stripe identifier {:?}", band.name),
        });
    }
    if band.material.trim().is_empty() {
        return Err(PackError::LiveryImport {
            detail: format!("stripe {:?} has empty material", band.name),
        });
    }
    let has_yz = band.upper_yz.len() >= 2 && band.lower_yz.len() >= 2;
    let has_px =
        band.crop_px.is_some() && band.upper.len() >= 2 && band.lower.len() >= 2 && has_photo;
    if !has_yz && !has_px {
        return Err(PackError::LiveryImport {
            detail: format!(
                "stripe {:?} requires >= 2 upper_yz/lower_yz points or photo-anchored crop_px points",
                band.name
            ),
        });
    }
    Ok(())
}

/// Validated external livery bundle ready to register in a [`crate::LiveryCatalog`]
/// and compose onto a USD airframe on both native and `wasm32` targets.
#[derive(Debug, Clone, PartialEq)]
pub struct ExternalLiveryBundle {
    /// Canonical lowercase livery id (e.g. `"d-efgh"`).
    pub id: String,
    /// Optional target variant or aircraft family (`"c172s"`, `"c172p"`, `"c172"`).
    pub target_aircraft_id: Option<String>,
    /// Synthesized catalog entry.
    pub entry: VehicleLivery,
    /// Validated declarative livery specification.
    pub spec: LiverySpec,
    /// Optional pre-generated `*_livery.usda` layer text; if `None`, runtime USD composition
    /// generates the projected mesh layer directly from `spec`.
    pub usda_overlay: Option<String>,
}

impl ExternalLiveryBundle {
    /// Builds a validated bundle from single-file `*_livery.json` bytes (native & `wasm32`).
    pub fn from_spec_bytes(json_bytes: &[u8]) -> Result<Self, PackError> {
        let spec = LiverySpec::from_json_bytes(json_bytes)?;
        Ok(Self {
            id: spec.canonical_id(),
            target_aircraft_id: spec.aircraft_id.clone(),
            entry: spec.to_catalog_entry(),
            spec,
            usda_overlay: None,
        })
    }

    /// Builds a validated bundle from `*_livery.json` bytes plus a pre-authored `*_livery.usda`
    /// overlay layer (native & `wasm32`).
    pub fn from_spec_and_usda_bytes(
        json_bytes: &[u8],
        usda_bytes: &[u8],
    ) -> Result<Self, PackError> {
        let mut bundle = Self::from_spec_bytes(json_bytes)?;
        let text = std::str::from_utf8(usda_bytes).map_err(|err| PackError::LiveryImport {
            detail: format!("USDA overlay is not valid UTF-8: {err}"),
        })?;
        if !text.trim_start().starts_with("#usda 1.0") {
            return Err(PackError::LiveryImport {
                detail: "USDA overlay must begin with '#usda 1.0'".to_owned(),
            });
        }
        bundle.usda_overlay = Some(text.to_owned());
        Ok(bundle)
    }

    /// Loads an external livery from either a `.json` file or a directory containing
    /// `livery.json` / `*_livery.json` (plus optional `livery.usda` / `*_livery.usda`).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn from_path(path: &std::path::Path) -> Result<Self, PackError> {
        if path.is_file() {
            let bytes = std::fs::read(path).map_err(|err| PackError::LiveryImport {
                detail: format!("could not read {}: {err}", path.display()),
            })?;
            return Self::from_spec_bytes(&bytes);
        }
        let entries = std::fs::read_dir(path).map_err(|err| PackError::LiveryImport {
            detail: format!("could not read directory {}: {err}", path.display()),
        })?;
        let (mut json_path, mut usda_path) = (None, None);
        for entry in entries.flatten() {
            let p = entry.path();
            let Some(name) = p.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            if name == "livery.json" || name.ends_with("_livery.json") {
                json_path = Some(p);
            } else if name == "livery.usda" || name.ends_with("_livery.usda") {
                usda_path = Some(p);
            }
        }
        let Some(spec_file) = json_path else {
            return Err(PackError::LiveryImport {
                detail: format!("no livery.json or *_livery.json in {}", path.display()),
            });
        };
        let json_bytes = std::fs::read(&spec_file).map_err(|err| PackError::LiveryImport {
            detail: format!("could not read {}: {err}", spec_file.display()),
        })?;
        if let Some(usda_file) = usda_path {
            let usda_bytes = std::fs::read(&usda_file).map_err(|err| PackError::LiveryImport {
                detail: format!("could not read {}: {err}", usda_file.display()),
            })?;
            Self::from_spec_and_usda_bytes(&json_bytes, &usda_bytes)
        } else {
            Self::from_spec_bytes(&json_bytes)
        }
    }
}
