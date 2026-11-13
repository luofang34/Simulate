//! File bridge between an `ace-rs` conceptual design (`vehicle.json`) and its
//! closed-loop 6-DoF flight here.
//!
//! `ace-rs` builds the flight package itself (its `ace-export` crate writes a
//! verified `syspack` directory); this side only reads the design's declared
//! parameters, checks that a package was exported from that design, and
//! compares the declared values with what the package measurably does in
//! flight. A value the flight did not measure is reported as not measured,
//! never estimated.

mod feedback;
mod inputs;

use std::collections::BTreeMap;
use std::path::Path;

use sha2::{Digest, Sha256};

use crate::calibration::results::DesignPerformanceReport;
use crate::{AIRSPEED_ENVELOPE_ASSET, LoadedSyspack, PackError};

pub use feedback::{AceFeedbackReport, AceFeedbackRow, build_ace_feedback_report};
pub use inputs::AceVehicleInputs;

/// Why an `ace-rs` design and a flight package could not be compared.
#[derive(Debug, thiserror::Error)]
pub enum AceBridgeError {
    /// The file is not JSON, or not an `ace-rs` vehicle document.
    #[error("{path}: not an ace-rs vehicle document")]
    Json {
        /// Where the document came from.
        path: String,
        /// Decoder error.
        #[source]
        source: serde_json::Error,
    },
    /// A document-level field (`id`, `name`, `parameters`) is missing.
    #[error("{path}: missing `{field}`")]
    MissingField {
        /// Where the document came from.
        path: String,
        /// The missing field.
        field: &'static str,
    },
    /// A parameter the comparison needs has no numeric value.
    #[error("vehicle {vehicle_id}: required parameter `{parameter}` has no numeric value")]
    MissingParameter {
        /// Vehicle identifier.
        vehicle_id: String,
        /// Parameter name, such as `wing.area`.
        parameter: String,
    },
    /// A parameter's unit is not one this bridge converts for its dimension.
    #[error(
        "vehicle {vehicle_id}: parameter `{parameter}` has unit `{unit}`, not a {dimension} unit"
    )]
    UnsupportedUnit {
        /// Vehicle identifier.
        vehicle_id: String,
        /// Parameter name.
        parameter: String,
        /// The unit found.
        unit: String,
        /// The dimension the parameter must have.
        dimension: &'static str,
    },
    /// `engine.count` is not a positive whole number.
    #[error("vehicle {vehicle_id}: engine.count {count} is not a positive whole number")]
    InvalidEngineCount {
        /// Vehicle identifier.
        vehicle_id: String,
        /// The declared value.
        count: f64,
    },
    /// The package was not exported from this design.
    #[error("package `{package_id}` was not exported from ace-rs vehicle `{vehicle_id}`")]
    PackageMismatch {
        /// Vehicle identifier.
        vehicle_id: String,
        /// Package identifier.
        package_id: String,
    },
    /// The design and the package's flight model disagree on engine count.
    #[error(
        "vehicle {vehicle_id} declares {declared} engine(s) but package `{package_id}` flies {flown}"
    )]
    EngineCountMismatch {
        /// Vehicle identifier.
        vehicle_id: String,
        /// Package identifier.
        package_id: String,
        /// `engine.count` from the design.
        declared: usize,
        /// Engines the flight model declares.
        flown: usize,
    },
}

/// Writes a complete, SHA-256-verified `syspack` directory to `out_dir`, embedding
/// the newly synthesized `performance/airspeed_envelope.json` from `report`.
pub fn write_syspack_with_manual_assets(
    package: &LoadedSyspack,
    report: &DesignPerformanceReport,
    out_dir: &Path,
) -> Result<(), PackError> {
    let mut manifest = package.manifest().clone();
    let fdm_asset = match &manifest.dynamics {
        syspack::DynamicsSpec::JsbsimXml { entry_asset, .. } => Some(entry_asset.clone()),
        _ => manifest
            .assets
            .keys()
            .find(|k| k.ends_with(".xml"))
            .cloned(),
    }
    .ok_or_else(|| PackError::InvalidCalibration {
        path: manifest.id.clone(),
        detail: "package has no JSBSim XML asset".to_owned(),
    })?;
    let fdm_bytes = package
        .asset(&fdm_asset)
        .ok_or_else(|| PackError::InvalidCalibration {
            path: fdm_asset.clone(),
            detail: "missing JSBSim XML bytes".to_owned(),
        })?;
    let env_bytes = encode(AIRSPEED_ENVELOPE_ASSET, &report.synthesized_envelope)?;

    let mut asset_files: BTreeMap<String, Vec<u8>> = BTreeMap::new();
    asset_files.insert(fdm_asset, fdm_bytes.to_vec());
    asset_files.insert(AIRSPEED_ENVELOPE_ASSET.to_owned(), env_bytes);

    for (rel_path, bytes) in &asset_files {
        manifest.assets.insert(
            rel_path.clone(),
            syspack::AssetDigest {
                sha256: hex_sha256(bytes),
                size_bytes: bytes.len() as u64,
            },
        );
    }
    for (rel_path, bytes) in &asset_files {
        write_file(&out_dir.join(rel_path), bytes)?;
    }
    write_file(
        &out_dir.join("syspack.json"),
        &encode("syspack.json", &manifest)?,
    )
}

fn encode<T: serde::Serialize>(path: &str, value: &T) -> Result<Vec<u8>, PackError> {
    serde_json::to_vec_pretty(value).map_err(|e| PackError::InvalidCalibration {
        path: path.to_owned(),
        detail: e.to_string(),
    })
}

fn write_file(path: &Path, bytes: &[u8]) -> Result<(), PackError> {
    let io = |e: std::io::Error, at: &Path| PackError::InvalidCalibration {
        path: at.display().to_string(),
        detail: e.to_string(),
    };
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| io(e, parent))?;
    }
    std::fs::write(path, bytes).map_err(|e| io(e, path))
}

fn hex_sha256(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut s = String::with_capacity(64);
    for b in digest {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

#[cfg(test)]
mod tests;
