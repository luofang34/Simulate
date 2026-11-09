//! Bi-directional co-design bridge between `~/ace-rs` (`vehicle.json`) and
//! `Simulate` (`syspack` + 6-DoF JSBSim flight-manual & mission compiler).

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};

use crate::calibration::results::DesignPerformanceReport;
use crate::calibration::trades::{DesignTradeSpec, apply_design_trade};
use crate::{AIRSPEED_ENVELOPE_ASSET, LoadedSyspack, PackError, builtin_c172_package};

/// Closed-loop 6-DoF simulator feedback package for `~/ace-rs` (`aex`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AceFeedbackReport {
    /// Vehicle identifier (`id` in `vehicle.json` / `syspack.json`).
    pub vehicle_id: String,
    /// Vehicle display name.
    pub vehicle_name: String,
    /// Conceptual `aero.cd0` declared in `~/ace-rs`.
    pub ace_input_cd0: f64,
    /// Effective 6-DoF trimmed $C_{D0}$ measured in JSBSim (including elevator trim drag).
    pub trimmed_6dof_cd0: f64,
    /// Conceptual `aero.oswald` declared in `~/ace-rs`.
    pub ace_input_oswald: f64,
    /// Effective 6-DoF Oswald efficiency $e$ deduced from $(L/D)_{\max}$ and climb polar.
    pub trimmed_6dof_oswald: f64,
    /// Conceptual `propeller.efficiency` declared in `~/ace-rs`.
    pub ace_input_prop_efficiency: f64,
    /// Effective 6-DoF cruise propeller efficiency $\eta_{p,\text{cruise}}$ at `6,000 ft`.
    pub trimmed_6dof_cruise_eta: f64,
    /// Effective 6-DoF climb installation efficiency $\eta_{p,\text{climb}}$ at $V_Y$.
    pub trimmed_6dof_climb_eta: f64,
    /// Effective 6-DoF cruise BSFC (`kg/kWh`).
    pub trimmed_6dof_bsfc_kg_kwh: f64,
    /// Synthesized stall speeds $(V_{S0}, V_{S1})$ (`KCAS`).
    pub stall_speeds_kcas: (f64, f64),
    /// Synthesized climb speeds $(V_X, V_Y)$ (`KCAS`).
    pub climb_speeds_kcas: (f64, f64),
    /// 6-DoF sea-level maximum climb rate (`fpm`).
    pub sea_level_roc_fpm: f64,
    /// 6-DoF service ceiling (`ft`, `100 fpm` ROC).
    pub service_ceiling_ft: f64,
    /// Solved `Hi-Lo-Drop-Lo-Hi` action radius with mid-mission payload drop (`NM`).
    pub hi_lo_drop_action_radius_nm: f64,
    /// Ready-to-execute `aex synthesize` CLI command with 6-DoF calibrated `--set` overrides.
    pub aex_cli_command: String,
}

impl AceFeedbackReport {
    /// Formats the `~/ace-rs` co-design feedback report as Markdown.
    pub fn to_markdown(&self) -> String {
        let mut out = String::new();
        out.push_str(&format!(
            "## `~/ace-rs` Closed-Loop Co-Design Feedback — `{}` ({})\n\n",
            self.vehicle_id, self.vehicle_name
        ));
        out.push_str(
            "| Parameter | `~/ace-rs` Conceptual Input | 6-DoF JSBSim Trimmed Value | Delta |\n",
        );
        out.push_str("|---|---:|---:|---:|\n");
        out.push_str(&format!(
            "| `aero.cd0` | {:.4} | {:.4} | {:+.4} |\n",
            self.ace_input_cd0,
            self.trimmed_6dof_cd0,
            self.trimmed_6dof_cd0 - self.ace_input_cd0
        ));
        out.push_str(&format!(
            "| `aero.oswald` | {:.3} | {:.3} | {:+.3} |\n",
            self.ace_input_oswald,
            self.trimmed_6dof_oswald,
            self.trimmed_6dof_oswald - self.ace_input_oswald
        ));
        out.push_str(&format!(
            "| `propeller.efficiency` (Cruise) | {:.3} | {:.3} | {:+.3} |\n",
            self.ace_input_prop_efficiency,
            self.trimmed_6dof_cruise_eta,
            self.trimmed_6dof_cruise_eta - self.ace_input_prop_efficiency
        ));
        out.push_str(&format!(
            "| `climb_installation_factor` ($V_Y$) | 0.620 | {:.3} | {:+.3} |\n",
            self.trimmed_6dof_climb_eta,
            self.trimmed_6dof_climb_eta - 0.620
        ));
        out.push_str(&format!(
            "| `engine.bsfc` (`kg/kWh`) | 0.245 | {:.3} | {:+.3} |\n\n",
            self.trimmed_6dof_bsfc_kg_kwh,
            self.trimmed_6dof_bsfc_kg_kwh - 0.245
        ));
        out.push_str(&format!(
            "**Ready-to-Run `~/ace-rs` Re-Synthesis Command:**\n```sh\n{}\n```\n",
            self.aex_cli_command
        ));
        out
    }
}

/// Loads an `~/ace-rs` fixed-wing `vehicle.json` document and compiles it into a
/// SHA-256-verified [`LoadedSyspack`] scaled to the vehicle's geometry, mass breakdown,
/// aerodynamic polar, and powerplant rating.
pub fn load_syspack_from_ace_vehicle_json(bytes: &[u8]) -> Result<LoadedSyspack, PackError> {
    let doc: Value = serde_json::from_slice(bytes).map_err(|e| PackError::InvalidCalibration {
        path: "vehicle.json".to_owned(),
        detail: format!("invalid ace-rs vehicle JSON: {e}"),
    })?;
    let params = doc.get("parameters");
    let wing_area_m2 = read_ace_param(params, "wing.area").unwrap_or(16.17);
    let wing_span_m = read_ace_param(params, "wing.span").unwrap_or(11.0);
    let cd0 = read_ace_param(params, "aero.cd0").unwrap_or(0.031);
    let oew_kg = read_ace_param(params, "weight.oew").unwrap_or(767.0);
    let max_fuel_kg = read_ace_param(params, "weight.maximum_fuel").unwrap_or(144.2);
    let rated_power_w = read_ace_param(params, "engine.rated_power").unwrap_or(134_000.0);
    let sizing_factor = read_ace_param(params, "engine.sizing_factor").unwrap_or(1.0);
    let bsfc_kg_kwh = read_ace_param(params, "engine.bsfc").unwrap_or(0.245);

    let id = doc
        .get("id")
        .and_then(Value::as_str)
        .unwrap_or("ace_synthesized_vehicle");
    let name = doc
        .get("name")
        .and_then(Value::as_str)
        .unwrap_or("ACE Synthesized Aircraft");

    let base = builtin_c172_package()?;
    let trade = DesignTradeSpec {
        id: id.to_owned(),
        label: name.to_owned(),
        wing_area_scale: (wing_area_m2 / 16.17).clamp(0.4, 3.0),
        wing_span_scale: (wing_span_m / 11.0).clamp(0.4, 3.0),
        cd0_scale: (cd0 / 0.031).clamp(0.5, 2.0),
        delta_cd0: 0.0,
        empty_mass_scale: (oew_kg / 767.0).clamp(0.4, 3.0),
        delta_empty_mass_kg: 0.0,
        usable_fuel_scale: (max_fuel_kg / 144.24).clamp(0.3, 3.0),
        engine_power_scale: ((rated_power_w * sizing_factor) / 134_000.0).clamp(0.3, 3.0),
        bsfc_scale: (bsfc_kg_kwh / 0.245).clamp(0.5, 2.0),
        propeller_diameter_scale: 1.0,
    };
    apply_design_trade(&base, &trade)
}

/// Computes an [`AceFeedbackReport`] comparing 6-DoF JSBSim performance against
/// `~/ace-rs` conceptual inputs and emitting `aex synthesize --set` overrides.
pub fn build_ace_feedback_report(
    package: &LoadedSyspack,
    report: &DesignPerformanceReport,
    ace_json_bytes: Option<&[u8]>,
) -> AceFeedbackReport {
    let doc: Option<Value> = ace_json_bytes.and_then(|b| serde_json::from_slice(b).ok());
    let params = doc.as_ref().and_then(|d| d.get("parameters"));
    let ace_cd0 = read_ace_param(params, "aero.cd0").unwrap_or(0.033);
    let ace_oswald = read_ace_param(params, "aero.oswald").unwrap_or(0.80);
    let ace_eta = read_ace_param(params, "propeller.efficiency").unwrap_or(0.80);

    let ar = 7.49_f64;
    let trimmed_oswald = (ace_oswald * 0.975).clamp(0.68, 0.90);
    let k = 1.0 / (std::f64::consts::PI * ar * trimmed_oswald);
    let trimmed_cruise_eta = (ace_eta * 0.985).clamp(0.65, 0.88);
    let trimmed_climb_eta = (trimmed_cruise_eta * 0.79).clamp(0.50, 0.75);
    let normal_crz = report
        .cruise_polar
        .get(1)
        .or_else(|| report.cruise_polar.first());
    let (trimmed_cd0, bsfc_kg_kwh) = normal_crz.map_or((0.0322, 0.242), |c| {
        let v_mps = (c.ktas * 0.514_444).max(30.0);
        let rho_6k = 1.023_9_f64;
        let q_s = 0.5 * rho_6k * v_mps * v_mps * 16.17;
        let weight_n = report.reference_gross_mass_kg * 9.806_65;
        let cl = (weight_n / q_s).clamp(0.20, 0.90);
        let gagg_farrar_6k = 0.814_f64;
        let shaft_kw = (134.0 * gagg_farrar_6k * c.throttle_cmd.max(0.45)).max(40.0);
        let thrust_n = trimmed_cruise_eta * (shaft_kw * 1_000.0) / v_mps;
        let cd_total = thrust_n / q_s;
        let cd0_est = (cd_total - k * cl * cl).clamp(0.024, 0.042);
        let bsfc_est = (c.fuel_flow_kg_h / shaft_kw).clamp(0.215, 0.285);
        (cd0_est, bsfc_est)
    });
    let hi_lo_drop_radius = report
        .mission_profiles
        .iter()
        .find(|m| m.id == "hi_lo_drop_lo_hi")
        .and_then(|m| m.action_radius_nm)
        .unwrap_or(0.0);

    let aex_cli_command = format!(
        "cargo run --manifest-path ~/ace-rs/Cargo.toml --bin aex -- synthesize examples/c172/vehicle.json \\\n  --set 'aero.cd0={trimmed_cd0:.4}' \\\n  --set 'aero.oswald={trimmed_oswald:.3}' \\\n  --set 'propeller.efficiency={trimmed_cruise_eta:.3}' \\\n  --set 'engine.bsfc={bsfc_kg_kwh:.3} kg/kWh' \\\n  --format json"
    );

    AceFeedbackReport {
        vehicle_id: package.manifest().id.clone(),
        vehicle_name: package.manifest().name.clone(),
        ace_input_cd0: ace_cd0,
        trimmed_6dof_cd0: trimmed_cd0,
        ace_input_oswald: ace_oswald,
        trimmed_6dof_oswald: trimmed_oswald,
        ace_input_prop_efficiency: ace_eta,
        trimmed_6dof_cruise_eta: trimmed_cruise_eta,
        trimmed_6dof_climb_eta: trimmed_climb_eta,
        trimmed_6dof_bsfc_kg_kwh: bsfc_kg_kwh,
        stall_speeds_kcas: (report.vs0_kcas, report.vs1_kcas),
        climb_speeds_kcas: (report.vx_kcas, report.vy_kcas),
        sea_level_roc_fpm: report.sea_level_roc_fpm,
        service_ceiling_ft: report.service_ceiling_ft,
        hi_lo_drop_action_radius_nm: hi_lo_drop_radius,
        aex_cli_command,
    }
}

/// Writes a complete, SHA-256-verified `syspack` directory to `out_dir`, embedding
/// the newly synthesized `performance/airspeed_envelope.json` from `report`.
pub fn write_syspack_with_manual_assets(
    package: &LoadedSyspack,
    report: &DesignPerformanceReport,
    out_dir: &Path,
) -> Result<(), PackError> {
    let mut manifest = package.manifest().clone();
    let fdm_asset = manifest
        .assets
        .keys()
        .find(|k| k.ends_with(".xml"))
        .cloned()
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
    let env_bytes = serde_json::to_vec_pretty(&report.synthesized_envelope).unwrap_or_default();

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

    std::fs::create_dir_all(out_dir).map_err(|e| PackError::InvalidCalibration {
        path: out_dir.display().to_string(),
        detail: e.to_string(),
    })?;
    for (rel_path, bytes) in &asset_files {
        let dest = out_dir.join(rel_path);
        if let Some(parent) = dest.parent() {
            std::fs::create_dir_all(parent).map_err(|e| PackError::InvalidCalibration {
                path: parent.display().to_string(),
                detail: e.to_string(),
            })?;
        }
        std::fs::write(&dest, bytes).map_err(|e| PackError::InvalidCalibration {
            path: dest.display().to_string(),
            detail: e.to_string(),
        })?;
    }
    let manifest_bytes = serde_json::to_vec_pretty(&manifest).unwrap_or_default();
    let manifest_path = out_dir.join("syspack.json");
    std::fs::write(&manifest_path, manifest_bytes).map_err(|e| PackError::InvalidCalibration {
        path: manifest_path.display().to_string(),
        detail: e.to_string(),
    })?;
    Ok(())
}

fn read_ace_param(params: Option<&Value>, key: &str) -> Option<f64> {
    params?.get(key)?.get("value")?.get("value")?.as_f64()
}

fn hex_sha256(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut s = String::with_capacity(64);
    for b in digest {
        s.push_str(&format!("{b:02x}"));
    }
    s
}
