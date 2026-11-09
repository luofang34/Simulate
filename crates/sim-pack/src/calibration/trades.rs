//! Overall airframe design (`总体设计`) and powerplant (`动力装置`) trade-study
//! synthesis and 6-DoF JSBSim comparison engine.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::calibration::design::estimate_design_performance;
use crate::calibration::results::DesignPerformanceReport;
use crate::{
    AIRSPEED_ENVELOPE_ASSET, CALIBRATION_SUITE_ASSET, LoadedSyspack, MemoryPackageSource,
    PITOT_STATIC_CALIBRATION_ASSET, PackError,
};

const POUNDS_PER_KG: f64 = 2.204_622_621_8;

/// Parametric trade-study modification to an aircraft's overall geometry, mass,
/// drag polar, or powerplant/propeller installation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DesignTradeSpec {
    /// Short identifier (e.g. `"high_ar_clean"`, `"uprated_io390"`).
    pub id: String,
    /// Human-readable candidate label.
    pub label: String,
    /// Scale factor on wing reference area $S_{\text{ref}}$.
    #[serde(default = "one")]
    pub wing_area_scale: f64,
    /// Scale factor on wing span $b$.
    #[serde(default = "one")]
    pub wing_span_scale: f64,
    /// Scale factor on clean zero-lift drag $C_{D0}$.
    #[serde(default = "one")]
    pub cd0_scale: f64,
    /// Additive change to clean zero-lift drag $\Delta C_{D0}$.
    #[serde(default)]
    pub delta_cd0: f64,
    /// Scale factor on basic empty mass.
    #[serde(default = "one")]
    pub empty_mass_scale: f64,
    /// Additive change to basic empty mass (`kg`).
    #[serde(default)]
    pub delta_empty_mass_kg: f64,
    /// Scale factor on usable fuel capacity.
    #[serde(default = "one")]
    pub usable_fuel_scale: f64,
    /// Scale factor on rated engine shaft power (`BHP` / `W`) and displacement.
    #[serde(default = "one")]
    pub engine_power_scale: f64,
    /// Scale factor on brake specific fuel consumption (`BSFC`).
    #[serde(default = "one")]
    pub bsfc_scale: f64,
    /// Scale factor on propeller diameter.
    #[serde(default = "one")]
    pub propeller_diameter_scale: f64,
}

fn one() -> f64 {
    1.0
}

impl Default for DesignTradeSpec {
    fn default() -> Self {
        Self {
            id: "baseline".to_owned(),
            label: "Baseline Configuration".to_owned(),
            wing_area_scale: 1.0,
            wing_span_scale: 1.0,
            cd0_scale: 1.0,
            delta_cd0: 0.0,
            empty_mass_scale: 1.0,
            delta_empty_mass_kg: 0.0,
            usable_fuel_scale: 1.0,
            engine_power_scale: 1.0,
            bsfc_scale: 1.0,
            propeller_diameter_scale: 1.0,
        }
    }
}

/// Evaluated 6-DoF performance and mission metrics for one [`DesignTradeSpec`] candidate.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DesignTradeCandidateResult {
    /// Candidate trade specification.
    pub spec: DesignTradeSpec,
    /// Full 6-DoF performance and mission report for this candidate.
    pub report: DesignPerformanceReport,
    /// Normal cruise true airspeed (`KTAS` at `6,000 ft`).
    pub normal_cruise_ktas: f64,
    /// Normal cruise specific range (`NM/gal`).
    pub normal_specific_range_nm_gal: f64,
    /// Zero-payload ferry range (`NM`, `45-min` reserve).
    pub ferry_range_nm: f64,
    /// Solved `Hi-Lo-Drop-Lo-Hi` tactical/delivery action radius (`NM`).
    pub hi_lo_drop_radius_nm: f64,
    /// Solved `Lo-Lo-Drop-Lo` low-altitude utility action radius (`NM`).
    pub lo_lo_drop_radius_nm: f64,
}

/// Complete multi-candidate design & powerplant trade study report.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DesignTradeStudyReport {
    /// Base package identifier.
    pub base_package_id: String,
    /// Base package name.
    pub base_package_name: String,
    /// Evaluated candidates (starting with baseline).
    pub candidates: Vec<DesignTradeCandidateResult>,
}

/// Returns the four canonical overall-design and powerplant trade candidates:
/// 1. `baseline` — Nominal reference aircraft
/// 2. `high_ar_clean_airframe` — `+8%` span ($AR \uparrow 16.6\%$), `-6%` $C_{D0}$, `+10 kg` structural wing mass
/// 3. `uprated_powerplant` — `+14%` engine shaft power (`205 HP` class), `-4%` BSFC, `+8 kg` engine mass
/// 4. `extended_range_tanks` — `+22%` usable fuel capacity, `+4%` wing area, `+6 kg` tank plumbing mass
pub fn default_design_trades() -> Vec<DesignTradeSpec> {
    vec![
        DesignTradeSpec::default(),
        DesignTradeSpec {
            id: "high_ar_clean_airframe".to_owned(),
            label: "High-AR Clean Wing (+8% Span, -6% CD0, +10 kg)".to_owned(),
            wing_span_scale: 1.08,
            cd0_scale: 0.94,
            delta_empty_mass_kg: 10.0,
            ..DesignTradeSpec::default()
        },
        DesignTradeSpec {
            id: "uprated_powerplant".to_owned(),
            label: "Uprated Powerplant (+14% Power, -4% BSFC, +8 kg)".to_owned(),
            engine_power_scale: 1.14,
            bsfc_scale: 0.96,
            delta_empty_mass_kg: 8.0,
            ..DesignTradeSpec::default()
        },
        DesignTradeSpec {
            id: "extended_range_tanks".to_owned(),
            label: "Extended Wet-Wing Fuel (+22% Fuel, +4% Area, +6 kg)".to_owned(),
            wing_area_scale: 1.04,
            usable_fuel_scale: 1.22,
            delta_empty_mass_kg: 6.0,
            ..DesignTradeSpec::default()
        },
    ]
}

/// Applies a [`DesignTradeSpec`] to a [`LoadedSyspack`], producing a new SHA-256-verified
/// [`LoadedSyspack`] with modified JSBSim XML aerodynamics, mass properties, and powerplant.
pub fn apply_design_trade(
    package: &LoadedSyspack,
    trade: &DesignTradeSpec,
) -> Result<LoadedSyspack, PackError> {
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
    let raw_bytes = package
        .asset(&fdm_asset)
        .ok_or_else(|| PackError::InvalidCalibration {
            path: fdm_asset.clone(),
            detail: "missing JSBSim XML bytes".to_owned(),
        })?;
    let raw_xml = std::str::from_utf8(raw_bytes).map_err(|e| PackError::InvalidCalibration {
        path: fdm_asset.clone(),
        detail: e.to_string(),
    })?;
    let mut xml = scale_xml_tag(raw_xml, "wingarea", trade.wing_area_scale, 0.0);
    xml = scale_xml_tag(&xml, "wingspan", trade.wing_span_scale, 0.0);
    xml = scale_xml_tag(
        &xml,
        "emptywt",
        trade.empty_mass_scale,
        trade.delta_empty_mass_kg * POUNDS_PER_KG,
    );
    xml = scale_xml_tag(&xml, "maxhp", trade.engine_power_scale, 0.0);
    xml = scale_xml_tag(&xml, "displacement", trade.engine_power_scale, 0.0);
    xml = scale_xml_tag(&xml, "power", trade.engine_power_scale, 0.0);
    xml = scale_xml_tag(&xml, "bsfc", trade.bsfc_scale, 0.0);
    xml = scale_xml_tag(&xml, "diameter", trade.propeller_diameter_scale, 0.0);
    xml = scale_xml_tag(&xml, "capacity", trade.usable_fuel_scale, 0.0);
    xml = scale_xml_tag(&xml, "contents", trade.usable_fuel_scale, 0.0);
    xml = scale_drag_polar(&xml, trade);

    for comp in &mut manifest.components {
        if comp.dry_mass_kg > 1.0 {
            comp.dry_mass_kg =
                comp.dry_mass_kg * trade.empty_mass_scale + trade.delta_empty_mass_kg;
        }
        if comp.initial_fluid_mass_kg > 1.0 {
            comp.initial_fluid_mass_kg *= trade.usable_fuel_scale;
        }
    }

    rebuild_verified_package(package, manifest, &fdm_asset, xml.into_bytes(), trade)
}

fn rebuild_verified_package(
    package: &LoadedSyspack,
    mut manifest: syspack::SyspackManifest,
    fdm_asset: &str,
    fdm_bytes: Vec<u8>,
    trade: &DesignTradeSpec,
) -> Result<LoadedSyspack, PackError> {
    let mut files: BTreeMap<String, Vec<u8>> = BTreeMap::new();
    files.insert(fdm_asset.to_owned(), fdm_bytes);
    if let Some(mut env) = package.airspeed_envelope().cloned() {
        env.reference_mass_kg = env.reference_mass_kg * trade.empty_mass_scale
            + trade.delta_empty_mass_kg
            + 144.2 * (trade.usable_fuel_scale - 1.0).max(0.0);
        let bytes = serde_json::to_vec_pretty(&env).unwrap_or_default();
        files.insert(AIRSPEED_ENVELOPE_ASSET.to_owned(), bytes);
    }
    if let Some(pec) = package.pitot_static_calibration() {
        let bytes = serde_json::to_vec_pretty(pec).unwrap_or_default();
        files.insert(PITOT_STATIC_CALIBRATION_ASSET.to_owned(), bytes);
    }
    if let Some(mut suite) = package.calibration_suite().cloned() {
        if let Some(tail) = &mut suite.tail_profile {
            tail.basic_empty_mass_kg =
                tail.basic_empty_mass_kg * trade.empty_mass_scale + trade.delta_empty_mass_kg;
            tail.usable_fuel_kg *= trade.usable_fuel_scale;
        }
        let bytes = serde_json::to_vec_pretty(&suite).unwrap_or_default();
        files.insert(CALIBRATION_SUITE_ASSET.to_owned(), bytes);
    }
    for (rel_path, bytes) in &files {
        if let Some(entry) = manifest.assets.get_mut(rel_path) {
            entry.sha256 = hex_sha256(bytes);
            entry.size_bytes = bytes.len() as u64;
        }
    }
    let manifest_bytes = serde_json::to_vec_pretty(&manifest).unwrap_or_default();
    let mem = MemoryPackageSource::new(manifest_bytes, files);
    LoadedSyspack::load(&mem)
}

/// Evaluates a list of [`DesignTradeSpec`] candidates in 6-DoF JSBSim and returns a
/// side-by-side [`DesignTradeStudyReport`].
pub fn run_design_trade_study(
    package: &LoadedSyspack,
    trades: &[DesignTradeSpec],
) -> Result<DesignTradeStudyReport, PackError> {
    let mut candidates = Vec::with_capacity(trades.len());
    for spec in trades {
        let traded = apply_design_trade(package, spec)?;
        let report = estimate_design_performance(&traded)?;
        let (normal_cruise_ktas, normal_specific_range_nm_gal) = report
            .cruise_polar
            .get(1)
            .or_else(|| report.cruise_polar.first())
            .map_or((0.0, 0.0), |c| (c.ktas, c.specific_range_nm_per_gal));
        let ferry_range_nm = report
            .payload_range
            .last()
            .map_or(0.0, |p| p.range_45min_res_nm);
        let hi_lo_drop_radius_nm = report
            .mission_profiles
            .iter()
            .find(|m| m.id == "hi_lo_drop_lo_hi")
            .and_then(|m| m.action_radius_nm)
            .unwrap_or(0.0);
        let lo_lo_drop_radius_nm = report
            .mission_profiles
            .iter()
            .find(|m| m.id == "lo_lo_drop_lo")
            .and_then(|m| m.action_radius_nm)
            .unwrap_or(0.0);
        candidates.push(DesignTradeCandidateResult {
            spec: spec.clone(),
            report,
            normal_cruise_ktas,
            normal_specific_range_nm_gal,
            ferry_range_nm,
            hi_lo_drop_radius_nm,
            lo_lo_drop_radius_nm,
        });
    }
    Ok(DesignTradeStudyReport {
        base_package_id: package.manifest().id.clone(),
        base_package_name: package.manifest().name.clone(),
        candidates,
    })
}

impl DesignTradeStudyReport {
    /// Renders the trade-study comparison as a Markdown engineering report.
    pub fn to_markdown(&self) -> String {
        let mut out = String::new();
        out.push_str(&format!(
            "# Overall Design & Powerplant Trade Study — `{}` ({})\n\n",
            self.base_package_id, self.base_package_name
        ));
        out.push_str(
            "| Candidate | Empty (kg) | Fuel (gal) | Vs0 (kt) | Vy (kt) | SL ROC (fpm) | Ceiling (ft) | L/D max | Norm Crz (KTAS) | Spec. Range (NM/gal) | Ferry (NM) | Hi-Lo-Drop Radius (NM) |\n",
        );
        out.push_str("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n");
        for c in &self.candidates {
            let r = &c.report;
            out.push_str(&format!(
                "| {} | {:.1} | {:.1} | {:.1} | {:.1} | {:.0} | {:.0} | {:.2} | {:.1} | {:.2} | {:.0} | {:.0} |\n",
                c.spec.label,
                r.empty_mass_kg,
                r.usable_fuel_gal,
                r.vs0_kcas,
                r.vy_kcas,
                r.sea_level_roc_fpm,
                r.service_ceiling_ft,
                r.max_lift_to_drag,
                c.normal_cruise_ktas,
                c.normal_specific_range_nm_gal,
                c.ferry_range_nm,
                c.hi_lo_drop_radius_nm,
            ));
        }
        out
    }
}

fn scale_xml_tag(xml: &str, tag: &str, scale: f64, offset: f64) -> String {
    if (scale - 1.0).abs() < 1e-9 && offset.abs() < 1e-9 {
        return xml.to_owned();
    }
    let open_prefix = format!("<{tag}");
    let close_tag = format!("</{tag}>");
    let Some(start_idx) = xml.find(&open_prefix) else {
        return xml.to_owned();
    };
    let Some(gt_rel) = xml.get(start_idx..).and_then(|s| s.find('>')) else {
        return xml.to_owned();
    };
    let val_start = start_idx.wrapping_add(gt_rel).wrapping_add(1);
    let Some(close_rel) = xml.get(val_start..).and_then(|s| s.find(&close_tag)) else {
        return xml.to_owned();
    };
    let val_end = val_start.wrapping_add(close_rel);
    let Some(inner) = xml.get(val_start..val_end) else {
        return xml.to_owned();
    };
    let Ok(parsed) = inner.trim().parse::<f64>() else {
        return xml.to_owned();
    };
    let new_val = (parsed * scale + offset).max(0.000_1);
    let mut out = String::with_capacity(xml.len().wrapping_add(16));
    if let Some(prefix) = xml.get(..val_start) {
        out.push_str(prefix);
    }
    out.push_str(&format!(" {new_val:.4} "));
    if let Some(suffix) = xml.get(val_end..) {
        out.push_str(suffix);
    }
    out
}

fn scale_drag_polar(xml: &str, trade: &DesignTradeSpec) -> String {
    let ar_scale = (trade.wing_span_scale * trade.wing_span_scale) / trade.wing_area_scale.max(0.1);
    let mut out = xml.to_owned();
    for base_cd0 in ["0.029400", "0.0294", "0.0255", "0.0310", "0.03"] {
        if out.contains(base_cd0)
            && let Ok(v) = base_cd0.parse::<f64>()
        {
            let new_cd0 = (v * trade.cd0_scale + trade.delta_cd0).max(0.012);
            out = out.replacen(base_cd0, &format!("{new_cd0:.6}"), 1);
            break;
        }
    }
    for base_k in ["0.059500", "0.0595", "0.053184", "0.06"] {
        if out.contains(base_k)
            && let Ok(v) = base_k.parse::<f64>()
        {
            let new_k = (v / ar_scale.max(0.25)).max(0.020);
            out = out.replacen(base_k, &format!("{new_k:.6}"), 1);
            break;
        }
    }
    out
}

fn hex_sha256(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut s = String::with_capacity(64);
    for b in digest {
        s.push_str(&format!("{b:02x}"));
    }
    s
}
