//! Declared-versus-flown comparison of an `ace-rs` design, built only from
//! what the closed-loop flight measured.

use std::path::Path;

use serde::{Deserialize, Serialize};

use super::{AceBridgeError, AceVehicleInputs};
use crate::calibration::results::{DesignCruisePoint, DesignPerformanceReport, GlideSample};

const G0: f64 = 9.806_65;
const RHO0: f64 = 1.225;
const MPS_PER_KT: f64 = 0.514_444;
const M_PER_FT: f64 = 0.3048;

/// One declared parameter beside the value the flight measured for it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AceFeedbackRow {
    /// `ace-rs` parameter name, such as `aero.cd0`.
    pub parameter: String,
    /// Unit of both values, spelled as `ace-rs` spells it.
    pub unit: String,
    /// The design's declared value, when it declares one.
    pub declared: Option<f64>,
    /// The flown value, or `None` when the flight did not measure it.
    pub flown: Option<f64>,
    /// How the flown value was measured.
    pub basis: String,
    /// Whether [`AceFeedbackReport::aex_edit_command`] writes the flown value
    /// back into the design.
    pub feeds_back: bool,
}

/// Closed-loop flight results of a package exported from an `ace-rs` design,
/// beside the design's declared values.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AceFeedbackReport {
    /// Vehicle and package identifier.
    pub vehicle_id: String,
    /// Vehicle display name.
    pub vehicle_name: String,
    /// Declared-versus-flown parameters.
    pub rows: Vec<AceFeedbackRow>,
    /// Flown stall speeds $(V_{S0}, V_{S1})$ (`KCAS`).
    pub stall_speeds_kcas: (f64, f64),
    /// Flown climb speeds $(V_X, V_Y)$ (`KCAS`).
    pub climb_speeds_kcas: (f64, f64),
    /// Flown sea-level maximum climb rate (`ft/min`).
    pub sea_level_roc_fpm: f64,
    /// Service ceiling from the flown climb profile (`ft`).
    pub service_ceiling_ft: f64,
    /// Flown idle-power $(L/D)_{\max}$.
    pub max_lift_to_drag: f64,
    /// `aex edit` command that writes the measured values into a copy of the
    /// design, or `None` when nothing it could take was measured.
    pub aex_edit_command: Option<String>,
}

/// Compares the design `inputs` (read from `vehicle_path`) with `report`, the
/// closed-loop flight of a package exported from it.
pub fn build_ace_feedback_report(
    inputs: &AceVehicleInputs,
    vehicle_path: &Path,
    report: &DesignPerformanceReport,
) -> Result<AceFeedbackReport, AceBridgeError> {
    inputs.check_flown(&report.package_id, &report.reference)?;
    let rows = comparison_rows(inputs, report);
    let aex_edit_command = aex_edit_command(vehicle_path, &rows);
    Ok(AceFeedbackReport {
        vehicle_id: inputs.id.clone(),
        vehicle_name: inputs.name.clone(),
        rows,
        stall_speeds_kcas: (report.vs0_kcas, report.vs1_kcas),
        climb_speeds_kcas: (report.vx_kcas, report.vy_kcas),
        sea_level_roc_fpm: report.sea_level_roc_fpm,
        service_ceiling_ft: report.service_ceiling_ft,
        max_lift_to_drag: report.max_lift_to_drag,
        aex_edit_command,
    })
}

/// Declared against flown values. Only the glide-fit Oswald efficiency feeds
/// back: the powered polar's slope does not recover the flight model's own
/// induced drag (the N642SP test holds the fed-back value to it), and neither
/// fit's CD0 is the clean parasite drag `aero.cd0` means (the glide includes
/// idle propeller drag, cruise any thrust-dependent drag).
fn comparison_rows(
    inputs: &AceVehicleInputs,
    report: &DesignPerformanceReport,
) -> Vec<AceFeedbackRow> {
    let weight_n = report.reference_gross_mass_kg * G0;
    let area = report.reference.wing_area_m2;
    let aspect = report.reference.wing_span_m.powi(2) / area;
    let powered = fit_polar(&cruise_coefficients(&report.cruise_polar, weight_n, area));
    let idle = fit_polar(&glide_coefficients(&report.glide_polar, weight_n, area));
    let oswald =
        |fit: Option<(f64, f64)>| fit.map(|(_, k)| 1.0 / (std::f64::consts::PI * aspect * k));
    let cruise = "least-squares CD = CD0 + k CL^2 over the level cruise points, D = summed thrust \
                  (includes any thrust-dependent drag)";
    let glide = "least-squares CD = CD0 + k CL^2 over the idle-power glide points \
                 (CD0 includes idle propeller drag)";
    let (cd0, e) = (Some(inputs.cd0), Some(inputs.oswald));
    vec![
        row(
            "wing.area",
            "m^2",
            Some(inputs.wing_area_m2),
            Some(area),
            "flight model reference area",
        ),
        row(
            "wing.span",
            "m",
            Some(inputs.wing_span_m),
            Some(report.reference.wing_span_m),
            "flight model span",
        ),
        row("aero.cd0", "1", cd0, powered.map(|(c, _)| c), cruise),
        row("aero.oswald", "1", e, oswald(powered), cruise),
        row("aero.cd0", "1", cd0, idle.map(|(c, _)| c), glide),
        fed_back(row("aero.oswald", "1", e, oswald(idle), glide)),
        fed_back(row(
            "propeller.efficiency",
            "1",
            inputs.propeller_efficiency,
            mean(report.cruise_polar.iter().filter_map(propulsive_efficiency)),
            "summed thrust x TAS / summed brake power, mean of the level cruise points",
        )),
        fed_back(row(
            "engine.bsfc",
            "kg/kWh",
            inputs.bsfc_kg_per_kwh,
            mean(report.cruise_polar.iter().filter_map(bsfc)),
            "summed engine fuel flow / summed brake power, mean of the level cruise points",
        )),
    ]
}

fn row(
    parameter: &str,
    unit: &str,
    declared: Option<f64>,
    flown: Option<f64>,
    basis: &str,
) -> AceFeedbackRow {
    AceFeedbackRow {
        parameter: parameter.to_owned(),
        unit: unit.to_owned(),
        declared,
        flown,
        basis: basis.to_owned(),
        feeds_back: false,
    }
}

fn fed_back(row: AceFeedbackRow) -> AceFeedbackRow {
    AceFeedbackRow {
        feeds_back: true,
        ..row
    }
}

/// `(CL, CD)` of each level cruise point: lift balances weight, drag balances
/// the summed thrust.
fn cruise_coefficients(points: &[DesignCruisePoint], weight_n: f64, area: f64) -> Vec<(f64, f64)> {
    points
        .iter()
        .filter(|p| p.thrust_n > 0.0)
        .map(|p| {
            let v = p.ktas * MPS_PER_KT;
            let q_s = 0.5 * isa_density(p.pressure_alt_ft * M_PER_FT) * v * v * area;
            (weight_n / q_s, p.thrust_n / q_s)
        })
        .collect()
}

/// `(CL, CD)` of each steady glide from its flight-path angle; calibrated
/// airspeed stands in for equivalent airspeed at glide speeds and heights.
fn glide_coefficients(samples: &[GlideSample], weight_n: f64, area: f64) -> Vec<(f64, f64)> {
    samples
        .iter()
        .filter(|s| s.lift_to_drag > 1.0)
        .map(|s| {
            let v = s.kcas * MPS_PER_KT;
            let q_s = 0.5 * RHO0 * v * v * area;
            let gamma = (1.0 / s.lift_to_drag).atan();
            (weight_n * gamma.cos() / q_s, weight_n * gamma.sin() / q_s)
        })
        .collect()
}

/// Least-squares `(CD0, k)` of `CD = CD0 + k CL^2`, or `None` with fewer than
/// three points, no spread in `CL^2`, or a fit that is not a drag polar.
pub(super) fn fit_polar(points: &[(f64, f64)]) -> Option<(f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let xs = points.iter().map(|(cl, _)| cl * cl);
    let mean_x = xs.clone().sum::<f64>() / n;
    let mean_y = points.iter().map(|(_, cd)| cd).sum::<f64>() / n;
    let sxx: f64 = xs.clone().map(|x| (x - mean_x).powi(2)).sum();
    let sxy: f64 = xs
        .zip(points)
        .map(|(x, (_, cd))| (x - mean_x) * (cd - mean_y))
        .sum();
    if sxx <= 1e-9 * mean_x.max(1e-9).powi(2) {
        return None;
    }
    let k = sxy / sxx;
    let cd0 = mean_y - k * mean_x;
    (k > 0.0 && cd0 > 0.0).then_some((cd0, k))
}

fn propulsive_efficiency(p: &DesignCruisePoint) -> Option<f64> {
    let power_w = p.shaft_power_kw? * 1_000.0;
    (power_w > 0.0 && p.thrust_n > 0.0).then(|| p.thrust_n * p.ktas * MPS_PER_KT / power_w)
}

fn bsfc(p: &DesignCruisePoint) -> Option<f64> {
    let power_kw = p.shaft_power_kw?;
    (p.direct_engine_fuel_flow && power_kw > 0.0).then(|| p.fuel_flow_kg_h / power_kw)
}

fn mean(values: impl Iterator<Item = f64>) -> Option<f64> {
    let (sum, n) = values.fold((0.0, 0_u32), |(s, n), v| (s + v, n.wrapping_add(1)));
    (n > 0).then(|| sum / f64::from(n))
}

/// ISA troposphere density (`kg/m^3`) at geopotential height `h_m`.
fn isa_density(h_m: f64) -> f64 {
    RHO0 * (1.0 - 2.255_77e-5 * h_m).powf(4.255_88)
}

/// `aex edit` writing the fed-back flown values into a copy of the design
/// beside it.
fn aex_edit_command(vehicle_path: &Path, rows: &[AceFeedbackRow]) -> Option<String> {
    let sets: Vec<String> = rows
        .iter()
        .filter(|r| r.feeds_back)
        .filter_map(|r| Some((r, r.flown?)))
        .map(|(r, v)| format!("  --set '{}={v:.4} {}'", r.parameter, r.unit))
        .collect();
    if sets.is_empty() {
        return None;
    }
    let stem = vehicle_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("vehicle");
    let save_as = vehicle_path.with_file_name(format!("{stem}-flown.json"));
    Some(format!(
        "aex edit {} \\\n{} \\\n  --save-as {}",
        vehicle_path.display(),
        sets.join(" \\\n"),
        save_as.display()
    ))
}

impl AceFeedbackReport {
    /// Formats the comparison as Markdown; unmeasured values read "not measured".
    pub fn to_markdown(&self) -> String {
        let fmt =
            |v: Option<f64>| v.map_or_else(|| "not measured".to_owned(), |v| format!("{v:.4}"));
        let mut out = format!(
            "## ace-rs design vs closed-loop flight: `{}` ({})\n\n",
            self.vehicle_id, self.vehicle_name
        );
        out.push_str("| Parameter | Unit | Declared | Flown | Basis |\n|---|---|---:|---:|---|\n");
        for r in &self.rows {
            out.push_str(&format!(
                "| `{}` | {} | {} | {} | {} |\n",
                r.parameter,
                r.unit,
                r.declared
                    .map_or_else(|| "not declared".to_owned(), |v| format!("{v:.4}")),
                fmt(r.flown),
                r.basis
            ));
        }
        out.push_str(&format!(
            "\nFlown: Vs0/Vs1 {:.1}/{:.1} KCAS, Vx/Vy {:.1}/{:.1} KCAS, SL ROC {:.0} fpm, \
             service ceiling {:.0} ft, idle (L/D)max {:.2}\n",
            self.stall_speeds_kcas.0,
            self.stall_speeds_kcas.1,
            self.climb_speeds_kcas.0,
            self.climb_speeds_kcas.1,
            self.sea_level_roc_fpm,
            self.service_ceiling_ft,
            self.max_lift_to_drag
        ));
        if let Some(cmd) = &self.aex_edit_command {
            out.push_str(&format!("\n```sh\n{cmd}\n```\n"));
        }
        out
    }
}
