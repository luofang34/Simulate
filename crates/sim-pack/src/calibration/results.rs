//! Result types and RMSE aggregation for closed-loop POH and empirical calibration,
//! plus new-design performance and range estimation output structures.

use serde::{Deserialize, Serialize};

use crate::AirspeedEnvelope;
use crate::calibration::schema::{CalibrationSuite, EmpiricalSegmentMode, TailProfile};

/// Simulated outcome of one POH takeoff-distance benchmark.
#[derive(Debug, Clone)]
pub struct TakeoffResult {
    /// Condition label from [`crate::TakeoffBenchmark`].
    pub label: String,
    /// Field pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Simulated ground roll (`ft`).
    pub sim_ground_roll_ft: f64,
    /// Published ground roll (`ft`).
    pub target_ground_roll_ft: f64,
    /// Simulated total distance to 50 ft (`ft`).
    pub sim_total_ft: f64,
    /// Published total distance to 50 ft (`ft`).
    pub target_total_ft: f64,
    /// Calibrated airspeed when the last wheel left the runway (`kt`), where
    /// the ground roll ends.
    pub sim_lift_off_kcas: f64,
    /// The benchmark's scheduled lift-off speed (`kt` CAS).
    pub target_lift_off_kcas: f64,
    /// The CG flown, fuselage station (`in`), when the card gives one.
    pub flown_cg_fs_in: Option<f64>,
    /// Whether both distances lie within `tolerances.takeoff_pct`.
    pub passed: bool,
}

/// Simulated outcome of a single stall benchmark.
#[derive(Debug, Clone)]
pub struct StallResult {
    /// Condition label from [`crate::StallBenchmark`].
    pub label: String,
    /// Flap angle (`deg`).
    pub flap_deg: f64,
    /// Roll bank angle (`deg`).
    pub bank_deg: f64,
    /// Simulated calibrated stall speed (`KCAS`).
    pub sim_kcas: f64,
    /// Target POH calibrated stall speed (`KCAS`).
    pub target_kcas: f64,
    /// Residual `sim_kcas - target_kcas` (`kt`).
    pub residual_kcas: f64,
    /// Simulated `PEC` indicated stall speed (`KIAS`), when `PEC` is present.
    pub sim_kias: Option<f64>,
    /// Target POH indicated stall speed (`KIAS`), when present.
    pub target_kias: Option<f64>,
    /// Residual `sim_kias - target_kias` (`kt`), when both exist.
    pub residual_kias: Option<f64>,
    /// The CG flown, fuselage station (`in`), when the card gives one.
    pub flown_cg_fs_in: Option<f64>,
    /// Whether residuals lie within `tolerances.stall_kt`.
    pub passed: bool,
}

/// Simulated outcome of a steady full-throttle climb benchmark.
#[derive(Debug, Clone, Copy)]
pub struct ClimbResult {
    /// Pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Settled calibrated climb speed (`KCAS`).
    pub sim_kcas: f64,
    /// Settled propeller speed (`RPM`, `0.0` on direct-drive non-propeller models).
    pub sim_rpm: f64,
    /// Simulated steady rate of climb (`ft/min`).
    pub sim_fpm: f64,
    /// Target POH rate of climb (`ft/min`).
    pub target_fpm: f64,
    /// Residual `sim_fpm - target_fpm` (`ft/min`).
    pub residual_fpm: f64,
    /// Whether `|residual_fpm| <= tolerances.climb_fpm`.
    pub passed: bool,
}

/// Simulated outcome of a steady level cruise benchmark.
#[derive(Debug, Clone, Copy)]
pub struct CruiseResult {
    /// Pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Commanded RPM (or `0.0` when fixed throttle is used).
    pub commanded_rpm: f64,
    /// Settled propeller speed (`RPM`).
    pub sim_rpm: f64,
    /// Simulated true airspeed (`KTAS`).
    pub sim_ktas: f64,
    /// Target POH true airspeed (`KTAS`).
    pub target_ktas: f64,
    /// Residual `sim_ktas - target_ktas` (`kt`).
    pub residual_ktas: f64,
    /// Simulated percentage of rated brake horsepower (`%BHP`), when applicable.
    pub sim_pct_bhp: Option<f64>,
    /// Target POH `%BHP`, when applicable.
    pub target_pct_bhp: Option<f64>,
    /// Simulated fuel flow (`gal/hr`), when applicable.
    pub sim_gph: Option<f64>,
    /// Target POH fuel flow (`gal/hr`), when applicable.
    pub target_gph: Option<f64>,
    /// Residual `sim_gph - target_gph` (`gal/hr`), when applicable.
    pub residual_gph: Option<f64>,
    /// Whether `KTAS`, `GPH`, and `%BHP` residuals all lie within suite tolerances.
    pub passed: bool,
}

/// Simulated outcome of a replayed empirical real-world flight segment.
#[derive(Debug, Clone)]
pub struct EmpiricalSegmentResult {
    /// Segment identifier.
    pub segment_id: String,
    /// Control mode.
    pub mode: EmpiricalSegmentMode,
    /// Pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Simulated groundspeed (`kt`, `sim_ktas + wind_along_track_kt`).
    pub sim_gs_kt: f64,
    /// Observed empirical groundspeed (`kt`).
    pub observed_gs_kt: f64,
    /// Groundspeed residual `sim_gs_kt - observed_gs_kt` (`kt`).
    pub residual_gs_kt: f64,
    /// Simulated rate of climb (`ft/min`).
    pub sim_roc_fpm: f64,
    /// Observed empirical rate of climb (`ft/min`), when applicable.
    pub observed_roc_fpm: Option<f64>,
    /// Rate-of-climb residual (`ft/min`), when applicable.
    pub residual_roc_fpm: Option<f64>,
    /// Whether groundspeed and climb-rate residuals satisfy segment tolerances.
    pub passed: bool,
}

/// Complete closed-loop POH and empirical validation report for an aircraft package.
#[derive(Debug, Clone)]
pub struct CalibrationReport {
    /// Aircraft designator (for example `"C172"`).
    pub type_designator: String,
    /// Reference handbook / TCDS / flight-test source.
    pub reference_document: String,
    /// Optional certificated tail profile (`N642SP`, etc.).
    pub tail_profile: Option<TailProfile>,
    /// Static WOT ground runup `(sim_rpm, min_rpm, max_rpm, passed)`.
    pub static_runup: Option<(f64, f64, f64, bool)>,
    /// Stall benchmark results across flap and bank settings.
    pub stalls: Vec<StallResult>,
    /// Full-throttle steady climb benchmark results.
    pub climbs: Vec<ClimbResult>,
    /// Level cruise `(Alt, RPM/Throttle)` benchmark results.
    pub cruises: Vec<CruiseResult>,
    /// Replayed real-world flight segment results.
    pub empirical_segments: Vec<EmpiricalSegmentResult>,
    /// POH takeoff-distance results.
    pub takeoffs: Vec<TakeoffResult>,
    /// Root-mean-square calibrated stall error (`kt`).
    pub stall_kcas_rmse_kt: f64,
    /// Root-mean-square indicated stall error (`kt`, `0.0` if no `PEC` targets).
    pub stall_kias_rmse_kt: f64,
    /// Root-mean-square steady climb rate error (`ft/min`).
    pub climb_rmse_fpm: f64,
    /// Root-mean-square cruise true-airspeed error (`KTAS`).
    pub cruise_rmse_ktas: f64,
    /// Root-mean-square cruise fuel-flow error (`gal/hr`).
    pub cruise_rmse_gph: f64,
    /// Root-mean-square empirical segment groundspeed error (`kt`).
    pub empirical_gs_rmse_kt: f64,
    /// Whether every configured check and benchmark passed within tolerance.
    pub all_passed: bool,
}

/// Finite-difference Jacobian sensitivity and damped Gauss-Newton parameter
/// recommendation for one tunable aircraft parameter.
#[derive(Debug, Clone)]
pub struct ParameterAdjustment {
    /// Parameter identifier (`volumetric_efficiency_scale`, `bsfc_scale`, `friction_hp_delta`, `equiv_mass_lbs`).
    pub parameter: String,
    /// Perturbation step applied during finite-difference probing.
    pub perturbation_step: f64,
    /// Mean climb-rate sensitivity $\partial \mathrm{ROC} / \partial \theta$ (`fpm` per unit $\theta$).
    pub mean_d_climb_fpm: f64,
    /// Mean cruise-speed sensitivity $\partial V_{\mathrm{KTAS}} / \partial \theta$ (`kt` per unit $\theta$).
    pub mean_d_cruise_ktas: f64,
    /// Mean fuel-flow sensitivity $\partial \dot{m}_f / \partial \theta$ (`GPH` per unit $\theta$).
    pub mean_d_cruise_gph: f64,
    /// Recommended damped Gauss-Newton adjustment $\Delta \theta$ reducing normalized weighted residual norm.
    pub recommended_delta: f64,
}

/// Parameter-identification report pairing baseline residuals with finite-difference
/// sensitivities and Gauss-Newton parameter updates.
#[derive(Debug, Clone)]
pub struct SensitivityReport {
    /// Baseline closed-loop calibration report.
    pub baseline: CalibrationReport,
    /// Weighted normalized objective $\sum (r_i / \sigma_i)^2$ before adjustment.
    pub baseline_cost: f64,
    /// Linearized predicted cost after applying `adjustments`.
    pub linearized_predicted_cost: f64,
    /// Per-parameter Jacobian summaries and Gauss-Newton step recommendations.
    pub adjustments: Vec<ParameterAdjustment>,
}

/// Simulated climb rate and climb gradient at one pressure altitude.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DesignClimbPoint {
    /// ISA pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Stabilized calibrated climb speed (`KCAS`).
    pub kcas: f64,
    /// Stabilized true climb speed (`KTAS`).
    pub ktas: f64,
    /// Stabilized rate of climb (`ft/min`).
    pub roc_fpm: f64,
    /// Flight-path climb angle $\gamma$ (`deg`).
    pub climb_angle_deg: f64,
    /// Still-air climb gradient (`ft/NM`).
    pub gradient_ft_per_nm: f64,
}

/// Simulated level-cruise polar point with specific range and endurance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DesignCruisePoint {
    /// Operating condition label (for example `"Economy Cruise (4,000 ft)"`).
    pub label: String,
    /// ISA pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Normalized throttle setting (`[0.0, 1.0]`).
    pub throttle_cmd: f64,
    /// Stabilized propeller RPM (when applicable).
    pub rpm: f64,
    /// Stabilized true airspeed (`KTAS`).
    pub ktas: f64,
    /// Fuel flow (`US gal/hr`).
    pub fuel_flow_gph: f64,
    /// Fuel mass flow (`kg/hr`).
    pub fuel_flow_kg_h: f64,
    /// Whether fuel flow came directly from the JSBSim engine node (`true`) or BSFC fallback (`false`).
    pub direct_engine_fuel_flow: bool,
    /// Specific range (`NM / US gal`).
    pub specific_range_nm_per_gal: f64,
    /// Specific range (`NM / kg`).
    pub specific_range_nm_per_kg: f64,
    /// Usable endurance (`hr`) allowing a `45-min` IFR reserve.
    pub endurance_45min_res_hr: f64,
    /// Still-air range (`NM`) allowing a `45-min` IFR reserve.
    pub range_45min_res_nm: f64,
    /// Dry-tanks still-air range (`NM`, zero reserve).
    pub range_no_res_nm: f64,
}

/// One corner of the aircraft's payload–range trade-off envelope.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PayloadRangePoint {
    /// Corner label (for example `"Point B — Full Usable Fuel (Normal Cruise)"`).
    pub label: String,
    /// Payload mass (`kg`).
    pub payload_kg: f64,
    /// Usable fuel mass loaded (`kg`).
    pub fuel_kg: f64,
    /// Takeoff gross mass (`kg`).
    pub takeoff_mass_kg: f64,
    /// Cruise true airspeed (`KTAS`).
    pub cruise_ktas: f64,
    /// Still-air range with `45-min` reserve (`NM`).
    pub range_45min_res_nm: f64,
}

/// Evaluated performance of a single mission segment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissionSegmentResult {
    /// Segment identifier.
    pub id: String,
    /// Segment kind label.
    pub kind_label: String,
    /// Target / operating pressure altitude (`ft`).
    pub alt_ft: f64,
    /// Aircraft gross mass entering the segment (`kg`).
    pub start_mass_kg: f64,
    /// Aircraft gross mass leaving the segment (`kg`).
    pub end_mass_kg: f64,
    /// Payload mass dropped during this segment (`kg`).
    pub payload_dropped_kg: f64,
    /// Mean true airspeed (`KTAS`).
    pub ktas: f64,
    /// Segment fuel burned (`kg`).
    pub fuel_burn_kg: f64,
    /// Segment fuel burned (`US gal`).
    pub fuel_burn_gal: f64,
    /// Segment duration (`min`).
    pub duration_min: f64,
    /// Segment still-air distance (`NM`).
    pub distance_nm: f64,
}

/// Complete evaluated mission profile result, including payload-drop effects and action radius.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissionProfileResult {
    /// Profile identifier.
    pub id: String,
    /// Human-readable profile title.
    pub name: String,
    /// Engineering description.
    pub description: String,
    /// Initial takeoff gross mass (`kg`).
    pub initial_mass_kg: f64,
    /// Initial payload mass (`kg`).
    pub initial_payload_kg: f64,
    /// Initial usable fuel mass (`kg`).
    pub initial_fuel_kg: f64,
    /// Total payload mass dropped mid-mission (`kg`).
    pub total_payload_dropped_kg: f64,
    /// Final landing gross mass (`kg`, including reserve fuel).
    pub final_mass_kg: f64,
    /// Total mission fuel burned excluding reserve (`kg`).
    pub total_fuel_burn_kg: f64,
    /// Reserve fuel required (`kg`).
    pub reserve_fuel_kg: f64,
    /// Remaining fuel margin above reserve (`kg`).
    pub remaining_fuel_margin_kg: f64,
    /// Total mission block time (`hr`, excluding reserve hold).
    pub total_duration_hr: f64,
    /// Total still-air distance flown (`NM`).
    pub total_distance_nm: f64,
    /// Solved maximum Action / Combat Radius (`NM`), when `solve_action_radius` is `true`.
    pub action_radius_nm: Option<f64>,
    /// Counterfactual Action Radius (`NM`) if mid-mission payload were retained all the way home.
    pub no_drop_action_radius_nm: Option<f64>,
    /// Action-radius gain (`NM`) attributable to the mid-mission payload drop.
    pub drop_radius_bonus_nm: Option<f64>,
    /// Low-altitude rate-of-climb gain (`fpm`) measured in 6-DoF JSBSim immediately after payload drop.
    pub post_drop_roc_gain_fpm: Option<f64>,
    /// Segment-by-segment breakdown.
    pub segments: Vec<MissionSegmentResult>,
}

/// Complete closed-loop performance, V-speed, ceiling, glide, and range estimate
/// synthesized directly from a fixed-wing [`crate::LoadedSyspack`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DesignPerformanceReport {
    /// Package identifier (`syspack.json` `id`).
    pub package_id: String,
    /// Human-readable aircraft name (`syspack.json` `name`).
    pub package_name: String,
    /// Dry basic empty mass (`kg`) from the component tree or tail profile.
    pub empty_mass_kg: f64,
    /// Usable fuel capacity (`kg`).
    pub usable_fuel_kg: f64,
    /// Usable fuel volume (`US gal` `100LL` equivalent).
    pub usable_fuel_gal: f64,
    /// Nominal payload mass (`kg`).
    pub nominal_payload_kg: f64,
    /// Reference gross mass (`kg`) at which V-speeds and polars are evaluated.
    pub reference_gross_mass_kg: f64,
    /// Power-off full-flap stall speed $V_{S0}$ (`KCAS`).
    pub vs0_kcas: f64,
    /// Power-off clean stall speed $V_{S1}$ (`KCAS`).
    pub vs1_kcas: f64,
    /// Sea-level best angle-of-climb speed $V_X$ (`KCAS`).
    pub vx_kcas: f64,
    /// Sea-level maximum climb angle $\gamma_{\max}$ (`deg`).
    pub max_climb_angle_deg: f64,
    /// Sea-level best rate-of-climb speed $V_Y$ (`KCAS`).
    pub vy_kcas: f64,
    /// Sea-level maximum rate of climb (`ft/min`).
    pub sea_level_roc_fpm: f64,
    /// Best power-off glide speed $V_G$ (`KCAS`).
    pub vg_kcas: f64,
    /// Maximum power-off lift-to-drag ratio $(L/D)_{\max}$.
    pub max_lift_to_drag: f64,
    /// Still-air power-off glide distance per `1,000 ft` altitude loss (`NM`).
    pub glide_nm_per_1000ft: f64,
    /// Design maneuvering speed $V_A$ at $+3.8g$ (`KCAS`).
    pub va_kcas: f64,
    /// Sea-level maximum level true airspeed (`KTAS`).
    pub max_level_speed_sl_ktas: f64,
    /// Estimated service ceiling (`ft`, where $RC = 100\text{ fpm}$).
    pub service_ceiling_ft: f64,
    /// Estimated absolute ceiling (`ft`, where $RC = 0\text{ fpm}$).
    pub absolute_ceiling_ft: f64,
    /// Multi-altitude `Vy` climb profile.
    pub climb_profile: Vec<DesignClimbPoint>,
    /// Multi-altitude cruise polar, specific range, and endurance table.
    pub cruise_polar: Vec<DesignCruisePoint>,
    /// Payload–range mission trade-off points.
    pub payload_range: Vec<PayloadRangePoint>,
    /// Multi-segment operational mission profiles (`Hi-Hi-Hi`, `Hi-Lo-Drop-Lo-Hi`, `Lo-Lo-Drop-Lo`, `Hi-Loiter-Hi`).
    #[serde(default)]
    pub mission_profiles: Vec<MissionProfileResult>,
    /// Auto-synthesized [`AirspeedEnvelope`] ready for `performance/airspeed_envelope.json`.
    pub synthesized_envelope: AirspeedEnvelope,
}

pub(super) struct RawSuiteEvaluation {
    pub static_rpm: Option<(f64, bool)>,
    pub stalls: Vec<StallResult>,
    pub climbs: Vec<ClimbResult>,
    pub cruises: Vec<CruiseResult>,
    pub empirical: Vec<EmpiricalSegmentResult>,
    pub takeoffs: Vec<TakeoffResult>,
}

pub(super) fn build_report(suite: &CalibrationSuite, raw: RawSuiteEvaluation) -> CalibrationReport {
    let static_runup = match (&suite.static_runup, raw.static_rpm) {
        (Some(spec), Some((rpm, ok))) => Some((rpm, spec.min_rpm, spec.max_rpm, ok)),
        _ => None,
    };
    let stall_kcas_rmse_kt = rmse(raw.stalls.iter().map(|s| s.residual_kcas));
    let stall_kias_rmse_kt = rmse(raw.stalls.iter().filter_map(|s| s.residual_kias));
    let climb_rmse_fpm = rmse(raw.climbs.iter().map(|c| c.residual_fpm));
    let cruise_rmse_ktas = rmse(raw.cruises.iter().map(|c| c.residual_ktas));
    let cruise_rmse_gph = rmse(raw.cruises.iter().filter_map(|c| c.residual_gph));
    let empirical_gs_rmse_kt = rmse(raw.empirical.iter().map(|e| e.residual_gs_kt));

    let all_passed = static_runup.is_none_or(|(_, _, _, ok)| ok)
        && raw.stalls.iter().all(|s| s.passed)
        && raw.climbs.iter().all(|c| c.passed)
        && raw.cruises.iter().all(|c| c.passed)
        && raw.empirical.iter().all(|e| e.passed)
        && raw.takeoffs.iter().all(|t| t.passed);

    CalibrationReport {
        type_designator: suite.type_designator.clone(),
        reference_document: suite.reference_document.clone(),
        tail_profile: suite.tail_profile.clone(),
        static_runup,
        stalls: raw.stalls,
        climbs: raw.climbs,
        cruises: raw.cruises,
        empirical_segments: raw.empirical,
        takeoffs: raw.takeoffs,
        stall_kcas_rmse_kt,
        stall_kias_rmse_kt,
        climb_rmse_fpm,
        cruise_rmse_ktas,
        cruise_rmse_gph,
        empirical_gs_rmse_kt,
        all_passed,
    }
}

fn rmse(iter: impl Iterator<Item = f64>) -> f64 {
    let mut sq = 0.0;
    let mut n = 0_u32;
    for v in iter {
        sq += v * v;
        n = n.wrapping_add(1);
    }
    if n == 0 {
        0.0
    } else {
        (sq / f64::from(n)).sqrt()
    }
}
