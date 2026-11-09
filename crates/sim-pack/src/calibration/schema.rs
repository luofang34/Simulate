//! Declarative, aircraft-agnostic POH and empirical flight-test benchmark schema
//! (`performance/poh_benchmarks.json`).

use serde::{Deserialize, Serialize};

use crate::PackError;
use crate::calibration::CALIBRATION_SUITE_ASSET;
use crate::calibration::climb_time::ClimbTimeBenchmark;

/// Tail-specific airframe, powerplant, avionics, and weight-and-balance metadata
/// anchoring a reference package to an actual certificated aircraft (for example
/// `N642SP`, 1998 Cessna 172S Skyhawk SP S/N `172S8050` at `N51`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TailProfile {
    /// Civil registration mark (for example `"N642SP"`).
    pub registration: String,
    /// Manufacturer airframe serial number (for example `"172S8050"`).
    pub serial_number: String,
    /// Certificated model year (for example `1998`).
    pub model_year: u16,
    /// Home aerodrome ICAO/FAA identifier (for example `"N51"`).
    pub home_base_icao: String,
    /// Installed powerplant and propeller summary.
    pub powerplant_summary: String,
    /// Installed avionics/air-data STC configuration (for example Garmin G3X `SA01899WI`).
    pub avionics_suite: String,
    /// Whether main/nose landing-gear speed fairings are installed (`+2 KTAS` in C172S POH).
    pub wheel_fairings_installed: bool,
    /// Current weighed basic empty mass (`kg`).
    pub basic_empty_mass_kg: f64,
    /// Current weighed basic empty longitudinal CG arm (`in` aft of datum).
    pub basic_empty_cg_in: f64,
    /// Usable fuel capacity (`kg`).
    pub usable_fuel_kg: f64,
}

/// Acceptance error bands applied when grading simulated residuals against targets.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CalibrationTolerances {
    /// Maximum absolute stall-speed error (`kt`, applied to both `KCAS` and `KIAS`).
    pub stall_kt: f64,
    /// Maximum absolute steady climb-rate error (`ft/min`).
    pub climb_fpm: f64,
    /// Maximum absolute steady cruise true-airspeed error (`KTAS`).
    pub cruise_ktas: f64,
    /// Maximum absolute cruise fuel-flow error (`gal/hr`).
    pub cruise_gph: f64,
    /// Maximum absolute cruise brake-horsepower percentage error (`%BHP`).
    pub cruise_pct_bhp: f64,
    /// Maximum takeoff-distance error, as a percentage of each POH distance.
    #[serde(default = "default_takeoff_pct")]
    pub takeoff_pct: f64,
    /// Maximum time-to-climb error, as a percentage of each reported time.
    #[serde(default = "default_takeoff_pct")]
    pub climb_time_pct: f64,
}

const fn default_takeoff_pct() -> f64 {
    10.0
}

/// POH takeoff-distance table entry on a paved, level, dry runway in still air.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TakeoffBenchmark {
    /// Condition label (for example `"Short field, flaps 10, 2,550 lb, SL ISA"`).
    pub label: String,
    /// Field pressure altitude (`ft`), flown at ISA temperature.
    pub pressure_alt_ft: f64,
    /// Aircraft gross mass (`kg`).
    pub gross_mass_kg: f64,
    /// Normalized flap command `[0.0, 1.0]`.
    pub flap_norm: f64,
    /// Lift-off speed (`KCAS`), where the pilot rotates.
    pub lift_off_kcas: f64,
    /// Speed held from lift-off to the obstacle (`KCAS`).
    pub obstacle_kcas: f64,
    /// Published ground roll (`ft`).
    pub target_ground_roll_ft: f64,
    /// Published total distance to clear a 50 ft obstacle (`ft`).
    pub target_total_50ft_ft: f64,
}

/// Static full-throttle ground runup propeller RPM certification window (e.g. FAA TCDS `3A12`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StaticRunupBenchmark {
    /// Field pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Minimum certificated static WOT propeller speed (`RPM`).
    pub min_rpm: f64,
    /// Maximum certificated static WOT propeller speed (`RPM`).
    pub max_rpm: f64,
}

/// Power-off stall speed benchmark at a specified flap deflection, bank angle, and mass.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StallBenchmark {
    /// Human-readable condition label (for example `"Flaps 30 deg, 0 deg bank"`).
    pub label: String,
    /// Flap setting in degrees (`0.0`, `10.0`, `30.0`, etc.).
    pub flap_deg: f64,
    /// Normalized flap setting in `[0.0, 1.0]` driving `fcs/flap-pos-norm`.
    pub flap_norm: f64,
    /// Coordinated roll bank angle (`deg`; load factor $n_z = 1 / \cos\phi$).
    pub bank_deg: f64,
    /// Aircraft gross mass (`kg`).
    pub gross_mass_kg: f64,
    /// Published POH indicated stall speed (`KIAS`), when `PEC` is present.
    pub target_kias: Option<f64>,
    /// Published POH calibrated stall speed (`KCAS`).
    pub target_kcas: f64,
}

/// Full-throttle steady climb rate benchmark at a target climb airspeed and pressure altitude.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ClimbBenchmark {
    /// ISA pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Aircraft gross mass (`kg`).
    pub gross_mass_kg: f64,
    /// Commanded climb calibrated airspeed (`KCAS`).
    pub climb_kcas: f64,
    /// Published POH indicated climb speed (`KIAS`) for reference.
    pub target_kias: f64,
    /// Whether mixture is leaned to POH best-power (`true` above `3,000 ft` or throughout).
    pub lean_best_power: bool,
    /// Published POH rate of climb (`ft/min`).
    pub target_fpm: f64,
    /// Manifold pressure the test held (`inHg`), for boosted engines whose climb
    /// power is a regulated manifold pressure rather than full throttle.
    #[serde(default)]
    pub target_map_inhg: Option<f64>,
    /// Crankshaft RPM the test held with the propeller governor, when the climb power
    /// was not the governor's maximum (a rated rather than take-off RPM).
    #[serde(default)]
    pub target_rpm: Option<f64>,
}

/// Steady level cruise benchmark at a pressure altitude and target propeller RPM (or throttle).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CruiseBenchmark {
    /// ISA pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Aircraft gross mass (`kg`).
    pub gross_mass_kg: f64,
    /// Target RPM as the cockpit tachometer reads it (crankshaft; equal to propeller
    /// RPM on a direct-drive engine), or `None` when fixed throttle is used.
    pub target_rpm: Option<f64>,
    /// Fixed throttle command `[0.0, 1.0]` used when `target_rpm` is `None`.
    pub fixed_throttle: Option<f64>,
    /// Published POH percentage of rated brake horsepower (`%BHP`), if applicable.
    pub target_pct_bhp: Option<f64>,
    /// Published POH true airspeed (`KTAS`).
    pub target_ktas: f64,
    /// Published POH leaned fuel flow (`gal/hr`), if applicable.
    pub target_gph: Option<f64>,
    /// Manifold pressure the test held (`inHg`). With a constant-speed propeller the
    /// throttle then holds this pressure while the governor holds `target_rpm`.
    #[serde(default)]
    pub target_map_inhg: Option<f64>,
}

/// Operating mode of a recorded real-world flight segment replayed in closed loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EmpiricalSegmentMode {
    /// Level cruise holding a recorded propeller speed (`RPM`).
    LevelCruiseRpm,
    /// Level cruise holding a fixed normalized throttle (`[0.0, 1.0]`).
    LevelCruiseThrottle,
    /// Full-throttle climb holding a recorded calibrated airspeed (`KCAS`).
    FullThrottleClimbKcas,
}

/// Real-world recorded flight segment (GPS/ADAHRS groundspeed & climb rate) used for
/// closed-loop simulation vs. empirical validation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EmpiricalFlightSegment {
    /// Unique segment identifier (for example `"n51_delaware_cruise_4047_4055"`).
    pub segment_id: String,
    /// Provenance of the recorded trajectory (aerodrome, media/log IDs, conditions).
    pub source_summary: String,
    /// Control mode flown during the segment.
    pub mode: EmpiricalSegmentMode,
    /// Mean pressure altitude (`ft`).
    pub pressure_alt_ft: f64,
    /// Estimated gross mass during the segment (`kg`).
    pub gross_mass_kg: f64,
    /// Commanded RPM (`LevelCruiseRpm`), throttle (`LevelCruiseThrottle`), or `KCAS` (`FullThrottleClimbKcas`).
    pub command_setpoint: f64,
    /// Estimated along-track wind component (`kt`, positive tailwind, negative headwind).
    pub wind_along_track_kt: f64,
    /// Empirically observed mean groundspeed (`kt`).
    pub observed_gs_kt: f64,
    /// Maximum acceptable groundspeed residual (`kt`).
    pub gs_tolerance_kt: f64,
    /// Empirically observed mean rate of climb (`ft/min`), when applicable.
    pub observed_roc_fpm: Option<f64>,
    /// Maximum acceptable rate-of-climb residual (`ft/min`), when `observed_roc_fpm` is set.
    pub roc_tolerance_fpm: Option<f64>,
}

/// Complete declarative POH + empirical flight-test suite shipped inside a system package.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationSuite {
    /// Package or ICAO aircraft identifier (for example `"C172"`).
    pub type_designator: String,
    /// Authoritative handbook / flight-test reference document.
    pub reference_document: String,
    /// Rated sea-level engine brake horsepower (`HP`), when piston/turboprop power `%BHP` is tracked.
    pub rated_bhp: Option<f64>,
    /// Optional specific tail-number anchor (`N642SP`, etc.).
    pub tail_profile: Option<TailProfile>,
    /// Pass/fail error tolerances by flight regime.
    pub tolerances: CalibrationTolerances,
    /// Optional static WOT ground runup RPM check.
    pub static_runup: Option<StaticRunupBenchmark>,
    /// Power-off stall benchmarks across flap settings and bank angles.
    pub stalls: Vec<StallBenchmark>,
    /// Steady full-throttle `Vy` climb benchmarks across altitude.
    pub climb_points: Vec<ClimbBenchmark>,
    /// Steady level cruise `(Alt, RPM/Throttle)` benchmarks.
    pub cruise_points: Vec<CruiseBenchmark>,
    /// Recorded real-world flight segments replayed in closed loop.
    pub empirical_segments: Vec<EmpiricalFlightSegment>,
    /// POH takeoff-distance benchmarks.
    #[serde(default)]
    pub takeoff_points: Vec<TakeoffBenchmark>,
    /// Time-to-climb benchmarks.
    #[serde(default)]
    pub climb_time_points: Vec<ClimbTimeBenchmark>,
    /// Model properties set for every handbook point (not the recorded segments)
    /// where the handbook's airplane differs from this one, such as speed fairings
    /// this tail no longer carries.
    #[serde(default)]
    pub handbook_configuration: Vec<HandbookProperty>,
}

/// One model property and the value the handbook's airplane had.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HandbookProperty {
    /// JSBSim property path (for example `systems/wheel-fairings-installed`).
    pub property: String,
    /// Value to set.
    pub value: f64,
}

impl CalibrationSuite {
    /// Parses and validates a [`CalibrationSuite`] JSON payload fail-closed.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, PackError> {
        let suite: Self =
            serde_json::from_slice(bytes).map_err(|source| PackError::CalibrationParse {
                path: CALIBRATION_SUITE_ASSET.to_owned(),
                source,
            })?;
        suite.validate()?;
        Ok(suite)
    }

    /// Validates finiteness, positive masses/tolerances, and non-empty test coverage.
    pub fn validate(&self) -> Result<(), PackError> {
        if self.type_designator.trim().is_empty() {
            return Err(invalid("type_designator must not be empty"));
        }
        let tol = &self.tolerances;
        for (name, val) in [
            ("stall_kt", tol.stall_kt),
            ("climb_fpm", tol.climb_fpm),
            ("cruise_ktas", tol.cruise_ktas),
            ("cruise_gph", tol.cruise_gph),
            ("cruise_pct_bhp", tol.cruise_pct_bhp),
            ("climb_time_pct", tol.climb_time_pct),
        ] {
            if !val.is_finite() || val <= 0.0 {
                return Err(invalid(&format!("tolerance {name} must be finite and > 0")));
            }
        }
        if self.stalls.is_empty()
            && self.climb_points.is_empty()
            && self.climb_time_points.is_empty()
            && self.cruise_points.is_empty()
            && self.empirical_segments.is_empty()
        {
            return Err(invalid(
                "calibration suite must contain at least one benchmark",
            ));
        }
        self.validate_points()
    }

    fn validate_points(&self) -> Result<(), PackError> {
        if let Some(runup) = &self.static_runup
            && (!runup.min_rpm.is_finite()
                || !runup.max_rpm.is_finite()
                || runup.min_rpm <= 0.0
                || runup.max_rpm < runup.min_rpm)
        {
            return Err(invalid("static_runup RPM bounds are invalid"));
        }
        for s in &self.stalls {
            if !s.gross_mass_kg.is_finite()
                || s.gross_mass_kg <= 0.0
                || !s.bank_deg.is_finite()
                || !(0.0..75.0).contains(&s.bank_deg)
                || !s.target_kcas.is_finite()
                || s.target_kcas <= 0.0
            {
                return Err(invalid(
                    "stall benchmark has non-finite or out-of-range fields",
                ));
            }
        }
        for c in &self.climb_points {
            if !c.pressure_alt_ft.is_finite()
                || !c.gross_mass_kg.is_finite()
                || c.gross_mass_kg <= 0.0
                || !c.climb_kcas.is_finite()
                || c.climb_kcas <= 0.0
                || !c.target_fpm.is_finite()
            {
                return Err(invalid(
                    "climb benchmark has non-finite or non-positive fields",
                ));
            }
        }
        if !self
            .climb_time_points
            .iter()
            .all(ClimbTimeBenchmark::is_valid)
        {
            return Err(invalid(
                "time-to-climb benchmark needs positive mass, speed and time and a rising altitude band",
            ));
        }
        for cr in &self.cruise_points {
            if !cr.pressure_alt_ft.is_finite()
                || !cr.gross_mass_kg.is_finite()
                || cr.gross_mass_kg <= 0.0
                || !cr.target_ktas.is_finite()
                || cr.target_ktas <= 0.0
                || (cr.target_rpm.is_none() && cr.fixed_throttle.is_none())
            {
                return Err(invalid(
                    "cruise benchmark requires finite positive mass/KTAS and target_rpm or fixed_throttle",
                ));
            }
        }
        for seg in &self.empirical_segments {
            if !seg.pressure_alt_ft.is_finite()
                || !seg.gross_mass_kg.is_finite()
                || seg.gross_mass_kg <= 0.0
                || !seg.command_setpoint.is_finite()
                || !seg.observed_gs_kt.is_finite()
                || !seg.gs_tolerance_kt.is_finite()
                || seg.gs_tolerance_kt <= 0.0
            {
                return Err(invalid("empirical flight segment has invalid parameters"));
            }
        }
        Ok(())
    }
}

fn invalid(detail: &str) -> PackError {
    PackError::InvalidCalibration {
        path: CALIBRATION_SUITE_ASSET.to_owned(),
        detail: detail.to_owned(),
    }
}

/// Operational kind of a single segment in a [`MissionProfileSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MissionSegmentKind {
    /// Ground warmup, taxi, and takeoff roll.
    WarmupTakeoff,
    /// Climb from the current altitude to `target_alt_ft` at $V_Y(h)$.
    Climb,
    /// Level cruise at `target_alt_ft` for `distance_nm` (or solved action radius).
    Cruise,
    /// High-power penetration or low-level dash at `target_alt_ft`.
    Dash,
    /// Mid-mission expendable / cargo / store release at `target_alt_ft`.
    PayloadDrop,
    /// On-station endurance loiter at `target_alt_ft` for `duration_min`.
    Loiter,
    /// Controlled descent to `target_alt_ft`.
    Descent,
    /// Terminal IFR / VFR holding reserve at `target_alt_ft` for `duration_min`.
    Reserve,
}

/// Specification of one segment in a [`MissionProfileSpec`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissionSegmentSpec {
    /// Segment identifier (e.g. `"outbound_hi_cruise"`, `"target_drop"`).
    pub id: String,
    /// Segment kind.
    pub kind: MissionSegmentKind,
    /// Operating or target pressure altitude (`ft`).
    pub target_alt_ft: f64,
    /// Fixed segment ground/air distance (`NM`), when applicable.
    #[serde(default)]
    pub distance_nm: f64,
    /// Fixed segment duration (`min`), when applicable.
    #[serde(default)]
    pub duration_min: f64,
    /// Throttle setting (`[0.0, 1.0]`).
    #[serde(default = "default_throttle")]
    pub throttle_norm: f64,
    /// Explicit payload mass (`kg`) released during a [`MissionSegmentKind::PayloadDrop`].
    #[serde(default)]
    pub drop_mass_kg: Option<f64>,
    /// Fraction (`[0.0, 1.0]`) of initial payload released during a [`MissionSegmentKind::PayloadDrop`].
    #[serde(default)]
    pub drop_fraction: Option<f64>,
    /// Parasite drag change $\Delta C_{D0}$ applied from this segment onward.
    #[serde(default)]
    pub delta_cd0: f64,
    /// Whether this segment scales with the solved mission Action Radius $R_{\text{action}}$.
    #[serde(default)]
    pub is_radius_leg: bool,
}

fn default_throttle() -> f64 {
    0.75
}

/// Complete multi-segment mission profile specification for flight manual compilation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissionProfileSpec {
    /// Profile identifier (e.g. `"hi_lo_drop_lo_hi"`).
    pub id: String,
    /// Human-readable profile title.
    pub name: String,
    /// Engineering description of the altitude/drop profile.
    pub description: String,
    /// Optional override for initial payload mass (`kg`).
    #[serde(default)]
    pub initial_payload_kg: Option<f64>,
    /// Optional override for initial usable fuel mass (`kg`).
    #[serde(default)]
    pub initial_fuel_kg: Option<f64>,
    /// When `true`, solves for the maximum Action Radius (`NM`) on `is_radius_leg` segments.
    #[serde(default)]
    pub solve_action_radius: bool,
    /// Ordered mission segments from warmup to reserve.
    pub segments: Vec<MissionSegmentSpec>,
}
