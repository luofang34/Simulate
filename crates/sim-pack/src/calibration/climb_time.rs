//! Time to climb, as WWII test reports give climb performance: the climb is
//! flown as a ladder of steady climbs, one freshly loaded model per rung, and the
//! time is the integral of energy height over the measured energy-height rate.
//! Integrating energy height rather than altitude counts the speed a constant-CAS
//! climb gains as it rises, and a rung per band keeps a supercharger's gear
//! change from being averaged across the whole climb.

use serde::{Deserialize, Serialize};

use crate::calibration::results::ClimbTimeResult;
use crate::calibration::schema::{CalibrationSuite, TakeoffBenchmark};
use crate::calibration::solver::{
    ClimbPower, FPS_PER_KT, ParameterOverrides, fly_steady_climb_powered, handbook_fdm,
};
use crate::calibration::takeoff::fly_takeoff_from_brake_release;
use crate::{LoadedSyspack, PackError};

/// Time to climb between two pressure altitudes at a held airspeed and power, the
/// way WWII test reports state climb performance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ClimbTimeBenchmark {
    /// Condition label (for example `"Rated power, 9,500 kg, SL to 3,000 m"`).
    pub label: String,
    /// Aircraft gross mass (`kg`), held for the whole climb.
    pub gross_mass_kg: f64,
    /// Pressure altitude the clock starts at (`ft`).
    pub start_alt_ft: f64,
    /// Pressure altitude the clock stops at (`ft`).
    pub end_alt_ft: f64,
    /// Calibrated airspeed of the climb schedule at sea level (`KCAS`).
    pub climb_kcas: f64,
    /// Change of the scheduled airspeed per 1,000 ft of pressure altitude
    /// (`KCAS`; negative for the falling schedule a bomber climbs on). Zero holds
    /// `climb_kcas` throughout.
    #[serde(default)]
    pub climb_kcas_per_1000ft: f64,
    /// Manifold pressure held while the throttle can reach it (`inHg`).
    #[serde(default)]
    pub target_map_inhg: Option<f64>,
    /// Crankshaft RPM held with the propeller governor.
    #[serde(default)]
    pub target_rpm: Option<f64>,
    /// Reported time (`min`).
    pub target_min: f64,
    /// When the clock starts at brake release rather than in the air: the take-off
    /// is flown to 50 ft at take-off power and its time added.
    #[serde(default)]
    pub from_brake_release: Option<BrakeReleaseStart>,
}

/// The take-off a [`ClimbTimeBenchmark`] timed from brake release begins with.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BrakeReleaseStart {
    /// Normalized take-off flap command `[0.0, 1.0]`.
    pub flap_norm: f64,
    /// Lift-off speed (`KCAS`).
    pub lift_off_kcas: f64,
    /// Speed held from lift-off to 50 ft (`KCAS`).
    pub obstacle_kcas: f64,
}

impl ClimbTimeBenchmark {
    /// Whether every field is finite, the band rises and the scheduled speed stays
    /// positive across it.
    pub(super) fn is_valid(&self) -> bool {
        let top_alt_ft = self.end_alt_ft.max(self.start_alt_ft);
        let top_kcas = self.climb_kcas + self.climb_kcas_per_1000ft * top_alt_ft / 1_000.0;
        self.gross_mass_kg.is_finite()
            && self.gross_mass_kg > 0.0
            && self.start_alt_ft.is_finite()
            && self.end_alt_ft.is_finite()
            && self.end_alt_ft > self.start_alt_ft
            && self.climb_kcas.is_finite()
            && self.climb_kcas > 0.0
            && self.climb_kcas_per_1000ft.is_finite()
            && top_kcas > 0.0
            && self.target_min.is_finite()
            && self.target_min > 0.0
    }
}

/// Widest altitude band between two rungs (`ft`).
const MAX_RUNG_SPACING_FT: f64 = 1_500.0;
const G_FPS2: f64 = 32.174_049;
/// Upper bound on rungs, far above any band a test report climbs.
const MAX_RUNGS: u32 = 100;

/// One rung: energy height (`ft`) and its rate (`ft/min`).
#[derive(Debug, Clone, Copy)]
struct Rung {
    energy_ft: f64,
    rate_fpm: f64,
}

pub(super) fn solve_climb_time(
    package: &LoadedSyspack,
    suite: &CalibrationSuite,
    bench: &ClimbTimeBenchmark,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<ClimbTimeResult, PackError> {
    let band = bench.end_alt_ft - bench.start_alt_ft;
    let intervals = (band / MAX_RUNG_SPACING_FT).ceil().max(1.0);
    let power = ClimbPower {
        map_hold_inhg: bench.target_map_inhg,
        rpm_hold: bench.target_rpm,
    };
    let mut rungs = Vec::new();
    for k in (0..=MAX_RUNGS)
        .map(f64::from)
        .take_while(|k| *k <= intervals)
    {
        let alt_ft = bench.start_alt_ft + band * k / intervals;
        let (_, ktas, rate_fpm, _) = fly_steady_climb_powered(
            &mut handbook_fdm(package, suite)?,
            alt_ft,
            bench.gross_mass_kg,
            bench.climb_kcas + bench.climb_kcas_per_1000ft * alt_ft / 1_000.0,
            false,
            base_non_payload_lbs,
            overrides,
            power,
        )?;
        let v_fps = ktas * FPS_PER_KT;
        rungs.push(Rung {
            energy_ft: alt_ft + v_fps * v_fps / (2.0 * G_FPS2),
            rate_fpm,
        });
    }
    let mut sim_min = integrate_minutes(&rungs);
    if let Some(start) = bench.from_brake_release {
        let takeoff = TakeoffBenchmark {
            label: bench.label.clone(),
            pressure_alt_ft: bench.start_alt_ft,
            gross_mass_kg: bench.gross_mass_kg,
            flap_norm: start.flap_norm,
            lift_off_kcas: start.lift_off_kcas,
            obstacle_kcas: start.obstacle_kcas,
            target_ground_roll_ft: 0.0,
            target_total_50ft_ft: 0.0,
        };
        let run = fly_takeoff_from_brake_release(
            &mut handbook_fdm(package, suite)?,
            &takeoff,
            base_non_payload_lbs,
            overrides,
        )?;
        // The ladder's first rung already flies at the scheduled speed; the
        // take-off ends slower and lower, so the energy between is climbed at the
        // first rung's rate.
        sim_min += run.elapsed_s / 60.0 + energy_gap_minutes(&rungs, run.energy_ft);
    }
    let residual_pct = 100.0 * (sim_min - bench.target_min) / bench.target_min;
    Ok(ClimbTimeResult {
        label: bench.label.clone(),
        band_ft: (bench.start_alt_ft, bench.end_alt_ft),
        sim_min,
        target_min: bench.target_min,
        residual_pct,
        passed: residual_pct.abs() <= suite.tolerances.climb_time_pct,
    })
}

/// Minutes the first rung's rate takes to climb from `energy_ft` to that rung's
/// energy height; zero when the take-off already ends above it.
fn energy_gap_minutes(rungs: &[Rung], energy_ft: f64) -> f64 {
    match rungs.first() {
        Some(first) if first.rate_fpm > 0.0 => {
            (first.energy_ft - energy_ft).max(0.0) / first.rate_fpm
        }
        Some(_) => f64::INFINITY,
        None => 0.0,
    }
}

/// Trapezoidal integral of `dE / rate` over the rungs; a rung that cannot climb
/// makes the time infinite, which no tolerance passes.
fn integrate_minutes(rungs: &[Rung]) -> f64 {
    rungs
        .windows(2)
        .map(|pair| match pair {
            [a, b] if a.rate_fpm > 0.0 && b.rate_fpm > 0.0 => {
                (b.energy_ft - a.energy_ft) * 0.5 * (1.0 / a.rate_fpm + 1.0 / b.rate_fpm)
            }
            _ => f64::INFINITY,
        })
        .sum()
}

#[cfg(test)]
mod tests;
