//! Training-grade trajectory, covariance (`NEES`), sensor/spoofing input, and
//! ICAO RNP (`NSE` / `FTE` / `TSE`) telemetry recorder for [`ChainTrial`].

mod extract;
mod nees;
mod rnp;
mod summary;
mod types;

#[cfg(test)]
mod tests;

use std::collections::{BTreeMap, BTreeSet};

use glam::DVec3;
use navigate_contract::{GeodeticPosition, NavigationSolution, SensorClass, SolutionQuality};
use sim_core::{MapCell, VehicleState};
use sim_geo::{Geodetic, LocalFrame, egm96_undulation_m, geodetic_to_ecef};
use sim_navigate::solution_position_ecef;
use sim_navigate::trial::GnssCheckState;

use crate::trial::{FLIGHT_CONTROLLER_SET, SetSlot};
use crate::{ChainTrial, ChainTrialError, ChainTrialSpec, Steering};
use extract::{build_inputs, build_truth, flight_controller_entry};
use nees::score_solution;
use rnp::{RnpContext, evaluate_rnp};
use summary::SummaryAccumulator;

pub use rnp::METRES_PER_NM;
pub use types::{
    CameraFrameTelemetry, GuidanceRnpTelemetry, InputTelemetry, MetricSummary, NavigatorSummary,
    NavigatorTelemetry, NeesConsistencySummary, SpoofingSummary, TELEMETRY_SCHEMA, TelemetryConfig,
    TelemetrySample, TelemetrySummary, TruthTelemetry,
};

const MAP_CELL_SIZE_M: f64 = 500.0;
const VISUAL_HOLD_WINDOW_S: f64 = 30.0;
const GNSS_HOLD_WINDOW_S: f64 = 2.0;

#[derive(Debug, Clone, Copy)]
struct VisualCellState {
    accepted_count: u64,
    cell: [i64; 2],
    independent: bool,
    fix_at_s: f64,
}

/// Stateful recorder that samples [`ChainTrial`] at any cadence and builds
/// [`TelemetrySummary`] across the run.
#[derive(Debug)]
pub struct TelemetryRecorder {
    config: TelemetryConfig,
    steered_set: String,
    start_fix: GeodeticPosition,
    anchor_ecef: DVec3,
    anchor_frame: LocalFrame,
    visited_cells: BTreeMap<String, BTreeSet<(i64, i64)>>,
    last_visual: BTreeMap<String, VisualCellState>,
    last_gnss_at_s: Option<f64>,
    accumulator: SummaryAccumulator,
}

impl TelemetryRecorder {
    /// Creates a recorder initialized from `spec` and `config`.
    #[must_use]
    pub fn new(spec: &ChainTrialSpec, config: TelemetryConfig) -> Self {
        let s = spec.start;
        let start_geo = Geodetic::from_degrees(s.lat_deg, s.lon_deg, s.height_m);
        let start_fix =
            GeodeticPosition::new(start_geo.lat_rad, start_geo.lon_rad, start_geo.height_m);
        let steered_set = match &spec.steering {
            Steering::FlightController => FLIGHT_CONTROLLER_SET.to_owned(),
            Steering::Set(name) => name.clone(),
        };
        Self {
            config,
            steered_set,
            start_fix,
            anchor_ecef: geodetic_to_ecef(&start_geo),
            anchor_frame: LocalFrame::at(&start_geo),
            visited_cells: BTreeMap::new(),
            last_visual: BTreeMap::new(),
            last_gnss_at_s: None,
            accumulator: SummaryAccumulator::default(),
        }
    }

    /// Captures one [`TelemetrySample`] without external DEM terrain lookup
    /// (`terrain_elev_m = 0.0`).
    ///
    /// # Errors
    ///
    /// Returns [`ChainTrialError`] if reading airframe atmosphere or navigator
    /// state fails.
    pub fn sample(&mut self, trial: &mut ChainTrial) -> Result<TelemetrySample, ChainTrialError> {
        self.sample_with_terrain(trial, |_, _| None)
    }

    /// Captures one [`TelemetrySample`], querying `terrain_msl_fn(lat_deg, lon_deg)`
    /// for orthometric terrain height above MSL (metres) when available.
    ///
    /// # Errors
    ///
    /// Returns [`ChainTrialError`] if reading airframe atmosphere or navigator
    /// state fails.
    pub fn sample_with_terrain(
        &mut self,
        trial: &mut ChainTrial,
        terrain_msl_fn: impl Fn(f64, f64) -> Option<f64>,
    ) -> Result<TelemetrySample, ChainTrialError> {
        let truth = build_truth(trial, &terrain_msl_fn)?;
        if let Some(fix) = trial.sensor_latch().gnss_fix {
            self.last_gnss_at_s = Some(fix.at_s);
        }
        let inputs = build_inputs(trial, &truth);
        let holding_active = trial
            .navigation()
            .and_then(|d| d.holding_pattern.as_ref())
            .is_some();
        let truth_state = trial.truth().state;
        let fc_est = trial.chain().board().estimated_state();
        let (navigators, steered_est_ecef) =
            self.build_navigators(trial, truth.t_s, &truth_state, fc_est, holding_active)?;
        let steered_nav = navigators.iter().find(|n| n.set == self.steered_set);
        let guidance_rnp = evaluate_rnp(&RnpContext {
            steered_set: &self.steered_set,
            display: trial.navigation(),
            start_fix: self.start_fix,
            truth_ecef: truth_state.position_ecef,
            steered_est_ecef,
            steered_nav,
            configured_rnp_nm: self.config.rnp_limit_nm,
        });
        let sample = TelemetrySample {
            schema: TELEMETRY_SCHEMA,
            seed: self.config.seed,
            truth,
            inputs,
            navigators,
            guidance_rnp,
        };
        self.accumulator.push(&sample);
        Ok(sample)
    }

    /// Produces the run-level statistical summary over all recorded samples.
    #[must_use]
    pub fn finish(&self) -> TelemetrySummary {
        self.accumulator.finish(self.config.seed)
    }

    fn build_navigators(
        &mut self,
        trial: &mut ChainTrial,
        t_s: f64,
        truth: &VehicleState,
        fc_est: Option<VehicleState>,
        holding_active: bool,
    ) -> Result<(Vec<NavigatorTelemetry>, Option<DVec3>), ChainTrialError> {
        let mut out = Vec::new();
        let mut steered_ecef = None;
        let (slots, log) = trial.set_slots_mut();
        for slot in slots.iter_mut() {
            let Some(est) = slot.run.solution(log)? else {
                continue;
            };
            let solution = est.get();
            if slot.name == self.steered_set {
                steered_ecef = Some(solution_position_ecef(solution));
            }
            let visual_state = self.update_visual_cell(slot, solution, t_s);
            let is_steered = slot.name == self.steered_set;
            out.push(navigator_entry(
                slot,
                solution,
                truth,
                t_s,
                visual_state,
                self.last_gnss_at_s,
                holding_active && is_steered,
            ));
        }
        if let Some(fc) = fc_est {
            if self.steered_set == FLIGHT_CONTROLLER_SET {
                steered_ecef = Some(fc.position_ecef);
            }
            out.push(flight_controller_entry(&fc, truth));
        }
        Ok((out, steered_ecef))
    }

    fn update_visual_cell(
        &mut self,
        slot: &SetSlot,
        solution: &NavigationSolution,
        t_s: f64,
    ) -> Option<VisualCellState> {
        let accepted = slot
            .run
            .terrain_fixes()
            .get("accepted")
            .copied()
            .unwrap_or(0);
        if accepted == 0 {
            return None;
        }
        if let Some(prev) = self.last_visual.get(&slot.name).copied()
            && prev.accepted_count == accepted
        {
            return Some(prev);
        }
        let pos_ecef = slot
            .run
            .last_terrain_match()
            .and_then(|m| m.fix())
            .map_or_else(
                || solution_position_ecef(solution),
                |fix| {
                    geodetic_to_ecef(&Geodetic {
                        lat_rad: fix.position.latitude_rad,
                        lon_rad: fix.position.longitude_rad,
                        height_m: fix.position.altitude_m,
                    })
                },
            );
        let ned = self
            .anchor_frame
            .ecef_to_ned_vector(pos_ecef - self.anchor_ecef);
        let enu = DVec3::new(ned.y, ned.x, -ned.z);
        let cell = MapCell::containing(enu, MAP_CELL_SIZE_M);
        let independent = self
            .visited_cells
            .entry(slot.name.clone())
            .or_default()
            .insert((cell.cell_x, cell.cell_y));
        let state = VisualCellState {
            accepted_count: accepted,
            cell: [cell.cell_x, cell.cell_y],
            independent,
            fix_at_s: t_s,
        };
        self.last_visual.insert(slot.name.clone(), state);
        Some(state)
    }
}

fn navigator_entry(
    slot: &SetSlot,
    solution: &NavigationSolution,
    truth: &VehicleState,
    t_s: f64,
    visual_state: Option<VisualCellState>,
    last_gnss_at_s: Option<f64>,
    in_steered_hold: bool,
) -> NavigatorTelemetry {
    let scored = score_solution(solution, truth);
    let p = solution.position;
    let est_alt_msl_m = p.altitude_m - egm96_undulation_m(p.latitude_rad, p.longitude_rad);
    let contributing = contributing_names(solution.integrity.contributing);
    let has_gnss = solution.integrity.contributing.contains(SensorClass::Gnss);
    let has_vis = solution
        .integrity
        .contributing
        .contains(SensorClass::VisualLandmark)
        || solution
            .integrity
            .contributing
            .contains(SensorClass::VisualOdometry);
    let position_source = classify_position_source(solution, has_gnss, has_vis, in_steered_hold);
    let gnss_integrity = match slot.run.gnss_check().map(|c| c.state) {
        Some(GnssCheckState::Consistent) => "normal",
        Some(GnssCheckState::Disagreeing) => "degraded",
        Some(GnssCheckState::Excluded) => "excluded",
        None => "not_monitored",
    };
    let source_hold_remaining_s = if has_vis {
        visual_state.map(|vs| (VISUAL_HOLD_WINDOW_S - (t_s - vs.fix_at_s)).max(0.0))
    } else if has_gnss {
        last_gnss_at_s.map(|gs| (GNSS_HOLD_WINDOW_S - (t_s - gs)).max(0.0))
    } else {
        None
    };
    NavigatorTelemetry {
        set: slot.name.clone(),
        est_lat_deg: p.latitude_rad.to_degrees(),
        est_lon_deg: p.longitude_rad.to_degrees(),
        est_alt_msl_m,
        est_vel_ned_mps: [
            solution.velocity.north_mps,
            solution.velocity.east_mps,
            solution.velocity.down_mps,
        ],
        error_enu_m: scored.error_enu_m,
        horizontal_error_m: scored.horizontal_error_m,
        vertical_error_m: scored.vertical_error_m,
        velocity_error_mps: scored.velocity_error_mps,
        sigma_east_m: Some(scored.sigma_east_m),
        sigma_north_m: Some(scored.sigma_north_m),
        sigma_up_m: Some(scored.sigma_up_m),
        sigma_horiz_m: Some(scored.sigma_horiz_m),
        epu_95_m: Some(scored.epu_95_m),
        vpu_95_m: Some(scored.vpu_95_m),
        nees_horiz_2d: scored.nees_horiz_2d,
        nees_pos_3d: scored.nees_pos_3d,
        calibration_ratio: scored.calibration_ratio,
        position_source: position_source.to_owned(),
        gnss_integrity: gnss_integrity.to_owned(),
        contributing_sources: contributing,
        source_hold_remaining_s,
        map_cell_id: visual_state.map(|vs| vs.cell),
        independent_fix: visual_state.map(|vs| vs.independent),
    }
}

fn classify_position_source(
    solution: &NavigationSolution,
    has_gnss: bool,
    has_vis: bool,
    in_steered_hold: bool,
) -> &'static str {
    if in_steered_hold && !has_gnss && !has_vis {
        return "orbit_hold";
    }
    if solution.integrity.quality == SolutionQuality::Unusable {
        return "dead_reckoning";
    }
    let has_celestial = solution
        .integrity
        .contributing
        .contains(SensorClass::Celestial);
    match (has_gnss, has_vis, has_celestial) {
        (true, false, false) => "gnss",
        (false, true, false) => "terrain",
        (false, false, true) => "celestial",
        (false, false, false) => "inertial",
        _ => "blended",
    }
}

fn contributing_names(comp: navigate_contract::SourceComposition) -> Vec<String> {
    let table = [
        (SensorClass::Gnss, "gnss"),
        (SensorClass::VisualLandmark, "terrain"),
        (SensorClass::VisualOdometry, "visual_odometry"),
        (SensorClass::Celestial, "celestial"),
        (SensorClass::Barometric, "baro"),
        (SensorClass::Inertial, "inertial"),
    ];
    table
        .into_iter()
        .filter(|(class, _)| comp.contains(*class))
        .map(|(_, name)| name.to_owned())
        .collect()
}
