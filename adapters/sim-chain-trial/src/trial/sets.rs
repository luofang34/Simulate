//! The navigation sets beside the flight, and the flight controller's own
//! estimate scored as one more set.

use std::collections::BTreeMap;

use navigate_contract::NavigationSolution;
use sim_core::VehicleState;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_navigate::trial::{
    ProgressPosition, ProgressRecord, SampleRecord, SensorKind, SensorState, SensorStates, SetRun,
};
use sim_navigate::{NavMeans, compare_with_truth};

use crate::{ChainTrialError, DeviceOutage};

/// The name the flight controller's estimate is scored under.
pub const FLIGHT_CONTROLLER_SET: &str = "flight-controller";

/// One set's navigator and what it last reported.
pub(super) struct SetSlot {
    pub(super) name: String,
    pub(super) run: SetRun,
    /// Which of the sensors the set has.
    pub(super) has: [bool; 4],
    pub(super) reported_fixes: BTreeMap<String, u64>,
}

const KINDS: [SensorKind; 4] = [
    SensorKind::Gnss,
    SensorKind::Baro,
    SensorKind::Stars,
    SensorKind::TerrainCamera,
];

impl SetSlot {
    pub(super) fn new(name: String, run: SetRun, has: impl Fn(SensorKind) -> bool) -> Self {
        Self {
            has: KINDS.map(&has),
            name,
            run,
            reported_fixes: BTreeMap::new(),
        }
    }

    /// Whether the set has `kind`.
    pub(super) fn has(&self, kind: SensorKind) -> bool {
        KINDS
            .iter()
            .position(|k| *k == kind)
            .and_then(|index| self.has.get(index).copied())
            .unwrap_or(false)
    }

    /// The set's terrain fixes when they changed since the last call.
    pub(super) fn changed_fixes(&mut self) -> Option<BTreeMap<String, u64>> {
        let now = self.run.terrain_fixes();
        if *now == self.reported_fixes {
            return None;
        }
        self.reported_fixes = now.clone();
        Some(now.clone())
    }

    /// What each sensor is doing for the set at `t_s`: an outage of the
    /// device shows as withheld, whatever the set's own schedules say.
    pub(super) fn states(&self, t_s: f64, outages: &[DeviceOutage]) -> SensorStates {
        let state = |index: usize, kind: SensorKind| {
            let has = self.has.get(index).copied().unwrap_or(false);
            if has
                && outages
                    .iter()
                    .any(|o| o.sensor == kind && o.covers_up_to(t_s))
            {
                SensorState::ScheduledUnavailable
            } else {
                self.run.sensor_state(kind, t_s)
            }
        };
        SensorStates {
            gnss: state(0, SensorKind::Gnss),
            baro: state(1, SensorKind::Baro),
            stars: state(2, SensorKind::Stars),
            terrain_camera: state(3, SensorKind::TerrainCamera),
        }
    }
}

/// Whether a set of `means` has `kind`.
pub(super) fn has_sensor(means: &[NavMeans], kind: SensorKind) -> bool {
    means.iter().any(|m| *m == kind.means())
}

/// A set's sample: its solution against `truth`.
pub(super) fn solution_sample(
    (t_s, set): (f64, &str),
    truth: &VehicleState,
    solution: &NavigationSolution,
    sensors: SensorStates,
) -> Result<ProgressRecord, ChainTrialError> {
    let comparison = compare_with_truth(solution, truth.position_ecef).map_err(|error| {
        ChainTrialError::Invalid {
            detail: format!("set {set}: {error}"),
        }
    })?;
    let (pn, pe, _) = solution.position_cov.diagonal();
    let p = solution.position;
    Ok(ProgressRecord::Sample(SampleRecord {
        t_s,
        set: set.to_owned(),
        truth: position_of(truth),
        estimate: ProgressPosition {
            lat_deg: p.latitude_rad.to_degrees(),
            lon_deg: p.longitude_rad.to_degrees(),
            altitude_m: p.altitude_m,
        },
        horizontal_error_m: comparison.horizontal_error_m,
        vertical_error_m: comparison.vertical_error_m,
        sigma_horizontal_m: Some((pn + pe).sqrt()),
        cross_track_true_m: None,
        sensors,
    }))
}

/// The flight controller's estimate against `truth`. Aviate states no
/// covariance through this entry, so the sample carries no sigma.
pub(super) fn flight_controller_sample(
    t_s: f64,
    truth: &VehicleState,
    estimate: &VehicleState,
    sensors: SensorStates,
) -> ProgressRecord {
    let frame = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
    let error = frame.ecef_to_ned_vector(estimate.position_ecef - truth.position_ecef);
    ProgressRecord::Sample(SampleRecord {
        t_s,
        set: FLIGHT_CONTROLLER_SET.to_owned(),
        truth: position_of(truth),
        estimate: position_of(estimate),
        horizontal_error_m: error.x.hypot(error.y),
        vertical_error_m: -error.z,
        sigma_horizontal_m: None,
        cross_track_true_m: None,
        sensors,
    })
}

fn position_of(state: &VehicleState) -> ProgressPosition {
    let position = ecef_to_geodetic(state.position_ecef);
    ProgressPosition {
        lat_deg: position.lat_rad.to_degrees(),
        lon_deg: position.lon_rad.to_degrees(),
        altitude_m: position.height_m,
    }
}
