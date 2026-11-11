//! One holding burn as a plan: the request the keeper takes, and the job that
//! flies the vessel to the burn and solves it a leg at a time.

use std::fmt;
use std::sync::Arc;

use sim_core::SimInstant;
use sim_orbital::{
    ContinuousPlantState, HoldingBurnSearch, LatchedBurnLaw, PlayerCommandPayload, SrpSurface,
    TdbEpoch,
};

use super::{KeeperModel, L2KeepingBurn, holding_error};
use crate::UniverseError;
use crate::planning::{PlanJob, PlanOutput, PlanReport, PlanRequest, PlannedCommand};

/// Exhaust velocity the burns are flown with; under constant-mass propellant
/// accounting it changes nothing, and otherwise it is a storable bipropellant's.
const EXHAUST_VEL_M_S: f64 = 3_000.0;

/// Longest coast towards the deadline flown in one slice (`s`), as long as one
/// lookahead leg of the search.
const COAST_SLICE_S: f64 = 2.0 * 86_400.0;

/// The vessel as the keeper took it, and when its burn is due.
pub(crate) struct L2HoldingRequest {
    pub(crate) model: Arc<KeeperModel>,
    pub(crate) state: ContinuousPlantState,
    pub(crate) taken_at: TdbEpoch,
    pub(crate) deadline: SimInstant,
    pub(crate) deadline_epoch: TdbEpoch,
    pub(crate) srp: Option<SrpSurface>,
    pub(crate) insertion: bool,
}

impl fmt::Debug for L2HoldingRequest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("L2HoldingRequest")
            .field("vessel", &self.model.vessel)
            .field("deadline", &self.deadline)
            .field("insertion", &self.insertion)
            .finish_non_exhaustive()
    }
}

impl PlanRequest for L2HoldingRequest {
    fn deadline(&self) -> SimInstant {
        self.deadline
    }

    fn vessel(&self) -> Option<sim_orbital::VesselId> {
        Some(self.model.vessel)
    }

    fn start(&self) -> Box<dyn PlanJob> {
        Box::new(L2HoldingJob {
            request: L2HoldingRequest {
                model: Arc::clone(&self.model),
                ..*self
            },
            stage: Stage::Coasting {
                state: self.state,
                at: self.taken_at,
            },
        })
    }
}

enum Stage {
    /// Flying the state the keeper took on to the deadline.
    Coasting {
        state: ContinuousPlantState,
        at: TdbEpoch,
    },
    /// Solving the burn at the deadline.
    Solving {
        state: ContinuousPlantState,
        search: Box<HoldingBurnSearch>,
    },
}

struct L2HoldingJob {
    request: L2HoldingRequest,
    stage: Stage,
}

impl PlanJob for L2HoldingJob {
    fn run_slice(&mut self) -> Result<Option<PlanOutput>, UniverseError> {
        let request = &self.request;
        let dynamics = request.model.dynamics(request.srp);
        match &mut self.stage {
            Stage::Coasting { state, at } => {
                let left_s = request.deadline_epoch.signed_offset_seconds_from(*at);
                if left_s > 0.0 {
                    let to = if left_s > COAST_SLICE_S {
                        at.advanced_seconds(COAST_SLICE_S)
                    } else {
                        request.deadline_epoch
                    };
                    *state = dynamics.coast(state, *at, to).map_err(holding_error)?;
                    *at = to;
                    return Ok(None);
                }
                let search =
                    HoldingBurnSearch::new(request.model.target, *state, request.deadline_epoch)
                        .map_err(holding_error)?;
                self.stage = Stage::Solving {
                    state: *state,
                    search: Box::new(search),
                };
                Ok(None)
            }
            Stage::Solving { state, search } => {
                let Some(delta_v_m_s) = search.advance(&dynamics).map_err(holding_error)? else {
                    return Ok(None);
                };
                request.burn(state, delta_v_m_s).map(Some)
            }
        }
    }
}

impl L2HoldingRequest {
    /// The burn of `delta_v_m_s` at the deadline, flown as constant thrust along the
    /// line of the two bodies sized for `state`'s whole mass.
    fn burn(
        &self,
        state: &ContinuousPlantState,
        delta_v_m_s: f64,
    ) -> Result<PlanOutput, UniverseError> {
        if delta_v_m_s == 0.0 {
            return Ok(PlanOutput::default());
        }
        let model = &self.model;
        let along = model
            .dynamics(self.srp)
            .burn_direction_icrf(self.deadline_epoch)
            .map_err(holding_error)?;
        let mass_kg = model
            .config
            .mass_model
            .total_mass_kg(state.propellant_mass_kg);
        let burn_s = model.burn_span.as_seconds().max(f64::MIN_POSITIVE);
        let thrust = PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
            dir_icrf: (along * delta_v_m_s.signum()).to_array(),
            thrust_n: mass_kg * delta_v_m_s.abs() / burn_s,
            exhaust_vel_m_s: EXHAUST_VEL_M_S,
            rw_torque_cmd_body_nm: [0.0; 3],
        });
        let commands = vec![
            PlannedCommand {
                vessel: model.vessel,
                at: self.deadline,
                payload: thrust,
            },
            PlannedCommand {
                vessel: model.vessel,
                at: self.deadline.advanced(model.burn_span),
                payload: PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::Coast),
            },
        ];
        Ok(PlanOutput {
            commands,
            report: Some(PlanReport::L2Holding(L2KeepingBurn {
                at: self.deadline,
                delta_v_m_s,
                insertion: self.insertion,
            })),
        })
    }
}
