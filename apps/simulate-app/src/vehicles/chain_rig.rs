//! The C172 on the vehicle's own control chain: Pilotage's host core flies the first
//! shareable link's route through the Aviate flight controller while Navigate's sets
//! navigate on the same devices, the same trial the browser page steps.
//!
//! Nobody flies this vehicle by hand; the app steps the trial and draws its true state.

use std::collections::BTreeMap;

use glam::DVec3;
use sim_chain_trial::{ChainTrial, StepOutcome, first_link};
use sim_core::{AirframeArticulation, EngineStatus, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_flight_chain::C172_RATE_HZ;
use sim_navigate::trial::{DirectoryStores, ProgressRecord, parse_utc};
use tracing::info;

use crate::error::AppSimError;

/// One navigation set's latest standing against truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetStanding {
    /// Horizontal error of the set's estimate, metres.
    pub horizontal_error_m: f64,
    /// The set's own horizontal 1-sigma, when it states one.
    pub sigma_horizontal_m: Option<f64>,
}

/// The chain trial and what the operator reads of it.
pub struct ChainRig {
    /// On the heap: the trial carries the board, the host core and every navigator.
    trial: Box<ChainTrial>,
    phase: Option<String>,
    standings: BTreeMap<String, SetStanding>,
    /// The link's epoch, Unix seconds: the flight's civil time at its start.
    epoch_utc_s: f64,
}

/// The C172 at the first link's start, its plan about to be handed to Pilotage.
pub(super) fn build_chain_rig() -> Result<ChainRig, AppSimError> {
    let spec = first_link()?;
    let epoch_utc_s = parse_utc(&spec.request.epoch_utc)
        .map_err(sim_chain_trial::ChainTrialError::from)?
        .unix_seconds();
    let trial = ChainTrial::new(&spec, &DirectoryStores)?;
    Ok(ChainRig {
        trial: Box::new(trial),
        phase: None,
        standings: BTreeMap::new(),
        epoch_utc_s,
    })
}

impl ChainRig {
    /// One cycle of the flight controller.
    pub(super) const fn period(&self) -> TickSpan {
        TickSpan::from_ticks(TICKS_PER_SECOND / C172_RATE_HZ)
    }

    /// The link's epoch, Unix seconds.
    pub(super) const fn epoch_utc_s(&self) -> f64 {
        self.epoch_utc_s
    }

    /// The true vehicle state.
    pub(super) fn state(&self) -> VehicleState {
        self.trial.chain().true_state()
    }

    /// The true specific force in body forward-right-down axes, m/s².
    pub(super) fn specific_force_frd_m_s2(&self) -> DVec3 {
        self.trial
            .chain()
            .board()
            .airframe()
            .specific_force_body_m_s2()
    }

    /// The flight model's engine.
    pub(super) fn engine_status(&self) -> EngineStatus {
        self.trial.chain().board().airframe().fdm().engine_status()
    }

    /// The flight model's surfaces, gear and propeller.
    pub(super) fn articulation(&self) -> AirframeArticulation {
        self.trial.chain().board().airframe().fdm().articulation()
    }

    /// Whether the flight controller is armed.
    pub(super) fn armed(&self) -> bool {
        self.trial.chain().board().controller().is_armed()
    }

    /// Pilotage's mission phase, in its own words.
    pub(super) fn phase(&self) -> Option<&str> {
        self.phase.as_deref()
    }

    /// Each navigation set's latest standing, by set name.
    pub(super) const fn standings(&self) -> &BTreeMap<String, SetStanding> {
        &self.standings
    }

    /// One flight-controller cycle, and the trial's reports of it.
    pub(super) fn step(&mut self) -> Result<(), AppSimError> {
        // The desktop trial has no terrain camera, so no frame is ever asked for.
        let outputs = match self.trial.step()? {
            StepOutcome::Advanced(outputs) | StepOutcome::FrameWanted(_, outputs) => outputs,
        };
        for update in outputs.mission {
            for event in &update.events {
                info!(at_s = update.t_s, event, "Pilotage mission");
            }
            if update.phase != self.phase {
                info!(at_s = update.t_s, phase = ?update.phase, "mission phase");
                self.phase = update.phase;
            }
        }
        for record in outputs.records {
            if let ProgressRecord::Sample(sample) = record {
                self.standings.insert(
                    sample.set,
                    SetStanding {
                        horizontal_error_m: sample.horizontal_error_m,
                        sigma_horizontal_m: sample.sigma_horizontal_m,
                    },
                );
            }
        }
        Ok(())
    }
}
