//! The C172 on the vehicle's own control chain: Pilotage's host core flies the first
//! shareable link's route through the Aviate flight controller while Navigate's sets
//! navigate on the same devices, the same trial the browser page steps.
//!
//! Nobody flies this vehicle by hand. On desktop the app steps the trial; in a link's page
//! the page steps it and the app draws what the page publishes ([`PageFeed`]).

use std::collections::BTreeMap;

use glam::DVec3;
use sim_chain_trial::{ChainTrial, StepOutcome, first_link};
use sim_core::{AirframeArticulation, EngineStatus, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_flight_chain::C172_RATE_HZ;
use sim_navigate::trial::{DirectoryStores, ProgressRecord, parse_utc};
use tracing::info;

use crate::error::AppSimError;

mod page;

pub use page::{ChainSnapshot, PageFeed};

/// One navigation set's latest standing against truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetStanding {
    /// Horizontal error of the set's estimate, metres.
    pub horizontal_error_m: f64,
    /// The set's own horizontal 1-sigma, when it states one.
    pub sigma_horizontal_m: Option<f64>,
}

/// Who steps the chain: the app, or a link's page.
enum Source {
    /// The app's own trial. On the heap: it carries the board, the host core and every
    /// navigator.
    Own(Box<ChainTrial>),
    /// The page's trial, read through what the page publishes.
    Page(PageFeed),
}

/// The chain and what the operator reads of it.
pub struct ChainRig {
    source: Source,
    /// The chain as last stepped or published.
    seen: ChainSnapshot,
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
        seen: ChainSnapshot::of(&trial, None),
        source: Source::Own(Box::new(trial)),
        standings: BTreeMap::new(),
        epoch_utc_s,
    })
}

/// The C172 a link's page flies, drawn from `feed`.
pub(super) fn page_chain_rig(feed: PageFeed) -> ChainRig {
    ChainRig {
        seen: feed.latest(),
        epoch_utc_s: feed.epoch_utc_s(),
        source: Source::Page(feed),
        standings: BTreeMap::new(),
    }
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
    pub(super) const fn state(&self) -> VehicleState {
        self.seen.state
    }

    /// The true specific force in body forward-right-down axes, m/s².
    pub(super) const fn specific_force_frd_m_s2(&self) -> DVec3 {
        self.seen.specific_force_frd_m_s2
    }

    /// The flight model's engine.
    pub(super) const fn engine_status(&self) -> EngineStatus {
        self.seen.engine
    }

    /// The flight model's surfaces, gear and propeller.
    pub(super) const fn articulation(&self) -> AirframeArticulation {
        self.seen.articulation
    }

    /// Whether the flight controller is armed.
    pub(super) const fn armed(&self) -> bool {
        self.seen.armed
    }

    /// Pilotage's mission phase, in its own words.
    pub(super) fn phase(&self) -> Option<&str> {
        self.seen.phase.as_deref()
    }

    /// Each navigation set's latest standing, by set name; a page's flight reports its
    /// sets on the page.
    pub(super) const fn standings(&self) -> &BTreeMap<String, SetStanding> {
        &self.standings
    }

    /// One flight-controller cycle of the app's own trial, and its reports; for a page's
    /// flight, the page's latest snapshot.
    pub(super) fn step(&mut self) -> Result<(), AppSimError> {
        let trial = match &mut self.source {
            Source::Own(trial) => trial,
            Source::Page(feed) => {
                self.seen = feed.latest();
                return Ok(());
            }
        };
        // The desktop trial has no terrain camera, so no frame is ever asked for.
        let outputs = match trial.step()? {
            StepOutcome::Advanced(outputs) | StepOutcome::FrameWanted(_, outputs) => outputs,
        };
        let mut phase = self.seen.phase.take();
        for update in outputs.mission {
            for event in &update.events {
                info!(at_s = update.t_s, event, "Pilotage mission");
            }
            if update.phase != phase {
                info!(at_s = update.t_s, phase = ?update.phase, "mission phase");
                phase = update.phase;
            }
        }
        self.seen = ChainSnapshot::of(trial, phase);
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
