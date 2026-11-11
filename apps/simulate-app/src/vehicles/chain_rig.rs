//! The C172 on the vehicle's own control chain: Pilotage's host core flies the first
//! shareable link's route through the Aviate flight controller while Navigate's sets
//! navigate on the same devices, the same trial the browser page steps.
//!
//! Nobody flies this vehicle by hand. On desktop the app steps the trial; in a link's page
//! the page steps it and the app draws what the page publishes ([`PageFeed`]).

use std::collections::BTreeMap;

use glam::DVec3;
use sim_chain_trial::{ChainTrial, StepOutcome, TruthSnapshot, first_link};
use sim_core::{
    AirframeArticulation, EngineStatus, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_flight_chain::{C172_RATE_HZ, FcAvionics};
use sim_navigate::trial::{DirectoryStores, ProgressRecord, parse_utc};
use tracing::info;

use crate::error::AppSimError;

mod page;

pub use page::{PageFeed, PageSnapshot};

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
    seen: PageSnapshot,
    standings: BTreeMap<String, SetStanding>,
    /// The link's epoch, Unix seconds: the flight's civil time at its start.
    epoch_utc_s: f64,
    /// The flight controller's last usable estimate, or the declared start before its
    /// first: what the displays hold while the estimator grades its estimate unusable.
    held_estimate: VehicleState,
}

/// The C172 at the first link's start, its plan about to be handed to Pilotage.
pub(super) fn build_chain_rig() -> Result<ChainRig, AppSimError> {
    let spec = first_link()?;
    let epoch_utc_s = parse_utc(&spec.request.epoch_utc)
        .map_err(sim_chain_trial::ChainTrialError::from)?
        .unix_seconds();
    let trial = ChainTrial::new(&spec, &DirectoryStores)?;
    Ok(ChainRig::from_trial(Box::new(trial), epoch_utc_s))
}

/// The C172 a link's page flies, drawn from `feed`.
pub(super) fn page_chain_rig(feed: PageFeed) -> ChainRig {
    let seen = feed.latest();
    ChainRig {
        held_estimate: seen.avionics.estimate.unwrap_or(seen.truth.state),
        seen,
        epoch_utc_s: feed.epoch_utc_s(),
        source: Source::Page(feed),
        standings: BTreeMap::new(),
    }
}

impl ChainRig {
    /// The vehicle flown by `trial`, which a host built and may keep stepping through
    /// this rig, at the link epoch `epoch_utc_s` (Unix seconds). The trial starts at its
    /// declared start, which the displays hold until the flight controller's first
    /// estimate.
    #[must_use]
    pub fn from_trial(trial: Box<ChainTrial>, epoch_utc_s: f64) -> Self {
        let seen = PageSnapshot::of(&trial, None);
        Self {
            held_estimate: seen.avionics.estimate.unwrap_or(seen.truth.state),
            seen,
            source: Source::Own(trial),
            standings: BTreeMap::new(),
            epoch_utc_s,
        }
    }

    /// The flight controller's account of its last cycle.
    #[must_use]
    pub const fn avionics(&self) -> FcAvionics {
        self.seen.avionics
    }

    /// The estimate the displays draw: the flight controller's, or the last one it gave.
    #[must_use]
    pub const fn held_estimate(&self) -> VehicleState {
        self.held_estimate
    }

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
        self.seen.truth.state
    }

    /// The true specific force in body forward-right-down axes, m/s².
    pub(super) const fn specific_force_frd_m_s2(&self) -> DVec3 {
        self.seen.truth.specific_force_body_m_s2
    }

    /// The flight model's engine.
    pub(super) const fn engine_status(&self) -> EngineStatus {
        self.seen.truth.engine
    }

    /// The flight model's surfaces, gear and propeller.
    pub(super) const fn articulation(&self) -> AirframeArticulation {
        self.seen.truth.articulation
    }

    /// Whether the flight controller is armed, in its own word.
    pub(super) const fn armed(&self) -> bool {
        self.seen.avionics.armed
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
        match &mut self.source {
            Source::Own(trial) => {
                let phase = self.seen.phase.take();
                let (seen, standings) = step_own(trial, phase)?;
                self.seen = seen;
                self.standings.extend(standings);
            }
            Source::Page(feed) => self.seen = feed.latest(),
        }
        if let Some(estimate) = self.seen.avionics.estimate {
            self.held_estimate = estimate;
        }
        Ok(())
    }
}

/// Route time of `truth`, seconds.
pub(crate) fn route_s(truth: &TruthSnapshot) -> f64 {
    truth.at.duration_since(SimInstant::START).as_seconds()
}

/// One cycle of `trial` from mission `phase`: the chain after it and the sets' standings
/// it reported.
fn step_own(
    trial: &mut ChainTrial,
    mut phase: Option<String>,
) -> Result<(PageSnapshot, Vec<(String, SetStanding)>), AppSimError> {
    // The desktop trial has no terrain camera, so no frame is ever asked for.
    let outputs = match trial.step()? {
        StepOutcome::Advanced(outputs) | StepOutcome::FrameWanted(_, outputs) => outputs,
    };
    for update in outputs.mission {
        for event in &update.events {
            info!(at_s = update.t_s, event, "Pilotage mission");
        }
        if update.phase != phase {
            info!(at_s = update.t_s, phase = ?update.phase, "mission phase");
            phase = update.phase;
        }
    }
    let standings = outputs
        .records
        .into_iter()
        .filter_map(|record| match record {
            ProgressRecord::Sample(sample) => Some((
                sample.set,
                SetStanding {
                    horizontal_error_m: sample.horizontal_error_m,
                    sigma_horizontal_m: sample.sigma_horizontal_m,
                },
            )),
            _ => None,
        })
        .collect();
    Ok((PageSnapshot::of(trial, phase), standings))
}
