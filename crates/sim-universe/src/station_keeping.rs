//! Holding an orbital vessel about a collinear $L_2$ point inside a [`Universe`].
//!
//! The keeper is a [`PlanSchedule`]: every maintenance interval it takes the
//! vessel's committed state and asks for a plan whose deadline is a fixed
//! planning delay later. The plan's job flies that state on to the deadline and
//! solves the holding burn there with [`sim_orbital::HoldingBurnSearch`], against
//! the forces the universe itself integrates: its gravity and ephemeris, and the
//! radiation-pressure surface on the vessel's own checkpoint. The burn is flown
//! as a short finite burn stamped at the deadline, so where and when it is made
//! depends only on the state the keeper took, never on where or how fast the
//! solve ran.

mod job;

use std::fmt;
use std::sync::Arc;

use glam::DVec3;
use sim_core::{SimInstant, TickSpan};
use sim_orbital::{
    BodyOrientationCatalog, Dop853Tolerances, EphemerisCatalog, ForceModel, L2HoldingDynamics,
    L2HoldingTarget, LagrangeError, NaifId, SrpSurface, VesselConfig, VesselId,
    l2_holding_tolerances,
};
use sim_spacetime::SolarFluxModel;

use crate::planning::{PlanRequest, PlanSchedule};
use crate::{OrbitalContext, Universe, UniverseError};

use job::L2HoldingRequest;

/// How long after taking a vessel's state the keeper's burn is made, unless set
/// otherwise: long enough for a solve to finish on any executor without holding
/// time warp in an ordinary run.
pub const DEFAULT_PLAN_DELAY: TickSpan = TickSpan::from_seconds_and_subsec_ticks(86_400, 0);

/// One burn the keeper commanded.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct L2KeepingBurn {
    /// When the burn starts.
    pub at: SimInstant,
    /// Velocity change along the line from the primary through the secondary (`m/s`),
    /// positive away from the primary.
    pub delta_v_m_s: f64,
    /// Whether this was the first burn, which puts the vessel on a bounded orbit,
    /// rather than one that keeps it there.
    pub insertion: bool,
}

/// What the keeper flies with: the bodies, the forces and the vessel, all of which
/// a plan's job carries with it.
pub(crate) struct KeeperModel {
    pub(crate) vessel: VesselId,
    pub(crate) primary: NaifId,
    pub(crate) secondary: NaifId,
    pub(crate) ephemeris: Arc<dyn EphemerisCatalog>,
    pub(crate) orientation: Arc<dyn BodyOrientationCatalog>,
    pub(crate) force_model: ForceModel,
    pub(crate) flux: SolarFluxModel,
    pub(crate) config: VesselConfig,
    pub(crate) tolerances: Dop853Tolerances,
    pub(crate) target: L2HoldingTarget,
    pub(crate) burn_span: TickSpan,
}

impl KeeperModel {
    pub(crate) fn dynamics(&self, srp: Option<SrpSurface>) -> L2HoldingDynamics<'_> {
        L2HoldingDynamics {
            primary: self.primary,
            secondary: self.secondary,
            force_model: &self.force_model,
            flux: &self.flux,
            ephemeris: self.ephemeris.as_ref(),
            orientation: self.orientation.as_ref(),
            mass_model: &self.config.mass_model,
            propellant_mode: self.config.propellant_mode,
            rw_mode: self.config.rw_mode,
            tolerances: &self.tolerances,
            srp,
        }
    }
}

/// Holds one orbital vessel of a [`Universe`] about the $L_2$ point of two bodies;
/// attach it with [`Universe::attach_plan_schedule`]. Its first plan is the
/// insertion, which puts the vessel on a bounded orbit from wherever it is.
#[derive(Clone)]
pub struct L2StationKeeper {
    model: Arc<KeeperModel>,
    plan_delay: TickSpan,
    next_burn: Option<SimInstant>,
}

impl fmt::Debug for L2StationKeeper {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("L2StationKeeper")
            .field("vessel", &self.model.vessel)
            .field("plan_delay", &self.plan_delay)
            .field("next_burn", &self.next_burn)
            .finish_non_exhaustive()
    }
}

impl L2StationKeeper {
    /// A keeper for `vessel`, flown with `config` in the universe built from
    /// `context`, burning for `burn_span` at a time towards `target`, each burn
    /// made [`DEFAULT_PLAN_DELAY`] after the state it was solved from.
    #[must_use]
    pub fn new(
        vessel: VesselId,
        (primary, secondary): (NaifId, NaifId),
        context: &OrbitalContext,
        config: VesselConfig,
        target: L2HoldingTarget,
        burn_span: TickSpan,
    ) -> Self {
        let model = KeeperModel {
            vessel,
            primary,
            secondary,
            ephemeris: Arc::clone(&context.ephemeris),
            orientation: Arc::clone(&context.orientation),
            force_model: context.force_model.clone(),
            flux: SolarFluxModel::standard(),
            config,
            tolerances: l2_holding_tolerances(),
            target,
            burn_span,
        };
        Self {
            model: Arc::new(model),
            plan_delay: DEFAULT_PLAN_DELAY,
            next_burn: None,
        }
    }

    /// This keeper making each burn `delay` after the state it was solved from.
    #[must_use]
    pub const fn with_plan_delay(mut self, delay: TickSpan) -> Self {
        self.plan_delay = delay;
        self
    }

    /// The vessel's position from $L_2$ in the rotating frame of the two bodies, at the
    /// universe's current instant (`m`).
    ///
    /// # Errors
    /// The vessel is not orbital or not about the secondary, or the frame failed.
    pub fn offset_from_l2_m(&self, universe: &Universe) -> Result<DVec3, UniverseError> {
        let state = universe.inertial_state(self.model.vessel)?;
        let epoch = universe.epoch().tdb_at(universe.now());
        let srp = srp_surface(universe, self.model.vessel);
        let (offset, _) = self
            .model
            .dynamics(srp)
            .offset_from_l2(&state, epoch)
            .map_err(holding_error)?;
        Ok(offset)
    }

    /// Moves the maintenance grid on past `now`; burns stay on the grid of epochs
    /// from the first, and a keeper not asked for a while plans once, not once per
    /// missed epoch.
    fn move_on(&mut self, now: SimInstant) {
        let interval = TickSpan::from_seconds_and_subsec_ticks(
            self.model.target.maintenance_interval_s.max(1.0).round() as u64,
            0,
        );
        let mut next = self.next_burn.unwrap_or(now).advanced(interval);
        while next <= now {
            next = next.advanced(interval);
        }
        self.next_burn = Some(next);
    }
}

impl PlanSchedule for L2StationKeeper {
    fn next_due(&self) -> Option<SimInstant> {
        Some(self.next_burn.unwrap_or(SimInstant::START))
    }

    fn plan(&mut self, universe: &Universe) -> Result<Option<Arc<dyn PlanRequest>>, UniverseError> {
        let now = universe.now();
        let insertion = self.next_burn.is_none();
        self.move_on(now);
        let state = universe.inertial_state(self.model.vessel)?;
        let deadline = now.advanced(self.plan_delay);
        let request = L2HoldingRequest {
            model: Arc::clone(&self.model),
            state,
            taken_at: universe.epoch().tdb_at(now),
            deadline,
            deadline_epoch: universe.epoch().tdb_at(deadline),
            srp: srp_surface(universe, self.model.vessel),
            insertion,
        };
        Ok(Some(Arc::new(request)))
    }

    fn clone_box(&self) -> Box<dyn PlanSchedule> {
        Box::new(self.clone())
    }
}

fn srp_surface(universe: &Universe, vessel: VesselId) -> Option<SrpSurface> {
    universe
        .orbital()
        .committed_checkpoint(vessel)
        .and_then(|checkpoint| checkpoint.gnc.srp_surface)
}

pub(crate) fn holding_error(error: LagrangeError) -> UniverseError {
    UniverseError::Holding(Box::new(error))
}
