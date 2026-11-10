//! Holding an orbital vessel about a collinear $L_2$ point inside a [`Universe`].
//!
//! Every maintenance interval the keeper solves the holding burn with
//! [`L2HoldingDynamics::holding_burn_m_s`] against the forces the universe
//! itself integrates: the universe's gravity and ephemeris, and the
//! radiation-pressure surface on the vessel's own checkpoint. The burn is
//! flown as a short finite burn through the universe's command log.

use std::sync::Arc;

use glam::DVec3;
use sim_core::{SimInstant, TickSpan};
use sim_orbital::{
    BodyOrientationCatalog, ContinuousPlantState, Dop853Tolerances, EphemerisCatalog, ForceModel,
    L2HoldingDynamics, L2HoldingTarget, LagrangeError, LatchedBurnLaw, NaifId,
    PlayerCommandPayload, SrpSurface, TdbEpoch, VesselConfig, VesselId, l2_holding_tolerances,
};
use sim_spacetime::SolarFluxModel;

use crate::{OrbitalContext, Universe, UniverseError};

/// Exhaust velocity the burns are flown with; under constant-mass propellant
/// accounting it changes nothing, and otherwise it is a storable bipropellant's.
const EXHAUST_VEL_M_S: f64 = 3_000.0;

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

/// Holds one orbital vessel of a [`Universe`] about the $L_2$ point of two bodies.
pub struct L2StationKeeper {
    vessel: VesselId,
    primary: NaifId,
    secondary: NaifId,
    ephemeris: Arc<dyn EphemerisCatalog>,
    orientation: Arc<dyn BodyOrientationCatalog>,
    force_model: ForceModel,
    flux: SolarFluxModel,
    config: VesselConfig,
    tolerances: Dop853Tolerances,
    target: L2HoldingTarget,
    burn_span: TickSpan,
    next_burn: Option<SimInstant>,
}

impl L2StationKeeper {
    /// A keeper for `vessel`, flown with `config` in the universe built from
    /// `context`, burning for `burn_span` at a time towards `target`.
    #[must_use]
    pub fn new(
        vessel: VesselId,
        (primary, secondary): (NaifId, NaifId),
        context: &OrbitalContext,
        config: VesselConfig,
        target: L2HoldingTarget,
        burn_span: TickSpan,
    ) -> Self {
        Self {
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
            next_burn: None,
        }
    }

    /// Burns when a maintenance epoch is due; call it after each advance of the
    /// universe. The first call is the insertion, which puts the vessel on a bounded
    /// orbit from wherever it is; the next is due a maintenance interval later.
    ///
    /// # Errors
    /// The vessel is not orbital or not about the secondary, or the burn could not be
    /// solved or commanded.
    pub fn update(
        &mut self,
        universe: &mut Universe,
    ) -> Result<Option<L2KeepingBurn>, UniverseError> {
        let now = universe.now();
        let insertion = self.next_burn.is_none();
        if self.next_burn.is_some_and(|due| now < due) {
            return Ok(None);
        }
        // Burns stay on the grid of maintenance epochs from the insertion; a keeper
        // that was not asked for a while makes one burn, not one per missed epoch.
        let interval = TickSpan::from_seconds_and_subsec_ticks(
            self.target.maintenance_interval_s.max(1.0).round() as u64,
            0,
        );
        let mut next = self.next_burn.unwrap_or(now).advanced(interval);
        while next <= now {
            next = next.advanced(interval);
        }
        self.next_burn = Some(next);
        let state = universe.inertial_state(self.vessel)?;
        let epoch = universe.epoch().tdb_at(now);
        self.burn(universe, &state, epoch, insertion)
    }

    /// The vessel's position from $L_2$ in the rotating frame of the two bodies, at the
    /// universe's current instant (`m`).
    ///
    /// # Errors
    /// The vessel is not orbital or not about the secondary, or the frame failed.
    pub fn offset_from_l2_m(&self, universe: &Universe) -> Result<DVec3, UniverseError> {
        let state = universe.inertial_state(self.vessel)?;
        let epoch = universe.epoch().tdb_at(universe.now());
        let srp = self.srp_surface(universe);
        let (offset, _) = self
            .dynamics(srp)
            .offset_from_l2(&state, epoch)
            .map_err(holding_error)?;
        Ok(offset)
    }

    fn srp_surface(&self, universe: &Universe) -> Option<SrpSurface> {
        universe
            .orbital()
            .committed_checkpoint(self.vessel)
            .and_then(|checkpoint| checkpoint.gnc.srp_surface)
    }

    fn dynamics(&self, srp: Option<SrpSurface>) -> L2HoldingDynamics<'_> {
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

    /// Solves the holding burn for `state` and flies it as a constant-thrust burn
    /// along the line of the two bodies, sized for the vessel's whole mass.
    fn burn(
        &self,
        universe: &mut Universe,
        state: &ContinuousPlantState,
        epoch: TdbEpoch,
        insertion: bool,
    ) -> Result<Option<L2KeepingBurn>, UniverseError> {
        let dynamics = self.dynamics(self.srp_surface(universe));
        let delta_v_m_s = dynamics
            .holding_burn_m_s(&self.target, state, epoch)
            .map_err(holding_error)?;
        if delta_v_m_s == 0.0 {
            return Ok(None);
        }
        let along = dynamics.burn_direction_icrf(epoch).map_err(holding_error)?;
        let mass_kg = self
            .config
            .mass_model
            .total_mass_kg(state.propellant_mass_kg);
        let burn_s = self.burn_span.as_seconds().max(f64::MIN_POSITIVE);
        let now = universe.now();
        universe.command(
            self.vessel,
            now,
            PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
                dir_icrf: (along * delta_v_m_s.signum()).to_array(),
                thrust_n: mass_kg * delta_v_m_s.abs() / burn_s,
                exhaust_vel_m_s: EXHAUST_VEL_M_S,
                rw_torque_cmd_body_nm: [0.0; 3],
            }),
        )?;
        universe.command(
            self.vessel,
            now.advanced(self.burn_span),
            PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::Coast),
        )?;
        Ok(Some(L2KeepingBurn {
            at: now,
            delta_v_m_s,
            insertion,
        }))
    }
}

fn holding_error(error: LagrangeError) -> UniverseError {
    UniverseError::Holding(Box::new(error))
}
