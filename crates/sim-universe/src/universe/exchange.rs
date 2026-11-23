//! Keeping carried plants and the orbital tier in step.
//!
//! The orbital tier stops at every carried plant's next exchange instant.
//! There the plant is shown the state the scheduler committed, advances its
//! internals to it, and its loads and mass become the vessel's externally held
//! loads from that same instant, beside whatever burn a plan has latched.
//! Internals therefore only ever run on this thread between committed
//! instants: speculative proposals stay pure functions of their checkpoints,
//! and no rollback reaches inside a plant.

use sim_core::{SimInstant, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    DerivativeOrder, ExternalLoads, ExternalMassProperties, MassPropertiesModel, NaifId, VesselId,
};
use sim_spacetime::SolarFluxModel;

use super::{Flight, Universe};
use crate::UniverseError;
use crate::carry::{CarriedLoads, CarryContext};
use crate::handoff::{naif_body, to_body_fixed};
use crate::local::{LocalPlant, OrbitalBody};

/// `loads` as the orbital tier holds them until the next exchange.
pub(super) fn external_loads(loads: &CarriedLoads) -> ExternalLoads {
    ExternalLoads::new(
        loads.force_body_n,
        loads.torque_body_nm,
        ExternalMassProperties::new(loads.body.mass_kg, loads.body.inertia_cg_body),
    )
}

/// `body` as a mass the orbital tier holds constant: all of it dry, because
/// whatever drains it is integrated by the plant that reported it.
pub(super) fn mass_model(body: &OrbitalBody) -> MassPropertiesModel {
    MassPropertiesModel {
        dry_mass_kg: body.mass_kg,
        dry_inertia_cols: body.inertia_cg_body.to_cols_array_2d(),
        propellant_inertia_per_kg_cols: [[0.0; 3]; 3],
        uncompensated_idot_factor: 0.0,
    }
}

impl Universe {
    /// `id`'s plant as the type that built it while the orbital tier carries
    /// it, or `None` when the vessel is local, unknown, has no carried plant,
    /// or is flown by another type. Its internals describe the plant's own
    /// [`LocalPlant::now`], the last exchange instant.
    #[must_use]
    pub fn carried<P: LocalPlant + 'static>(&self, id: VesselId) -> Option<&P> {
        match &self.vessels.get(&id)?.flight {
            Flight::Orbital { carried } => carried.as_ref()?.as_any().downcast_ref(),
            Flight::Local(_) => None,
        }
    }

    /// [`Self::carried`], to command the plant while it is aloft. A change
    /// takes effect from the plant's next exchange, so make it at an instant
    /// the run reaches however it is sliced.
    pub fn carried_mut<P: LocalPlant + 'static>(&mut self, id: VesselId) -> Option<&mut P> {
        match &mut self.vessels.get_mut(&id)?.flight {
            Flight::Orbital { carried } => carried.as_mut()?.as_any_mut().downcast_mut(),
            Flight::Local(_) => None,
        }
    }

    /// Advances every vessel to `until`, stopping the orbital tier at each
    /// carried plant's exchange instant on the way.
    pub(super) fn advance_all_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        for vessel in self.vessels.values_mut() {
            if let Flight::Local(plant) = &mut vessel.flight {
                plant.advance_to(until)?;
            }
        }
        loop {
            let stop = self.next_exchange(until).unwrap_or(until);
            self.commit_orbital_to(stop)?;
            self.now = stop;
            self.exchange_due()?;
            if stop >= until {
                break;
            }
        }
        self.refresh_all_buses();
        Ok(())
    }

    fn commit_orbital_to(&mut self, at: SimInstant) -> Result<(), UniverseError> {
        self.orbital
            .advance_committed_to(at)
            .map(|_metrics| ())
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))
    }

    /// When `plant` next exchanges: one carry period after the instant its
    /// internals describe, which rides in the plant's own snapshot.
    fn exchange_instant(plant: &dyn LocalPlant) -> Option<SimInstant> {
        let period = plant.carry_period()?;
        (period.ticks_u128() > 0).then(|| plant.now().advanced(period))
    }

    /// The earliest exchange instant no later than `until`.
    fn next_exchange(&self, until: SimInstant) -> Option<SimInstant> {
        self.vessels
            .values()
            .filter_map(|vessel| match &vessel.flight {
                Flight::Orbital { carried } => Self::exchange_instant(carried.as_deref()?),
                Flight::Local(_) => None,
            })
            .min()
            .map(|at| at.max(self.now))
            .filter(|&at| at <= until)
    }

    /// Exchanges with every carried plant whose instant has come, in id
    /// order.
    fn exchange_due(&mut self) -> Result<(), UniverseError> {
        let due: Vec<VesselId> = self
            .vessels
            .iter()
            .filter_map(|(&id, vessel)| match &vessel.flight {
                Flight::Orbital { carried } => {
                    let at = Self::exchange_instant(carried.as_deref()?)?;
                    (at <= self.now).then_some(id)
                }
                Flight::Local(_) => None,
            })
            .collect();
        for id in due {
            if let Some(loads) = self.carry_to_now(id)? {
                self.latch(id, &loads)?;
            }
        }
        Ok(())
    }

    /// Brings `id`'s carried plant to [`Self::now`] without latching, for a
    /// vessel about to leave the orbital tier.
    pub(super) fn bring_carried_to_now(&mut self, id: VesselId) -> Result<(), UniverseError> {
        let behind = match &self.vessel(id)?.flight {
            Flight::Orbital { carried } => carried
                .as_deref()
                .is_some_and(|plant| plant.now() < self.now),
            Flight::Local(_) => false,
        };
        if behind {
            self.carry_to_now(id)?;
        }
        Ok(())
    }

    pub(super) fn take_carried(&mut self, id: VesselId) -> Option<Box<dyn LocalPlant>> {
        match &mut self.vessels.get_mut(&id)?.flight {
            Flight::Orbital { carried } => carried.take(),
            Flight::Local(_) => None,
        }
    }

    /// Shows `id`'s carried plant the state committed at [`Self::now`] and
    /// advances its internals to it. `None` for a vessel with no such plant.
    fn carry_to_now(&mut self, id: VesselId) -> Result<Option<CarriedLoads>, UniverseError> {
        let Some(mut plant) = self.take_carried(id) else {
            return Ok(None);
        };
        let carried = self.carry_plant(id, plant.as_mut());
        if let Flight::Orbital { carried: slot } = &mut self.vessel_mut(id)?.flight {
            *slot = Some(plant);
        }
        carried.map(Some)
    }

    fn carry_plant(
        &self,
        id: VesselId,
        plant: &mut dyn LocalPlant,
    ) -> Result<CarriedLoads, UniverseError> {
        let body = plant.body();
        let state = self.carried_base_state(id, body)?;
        let context = self.carry_context(body, state, self.now)?;
        let loads = plant.carry(&context)?;
        if plant.now() != self.now {
            return Err(UniverseError::BetweenSteps { vessel: id });
        }
        Ok(loads)
    }

    /// The orbital vessel `id`'s state at [`Self::now`] fixed to `body`, the
    /// plant's own, whatever origin the scheduler integrates it about. Between
    /// integration step ends this is the scheduler's dense output.
    fn carried_base_state(
        &self,
        id: VesselId,
        body: BodyEllipsoid,
    ) -> Result<VehicleState, UniverseError> {
        let naif = naif_body(body).ok_or(UniverseError::UnmodelledBody)?;
        let mut plant = self
            .orbital
            .sample_vessel_state(id, self.now)
            .ok_or(UniverseError::UnknownVessel(id))?;
        if plant.primary_body != naif {
            let vessel = self
                .orbital
                .sample_vessel_barycentric(id, self.now)
                .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
            let origin = self.ephemeris.geometric_state_icrf(
                naif,
                NaifId::SSB,
                self.epoch.tdb_at(self.now),
                DerivativeOrder::Velocity,
            )?;
            plant.primary_body = naif;
            plant.pos_rel_icrf_m = vessel.pos_m - origin.pos_m;
            plant.vel_rel_icrf_m_s = vessel.vel_m_s - origin.vel_m_s;
        }
        let (_, rotation) = self.rotation(body, self.now)?;
        Ok(to_body_fixed(&plant, naif, &rotation)?.0)
    }

    /// What a plant at `state`, fixed to `body`, is told at `at`. Sunlight is
    /// sampled here, at the exchange instant alone, so it never depends on
    /// when a display last asked for it.
    pub(super) fn carry_context(
        &self,
        body: BodyEllipsoid,
        state: VehicleState,
        at: SimInstant,
    ) -> Result<CarryContext, UniverseError> {
        let (naif, rotation) = self.rotation(body, at)?;
        let tdb = self.epoch.tdb_at(at);
        let origin = self.ephemeris.geometric_state_icrf(
            naif,
            NaifId::SSB,
            tdb,
            DerivativeOrder::Position,
        )?;
        let position_ssb_m =
            origin.pos_m + rotation.r_icrf_to_fixed.transpose() * state.position_ecef;
        let sunlight =
            SolarFluxModel::standard().sample(position_ssb_m, tdb, self.ephemeris.as_ref())?;
        Ok(CarryContext {
            at,
            body,
            state,
            sun_direction_fixed: rotation.r_icrf_to_fixed * sunlight.to_sun_icrf,
            solar_irradiance_w_m2: sunlight.unobstructed_w_m2,
            solar_lit_fraction: sunlight.lit_fraction,
        })
    }

    /// Holds `loads` on `id` from [`Self::now`]. Loads with the same bits
    /// as those held change nothing, so a carried vessel that only coasts
    /// keeps the scheduler's long integration steps.
    fn latch(&mut self, id: VesselId, loads: &CarriedLoads) -> Result<(), UniverseError> {
        self.orbital
            .set_external_loads(id, external_loads(loads))
            .map(|_changed| ())
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))
    }
}
