//! Every vessel of a run on one master clock, each in the fidelity tier that
//! suits where it is.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use sim_bodies::BodyCatalog;
use sim_core::{SimInstant, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    BodyOrientationCatalog, ContentHash, ContinuousPlantState, Dop853Tolerances,
    EnvironmentLoadSpec, EphemerisCatalog, ForceModel, NaifId, OrbitalUniverse,
    PlayerCommandPayload, SimulationManifestDigest, VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{BodyRotationState, SimEpoch};

use crate::UniverseError;
use crate::bus::{VesselBusConfig, VesselBusState};
use crate::handoff::to_body_fixed;
use crate::local::{LocalFactory, LocalPlant};
use crate::planning::Planner;
use crate::proximity::{ProximityStepReport, UniverseInterVesselLink};
use crate::tiering::{TierChange, TierPolicy};

mod carried_arc;
mod checkpoint;
mod exchange;
mod handoffs;
mod planned;
mod proximity_ops;
mod proximity_step;

pub use checkpoint::UniverseCheckpoint;

/// Which dynamics currently fly a vessel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tier {
    /// `sim-core` flight dynamics in the central body's rotating frame.
    Local,
    /// `sim-orbital` propagation in ICRF axes about the central body.
    Orbital,
}

/// Where the orbital tier gets the solar system from, and how it steps.
pub struct OrbitalContext {
    /// Body positions.
    pub ephemeris: Arc<dyn EphemerisCatalog>,
    /// Body orientations; the handoff reads the same catalog, so a vessel
    /// does not jump when it changes tier.
    pub orientation: Arc<dyn BodyOrientationCatalog>,
    /// Gravity the orbital tier integrates.
    pub force_model: ForceModel,
    /// The orbital tier's integration steps end on this grid, so its states
    /// do not depend on how a frame loop slices time.
    pub quantum: TickSpan,
    /// Integrator tolerances.
    pub tolerances: Dop853Tolerances,
}

pub(crate) enum Flight {
    Local(Box<dyn LocalPlant>),
    /// Held by the orbital scheduler, which knows the vessel's central body.
    /// A plant that can be carried stays here, dormant but for its exchanges
    /// with the scheduler, and flies the vessel again when it comes down.
    Orbital {
        carried: Option<Box<dyn LocalPlant>>,
        /// Base states sampled at each `plant.period()` step since the last
        /// exchange, so multi-step carries integrate along the orbital trajectory.
        samples: Vec<(SimInstant, VehicleState)>,
    },
}

pub(crate) struct Vessel {
    pub(crate) flight: Flight,
    pub(crate) rebuild: LocalFactory,
    pub(crate) bus: VesselBusState,
}

pub(crate) struct RetiredVessel {
    meta: proximity_step::ExtractedPlantMeta,
    survivor_world_donor: VesselId,
}

/// Every vessel of a run, advanced together on the master clock.
///
/// Uncoupled local vessels step in id order, each on its own period; coupled
/// vessels step synchronously through `sim-core` proximity groups. Orbital
/// vessels belong to one [`OrbitalUniverse`], which owns their events,
/// commands and integration grid.
pub struct Universe {
    epoch: SimEpoch,
    orbital: OrbitalUniverse,
    bodies: Arc<BodyCatalog>,
    pub(crate) ephemeris: Arc<dyn EphemerisCatalog>,
    orientation: Arc<dyn BodyOrientationCatalog>,
    tolerances: Dop853Tolerances,
    pub(crate) vessels: BTreeMap<VesselId, Vessel>,
    now: SimInstant,
    policy: Option<TierPolicy>,
    tier_changes: Vec<TierChange>,
    state_reinitializations: u64,
    planner: Planner,
    orbital_workers: usize,
    pub(crate) links: Vec<UniverseInterVesselLink>,
    pub(crate) active_vessel: Option<VesselId>,
    pub(crate) next_vessel_id: u64,
    pub(crate) next_link_id: u64,
    pub(crate) proximity_spawned: BTreeSet<VesselId>,
    pub(crate) retired_ever: BTreeSet<VesselId>,
    pub(crate) auto_demoted: BTreeSet<VesselId>,
    pub(crate) retired_vessels: BTreeMap<VesselId, RetiredVessel>,
    pub(crate) proximity_report: ProximityStepReport,
}

impl Universe {
    /// An empty universe on `epoch`'s clock.
    #[must_use]
    pub fn new(epoch: SimEpoch, context: OrbitalContext) -> Self {
        let clock = epoch.clock();
        let bodies = Arc::clone(context.force_model.body_catalog());
        let manifest = SimulationManifestDigest::compute(
            clock.base_tdb(),
            context.ephemeris.content_hash(),
            context.force_model.constants().content_hash(),
            context.orientation.content_hash(),
            epoch.scales().content_hash(),
            ContentHash::of_bytes(b"sim-universe:no-collision-terrain"),
            context.force_model.content_hash(),
            context.tolerances.content_hash(),
        );
        let orbital = OrbitalUniverse::new(
            manifest,
            clock,
            context.force_model,
            Arc::clone(&context.ephemeris),
            Arc::clone(&context.orientation),
        )
        .with_horizon_quantum_ticks(context.quantum.ticks_u128());
        Self {
            epoch,
            orbital,
            bodies,
            ephemeris: context.ephemeris,
            orientation: context.orientation,
            tolerances: context.tolerances,
            vessels: BTreeMap::new(),
            now: SimInstant::START,
            policy: None,
            tier_changes: Vec::new(),
            state_reinitializations: 0,
            planner: Planner::new(),
            orbital_workers: 1,
            links: Vec::new(),
            active_vessel: None,
            next_vessel_id: 1,
            next_link_id: 1,
            proximity_spawned: BTreeSet::new(),
            retired_ever: BTreeSet::new(),
            auto_demoted: BTreeSet::new(),
            retired_vessels: BTreeMap::new(),
            proximity_report: ProximityStepReport::default(),
        }
    }

    /// Number of times [`Self::restore`] has overwritten the universe's state (`0` during uninterrupted runs).
    #[must_use]
    pub const fn state_reinitializations(&self) -> u64 {
        self.state_reinitializations
    }

    /// Speculates orbital vessels on `workers` threads. The committed states
    /// are the same bits for any count; one thread is the default.
    #[must_use]
    pub fn with_orbital_workers(mut self, workers: usize) -> Self {
        self.orbital_workers = workers.max(1);
        self
    }

    /// Moves vessels between tiers by `policy` as the universe advances.
    /// Without one, vessels change tier only when asked to.
    #[must_use]
    pub fn with_tier_policy(mut self, policy: TierPolicy) -> Self {
        self.policy = Some(policy);
        self
    }

    /// Moves vessels between tiers by the regime limits each body of this
    /// run declares ([`TierPolicy::for_bodies`]).
    #[must_use]
    pub fn with_regime_tier_policy(self) -> Self {
        let policy = TierPolicy::for_bodies(&self.bodies);
        self.with_tier_policy(policy)
    }

    /// The tier changes the policy has made since this was last called, in
    /// the order they happened.
    pub fn take_tier_changes(&mut self) -> Vec<TierChange> {
        std::mem::take(&mut self.tier_changes)
    }

    /// The instant every vessel has been advanced to.
    #[must_use]
    pub const fn now(&self) -> SimInstant {
        self.now
    }

    /// The bodies of this run, as the orbital force model's constants
    /// describe them: one catalog for both tiers, so a vessel changing tier
    /// keeps its body, its fields and its air.
    #[must_use]
    pub const fn bodies(&self) -> &Arc<BodyCatalog> {
        &self.bodies
    }

    /// The run's clock.
    #[must_use]
    pub const fn epoch(&self) -> &SimEpoch {
        &self.epoch
    }

    /// The scheduler that owns every orbital-tier vessel: its events,
    /// stamped commands, snapshots and render samples.
    #[must_use]
    pub const fn orbital(&self) -> &OrbitalUniverse {
        &self.orbital
    }

    /// Adds a vessel flown by `plant`; `rebuild` makes its local tier again
    /// when it comes back from orbit.
    pub fn insert_local(
        &mut self,
        id: VesselId,
        plant: Box<dyn LocalPlant>,
        rebuild: LocalFactory,
    ) -> Result<(), UniverseError> {
        if self.vessels.contains_key(&id) {
            return Err(UniverseError::DuplicateVessel(id));
        }
        let bus_cfg = VesselBusConfig {
            srp_surface: plant.srp_surface(),
            ..VesselBusConfig::default()
        };
        let bus = VesselBusState::new(bus_cfg, self.now);
        let flight = Flight::Local(plant);
        self.vessels.insert(
            id,
            Vessel {
                flight,
                rebuild,
                bus,
            },
        );
        self.next_vessel_id = self.next_vessel_id.max(id.0.saturating_add(1));
        if self.active_vessel.is_none() {
            self.active_vessel = Some(id);
        }
        self.refresh_vessel_bus(id);
        Ok(())
    }

    /// Adds a vessel starting directly in the orbital tier at [`Self::now`].
    pub fn insert_orbital(
        &mut self,
        id: VesselId,
        config: VesselConfig,
        state: ContinuousPlantState,
        rebuild: LocalFactory,
    ) -> Result<(), UniverseError> {
        self.insert_orbital_with_bus(id, config, state, VesselBusConfig::default(), rebuild)
    }

    /// Adds a vessel starting directly in the orbital tier at [`Self::now`] with explicit bus configuration.
    pub fn insert_orbital_with_bus(
        &mut self,
        id: VesselId,
        config: VesselConfig,
        state: ContinuousPlantState,
        bus_config: VesselBusConfig,
        rebuild: LocalFactory,
    ) -> Result<(), UniverseError> {
        if self.vessels.contains_key(&id) {
            return Err(UniverseError::DuplicateVessel(id));
        }
        let mut checkpoint = VesselCheckpoint::new(id, self.now, state, 0);
        if let Some(srp) = bus_config.srp_surface {
            checkpoint =
                checkpoint.with_environment_load(EnvironmentLoadSpec::SolarRadiationPressure(srp));
        }
        self.orbital
            .insert_vessel(config, checkpoint)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        let bus = VesselBusState::new(bus_config, self.now);
        self.vessels.insert(
            id,
            Vessel {
                flight: Flight::Orbital {
                    carried: None,
                    samples: Vec::new(),
                },
                rebuild,
                bus,
            },
        );
        self.next_vessel_id = self.next_vessel_id.max(id.0.saturating_add(1));
        if self.active_vessel.is_none() {
            self.active_vessel = Some(id);
        }
        self.refresh_vessel_bus(id);
        Ok(())
    }

    /// The vessels, in stepping order.
    pub fn ids(&self) -> impl Iterator<Item = VesselId> + '_ {
        self.vessels.keys().copied()
    }

    /// `id`'s local plant as the type that built it, or `None` when the
    /// vessel is orbital, unknown, or flown by another type of plant. A plant
    /// the orbital tier carries is reached through [`Self::carried`].
    #[must_use]
    pub fn local<P: LocalPlant + 'static>(&self, id: VesselId) -> Option<&P> {
        match &self.vessels.get(&id)?.flight {
            Flight::Local(plant) => plant.as_any().downcast_ref(),
            Flight::Orbital { .. } => None,
        }
    }

    /// [`Self::local`], to change what only that plant type exposes: its
    /// stamped inputs, its sensors, its own checkpoints.
    pub fn local_mut<P: LocalPlant + 'static>(&mut self, id: VesselId) -> Option<&mut P> {
        match &mut self.vessels.get_mut(&id)?.flight {
            Flight::Local(plant) => plant.as_any_mut().downcast_mut(),
            Flight::Orbital { .. } => None,
        }
    }

    /// Which tier flies `id`.
    pub fn tier(&self, id: VesselId) -> Result<Tier, UniverseError> {
        Ok(match self.vessel(id)?.flight {
            Flight::Local(_) => Tier::Local,
            Flight::Orbital { .. } => Tier::Orbital,
        })
    }

    /// `id`'s state fixed to its central body, whichever tier flies it.
    /// Fails for an orbit centred on an origin with no body-fixed frame.
    pub fn vehicle_state(&self, id: VesselId) -> Result<VehicleState, UniverseError> {
        match &self.vessel(id)?.flight {
            Flight::Local(plant) => Ok(plant.state()),
            Flight::Orbital { .. } => {
                let plant = self.inertial_state(id)?;
                let body = self
                    .bodies
                    .ellipsoid_of(plant.primary_body)
                    .ok_or(UniverseError::UnmodelledBody)?;
                let (naif, rotation) = self.rotation(body, self.now)?;
                Ok(to_body_fixed(&plant, naif, &rotation)?.0)
            }
        }
    }

    /// The origin `id`'s motion is described about: its body for a local
    /// vessel, and for an orbital one whichever body or barycentre the
    /// scheduler currently integrates it around.
    pub fn central_body(&self, id: VesselId) -> Result<NaifId, UniverseError> {
        match &self.vessel(id)?.flight {
            Flight::Local(plant) => self
                .bodies
                .naif_of(&plant.body())
                .ok_or(UniverseError::UnmodelledBody),
            Flight::Orbital { .. } => Ok(self.inertial_state(id)?.primary_body),
        }
    }

    /// An orbital vessel's state in ICRF axes about its central body, at
    /// [`Self::now`].
    pub fn inertial_state(&self, id: VesselId) -> Result<ContinuousPlantState, UniverseError> {
        match self.vessel(id)?.flight {
            Flight::Local(_) => Err(UniverseError::NotOrbital(id)),
            Flight::Orbital { .. } => self
                .orbital
                .sample_vessel_state(id, self.now)
                .ok_or(UniverseError::UnknownVessel(id)),
        }
    }

    /// Stamps `command` for the orbital vessel `id` at `at`, which must not
    /// precede [`Self::now`]. It takes effect when the universe advances to
    /// `at`, at that exact instant however the advance is sliced.
    pub fn command(
        &mut self,
        id: VesselId,
        at: SimInstant,
        command: PlayerCommandPayload,
    ) -> Result<(), UniverseError> {
        match self.vessel(id)?.flight {
            Flight::Local(_) => Err(UniverseError::NotOrbital(id)),
            Flight::Orbital { .. } => self
                .orbital
                .stamp_player_command(at, id, command)
                .map(|_stamped| ())
                .map_err(|e| UniverseError::Scheduler(Box::new(e))),
        }
    }

    pub(crate) fn rotation(
        &self,
        body: BodyEllipsoid,
        at: SimInstant,
    ) -> Result<(NaifId, BodyRotationState), UniverseError> {
        let naif = self
            .bodies
            .naif_of(&body)
            .ok_or(UniverseError::UnmodelledBody)?;
        let rotation = self
            .orientation
            .body_rotation(naif, self.epoch.tdb_at(at))?;
        Ok((naif, rotation))
    }

    pub(crate) fn vessel(&self, id: VesselId) -> Result<&Vessel, UniverseError> {
        self.vessels
            .get(&id)
            .ok_or(UniverseError::UnknownVessel(id))
    }

    pub(crate) fn vessel_mut(&mut self, id: VesselId) -> Result<&mut Vessel, UniverseError> {
        self.vessels
            .get_mut(&id)
            .ok_or(UniverseError::UnknownVessel(id))
    }
}

#[cfg(test)]
mod tests;
