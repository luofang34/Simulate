//! Every vessel of a run on one master clock, each in the fidelity tier that
//! suits where it is.

use std::collections::BTreeMap;
use std::sync::Arc;

use sim_core::{SimInstant, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    BodyOrientationCatalog, ContentHash, ContinuousPlantState, Dop853Tolerances, EphemerisCatalog,
    ForceModel, MassPropertiesModel, NaifId, OrbitalUniverse, PlayerCommandPayload, PropellantMode,
    ReactionWheelMode, SimulationManifestDigest, VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{BodyRotationState, SimEpoch};

use crate::UniverseError;
use crate::handoff::{InertialExtras, body_ellipsoid, naif_body, to_body_fixed, to_inertial};
use crate::local::{LocalFactory, LocalPlant};
use crate::tiering::{TierChange, TierPolicy};

mod checkpoint;

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

enum Flight {
    Local(Box<dyn LocalPlant>),
    /// Held by the orbital scheduler, which knows the vessel's central body.
    Orbital,
}

struct Vessel {
    flight: Flight,
    rebuild: LocalFactory,
}

/// Every vessel of a run, advanced together on the master clock.
///
/// Local vessels step in id order, each on its own period. Orbital vessels
/// belong to one [`OrbitalUniverse`], which owns their events, commands and
/// integration grid. Either way the result depends only on the instants
/// asked for, never on the order vessels were added or how the caller
/// slices time.
pub struct Universe {
    epoch: SimEpoch,
    orbital: OrbitalUniverse,
    orientation: Arc<dyn BodyOrientationCatalog>,
    tolerances: Dop853Tolerances,
    vessels: BTreeMap<VesselId, Vessel>,
    now: SimInstant,
    policy: Option<TierPolicy>,
    tier_changes: Vec<TierChange>,
}

impl Universe {
    /// An empty universe on `epoch`'s clock.
    #[must_use]
    pub fn new(epoch: SimEpoch, context: OrbitalContext) -> Self {
        let clock = epoch.clock();
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
            context.ephemeris,
            Arc::clone(&context.orientation),
        )
        .with_horizon_quantum_ticks(context.quantum.ticks_u128());
        Self {
            epoch,
            orbital,
            orientation: context.orientation,
            tolerances: context.tolerances,
            vessels: BTreeMap::new(),
            now: SimInstant::START,
            policy: None,
            tier_changes: Vec::new(),
        }
    }

    /// Moves vessels between tiers by `policy` as the universe advances.
    /// Without one, vessels change tier only when asked to.
    #[must_use]
    pub fn with_tier_policy(mut self, policy: TierPolicy) -> Self {
        self.policy = Some(policy);
        self
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
        let flight = Flight::Local(plant);
        self.vessels.insert(id, Vessel { flight, rebuild });
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
        if self.vessels.contains_key(&id) {
            return Err(UniverseError::DuplicateVessel(id));
        }
        let checkpoint = VesselCheckpoint::new(id, self.now, state, 0);
        self.orbital
            .insert_vessel(config, checkpoint)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        self.vessels.insert(
            id,
            Vessel {
                flight: Flight::Orbital,
                rebuild,
            },
        );
        Ok(())
    }

    /// The vessels, in stepping order.
    pub fn ids(&self) -> impl Iterator<Item = VesselId> + '_ {
        self.vessels.keys().copied()
    }

    /// `id`'s local plant as the type that built it, or `None` when the
    /// vessel is orbital, unknown, or flown by another type of plant.
    #[must_use]
    pub fn local<P: LocalPlant + 'static>(&self, id: VesselId) -> Option<&P> {
        match &self.vessels.get(&id)?.flight {
            Flight::Local(plant) => plant.as_any().downcast_ref(),
            Flight::Orbital => None,
        }
    }

    /// [`Self::local`], to change what only that plant type exposes: its
    /// stamped inputs, its sensors, its own checkpoints.
    pub fn local_mut<P: LocalPlant + 'static>(&mut self, id: VesselId) -> Option<&mut P> {
        match &mut self.vessels.get_mut(&id)?.flight {
            Flight::Local(plant) => plant.as_any_mut().downcast_mut(),
            Flight::Orbital => None,
        }
    }

    /// Which tier flies `id`.
    pub fn tier(&self, id: VesselId) -> Result<Tier, UniverseError> {
        Ok(match self.vessel(id)?.flight {
            Flight::Local(_) => Tier::Local,
            Flight::Orbital => Tier::Orbital,
        })
    }

    /// `id`'s state fixed to its central body, whichever tier flies it.
    /// Fails for an orbit centred on an origin with no body-fixed frame.
    pub fn vehicle_state(&self, id: VesselId) -> Result<VehicleState, UniverseError> {
        match &self.vessel(id)?.flight {
            Flight::Local(plant) => Ok(plant.state()),
            Flight::Orbital => {
                let plant = self.inertial_state(id)?;
                let body =
                    body_ellipsoid(plant.primary_body).ok_or(UniverseError::UnmodelledBody)?;
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
            Flight::Local(plant) => naif_body(plant.body()).ok_or(UniverseError::UnmodelledBody),
            Flight::Orbital => Ok(self.inertial_state(id)?.primary_body),
        }
    }

    /// An orbital vessel's state in ICRF axes about its central body, at
    /// [`Self::now`].
    pub fn inertial_state(&self, id: VesselId) -> Result<ContinuousPlantState, UniverseError> {
        match self.vessel(id)?.flight {
            Flight::Local(_) => Err(UniverseError::NotOrbital(id)),
            Flight::Orbital => self
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
            Flight::Orbital => self
                .orbital
                .stamp_player_command(at, id, command)
                .map(|_stamped| ())
                .map_err(|e| UniverseError::Scheduler(Box::new(e))),
        }
    }

    /// Advances every vessel to `until`: local tiers by whole periods, the
    /// orbital tier to exactly `until`. A tier policy acts at each of its
    /// check instants on the way.
    pub fn advance_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        if until < self.now {
            return Err(UniverseError::Backwards);
        }
        while let Some(check) = self
            .policy
            .as_ref()
            .and_then(|policy| policy.next_check(self.now, until))
        {
            self.advance_all_to(check)?;
            self.apply_tier_policy()?;
        }
        self.advance_all_to(until)
    }

    fn advance_all_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        for vessel in self.vessels.values_mut() {
            if let Flight::Local(plant) = &mut vessel.flight {
                plant.advance_to(until)?;
            }
        }
        self.orbital
            .advance_committed_to(until)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        self.now = until;
        Ok(())
    }

    fn apply_tier_policy(&mut self) -> Result<(), UniverseError> {
        let Some(policy) = &self.policy else {
            return Ok(());
        };
        let mut wanted = Vec::new();
        for (&id, vessel) in &self.vessels {
            let (tier, body, state) = match &vessel.flight {
                Flight::Local(plant) => {
                    if plant.now() != self.now
                        || plant.holds_local_tier()
                        || plant.orbital_body().is_none()
                    {
                        continue;
                    }
                    (Tier::Local, plant.body(), plant.state())
                }
                Flight::Orbital => {
                    let primary = self.inertial_state(id)?.primary_body;
                    let Some(body) = body_ellipsoid(primary) else {
                        continue;
                    };
                    (Tier::Orbital, body, self.vehicle_state(id)?)
                }
            };
            let Some(naif) = naif_body(body) else {
                continue;
            };
            let height_m = body.fixed_to_geodetic(state.position_ecef).height_m;
            if let Some(to) = policy.wanted(tier, naif, height_m) {
                wanted.push((id, to));
            }
        }
        for (vessel, to) in wanted {
            match to {
                Tier::Orbital => self.promote_to_orbital(vessel)?,
                Tier::Local => self.demote_to_local(vessel)?,
            }
            let at = self.now;
            self.tier_changes.push(TierChange { vessel, at, to });
        }
        Ok(())
    }

    /// Hands `id` from flight dynamics to orbital propagation. Its local
    /// tier must have reached [`Self::now`], so both tiers describe the same
    /// instant.
    pub fn promote_to_orbital(&mut self, id: VesselId) -> Result<(), UniverseError> {
        let Flight::Local(plant) = &self.vessel(id)?.flight else {
            return Err(UniverseError::AlreadyInTier(id));
        };
        if plant.now() != self.now {
            return Err(UniverseError::BetweenSteps { vessel: id });
        }
        let body = plant.body();
        let (naif, rotation) = self.rotation(body, self.now)?;
        let state = to_inertial(&plant.state(), naif, &rotation, InertialExtras::default());
        let Some(carried) = plant.orbital_body() else {
            return Err(UniverseError::LocalOnly(id));
        };
        let config = VesselConfig {
            mass_model: MassPropertiesModel {
                dry_mass_kg: carried.mass_kg,
                dry_inertia_cols: carried.inertia_cg_body.to_cols_array_2d(),
                propellant_inertia_per_kg_cols: [[0.0; 3]; 3],
                uncompensated_idot_factor: 0.0,
            },
            propellant_mode: PropellantMode::InfiniteConstantMass,
            rw_mode: ReactionWheelMode::ConserveMomentum {
                h_max_nms: 0.0,
                tau_max_nm: 0.0,
            },
            tolerances: self.tolerances,
            bounding_radius_m: carried.bounding_radius_m,
            auto_soi_switch: true,
        };
        let checkpoint = VesselCheckpoint::new(id, self.now, state, 0);
        self.orbital
            .insert_vessel(config, checkpoint)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        self.vessel_mut(id)?.flight = Flight::Orbital;
        Ok(())
    }

    /// Hands `id` from orbital propagation back to flight dynamics about the
    /// body it currently orbits, built by the vessel's factory at
    /// [`Self::now`].
    pub fn demote_to_local(&mut self, id: VesselId) -> Result<(), UniverseError> {
        let Flight::Orbital = self.vessel(id)?.flight else {
            return Err(UniverseError::AlreadyInTier(id));
        };
        let primary = self.inertial_state(id)?.primary_body;
        let body = body_ellipsoid(primary).ok_or(UniverseError::UnmodelledBody)?;
        let (naif, rotation) = self.rotation(body, self.now)?;
        let (_config, checkpoint) = self
            .orbital
            .remove_vessel(id)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        let (state, _extras) = to_body_fixed(&checkpoint.plant, naif, &rotation)?;
        let plant = (self.vessel(id)?.rebuild)(body, &state, self.now, &self.epoch)?;
        self.vessel_mut(id)?.flight = Flight::Local(plant);
        Ok(())
    }

    fn rotation(
        &self,
        body: BodyEllipsoid,
        at: SimInstant,
    ) -> Result<(NaifId, BodyRotationState), UniverseError> {
        let naif = naif_body(body).ok_or(UniverseError::UnmodelledBody)?;
        let rotation = self
            .orientation
            .body_rotation(naif, self.epoch.tdb_at(at))?;
        Ok((naif, rotation))
    }

    fn vessel(&self, id: VesselId) -> Result<&Vessel, UniverseError> {
        self.vessels
            .get(&id)
            .ok_or(UniverseError::UnknownVessel(id))
    }

    fn vessel_mut(&mut self, id: VesselId) -> Result<&mut Vessel, UniverseError> {
        self.vessels
            .get_mut(&id)
            .ok_or(UniverseError::UnknownVessel(id))
    }
}

#[cfg(test)]
mod tests;
