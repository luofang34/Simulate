//! Moving a vessel between tiers, by request or by policy.
//!
//! A plant that can be carried goes up with its vessel and comes back down
//! flying it, so nothing inside it is lost to a change of integrator. Any
//! other plant is dropped on the way up and rebuilt by the vessel's factory.

use sim_core::SimInstant;
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    LatchedBurnLaw, PropellantMode, ReactionWheelMode, SrpSurface, VesselCheckpoint, VesselConfig,
    VesselId,
};

use super::exchange::{mass_model, wrench_law};
use super::{Flight, Tier, Universe};
use crate::UniverseError;
use crate::handoff::{InertialExtras, body_ellipsoid, naif_body, to_body_fixed, to_inertial};
use crate::local::LocalPlant;
use crate::tiering::{TierChange, planetary_dynamic_pressure_pa};

/// Whether `plant`'s own steps land on `at`, so its internals can be brought
/// to that instant.
fn reaches(plant: &dyn LocalPlant, at: SimInstant) -> bool {
    let period = plant.period().ticks_u128();
    at >= plant.now()
        && period > 0
        && at
            .duration_since(plant.now())
            .ticks_u128()
            .is_multiple_of(period)
}

/// What a tier policy reads of one vessel.
struct PolicyView {
    tier: Tier,
    body: BodyEllipsoid,
    height_m: f64,
    dynamic_pressure_pa: f64,
    ground_contact: bool,
    /// A carried plant asks for flight dynamics.
    holds_local: bool,
}

impl Universe {
    /// Advances to `until` with a tier policy acting at each of its check
    /// instants on the way, but no plans.
    pub(super) fn advance_unplanned_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
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

    /// What the policy reads of the orbital vessel `id`, or `None` for one
    /// that cannot change tier now: about an origin with no body-fixed frame,
    /// or carrying a plant whose steps do not land on this instant.
    fn orbital_policy_view(
        &self,
        id: VesselId,
        carried: Option<&dyn LocalPlant>,
    ) -> Result<Option<PolicyView>, UniverseError> {
        if carried.is_some_and(|plant| !reaches(plant, self.now)) {
            return Ok(None);
        }
        let primary = self.inertial_state(id)?.primary_body;
        let Some(body) = body_ellipsoid(primary) else {
            return Ok(None);
        };
        let state = self.vehicle_state(id)?;
        let height_m = body.fixed_to_geodetic(state.position_ecef).height_m;
        Ok(Some(PolicyView {
            tier: Tier::Orbital,
            body,
            height_m,
            dynamic_pressure_pa: planetary_dynamic_pressure_pa(
                primary,
                height_m,
                state.velocity_ecef.length(),
            ),
            ground_contact: height_m <= 0.0,
            // Only at an exchange do the plant's internals describe this instant.
            holds_local: carried
                .is_some_and(|plant| plant.now() == self.now && plant.holds_local_tier()),
        }))
    }

    /// What the policy reads of a local vessel, or `None` for one that stays
    /// where it is.
    fn local_policy_view(&self, plant: &dyn LocalPlant) -> Option<PolicyView> {
        if plant.now() != self.now || plant.holds_local_tier() || plant.orbital_body().is_none() {
            return None;
        }
        let body = plant.body();
        Some(PolicyView {
            tier: Tier::Local,
            body,
            height_m: body.fixed_to_geodetic(plant.state().position_ecef).height_m,
            dynamic_pressure_pa: plant.dynamic_pressure_pa(),
            ground_contact: plant.is_ground_contact(),
            holds_local: false,
        })
    }

    fn apply_tier_policy(&mut self) -> Result<(), UniverseError> {
        let Some(policy) = &self.policy else {
            return Ok(());
        };
        let mut wanted = Vec::new();
        for (&id, vessel) in &self.vessels {
            let view = match &vessel.flight {
                Flight::Local(plant) => self.local_policy_view(plant.as_ref()),
                Flight::Orbital { carried } => self.orbital_policy_view(id, carried.as_deref())?,
            };
            let Some(view) = view else {
                continue;
            };
            let Some(naif) = naif_body(view.body) else {
                continue;
            };
            let to = policy
                .wanted(
                    view.tier,
                    naif,
                    view.height_m,
                    view.dynamic_pressure_pa,
                    view.ground_contact,
                )
                .or_else(|| {
                    (view.holds_local && policy.bands.contains_key(&naif)).then_some(Tier::Local)
                });
            if let Some(to) = to {
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
    /// instant. A plant with a [`LocalPlant::carry_period`] goes with the
    /// vessel and keeps its internals running there; any other is dropped,
    /// and only its mass, inertia, size and radiation-pressure surface go on.
    pub fn promote_to_orbital(&mut self, id: VesselId) -> Result<(), UniverseError> {
        let Flight::Local(plant) = &self.vessel(id)?.flight else {
            return Err(UniverseError::AlreadyInTier(id));
        };
        if plant.now() != self.now {
            return Err(UniverseError::BetweenSteps { vessel: id });
        }
        if plant.orbital_body().is_none() {
            return Err(UniverseError::LocalOnly(id));
        }
        let srp = plant
            .srp_surface()
            .or_else(|| self.vessels.get(&id).and_then(|v| v.bus.srp_surface()));
        let aloft = Flight::Orbital { carried: None };
        let Flight::Local(mut plant) = std::mem::replace(&mut self.vessel_mut(id)?.flight, aloft)
        else {
            return Err(UniverseError::AlreadyInTier(id));
        };
        match self.insert_promoted(id, plant.as_mut(), srp) {
            Ok(carryable) => {
                self.vessel_mut(id)?.flight = Flight::Orbital {
                    carried: carryable.then_some(plant),
                };
                Ok(())
            }
            Err(error) => {
                self.vessel_mut(id)?.flight = Flight::Local(plant);
                Err(error)
            }
        }
    }

    /// Gives `plant`'s vessel to the orbital scheduler at [`Self::now`];
    /// `true` when the plant is to be carried along. A carried plant's present
    /// loads are latched from the first instant, so nothing coasts until its
    /// first exchange.
    fn insert_promoted(
        &mut self,
        id: VesselId,
        plant: &mut dyn LocalPlant,
        srp: Option<SrpSurface>,
    ) -> Result<bool, UniverseError> {
        let body = plant.body();
        let (naif, rotation) = self.rotation(body, self.now)?;
        let local = plant.state();
        let state = to_inertial(&local, naif, &rotation, InertialExtras::default());
        let carryable = plant
            .carry_period()
            .is_some_and(|period| period.ticks_u128() > 0);
        let (law, carried) = if carryable {
            let context = self.carry_context(body, local, self.now)?;
            let loads = plant.carry(&context)?;
            (wrench_law(&loads), loads.body)
        } else {
            let carried = plant.orbital_body().ok_or(UniverseError::LocalOnly(id))?;
            (LatchedBurnLaw::Coast, carried)
        };
        let config = VesselConfig {
            mass_model: mass_model(&carried),
            propellant_mode: PropellantMode::InfiniteConstantMass,
            rw_mode: ReactionWheelMode::ConserveMomentum {
                h_max_nms: 0.0,
                tau_max_nm: 0.0,
            },
            tolerances: self.tolerances,
            bounding_radius_m: carried.bounding_radius_m,
            auto_soi_switch: true,
        };
        let mut checkpoint = VesselCheckpoint::new(id, self.now, state, 0);
        checkpoint.gnc.latched_law = law;
        if let Some(surface) = srp {
            checkpoint = checkpoint.with_srp_surface(surface);
        }
        self.orbital
            .insert_vessel(config, checkpoint)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        Ok(carryable)
    }

    /// Hands `id` from orbital propagation back to flight dynamics about the
    /// body it currently orbits, at [`Self::now`]. A carried plant is brought
    /// to this instant and takes the orbital state as its own; it must be at
    /// one of its step instants. A vessel without one, or whose plant cannot
    /// adopt the state, is built by its factory.
    pub fn demote_to_local(&mut self, id: VesselId) -> Result<(), UniverseError> {
        let Flight::Orbital { carried } = &self.vessel(id)?.flight else {
            return Err(UniverseError::AlreadyInTier(id));
        };
        if carried
            .as_deref()
            .is_some_and(|plant| !reaches(plant, self.now))
        {
            return Err(UniverseError::BetweenSteps { vessel: id });
        }
        let primary = self.inertial_state(id)?.primary_body;
        let body = body_ellipsoid(primary).ok_or(UniverseError::UnmodelledBody)?;
        let (naif, rotation) = self.rotation(body, self.now)?;
        self.bring_carried_to_now(id)?;
        let (_config, checkpoint) = self
            .orbital
            .remove_vessel(id)
            .map_err(|e| UniverseError::Scheduler(Box::new(e)))?;
        let (state, _extras) = to_body_fixed(&checkpoint.plant, naif, &rotation)?;
        let mut kept = None;
        if let Some(mut plant) = self.take_carried(id)
            && plant.adopt_base(body, &state, self.now)?
        {
            kept = Some(plant);
        }
        let plant = match kept {
            Some(plant) => plant,
            None => (self.vessel(id)?.rebuild)(body, &state, self.now, &self.epoch)?,
        };
        self.vessel_mut(id)?.flight = Flight::Local(plant);
        Ok(())
    }
}
