//! Discrete event mutation handlers for [`crate::OrbitalUniverse`].

use sim_spacetime::SimInstant;

use crate::{
    checkpoint::{ExternalLoads, PropagatorKind, VesselCheckpoint, VesselConfig},
    ephemeris::EphemerisCatalog,
    events::{
        EventOrderKey, PlayerCommandPayload, ScheduledEvent, UniverseEventPayload, phase_ranks,
    },
    gravity::ForceModel,
    ids::{NaifId, VesselId},
    time::TdbEpoch,
    universe::{OrbitalUniverse, SchedulerError},
};

pub(crate) fn apply_checkpoint_event_mutation(
    cp: &mut VesselCheckpoint,
    committed_instant: SimInstant,
    ep: TdbEpoch,
    payload: UniverseEventPayload,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    vid: VesselId,
) -> Result<Option<ScheduledEvent>, SchedulerError> {
    let mut follow_up = None;
    match payload {
        UniverseEventPayload::PlayerCommand(cmd) => {
            apply_player_cmd(cp, cmd, ep, force_model, ephemeris, vid)?;
        }
        UniverseEventPayload::DiscreteGncUpdate => {
            let _ = cp.rng.next_u64();
            cp.generations.bump_gnc_gen();
            cp.generations.bump_state_gen();
            cp.gnc.next_gnc_ticks = if cp.gnc.gnc_period_ticks > 0 {
                Some(
                    committed_instant
                        .ticks_u128()
                        .wrapping_add(cp.gnc.gnc_period_ticks),
                )
            } else {
                None
            };
        }
        UniverseEventPayload::SwitchPrimaryBody { new_primary } => {
            switch_cp_primary(cp, new_primary, ep, force_model, ephemeris, vid)?;
            cp.generations.bump_state_gen();
        }
        UniverseEventPayload::ModeTransition {
            next_mode_code,
            remaining_chain,
        } => {
            follow_up =
                apply_mode_transition(cp, committed_instant, vid, next_mode_code, remaining_chain);
        }
        UniverseEventPayload::Marker { .. } => {}
    }
    Ok(follow_up)
}

/// Refuses a checkpoint whose propagation is closed-form (an unperturbed Kepler arc, rest
/// on a surface) while it holds anything that would change its motion: such a path
/// evaluates no force model, so the loads would be dropped without a trace. Whether it
/// refuses depends on the checkpoint alone, so every replay refuses at the same instant.
pub(crate) fn refuse_unapplied_loads(
    checkpoint: &VesselCheckpoint,
    loads: &ExternalLoads,
) -> Result<(), SchedulerError> {
    let propagator = checkpoint.integrator_hint.propagator;
    let closed_form = matches!(
        propagator,
        PropagatorKind::DormantKepler | PropagatorKind::PinnedSurfaceRest
    );
    if closed_form && (loads.acts_on_motion() || !checkpoint.environment.is_empty()) {
        return Err(SchedulerError::LoadsNotPropagated {
            vessel_id: checkpoint.vessel_id,
            propagator,
            instant: checkpoint.instant,
        });
    }
    Ok(())
}

fn apply_mode_transition(
    cp: &mut VesselCheckpoint,
    committed_instant: SimInstant,
    vid: VesselId,
    next_mode_code: u32,
    remaining_chain: u32,
) -> Option<ScheduledEvent> {
    cp.gnc.mode_code = next_mode_code;
    cp.generations.bump_state_gen();
    if remaining_chain == 0 {
        return None;
    }
    let seq = cp.generations.next_emit_seq();
    let key = EventOrderKey::single(
        committed_instant,
        phase_ranks::CUSTOM_DETECTOR,
        vid,
        99,
        seq,
    );
    let next_rem = if remaining_chain == u32::MAX {
        u32::MAX
    } else {
        remaining_chain - 1
    };
    Some(ScheduledEvent::single(
        key,
        vid,
        Some(cp.generations.state_gen),
        UniverseEventPayload::ModeTransition {
            next_mode_code: next_mode_code.wrapping_add(1),
            remaining_chain: next_rem,
        },
    ))
}

fn apply_player_cmd(
    cp: &mut VesselCheckpoint,
    cmd: PlayerCommandPayload,
    ep: TdbEpoch,
    fm: &ForceModel,
    ephem: &dyn EphemerisCatalog,
    vid: VesselId,
) -> Result<(), SchedulerError> {
    match cmd {
        PlayerCommandPayload::SetBurnLaw(law) => cp.gnc.latched_law = law,
        PlayerCommandPayload::SwitchPrimary(new_p) => {
            switch_cp_primary(cp, new_p, ep, fm, ephem, vid)?;
        }
        PlayerCommandPayload::SetGncMode(m) => cp.gnc.mode_code = m,
        PlayerCommandPayload::SetFins {
            fin_angle_rad,
            fins_locked_zero,
        } => {
            cp.actuators.fins_locked_zero = fins_locked_zero;
            cp.actuators.fin_angle_rad = if fins_locked_zero {
                [0.0; 4]
            } else {
                fin_angle_rad
            };
        }
        PlayerCommandPayload::SetNosecone {
            carrier_rad,
            wrist_rad,
        } => {
            cp.actuators.nose_carrier_rad = carrier_rad;
            cp.actuators.nose_wrist_rad = wrist_rad;
        }
    }
    cp.generations.bump_state_gen();
    Ok(())
}

fn switch_cp_primary(
    cp: &mut VesselCheckpoint,
    new_primary: NaifId,
    ep: TdbEpoch,
    fm: &ForceModel,
    ephem: &dyn EphemerisCatalog,
    vid: VesselId,
) -> Result<(), SchedulerError> {
    let (new_r, new_v) = fm
        .switch_primary_frame(
            cp.plant.primary_body,
            new_primary,
            cp.plant.pos_rel_icrf_m,
            cp.plant.vel_rel_icrf_m_s,
            ep,
            ephem,
        )
        .map_err(|source| SchedulerError::FrameSwitch {
            vessel_id: vid,
            source,
        })?;
    cp.plant.primary_body = new_primary;
    cp.plant.pos_rel_icrf_m = new_r;
    cp.plant.vel_rel_icrf_m_s = new_v;
    cp.pinned_surface = None;
    Ok(())
}

impl OrbitalUniverse {
    /// Takes `vessel_id` out of the universe at `committed_instant`, returning its
    /// configuration and a checkpoint integrated to exactly that instant, so another
    /// dynamics tier can carry the vessel on without a state jump. Its queued events are
    /// dropped and it stops being the camera focus.
    pub fn remove_vessel(
        &mut self,
        vessel_id: VesselId,
    ) -> Result<(VesselConfig, VesselCheckpoint), SchedulerError> {
        if !self.vessels.contains_key(&vessel_id) {
            return Err(SchedulerError::UnknownVessel(vessel_id));
        }
        self.rollback_vessel(vessel_id, self.committed_instant)?;
        let timeline = self
            .vessels
            .remove(&vessel_id)
            .ok_or(SchedulerError::UnknownVessel(vessel_id))?;
        self.event_queue
            .retain(|key, _| !key.participants.contains(&vessel_id));
        if self.active_camera_vessel == Some(vessel_id) {
            self.active_camera_vessel = None;
        }
        Ok((timeline.config, timeline.committed_checkpoint))
    }
}

impl OrbitalUniverse {
    /// Holds `loads` on `vessel_id` from `committed_instant` until they are set again: an
    /// owner outside the orbital tier reporting the force, torque and mass its own model
    /// produced at this instant. Returns `false`, changing nothing, when the vessel already
    /// holds the same bits, so a quiet owner leaves the vessel its long integration steps.
    ///
    /// The change is part of the vessel's checkpoint rather than the command log, so an owner
    /// reporting at every exchange grows no log. It applies exactly at `committed_instant`,
    /// after every event at that instant has fired (an advance returns only once they have),
    /// and before any event stamped later for the same instant. It touches no field a player
    /// command or GNC tick writes, so their relative order at one instant does not change the
    /// trajectory. Speculation from this instant on is discarded and the vessel's `state_gen`
    /// advances, so events proposed on the superseded trajectory are skipped.
    ///
    /// A vessel on an unperturbed Kepler arc or at rest on a surface is refused loads that
    /// act on its motion ([`SchedulerError::LoadsNotPropagated`]): its propagation would
    /// not apply them.
    pub fn set_external_loads(
        &mut self,
        vessel_id: VesselId,
        loads: ExternalLoads,
    ) -> Result<bool, SchedulerError> {
        let held = self
            .vessels
            .get(&vessel_id)
            .ok_or(SchedulerError::UnknownVessel(vessel_id))?
            .committed_checkpoint
            .external
            .loads;
        if !loads.is_valid() {
            return Err(SchedulerError::InvalidExternalLoads {
                vessel_id,
                instant: self.committed_instant,
            });
        }
        if held.same_bits(&loads) {
            return Ok(false);
        }
        self.rollback_vessel(vessel_id, self.committed_instant)?;
        if let Some(timeline) = self.vessels.get(&vessel_id) {
            refuse_unapplied_loads(&timeline.committed_checkpoint, &loads)?;
        }
        let timeline = self
            .vessels
            .get_mut(&vessel_id)
            .ok_or(SchedulerError::UnknownVessel(vessel_id))?;
        let checkpoint = &mut timeline.committed_checkpoint;
        checkpoint.external.loads = loads;
        checkpoint.external.generation = checkpoint.external.generation.wrapping_add(1);
        checkpoint.generations.bump_state_gen();
        timeline.frontier_checkpoint = checkpoint.clone();
        Ok(true)
    }

    /// Applies `impulse_body_nms`, an angular impulse about the centre of mass in body
    /// axes, to `vessel_id` at `committed_instant`: its body rate changes by the inertia
    /// it holds now, inverted, times the impulse. This is how an owner whose internal
    /// parts moved hands the reaction to the body the orbital tier flies, together with
    /// the internal momentum it reports through [`Self::set_external_loads`]; set those
    /// loads first, so the impulse meets the inertia that goes with them.
    ///
    /// Like the held loads it is part of the vessel's checkpoint and not of the command
    /// log, and it applies after every event at this instant has fired. A zero impulse
    /// changes nothing and returns `false`. Speculation from this instant on is
    /// discarded and the vessel's `state_gen` advances.
    pub fn exchange_angular_momentum(
        &mut self,
        vessel_id: VesselId,
        impulse_body_nms: glam::DVec3,
    ) -> Result<bool, SchedulerError> {
        let instant = self.committed_instant;
        let invalid = SchedulerError::InvalidExternalLoads { vessel_id, instant };
        if !impulse_body_nms.is_finite() {
            return Err(invalid);
        }
        if !self.vessels.contains_key(&vessel_id) {
            return Err(SchedulerError::UnknownVessel(vessel_id));
        }
        if impulse_body_nms == glam::DVec3::ZERO {
            return Ok(false);
        }
        self.rollback_vessel(vessel_id, instant)?;
        let timeline = self
            .vessels
            .get_mut(&vessel_id)
            .ok_or(SchedulerError::UnknownVessel(vessel_id))?;
        let checkpoint = &mut timeline.committed_checkpoint;
        if checkpoint.integrator_hint.propagator != PropagatorKind::FullCoupled6Dof {
            return Err(SchedulerError::LoadsNotPropagated {
                vessel_id,
                propagator: checkpoint.integrator_hint.propagator,
                instant,
            });
        }
        let inertia = checkpoint
            .mass_model(&timeline.config)
            .inertia_tensor(checkpoint.plant.propellant_mass_kg);
        let rate_change = inertia.inverse() * impulse_body_nms;
        if !rate_change.is_finite() {
            return Err(invalid);
        }
        checkpoint.plant.omega_body_rad_s += rate_change;
        checkpoint.external.generation = checkpoint.external.generation.wrapping_add(1);
        checkpoint.generations.bump_state_gen();
        timeline.frontier_checkpoint = checkpoint.clone();
        Ok(true)
    }
}
