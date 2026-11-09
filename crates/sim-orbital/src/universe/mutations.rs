//! Discrete event mutation handlers for [`crate::OrbitalUniverse`].

use sim_spacetime::SimInstant;

use crate::{
    checkpoint::VesselCheckpoint,
    ephemeris::EphemerisCatalog,
    events::{
        EventOrderKey, PlayerCommandPayload, ScheduledEvent, UniverseEventPayload, phase_ranks,
    },
    gravity::ForceModel,
    ids::{NaifId, VesselId},
    time::TdbEpoch,
    universe::SchedulerError,
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
