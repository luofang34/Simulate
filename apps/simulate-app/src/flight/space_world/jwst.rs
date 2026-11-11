//! A JWST-like observatory (`JWST_L2`) about Sun–Earth $L_2$: where it starts and
//! the keeper that holds it there inside [`SpaceWorld`].

use glam::{DQuat, DVec3};
use sim_core::{SimInstant, TickSpan};
use sim_orbital::{
    ContinuousPlantState, L2BoundedArcTargetSpec, L2HoldingTarget, MassPropertiesModel, NaifId,
    PropellantMode, ReactionWheelMode, SrpSurface, SynodicFrame, VesselConfig,
    target_l2_terminal_x_arc_dop853,
};
use sim_raketa::{RaketaEnvironment, RaketaPlant};
use sim_spacetime::SimEpoch;
use sim_universe::{L2KeepingBurn, L2StationKeeper, Universe, VesselBusConfig};
use tracing::{info, warn};

use super::super::free_world::SolarSystem;
use super::{JWST_L2, SpaceWorld, universe_error};
use crate::error::AppSimError;

const DAY_S: f64 = 86_400.0;

/// Horizon of the arc the observatory starts on; the insertion burn then puts it on a
/// bounded orbit.
const ARRIVAL_ARC_S: f64 = 60.0 * DAY_S;
/// Out-of-plane amplitude of that arc (`m`).
const ARRIVAL_AZ_M: f64 = 1.2e8;

/// Every three weeks a holding burn aims a revolution ahead, at zero velocity along
/// the Sun–Earth line at the second crossing of the plane.
const HOLDING_TARGET: L2HoldingTarget = L2HoldingTarget {
    maintenance_interval_s: 21.0 * DAY_S,
    crossings_ahead: 2,
    lookahead_s: 240.0 * DAY_S,
    x_bound_m: 6.0e8,
    max_burn_m_s: 30.0,
    min_burn_m_s: 1.0e-3,
};

/// How long each holding burn fires.
const BURN_SPAN: TickSpan = TickSpan::from_seconds_and_subsec_ticks(60, 0);

pub(super) const fn jwst_srp_surface() -> SrpSurface {
    SrpSurface {
        area_m2: 150.0,
        reflectivity_coefficient: 1.48,
    }
}

fn jwst_config(solar: &SolarSystem) -> VesselConfig {
    VesselConfig {
        mass_model: MassPropertiesModel::diagonal(
            6_200.0,
            [45_000.0, 45_000.0, 25_000.0],
            [0.0; 3],
        ),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        tolerances: solar.context().tolerances,
        bounding_radius_m: 11.0,
        auto_soi_switch: false,
    }
}

/// Adds the observatory to `universe` at the first tick of `epoch`, on an arc about
/// $L_2$, and returns the keeper that will hold it; the keeper's first update makes
/// the insertion burn.
pub(super) fn spawn_jwst_l2(
    universe: &mut Universe,
    epoch: &SimEpoch,
    solar: &SolarSystem,
    environment: &RaketaEnvironment,
) -> Result<L2StationKeeper, AppSimError> {
    let tdb_start = epoch.tdb_at(SimInstant::START);
    let frame =
        SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, tdb_start, solar.ephemeris())?;
    let ctx = solar.context();
    let arc = target_l2_terminal_x_arc_dop853(
        &L2BoundedArcTargetSpec::sun_earth_southern(tdb_start, ARRIVAL_ARC_S, ARRIVAL_AZ_M),
        &ctx.force_model,
        solar.ephemeris(),
        solar.orientation(),
    )?;
    let (pos_rel_earth, vel_rel_earth) =
        frame.synodic_to_icrf_rel_secondary(arc.initial_pos_syn_m, arc.initial_vel_syn_m_s);
    let jwst_state = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: pos_rel_earth,
        vel_rel_icrf_m_s: vel_rel_earth,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 150.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let config = jwst_config(solar);
    let mut bus_cfg = VesselBusConfig {
        srp_surface: Some(jwst_srp_surface()),
        ..VesselBusConfig::default()
    };
    bus_cfg.solar_array.area_m2 = 30.0;
    bus_cfg.solar_array.efficiency = 0.32;
    universe
        .insert_orbital_with_bus(
            JWST_L2,
            config,
            jwst_state,
            bus_cfg,
            RaketaPlant::factory(environment.clone()),
        )
        .map_err(universe_error)?;
    Ok(L2StationKeeper::new(
        JWST_L2,
        (NaifId::SUN, NaifId::EARTH),
        &ctx,
        config,
        HOLDING_TARGET,
        BURN_SPAN,
    ))
}

/// Reports a burn the keeper commanded, or why it could not; a keeper that cannot
/// solve (the lookahead runs past the kernels' coverage, or the observatory has
/// already left) must not end the pilot's flight.
fn report_keeping(outcome: Result<Option<L2KeepingBurn>, sim_universe::UniverseError>) {
    match outcome {
        Ok(Some(burn)) => info!(
            delta_v_m_s = burn.delta_v_m_s,
            insertion = burn.insertion,
            "JWST_L2 holding burn"
        ),
        Ok(None) => {}
        Err(error) => warn!(?error, "JWST_L2 station keeping skipped a burn"),
    }
}

impl SpaceWorld {
    /// Lets the keeper burn if a maintenance epoch of `JWST_L2` is due.
    pub(super) fn maintain_jwst_l2(&mut self) {
        report_keeping(self.jwst_keeper.update(&mut self.universe));
    }
}

/// The observatory alone in a universe of its own, as the flight builds it, with its
/// keeper: a year of it can be flown without the pilot's vessel stepping beside it.
#[cfg(test)]
pub(in crate::flight) struct JwstAlone {
    pub(in crate::flight) universe: Universe,
    pub(in crate::flight) keeper: L2StationKeeper,
}

#[cfg(test)]
impl JwstAlone {
    pub(in crate::flight) fn new(
        epoch: &SimEpoch,
        solar: &SolarSystem,
    ) -> Result<Self, AppSimError> {
        let mut universe = Universe::new(epoch.clone(), solar.context());
        let keeper = spawn_jwst_l2(&mut universe, epoch, solar, &solar.raketa_environment())?;
        Ok(Self { universe, keeper })
    }
}
