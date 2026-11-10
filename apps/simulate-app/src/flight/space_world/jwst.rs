//! Sun–Earth $L_2$ observatory (`JWST_L2`) initialization and periodic bounded-halo
//! station-keeping inside [`SpaceWorld`].

use glam::{DQuat, DVec3};
use sim_core::{SimInstant, TickSpan};
use sim_orbital::{
    CoastControlLaw, ContinuousPlantState, L2BoundedArcTargetSpec, LatchedBurnLaw,
    MassPropertiesModel, NaifId, PlayerCommandPayload, PropellantMode, ReactionWheelMode,
    SolarRadiationPressureLaw, SrpSurface, SynodicFrame, VesselConfig,
    approximate_l2_libration_seed, propagate_plant_interval, target_l2_terminal_x_arc_dop853,
};
use sim_raketa::{RaketaEnvironment, RaketaPlant};
use sim_spacetime::{SimEpoch, SolarFluxModel, TdbEpoch};
use sim_universe::{Universe, VesselBusConfig};

use super::super::free_world::SolarSystem;
use super::{JWST_L2, SpaceWorld, universe_error};
use crate::error::AppSimError;

pub(super) const fn jwst_srp_surface() -> SrpSurface {
    SrpSurface {
        area_m2: 150.0,
        reflectivity_coefficient: 1.48,
    }
}

pub(super) fn jwst_mass_model() -> MassPropertiesModel {
    MassPropertiesModel::diagonal(6_200.0, [45_000.0, 45_000.0, 25_000.0], [0.0; 3])
}

pub(super) fn spawn_jwst_l2(
    universe: &mut Universe,
    epoch: &SimEpoch,
    solar: &SolarSystem,
    environment: &RaketaEnvironment,
) -> Result<(), AppSimError> {
    let tdb_start = epoch.tdb_at(SimInstant::START);
    let frame =
        SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, tdb_start, solar.ephemeris())?;
    let ctx = solar.context();
    let spec = L2BoundedArcTargetSpec {
        primary: NaifId::SUN,
        secondary: NaifId::EARTH,
        start_epoch: tdb_start,
        horizon_s: 14.0 * 86_400.0,
        az_amplitude_m: 120_000_000.0,
        southern: true,
        tolerance_x_m: 1_000.0,
        max_iterations: 12,
    };
    let targeted = target_l2_terminal_x_arc_dop853(
        &spec,
        &ctx.force_model,
        solar.ephemeris(),
        solar.orientation(),
    )?;
    let (pos_rel_earth, vel_rel_earth) = frame
        .synodic_to_icrf_rel_secondary(targeted.initial_pos_syn_m, targeted.initial_vel_syn_m_s);
    let jwst_state = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: pos_rel_earth,
        vel_rel_icrf_m_s: vel_rel_earth,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 150.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let jwst_cfg = VesselConfig {
        mass_model: jwst_mass_model(),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        tolerances: ctx.tolerances,
        bounding_radius_m: 11.0,
        auto_soi_switch: false,
    };
    let mut bus_cfg = VesselBusConfig {
        srp_surface: Some(jwst_srp_surface()),
        ..VesselBusConfig::default()
    };
    bus_cfg.solar_array.area_m2 = 30.0;
    bus_cfg.solar_array.efficiency = 0.32;
    universe
        .insert_orbital_with_bus(
            JWST_L2,
            jwst_cfg,
            jwst_state,
            bus_cfg,
            RaketaPlant::factory(environment.clone()),
        )
        .map_err(universe_error)
}

impl SpaceWorld {
    pub(super) fn maintain_jwst_l2(&mut self) -> Result<(), AppSimError> {
        let now = self.universe.now();
        let at = self.universe.epoch().tdb_at(now);
        let jwst = self
            .universe
            .inertial_state(JWST_L2)
            .map_err(universe_error)?;
        let dv_icrf = compute_jwst_stationkeeping_dv(&jwst, at, &self.solar)?;
        if dv_icrf.length() > 1.0e-4 {
            let burn_s = 60.0;
            let burn_span = TickSpan::from_seconds_and_subsec_ticks(60, 0);
            let total_mass_kg = 6_200.0 + jwst.propellant_mass_kg;
            self.universe
                .command(
                    JWST_L2,
                    now,
                    PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
                        dir_icrf: dv_icrf.normalize_or_zero().to_array(),
                        thrust_n: total_mass_kg * dv_icrf.length() / burn_s,
                        exhaust_vel_m_s: 3_000.0,
                        rw_torque_cmd_body_nm: [0.0; 3],
                    }),
                )
                .map_err(universe_error)?;
            self.universe
                .command(
                    JWST_L2,
                    now.advanced(burn_span),
                    PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::Coast),
                )
                .map_err(universe_error)?;
        }
        Ok(())
    }
}

fn compute_jwst_stationkeeping_dv(
    jwst: &ContinuousPlantState,
    at: TdbEpoch,
    solar: &SolarSystem,
) -> Result<DVec3, AppSimError> {
    let frame = SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, at, solar.ephemeris())?;
    let ctx = solar.context();
    let mu_sun = ctx.force_model.constants().require_mu(NaifId::SUN)?;
    let mu_earth = ctx.force_model.constants().require_mu(NaifId::EARTH)?;
    let seed = approximate_l2_libration_seed(&frame, mu_sun, mu_earth, 120_000_000.0, true);
    let omega_p = std::f64::consts::TAU / seed.period_s.max(1.0);
    let (pos_syn, vel_syn) =
        frame.icrf_rel_secondary_to_synodic(jwst.pos_rel_icrf_m, jwst.vel_rel_icrf_m_s);
    let tf = at.advanced_seconds(7.0 * 86_400.0);
    let frame_tf = SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, tf, solar.ephemeris())?;
    let vx_damped = -0.35 * omega_p * (pos_syn.x - seed.l2_x_rel_secondary_m);
    let vy_solved = solve_secant_vy(vel_syn.y, |vy_test| {
        let v_syn = DVec3::new(vx_damped, vy_test, vel_syn.z);
        let (p0, v0) = frame.synodic_to_icrf_rel_secondary(pos_syn, v_syn);
        let init = ContinuousPlantState {
            pos_rel_icrf_m: p0,
            vel_rel_icrf_m_s: v0,
            ..*jwst
        };
        let srp = SolarRadiationPressureLaw {
            inner: &CoastControlLaw,
            surface: jwst_srp_surface(),
            flux: &SolarFluxModel::standard(),
            ephemeris: solar.ephemeris(),
        };
        let out = propagate_plant_interval(
            &init,
            at,
            tf,
            None,
            &ctx.tolerances,
            &jwst_mass_model(),
            PropellantMode::InfiniteConstantMass,
            ReactionWheelMode::ConserveMomentum {
                h_max_nms: 0.0,
                tau_max_nm: 0.0,
            },
            &ctx.force_model,
            solar.ephemeris(),
            solar.orientation(),
            &srp,
        )
        .map_err(|e| AppSimError::Lagrange(sim_orbital::LagrangeError::Propagation(e)))?;
        let (p_syn_f, _) = frame_tf.icrf_rel_secondary_to_synodic(
            out.final_state.pos_rel_icrf_m,
            out.final_state.vel_rel_icrf_m_s,
        );
        Ok(p_syn_f.x - seed.l2_x_rel_secondary_m)
    })?;
    let target_v_syn = DVec3::new(vx_damped, vy_solved, vel_syn.z);
    let (_, target_v_icrf) = frame.synodic_to_icrf_rel_secondary(pos_syn, target_v_syn);
    Ok(target_v_icrf - jwst.vel_rel_icrf_m_s)
}

fn solve_secant_vy<F>(initial_vy: f64, eval_res: F) -> Result<f64, AppSimError>
where
    F: Fn(f64) -> Result<f64, AppSimError>,
{
    let mut vy0 = initial_vy;
    let mut f0 = eval_res(vy0)?;
    let mut vy1 = vy0 + 2.0;
    for _ in 0..5 {
        if f0.abs() <= 5_000.0 {
            break;
        }
        let f1 = eval_res(vy1)?;
        let df = f1 - f0;
        if df.abs() < 1.0e-6 {
            break;
        }
        let vy_next = (vy1 - f1 * (vy1 - vy0) / df).clamp(initial_vy - 80.0, initial_vy + 80.0);
        vy0 = vy1;
        f0 = f1;
        vy1 = vy_next;
    }
    Ok(vy0)
}
