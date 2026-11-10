//! Scenario-initialized Mars atmospheric `EntryGCorridor`, terminal `PoweredDescentAutoland`,
//! `< 50 m` fin ground-lock, and `1 ms` compliant touchdown settling into `PinnedSurfaceRest`.

use glam::{DQuat, DVec3};
use sim_core::TickSpan;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_orbital::{
    AutolandGuidanceConfig, BodyOrientationCatalog, BuiltinSolarSystem2026_2028, CoastControlLaw,
    ContactFootSpec, ContinuousPlantState, EntryCorridorConfig, ForceModel, NaifId,
    PinnedSurfaceState, PropellantMode, STANDARD_GRAVITY_G0_M_S2, TdbEpoch, VesselContactState,
    evaluate_entry_corridor_guidance, evaluate_powered_descent_autoland,
    integrate_contact_interval, propagate_plant_interval, quat_normalize_portable,
};
use sim_scenery::PinnedCollisionTerrain;

use crate::{
    aero_actuators::{
        RAKETA_CG_Y_M, RAKETA_MASS_KG, RaketaStageLaw, raketa_default_rw_mode, raketa_mass_model,
    },
    gnc_boundary::RaketaOuterGuidanceSetpoint,
    mission::{RaketaMissionError, RaketaMissionTarget, closed_loop_tols},
};

/// A configured landing site and the collision terrain the contact model uses there.
pub(super) struct TargetSiteSpec {
    pub(super) body: NaifId,
    pub(super) ellipsoid: BodyEllipsoid,
    pub(super) terrain: PinnedCollisionTerrain,
    pub(super) site_geo: Geodetic,
    pub(super) ground_h_m: f64,
}

pub(super) fn run_staged_arrival_or_entry_demo(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    target: RaketaMissionTarget,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
) -> Result<(f64, u32), RaketaMissionError> {
    if target == RaketaMissionTarget::MarsSurface {
        let peak = run_staged_mars_entry_corridor_demo(solar, force_model, epoch, state)?;
        return Ok((peak, 1));
    }
    let law = RaketaStageLaw {
        commanded_main_accel_m_s2: 2.5 * STANDARD_GRAVITY_G0_M_S2,
        desired_body_y_icrf: -state.vel_rel_icrf_m_s.normalize_or_zero(),
        rcs_cmd_unit_body: DVec3::ZERO,
        fin_cmd_deg: [0.0; 4],
        is_landing_or_pad_mode: false,
        orientation: &solar.orientation,
    };
    let (_, lim, _, _) = law.evaluate_detailed(*epoch, state);
    let t_next = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(10, 0));
    let prop = propagate_plant_interval(
        state,
        *epoch,
        t_next,
        None,
        &closed_loop_tols(),
        &raketa_mass_model(),
        PropellantMode::InfiniteConstantMass,
        raketa_default_rw_mode(),
        force_model,
        &solar.ephemeris,
        &solar.orientation,
        &law,
    )?;
    *state = prop.final_state;
    *epoch = t_next;
    Ok((lim.resulting_proper_accel_m_s2, 0))
}

fn run_staged_mars_entry_corridor_demo(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
) -> Result<f64, RaketaMissionError> {
    let ell = BodyEllipsoid::MARS_IAU;
    let entry_geo = Geodetic {
        lat_rad: 18.38_f64.to_radians(),
        lon_rad: 77.58_f64.to_radians(),
        height_m: 42_000.0,
    };
    let rot = solar.orientation.body_rotation(NaifId::MARS, *epoch)?;
    let r_f2i = rot.r_icrf_to_fixed.transpose();
    let p_fixed = ell.geodetic_to_fixed(&entry_geo);
    let lf = LocalFrame::at(&entry_geo);
    let v_fixed = lf.enu_to_ecef() * DVec3::new(2_100.0, 0.0, -420.0);
    let (p_i, v_i) = rot.fixed_to_icrf_pv(p_fixed, v_fixed);
    // Explicit scenario state initialization at Mars entry interface (42 km altitude):
    state.pos_rel_icrf_m = p_i;
    state.vel_rel_icrf_m_s = v_i;
    state.q_body_to_icrf = quat_normalize_portable(DQuat::from_rotation_arc(
        DVec3::Y,
        -(r_f2i * v_fixed).normalize(),
    ));

    let cfg = EntryCorridorConfig::default();
    let mut peak_a = 0.0_f64;
    let mut step = 0_u32;
    while step < 6 {
        let probe_law = RaketaStageLaw {
            commanded_main_accel_m_s2: 0.0,
            desired_body_y_icrf: -state.vel_rel_icrf_m_s.normalize(),
            rcs_cmd_unit_body: DVec3::ZERO,
            fin_cmd_deg: [4.0, -4.0, 4.0, -4.0],
            is_landing_or_pad_mode: false,
            orientation: &solar.orientation,
        };
        let (_, _, aero, _) = probe_law.evaluate_detailed(*epoch, state);
        let aero_icrf =
            sim_orbital::quat_rotate_vec_portable(state.q_body_to_icrf, aero.force_body_n)
                / RAKETA_MASS_KG;
        let ec = evaluate_entry_corridor_guidance(
            state.pos_rel_icrf_m,
            state.vel_rel_icrf_m_s,
            aero_icrf,
            entry_geo.height_m,
            &cfg,
        );
        let active_law = RaketaOuterGuidanceSetpoint::from_entry_corridor(
            &ec,
            libm::sqrt(aero_icrf.length_squared()),
            cfg.max_proper_accel_m_s2,
            [4.0, -4.0, 4.0, -4.0],
        )
        .into_stage_law(&solar.orientation);
        let (_, lim, _, _) = active_law.evaluate_detailed(*epoch, state);
        peak_a = peak_a.max(lim.resulting_proper_accel_m_s2);
        let t_next = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(4, 0));
        let prop = propagate_plant_interval(
            state,
            *epoch,
            t_next,
            None,
            &closed_loop_tols(),
            &raketa_mass_model(),
            PropellantMode::InfiniteConstantMass,
            raketa_default_rw_mode(),
            force_model,
            &solar.ephemeris,
            &solar.orientation,
            &active_law,
        )?;
        *state = prop.final_state;
        *epoch = t_next;
        step = step.wrapping_add(1);
    }
    Ok(peak_a)
}

/// Initializes a scenario state at `18 m` AGL terminal approach above `target` and demonstrates
/// `PoweredDescentAutoland` guidance evaluation followed by compliant touchdown settling.
pub(super) fn run_staged_autoland_and_surface_settle_demo(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    target: RaketaMissionTarget,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
) -> Result<(PinnedSurfaceState, f64, bool, u32), RaketaMissionError> {
    let site = target_site_config(target);
    let approach_geo = Geodetic {
        height_m: site.ground_h_m + (RAKETA_CG_Y_M - 0.0369) + 18.0,
        ..site.site_geo
    };
    let rot = solar.orientation.body_rotation(site.body, *epoch)?;
    let r_f2i = rot.r_icrf_to_fixed.transpose();
    let lf = LocalFrame::at(&approach_geo);
    let up_icrf = (r_f2i * lf.up()).normalize();
    let (p_app, v_app) = rot.fixed_to_icrf_pv(
        site.ellipsoid.geodetic_to_fixed(&approach_geo),
        lf.enu_to_ecef() * DVec3::new(0.4, 0.0, -1.8),
    );
    state.primary_body = site.body;
    state.pos_rel_icrf_m = p_app;
    state.vel_rel_icrf_m_s = v_app;
    state.q_body_to_icrf = quat_normalize_portable(DQuat::from_rotation_arc(DVec3::Y, up_icrf));
    state.omega_body_rad_s = DVec3::ZERO;

    let pad_geo = Geodetic {
        height_m: site.ground_h_m + (RAKETA_CG_Y_M - 0.0369),
        ..site.site_geo
    };
    let (p_pad, v_pad) =
        rot.fixed_to_icrf_pv(site.ellipsoid.geodetic_to_fixed(&pad_geo), DVec3::ZERO);
    let g_icrf = force_model.relative_gravitational_accel_icrf(
        site.body,
        state.pos_rel_icrf_m,
        *epoch,
        &solar.ephemeris,
        &solar.orientation,
    )?;
    let al = evaluate_powered_descent_autoland(
        state.pos_rel_icrf_m,
        state.vel_rel_icrf_m_s,
        p_pad,
        v_pad,
        up_icrf,
        g_icrf,
        18.0,
        &AutolandGuidanceConfig::default(),
    );
    let law = RaketaOuterGuidanceSetpoint::from_powered_descent(
        &al,
        libm::sqrt((state.vel_rel_icrf_m_s - v_pad).length_squared()),
        [15.0, -15.0, 15.0, -15.0],
    )
    .into_stage_law(&solar.orientation);
    let (_, lim, _, fin_locked) = law.evaluate_detailed(*epoch, state);
    let (pinned, settle_peak) =
        settle_touchdown_on_terrain(solar, force_model, &site, &rot, &lf, epoch, state)?;
    Ok((
        pinned,
        lim.resulting_proper_accel_m_s2.max(settle_peak),
        al.lock_fins_neutral && fin_locked,
        2,
    ))
}

fn settle_touchdown_on_terrain(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    site: &TargetSiteSpec,
    rot: &sim_orbital::BodyRotationState,
    lf: &LocalFrame,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
) -> Result<(PinnedSurfaceState, f64), RaketaMissionError> {
    let td_geo = Geodetic {
        height_m: site.ground_h_m + (RAKETA_CG_Y_M - 0.0369) + 0.04,
        ..site.site_geo
    };
    let (p_td, v_td) = rot.fixed_to_icrf_pv(
        site.ellipsoid.geodetic_to_fixed(&td_geo),
        lf.enu_to_ecef() * DVec3::new(0.15, 0.0, -0.65),
    );
    state.pos_rel_icrf_m = p_td;
    state.vel_rel_icrf_m_s = v_td;
    let t_settle = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(6, 0));
    let feet = ContactFootSpec::raketa_four_fin_pads(RAKETA_CG_Y_M);
    let settled = integrate_contact_interval(
        state,
        VesselContactState::default(),
        &feet,
        *epoch,
        t_settle,
        &raketa_mass_model(),
        PropellantMode::InfiniteConstantMass,
        raketa_default_rw_mode(),
        force_model,
        &site.terrain,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )?;
    *state = settled.state;
    *epoch = t_settle;
    let pinned = settled
        .contact
        .pinned
        .ok_or(RaketaMissionError::TouchdownDidNotSettle { body: site.body })?;
    Ok((pinned, settled.peak_proper_accel_m_s2))
}

pub(super) fn target_site_config(target: RaketaMissionTarget) -> TargetSiteSpec {
    let (body, ellipsoid, terrain, site_geo) = match target {
        RaketaMissionTarget::MoonTranquility => (
            NaifId::MOON,
            BodyEllipsoid::MOON_IAU,
            PinnedCollisionTerrain::moon_iau(11, Vec::new()),
            Geodetic {
                lat_rad: 0.674_08_f64.to_radians(),
                lon_rad: 23.472_97_f64.to_radians(),
                height_m: 0.0,
            },
        ),
        RaketaMissionTarget::MarsSurface => (
            NaifId::MARS,
            BodyEllipsoid::MARS_IAU,
            PinnedCollisionTerrain::mars_iau(0.0, 0.0, Vec::new()),
            Geodetic {
                lat_rad: 18.38_f64.to_radians(),
                lon_rad: 77.58_f64.to_radians(),
                height_m: 0.0,
            },
        ),
    };
    let ground_h_m = terrain.sample_ground(&site_geo).ellipsoidal_height_m;
    TargetSiteSpec {
        body,
        ellipsoid,
        terrain,
        site_geo,
        ground_h_m,
    }
}
