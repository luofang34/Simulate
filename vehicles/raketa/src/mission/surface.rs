//! The uninterrupted mission from the Earth launch pad to rest on the destination surface,
//! flown as one propagated trajectory, and the measurements taken of it.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_geo::LocalFrame;
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContinuousPlantState, ForceModel,
    PinnedSurfaceState, STANDARD_GRAVITY_G0_M_S2, TdbEpoch,
};

use super::{
    continuous::fly_pad_to_destination_soi,
    landing::{TargetSiteSpec, target_site_config},
    voyage::{RaketaOriginChange, Voyage},
};
use crate::mission::{RaketaMissionError, RaketaMissionPhase, RaketaMissionTarget};

/// Measured outcome of [`run_uninterrupted_mission`]: one propagated trajectory from the
/// Earth pad to rest on the destination surface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaSurfaceMissionSummary {
    /// Mission target.
    pub target: RaketaMissionTarget,
    /// Phases flown, in order.
    pub phases_completed: Vec<RaketaMissionPhase>,
    /// Writes to the vessel state that did not come from a propagator or an origin change,
    /// counted by comparing each propagation's input with the previous output.
    pub explicit_state_reinitializations: u32,
    /// Every origin change, with the vessel's `SSB` state on both sides.
    pub origin_changes: Vec<RaketaOriginChange>,
    /// Launch epoch (`TDB`).
    pub start_epoch: TdbEpoch,
    /// Epoch of entry into the destination's sphere of influence (`TDB`).
    pub soi_entry_epoch: TdbEpoch,
    /// Epoch at which the braking burn ended (`TDB`).
    pub braking_end_epoch: TdbEpoch,
    /// Epoch of first foot contact (`TDB`).
    pub first_contact_epoch: TdbEpoch,
    /// Epoch at the end of the flight, settled in `PinnedSurfaceRest` (`TDB`).
    pub rest_epoch: TdbEpoch,
    /// Thrust $\Delta v$ from the pad to the parking orbit (`m/s`).
    pub ascent_delta_v_m_s: f64,
    /// Thrust $\Delta v$ of the trans-injection burn (`m/s`).
    pub injection_delta_v_m_s: f64,
    /// Thrust $\Delta v$ of the interplanetary or cislunar midcourse corrections (`m/s`).
    pub midcourse_delta_v_m_s: f64,
    /// Thrust $\Delta v$ of the approach aiming burns inside the sphere of influence (`m/s`).
    pub approach_targeting_delta_v_m_s: f64,
    /// Thrust $\Delta v$ of the braking burn that cancels the arrival velocity (`m/s`).
    pub braking_delta_v_m_s: f64,
    /// Capture $\Delta v$: approach aiming plus braking (`m/s`).
    pub capture_delta_v_m_s: f64,
    /// Thrust $\Delta v$ of the powered descent and touchdown (`m/s`).
    pub descent_delta_v_m_s: f64,
    /// Peak proper acceleration over the whole flight (`G`).
    pub peak_proper_accel_g: f64,
    /// Whether all fins stayed locked on the launch pad.
    pub pad_fin_lock_verified: bool,
    /// Whether all fins were locked neutral just before touchdown.
    pub landing_fin_lock_verified: bool,
    /// Downward speed relative to the surface just before first contact (`m/s`).
    pub touchdown_vertical_speed_m_s: f64,
    /// Horizontal speed relative to the surface just before first contact (`m/s`).
    pub touchdown_horizontal_speed_m_s: f64,
    /// Surface speed at the start of the contact interval in which `PinnedSurfaceRest`
    /// locked; the contact model rebuilds the state from the pinned pose, and this bounds
    /// the velocity that rebuild removes (`m/s`).
    pub pin_chunk_start_speed_m_s: f64,
    /// Body-fixed CG displacement across that interval, the pin rebuild included (`m`).
    pub pin_chunk_displacement_m: f64,
    /// Final propagated state about the destination body.
    pub final_state: ContinuousPlantState,
    /// Final pinned surface pose.
    pub final_pinned_state: PinnedSurfaceState,
    /// Lowest foot-pad clearance above the contact model's terrain at the end (`m`).
    pub final_foot_clearance_m: f64,
    /// Speed relative to the turning surface at the end (`m/s`).
    pub final_surface_speed_m_s: f64,
    /// Angle between the body axis and the local vertical at the end (`deg`).
    pub final_tilt_deg: f64,
    /// Planetodetic latitude of the landing point (`deg`).
    pub landing_lat_deg: f64,
    /// Planetodetic longitude of the landing point (`deg`).
    pub landing_lon_deg: f64,
    /// Surface distance from the landing point to the configured target site (`m`).
    pub landing_distance_from_site_m: f64,
}

/// Executes a continuous, zero-state-reset mission from the Earth pad to rest on the surface
/// of the Moon or Mars.
///
/// Pad, ascent, injection and transfer are flown as in [`super::run_continuous_pad_to_transfer`];
/// inside the destination's sphere of influence the vessel aims at a gate over the landing
/// site, brakes propulsively to rest above it, descends on the main engine with the nose
/// held vertical, and settles through the compliant contact model.
pub fn run_uninterrupted_mission(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    start_epoch: TdbEpoch,
    target: RaketaMissionTarget,
) -> Result<RaketaSurfaceMissionSummary, RaketaMissionError> {
    let (mut voyage, departure) =
        fly_pad_to_destination_soi(solar, force_model, start_epoch, target)?;
    let mut phases = departure.phases;
    let soi_entry_epoch = voyage.epoch;
    let site = target_site_config(target);
    let gate_alt_m = match target {
        RaketaMissionTarget::MoonTranquility => 5_000.0,
        RaketaMissionTarget::MarsSurface => 30_000.0,
    };
    let arrival = voyage.approach_and_brake(&site, gate_alt_m)?;
    let braking_end_epoch = voyage.epoch;
    phases.push(RaketaMissionPhase::ArrivalBrakingOrEntry);
    let landing = voyage.descend_and_land(&site, arrival.touchdown_fixed_m)?;
    phases.push(RaketaMissionPhase::PoweredDescentAutoland);
    phases.push(RaketaMissionPhase::TargetSurfacePinnedRest);
    voyage.audit_continuity();
    let rest = measure_rest(&voyage, &site)?;

    Ok(RaketaSurfaceMissionSummary {
        target,
        phases_completed: phases,
        explicit_state_reinitializations: voyage.state_reinitializations,
        origin_changes: voyage.origin_changes.clone(),
        start_epoch,
        soi_entry_epoch,
        braking_end_epoch,
        first_contact_epoch: landing.first_contact_epoch,
        rest_epoch: voyage.epoch,
        ascent_delta_v_m_s: departure.ascent_delta_v_m_s,
        injection_delta_v_m_s: voyage.injection_burn_delta_v_m_s,
        midcourse_delta_v_m_s: voyage.midcourse_correction_delta_v_m_s,
        approach_targeting_delta_v_m_s: arrival.targeting_delta_v_m_s,
        braking_delta_v_m_s: arrival.braking_delta_v_m_s,
        capture_delta_v_m_s: arrival.targeting_delta_v_m_s + arrival.braking_delta_v_m_s,
        descent_delta_v_m_s: landing.descent_delta_v_m_s,
        peak_proper_accel_g: voyage.peak_proper_accel_m_s2 / STANDARD_GRAVITY_G0_M_S2,
        pad_fin_lock_verified: departure.fin_lock_verified,
        landing_fin_lock_verified: landing.fins_locked_at_contact,
        touchdown_vertical_speed_m_s: landing.contact_vertical_speed_m_s,
        touchdown_horizontal_speed_m_s: landing.contact_horizontal_speed_m_s,
        pin_chunk_start_speed_m_s: landing.pin_chunk_start_speed_m_s,
        pin_chunk_displacement_m: landing.pin_chunk_displacement_m,
        final_state: voyage.state,
        final_pinned_state: landing.pinned,
        final_foot_clearance_m: rest.foot_clearance_m,
        final_surface_speed_m_s: rest.surface_speed_m_s,
        final_tilt_deg: rest.tilt_deg,
        landing_lat_deg: rest.lat_deg,
        landing_lon_deg: rest.lon_deg,
        landing_distance_from_site_m: rest.distance_from_site_m,
    })
}

struct Rest {
    foot_clearance_m: f64,
    surface_speed_m_s: f64,
    tilt_deg: f64,
    lat_deg: f64,
    lon_deg: f64,
    distance_from_site_m: f64,
}

fn measure_rest(voyage: &Voyage<'_>, site: &TargetSiteSpec) -> Result<Rest, RaketaMissionError> {
    let rot = voyage
        .solar
        .orientation
        .body_rotation(site.body, voyage.epoch)?;
    let (p_f, v_f) =
        rot.icrf_to_fixed_pv(voyage.state.pos_rel_icrf_m, voyage.state.vel_rel_icrf_m_s);
    let geo = site.ellipsoid.fixed_to_geodetic(p_f);
    let up = LocalFrame::at(&geo).up();
    let axis_fixed = rot.r_icrf_to_fixed * (voyage.state.q_body_to_icrf * DVec3::Y);
    let site_fixed = site.ellipsoid.geodetic_to_fixed(&site.site_geo);
    Ok(Rest {
        foot_clearance_m: voyage.foot_clearance(site)?,
        surface_speed_m_s: v_f.length(),
        tilt_deg: axis_fixed.angle_between(up).to_degrees(),
        lat_deg: geo.lat_rad.to_degrees(),
        lon_deg: geo.lon_rad.to_degrees(),
        distance_from_site_m: p_f.angle_between(site_fixed) * site.ellipsoid.semi_major_m,
    })
}
