//! Dedicated `1 ms` fixed-substep compliant ground contact integrator with
//! anchor-based stateful bristle stiction and `PinnedSurfaceRest` equilibrium.
//!
//! Keeps stiff landing-gear springs (`~2.4 MN/m`) completely out of adaptive
//! `DOP853`, guarantees **zero steady-state downhill creep** inside the static
//! friction cone $\mu_s$, and transitions settled landers to `PinnedSurfaceRest`.

mod substep;

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_spacetime::{CollisionTerrain, TICKS_PER_SECOND, TickSpan};

use crate::{
    ephemeris::EphemerisCatalog,
    gravity::{ForceModel, GravityError},
    ids::NaifId,
    orientation::BodyOrientationCatalog,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, MassPropertiesModel, PropagationAudit,
        PropellantMode, ReactionWheelMode, quat_normalize_portable, quat_rotate_vec_portable,
    },
    scalar_math::sqrt,
    time::TdbEpoch,
};

use substep::step_one_contact_substep;

/// Altitude clearance threshold (`m`) below which propagation switches to [`integrate_contact_interval`].
pub const CONTACT_PROXIMITY_THRESHOLD_M: f64 = 0.05;

/// Fixed substep duration (`1 ms = 0.001 s`) for compliant ground contact integration.
pub const CONTACT_SUBSTEP_SECONDS: f64 = 1.0e-3;

/// Number of consecutive slip-free low-energy `1 ms` substeps (`0.25 s`) required before locking `PinnedSurfaceRest`.
pub const PINNED_REST_SETTLE_SUBSTEPS: u32 = 250;

/// Maximum body-fixed speed (`m/s`) allowed to enter `PinnedSurfaceRest`.
pub const PINNED_REST_MAX_SPEED_M_S: f64 = 1.0e-3;

/// Maximum body-fixed angular rate (`rad/s`) allowed to enter `PinnedSurfaceRest`.
pub const PINNED_REST_MAX_OMEGA_RAD_S: f64 = 1.0e-3;

/// Geometry of one landing foot/pad relative to the vessel's nominal body origin/CG.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactFootSpec {
    /// Foot contact pad center in body coordinates (`m`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub pos_body_m: DVec3,
    /// Effective spherical pad contact radius (`m`, `>= 0`).
    pub pad_radius_m: f64,
}

impl ContactFootSpec {
    /// Four symmetric landing feet matching `Raketa`'s fin-tip pads (`radius = 1.6712 m`, `y_pad = 0.0369 m`
    /// relative to hull base, offset from CG `y_cg_m`).
    #[must_use]
    pub fn raketa_four_fin_pads(y_cg_m: f64) -> [Self; 4] {
        let r = 1.6712_f64;
        let y_rel = 0.0369_f64 - y_cg_m;
        [
            Self {
                pos_body_m: DVec3::new(r, y_rel, 0.0),
                pad_radius_m: 0.04,
            },
            Self {
                pos_body_m: DVec3::new(-r, y_rel, 0.0),
                pad_radius_m: 0.04,
            },
            Self {
                pos_body_m: DVec3::new(0.0, y_rel, r),
                pad_radius_m: 0.04,
            },
            Self {
                pos_body_m: DVec3::new(0.0, y_rel, -r),
                pad_radius_m: 0.04,
            },
        ]
    }
}

/// Per-foot stateful elastic-bristle stiction anchor in body-fixed planetary coordinates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BristleFootState {
    /// Whether this foot currently has an active ground stiction anchor.
    pub anchored: bool,
    /// Body-fixed (`ECEF` / `IAU_BODY`) anchor point `[x, y, z]` (`m`).
    pub anchor_fixed_m: [f64; 3],
    /// Whether this foot slipped outside the static Coulomb cone on the latest substep.
    pub slipping: bool,
}

/// Locked body-fixed surface pose for a vessel settled in `PinnedSurfaceRest`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PinnedSurfaceState {
    /// Surface body (`NaifId::EARTH`, `MOON`, or `MARS`).
    pub body: NaifId,
    /// CG position in body-fixed (`ECEF` / `IAU_BODY`) coordinates (`m`).
    pub pos_fixed_m: [f64; 3],
    /// Unit quaternion rotating vessel body axes to body-fixed planetary axes `[x, y, z, w]`.
    pub q_body_to_fixed_xyzw: [f64; 4],
}

impl PinnedSurfaceState {
    /// Captures the body-fixed surface anchor corresponding to `plant` at `epoch`.
    pub fn from_icrf_plant_state(
        plant: &ContinuousPlantState,
        epoch: TdbEpoch,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<Self, GravityError> {
        let rot = orientation.body_rotation(plant.primary_body, epoch)?;
        let (pos_fixed, _) = rot.icrf_to_fixed_pv(plant.pos_rel_icrf_m, plant.vel_rel_icrf_m_s);
        let q_i2f = quat_normalize_portable(DQuat::from_mat3(&rot.r_icrf_to_fixed));
        let q_b2f = quat_normalize_portable(q_i2f * plant.q_body_to_icrf);
        Ok(Self {
            body: plant.primary_body,
            pos_fixed_m: pos_fixed.to_array(),
            q_body_to_fixed_xyzw: q_b2f.to_array(),
        })
    }

    /// Reconstructs the instantaneous `ICRF` [`ContinuousPlantState`] of a pinned vessel at `epoch`.
    pub fn to_icrf_plant_state(
        &self,
        epoch: TdbEpoch,
        propellant_mass_kg: f64,
        h_rw_body_nms: DVec3,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<ContinuousPlantState, GravityError> {
        let rot = orientation.body_rotation(self.body, epoch)?;
        let r_f2i = rot.r_icrf_to_fixed.transpose();
        let p_fixed = DVec3::from_array(self.pos_fixed_m);
        let (pos_icrf, vel_icrf) = rot.fixed_to_icrf_pv(p_fixed, DVec3::ZERO);

        let q_b2f = quat_normalize_portable(DQuat::from_array(self.q_body_to_fixed_xyzw));
        let q_f2i = quat_normalize_portable(DQuat::from_mat3(&r_f2i));
        let q_b2i = quat_normalize_portable(q_f2i * q_b2f);
        let omega_body = quat_rotate_vec_portable(q_b2f.conjugate(), rot.omega_body_rad_s);

        Ok(ContinuousPlantState {
            primary_body: self.body,
            pos_rel_icrf_m: pos_icrf,
            vel_rel_icrf_m_s: vel_icrf,
            q_body_to_icrf: q_b2i,
            omega_body_rad_s: omega_body,
            propellant_mass_kg,
            h_rw_body_nms,
        })
    }

    /// Reconstructs both the `ICRF` [`ContinuousPlantState`] and its centripetal `ICRF` acceleration at `epoch`.
    pub fn to_icrf_plant_and_accel(
        &self,
        epoch: TdbEpoch,
        propellant_mass_kg: f64,
        h_rw_body_nms: DVec3,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<(ContinuousPlantState, DVec3), GravityError> {
        let st = self.to_icrf_plant_state(epoch, propellant_mass_kg, h_rw_body_nms, orientation)?;
        let rot = orientation.body_rotation(self.body, epoch)?;
        let omega_icrf = rot.r_icrf_to_fixed.transpose() * rot.omega_body_rad_s;
        let accel_icrf = omega_icrf.cross(st.vel_rel_icrf_m_s);
        Ok((st, accel_icrf))
    }
}

/// Complete contact/stiction runtime state carried across `ContactSubstep` horizons.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VesselContactState {
    /// Per-foot bristle anchor states (matched by index to [`ContactFootSpec`]).
    pub feet: Vec<BristleFootState>,
    /// Consecutive low-energy slip-free substeps accumulated toward `PinnedSurfaceRest`.
    pub settle_substeps: u32,
    /// Active pinned surface pose when settled (`Some` iff `PinnedSurfaceRest`).
    pub pinned: Option<PinnedSurfaceState>,
}

/// Outcome of advancing a vessel over a contact substep interval.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactStepOutput {
    /// Updated `ICRF` plant state at `end_epoch`.
    pub state: ContinuousPlantState,
    /// Updated bristle and pinned-rest state.
    pub contact: VesselContactState,
    /// Foot ground clearance at the end of the interval (`m`, negative = compressed); zero when the
    /// vessel is pinned at rest.
    pub min_foot_clearance_m: f64,
    /// Peak proper acceleration (`|(F_contact + F_thrust + F_aero)/m|`) experienced (`m/s^2`).
    pub peak_proper_accel_m_s2: f64,
    /// Accumulated idealization/exceedance flags.
    pub audit: PropagationAudit,
}

/// Effective gravity (`m/s^2`, body-fixed frame) at a pinned vessel's position: the body's
/// gravitation minus the centripetal term of the rotating frame. A vessel at rest on the
/// surface feels this as its proper acceleration with opposite sign.
fn pinned_effective_gravity_fixed(
    pinned: &PinnedSurfaceState,
    propellant_mass_kg: f64,
    h_rw_body_nms: DVec3,
    epoch: TdbEpoch,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<DVec3, GravityError> {
    let plant =
        pinned.to_icrf_plant_state(epoch, propellant_mass_kg, h_rw_body_nms, orientation)?;
    let rot = orientation.body_rotation(pinned.body, epoch)?;
    let g_icrf = force_model.relative_gravitational_accel_icrf(
        pinned.body,
        plant.pos_rel_icrf_m,
        epoch,
        ephemeris,
        orientation,
    )?;
    let p_fixed = DVec3::from_array(pinned.pos_fixed_m);
    let om = rot.omega_body_rad_s;
    Ok(rot.r_icrf_to_fixed * g_icrf - om.cross(om.cross(p_fixed)))
}

/// Proper acceleration magnitude (`m/s^2`) of a vessel at rest on the surface at `epoch`.
fn pinned_rest_proper_accel(
    pinned: &PinnedSurfaceState,
    plant: &ContinuousPlantState,
    epoch: TdbEpoch,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<f64, GravityError> {
    let g_eff = pinned_effective_gravity_fixed(
        pinned,
        plant.propellant_mass_kg,
        plant.h_rw_body_nms,
        epoch,
        force_model,
        ephemeris,
        orientation,
    )?;
    Ok(g_eff.length())
}

/// Evaluates whether a `PinnedSurfaceRest` vessel remains in static load equilibrium at `epoch`
/// (`true` = stays pinned; `false` = liftoff thrust or lateral load exceeds static friction cone).
#[allow(clippy::too_many_arguments)]
pub fn verify_pinned_static_equilibrium(
    pinned: &PinnedSurfaceState,
    propellant_mass_kg: f64,
    h_rw_body_nms: DVec3,
    epoch: TdbEpoch,
    mass_model: &MassPropertiesModel,
    force_model: &ForceModel,
    terrain: &dyn CollisionTerrain,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
) -> Result<bool, GravityError> {
    let plant =
        pinned.to_icrf_plant_state(epoch, propellant_mass_kg, h_rw_body_nms, orientation)?;
    let p_fixed = DVec3::from_array(pinned.pos_fixed_m);
    let g_eff_fixed = pinned_effective_gravity_fixed(
        pinned,
        propellant_mass_kg,
        h_rw_body_nms,
        epoch,
        force_model,
        ephemeris,
        orientation,
    )?;

    let wrench = control.evaluate_stage(0.0, epoch, &plant);
    let f_ctrl_body = wrench.main_thrust_body_n + wrench.rcs_force_body_n + wrench.env_force_body_n;
    let q_b2f = quat_normalize_portable(DQuat::from_array(pinned.q_body_to_fixed_xyzw));
    let f_ctrl_fixed = quat_rotate_vec_portable(q_b2f, f_ctrl_body);
    let m_tot = mass_model.total_mass_kg(propellant_mass_kg);
    let f_load_fixed = m_tot * g_eff_fixed + f_ctrl_fixed;

    let (_, ground) = terrain.sample_fixed(p_fixed);
    let n_up = ground.normal_fixed;
    let f_into_ground = -f_load_fixed.dot(n_up);
    if f_into_ground <= 1.0 {
        return Ok(false);
    }
    let f_tan = f_load_fixed + f_into_ground * n_up;
    Ok(sqrt(f_tan.length_squared()) <= ground.mu_static * f_into_ground)
}

/// Integrates compliant multi-foot ground contact with stateful elastic-bristle stiction
/// across `[start_epoch, end_epoch]` using fixed `1 ms` semi-implicit Euler substeps.
#[allow(clippy::too_many_arguments)]
pub fn integrate_contact_interval(
    initial: &ContinuousPlantState,
    mut contact: VesselContactState,
    feet_specs: &[ContactFootSpec],
    start_epoch: TdbEpoch,
    end_epoch: TdbEpoch,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    force_model: &ForceModel,
    terrain: &dyn CollisionTerrain,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
) -> Result<ContactStepOutput, GravityError> {
    if contact.feet.len() != feet_specs.len() {
        contact.feet = vec![BristleFootState::default(); feet_specs.len()];
    }
    if let Some(pinned) = contact.pinned {
        if verify_pinned_static_equilibrium(
            &pinned,
            initial.propellant_mass_kg,
            initial.h_rw_body_nms,
            start_epoch,
            mass_model,
            force_model,
            terrain,
            ephemeris,
            orientation,
            control,
        )? {
            let st = pinned.to_icrf_plant_state(
                end_epoch,
                initial.propellant_mass_kg,
                initial.h_rw_body_nms,
                orientation,
            )?;
            let rest_accel = pinned_rest_proper_accel(
                &pinned,
                initial,
                start_epoch,
                force_model,
                ephemeris,
                orientation,
            )?;
            // A pinned pose is the settled surface-contact pose, so the feet touch the ground.
            return Ok(ContactStepOutput {
                state: st,
                contact,
                min_foot_clearance_m: 0.0,
                peak_proper_accel_m_s2: rest_accel,
                audit: PropagationAudit::default(),
            });
        }
        contact.pinned = None;
        contact.settle_substeps = 0;
    }

    run_contact_substeps(
        initial,
        contact,
        feet_specs,
        start_epoch,
        end_epoch,
        mass_model,
        propellant_mode,
        rw_mode,
        force_model,
        terrain,
        ephemeris,
        orientation,
        control,
    )
}

#[allow(clippy::too_many_arguments)]
fn run_contact_substeps(
    initial: &ContinuousPlantState,
    mut contact: VesselContactState,
    feet_specs: &[ContactFootSpec],
    start_epoch: TdbEpoch,
    end_epoch: TdbEpoch,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    force_model: &ForceModel,
    terrain: &dyn CollisionTerrain,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
) -> Result<ContactStepOutput, GravityError> {
    let dur_s = end_epoch.signed_offset_seconds_from(start_epoch).max(0.0);
    let n_steps = ((dur_s / CONTACT_SUBSTEP_SECONDS).ceil() as u32).max(1);
    let dt = if dur_s > 0.0 {
        dur_s / (n_steps as f64)
    } else {
        0.0
    };
    let (mut state, mut audit) = (*initial, PropagationAudit::default());
    let (mut min_clear, mut peak_prop, mut step) = (f64::INFINITY, 0.0_f64, 0_u32);

    while step < n_steps && dt > 0.0 {
        let tau = (step as f64) * dt;
        let ticks = (tau * (TICKS_PER_SECOND as f64)).round() as u128;
        let ep = start_epoch.add_span(TickSpan::from_ticks_u128(ticks));
        let sub = step_one_contact_substep(
            &mut state,
            &mut contact,
            feet_specs,
            tau,
            ep,
            dt,
            mass_model,
            propellant_mode,
            rw_mode,
            force_model,
            terrain,
            ephemeris,
            orientation,
            control,
        )?;
        min_clear = sub.0;
        peak_prop = peak_prop.max(sub.1);
        audit = audit.merged(sub.2);
        if let Some(pinned) = contact.pinned {
            state = pinned.to_icrf_plant_state(
                end_epoch,
                state.propellant_mass_kg,
                state.h_rw_body_nms,
                orientation,
            )?;
            // The pinned remainder of the interval is at rest on the surface: zero clearance
            // and the body's surface proper acceleration.
            min_clear = min_clear.min(0.0);
            let rest_accel = pinned_rest_proper_accel(
                &pinned,
                &state,
                end_epoch,
                force_model,
                ephemeris,
                orientation,
            )?;
            peak_prop = peak_prop.max(rest_accel);
            break;
        }
        step = step.wrapping_add(1);
    }

    Ok(ContactStepOutput {
        state,
        contact,
        min_foot_clearance_m: min_clear,
        peak_proper_accel_m_s2: peak_prop,
        audit,
    })
}
