//! Flying the descent law down to the ground, handing over to the compliant contact model
//! just above it, cutting the engine at first contact and settling into
//! `PinnedSurfaceRest`, as the staged touchdown demonstration settles.

use glam::DVec3;
use sim_geo::LocalFrame;
use sim_orbital::{
    BodyOrientationCatalog, CoastControlLaw, PinnedSurfaceState, TdbEpoch, VesselContactState,
};

use super::{
    descent::{SurfaceDescentLaw, foot_clearance_m},
    landing::TargetSiteSpec,
    voyage::Voyage,
};
use crate::mission::{RaketaMissionError, closed_loop_tols};

/// Below this foot clearance the contact model takes over from `DOP853` (`m`).
const CONTACT_HANDOVER_M: f64 = 0.3;
/// Contact-model chunk length while waiting for first contact (`s`).
const CONTACT_CHUNK_S: f64 = 0.05;
/// Upper bound on descent windows before giving up.
const MAX_DESCENT_WINDOWS: u32 = 40_000;

/// What happened between rest above the site and rest on the ground.
pub(super) struct Landing {
    pub(super) first_contact_epoch: TdbEpoch,
    /// Vertical speed relative to the surface just before first contact (`m/s`, downward positive).
    pub(super) contact_vertical_speed_m_s: f64,
    /// Horizontal speed relative to the surface just before first contact (`m/s`).
    pub(super) contact_horizontal_speed_m_s: f64,
    /// Whether all fins were locked neutral in the last moments before contact.
    pub(super) fins_locked_at_contact: bool,
    pub(super) descent_delta_v_m_s: f64,
    /// Surface speed at the start of the contact chunk in which the pin locked (`m/s`).
    pub(super) pin_chunk_start_speed_m_s: f64,
    /// Body-fixed CG displacement across that chunk, pin snap included (`m`).
    pub(super) pin_chunk_displacement_m: f64,
    pub(super) pinned: PinnedSurfaceState,
}

impl Voyage<'_> {
    /// Descends onto `touchdown_fixed_m` and settles there.
    pub(super) fn descend_and_land(
        &mut self,
        site: &TargetSiteSpec,
        touchdown_fixed_m: DVec3,
    ) -> Result<Landing, RaketaMissionError> {
        let law = SurfaceDescentLaw {
            orientation: &self.solar.orientation,
            body: site.body,
            terrain: &site.terrain,
            mu_m3_s2: self.force_model.constants().require_mu(site.body)?,
            target_fixed_m: touchdown_fixed_m,
        };
        let mut descent_dv = self.descend_to_contact_handover(site, &law)?;

        let mut contact = VesselContactState::default();
        let mut before_contact = self.surface_velocity_split(site)?;
        let mut fins_locked = false;
        let mut touched = false;
        for _ in 0..2_000 {
            let stage = law.stage_law(self.epoch, &self.state);
            fins_locked = stage.evaluate_detailed(self.epoch, &self.state).3;
            let (out, dv) = self.fly_in_contact(CONTACT_CHUNK_S, contact, &site.terrain, &law)?;
            descent_dv += dv;
            contact = out.contact;
            if out.min_foot_clearance_m <= 0.0 || contact.pinned.is_some() {
                touched = true;
                break;
            }
            before_contact = self.surface_velocity_split(site)?;
        }
        if !touched {
            return Err(self.descent_stalled(site));
        }
        let first_contact_epoch = self.epoch;

        let mut pin_chunk = (f64::NAN, f64::NAN);
        for _ in 0..60 {
            if let Some(pinned) = contact.pinned {
                return Ok(Landing {
                    first_contact_epoch,
                    contact_vertical_speed_m_s: before_contact.0,
                    contact_horizontal_speed_m_s: before_contact.1,
                    fins_locked_at_contact: fins_locked,
                    descent_delta_v_m_s: descent_dv,
                    pin_chunk_start_speed_m_s: pin_chunk.0,
                    pin_chunk_displacement_m: pin_chunk.1,
                    pinned,
                });
            }
            let (down, across) = self.surface_velocity_split(site)?;
            let p_before = self.fixed_position(site)?;
            let (out, _) = self.fly_in_contact(0.5, contact, &site.terrain, &CoastControlLaw)?;
            contact = out.contact;
            pin_chunk = (
                down.hypot(across),
                (self.fixed_position(site)? - p_before).length(),
            );
        }
        Err(RaketaMissionError::TouchdownDidNotSettle { body: site.body })
    }

    /// Flies the descent law under `DOP853` until the feet are just above the ground,
    /// returning the thrust $\Delta v$ (`m/s`).
    fn descend_to_contact_handover(
        &mut self,
        site: &TargetSiteSpec,
        law: &SurfaceDescentLaw<'_>,
    ) -> Result<f64, RaketaMissionError> {
        let tols = closed_loop_tols();
        let mut dv = 0.0_f64;
        for _ in 0..MAX_DESCENT_WINDOWS {
            let clearance = self.foot_clearance(site)?;
            if clearance < CONTACT_HANDOVER_M {
                return Ok(dv);
            }
            let sink = self.surface_velocity_split(site)?.0.max(0.1);
            let window = (0.5 * (clearance - 0.5 * CONTACT_HANDOVER_M) / sink).clamp(0.02, 5.0);
            dv += self.fly_with_tols(window, &tols, law)?;
        }
        Err(self.descent_stalled(site))
    }

    fn descent_stalled(&self, site: &TargetSiteSpec) -> RaketaMissionError {
        RaketaMissionError::DescentDidNotReachSurface {
            body: site.body,
            epoch: self.epoch,
            foot_clearance_m: self.foot_clearance(site).unwrap_or(f64::NAN),
        }
    }

    /// Lowest foot-pad clearance above the contact model's terrain (`m`).
    pub(super) fn foot_clearance(&self, site: &TargetSiteSpec) -> Result<f64, RaketaMissionError> {
        let rot = self
            .solar
            .orientation
            .body_rotation(site.body, self.epoch)?;
        Ok(foot_clearance_m(&site.terrain, &rot, &self.state))
    }

    /// CG position in body-fixed coordinates (`m`).
    fn fixed_position(&self, site: &TargetSiteSpec) -> Result<DVec3, RaketaMissionError> {
        let rot = self
            .solar
            .orientation
            .body_rotation(site.body, self.epoch)?;
        Ok(rot.r_icrf_to_fixed * self.state.pos_rel_icrf_m)
    }

    /// `(downward, horizontal)` speed relative to the turning surface (`m/s`).
    pub(super) fn surface_velocity_split(
        &self,
        site: &TargetSiteSpec,
    ) -> Result<(f64, f64), RaketaMissionError> {
        let rot = self
            .solar
            .orientation
            .body_rotation(site.body, self.epoch)?;
        let (p_f, v_f) =
            rot.icrf_to_fixed_pv(self.state.pos_rel_icrf_m, self.state.vel_rel_icrf_m_s);
        let up = LocalFrame::at(&site.ellipsoid.fixed_to_geodetic(p_f)).up();
        let v_up = v_f.dot(up);
        Ok((-v_up, (v_f - up * v_up).length()))
    }
}
