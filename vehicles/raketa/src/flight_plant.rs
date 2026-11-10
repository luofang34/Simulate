//! `Raketa` as a vessel of a [`sim_universe::Universe`]: its pad rest, powered
//! and aerodynamic flight as a [`LocalPlant`], so a universe can fly it off a
//! surface, hand it to orbital propagation once it coasts above the
//! atmosphere, and take it back when it comes down.
//!
//! Touchdown is not modelled here. A free vessel that comes down to its
//! ground height stops the plant with [`RaketaPlantError::GroundImpact`].

use std::any::Any;
use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_orbital::{
    BodyOrientationCatalog, ContinuousPlantState, Dop853Error, Dop853Tolerances, EphemerisCatalog,
    ForceModel, GravityError, NaifId, OrientationError, PinnedSurfaceState, PropellantMode,
    STANDARD_GRAVITY_G0_M_S2, SrpSurface, propagate_plant_interval, quat_normalize_portable,
};
use sim_spacetime::SimEpoch;
use sim_universe::{
    InertialExtras, LocalFactory, LocalPlant, OrbitalBody, UniverseError, naif_body, to_body_fixed,
    to_inertial,
};

use crate::aero_actuators::{
    RAKETA_CG_Y_M, RAKETA_INERTIA_DIAGONAL_KG_M2, RAKETA_MASS_KG, RAKETA_REF_AREA_M2,
    RaketaStageLaw, raketa_default_rw_mode, raketa_mass_model,
};

/// `Raketa`'s flight-dynamics step, `1/50 s`.
pub const RAKETA_PLANT_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);

/// Radius of the sphere about the centre of gravity that encloses the hull, m.
pub const RAKETA_BOUNDING_RADIUS_M: f64 = 7.0;

/// Dynamic pressure, Pa, above which air holds the vessel in flight
/// dynamics. Below it the air's acceleration on `Raketa` is under
/// `1e-4 m/s²`, which orbital propagation leaves out.
pub const RAKETA_SENSIBLE_AIR_Q_PA: f64 = 0.1;

/// What the crew or a guidance law asks of `Raketa` until told otherwise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaketaCommand {
    /// Main-engine specific thrust, m/s², before the proper-acceleration
    /// limiter.
    pub main_accel_m_s2: f64,
    /// Direction to point the nose (body `+Y`) in ICRF axes; zero holds the
    /// current direction.
    pub nose_icrf: DVec3,
    /// Translational RCS command in body axes, each in `[-1, 1]`.
    pub rcs_unit_body: DVec3,
    /// Fin deflections, degrees.
    pub fin_deg: [f64; 4],
    /// Whether the fins lock neutral near the ground.
    pub landing_mode: bool,
}

impl Default for RaketaCommand {
    fn default() -> Self {
        Self {
            main_accel_m_s2: 0.0,
            nose_icrf: DVec3::ZERO,
            rcs_unit_body: DVec3::ZERO,
            fin_deg: [0.0; 4],
            landing_mode: false,
        }
    }
}

/// The solar system `Raketa` flies in. Share the catalogs with the universe
/// that owns the plant, or the vessel jumps when it changes tier.
#[derive(Clone)]
pub struct RaketaEnvironment {
    /// Body positions.
    pub ephemeris: Arc<dyn EphemerisCatalog>,
    /// Body orientations.
    pub orientation: Arc<dyn BodyOrientationCatalog>,
    /// Gravity.
    pub force_model: ForceModel,
}

/// Why a `Raketa` plant could not be built or stepped.
#[derive(Debug, thiserror::Error)]
pub enum RaketaPlantError {
    /// The body has no ephemeris or orientation in the orbital stack.
    #[error("Raketa cannot fly about a body the orbital stack does not model")]
    UnmodelledBody,
    /// The body's orientation was unavailable.
    #[error("body orientation unavailable for Raketa")]
    Orientation(#[from] OrientationError),
    /// Gravity or the pad anchor could not be evaluated.
    #[error("gravity evaluation failed for Raketa")]
    Gravity(#[from] GravityError),
    /// The integrator failed.
    #[error("Raketa propagation failed")]
    Propagation(#[from] Dop853Error<GravityError>),
    /// The frame change to or from the body's fixed axes failed.
    #[error("Raketa frame change failed")]
    Frame(#[source] Box<UniverseError>),
    /// The free vessel came down to its ground height.
    #[error("Raketa reached the surface at {height_m} m; touchdown is not modelled by this plant")]
    GroundImpact {
        /// Height of the centre of gravity above the ellipsoid, m.
        height_m: f64,
    },
}

/// `Raketa` in flight dynamics: pinned to a pad, or flying under thrust,
/// reaction wheels, RCS and air loads about one body.
pub struct RaketaPlant {
    environment: RaketaEnvironment,
    epoch: SimEpoch,
    body: BodyEllipsoid,
    primary: NaifId,
    now: SimInstant,
    plant: ContinuousPlantState,
    pinned: Option<PinnedSurfaceState>,
    command: RaketaCommand,
    fixed: VehicleState,
    q_inf_pa: f64,
    ground_height_m: f64,
}

impl RaketaPlant {
    /// At rest with its fins on the ground at `site` (ellipsoidal ground
    /// height in `site.height_m`), nose up, at `start` on `epoch`'s clock.
    pub fn on_pad(
        environment: RaketaEnvironment,
        epoch: SimEpoch,
        start: SimInstant,
        body: BodyEllipsoid,
        site: &Geodetic,
    ) -> Result<Self, RaketaPlantError> {
        let primary = naif_body(body).ok_or(RaketaPlantError::UnmodelledBody)?;
        let cg = Geodetic {
            height_m: site.height_m + RAKETA_CG_Y_M,
            ..*site
        };
        let up_fixed = LocalFrame::at(&cg).up();
        let q_body_to_fixed = quat_normalize_portable(DQuat::from_rotation_arc(DVec3::Y, up_fixed));
        let pinned = PinnedSurfaceState {
            body: primary,
            pos_fixed_m: body.geodetic_to_fixed(&cg).to_array(),
            q_body_to_fixed_xyzw: q_body_to_fixed.to_array(),
        };
        let plant = pinned.to_icrf_plant_state(
            epoch.tdb_at(start),
            0.0,
            DVec3::ZERO,
            environment.orientation.as_ref(),
        )?;
        let mut built = Self::assemble(
            environment,
            epoch,
            start,
            body,
            primary,
            plant,
            Some(pinned),
        )?;
        built.ground_height_m = site.height_m;
        Ok(built)
    }

    /// Flying free from `state`, fixed to `body`, at `start` on `epoch`'s
    /// clock: how a universe hands the vessel back from orbit.
    pub fn from_body_fixed(
        environment: RaketaEnvironment,
        epoch: SimEpoch,
        start: SimInstant,
        body: BodyEllipsoid,
        state: &VehicleState,
    ) -> Result<Self, RaketaPlantError> {
        let primary = naif_body(body).ok_or(RaketaPlantError::UnmodelledBody)?;
        let rotation = environment
            .orientation
            .body_rotation(primary, epoch.tdb_at(start))?;
        let plant = to_inertial(state, primary, &rotation, InertialExtras::default());
        Self::assemble(environment, epoch, start, body, primary, plant, None)
    }

    /// The factory a universe rebuilds the vessel's local tier with.
    pub fn factory(environment: RaketaEnvironment) -> LocalFactory {
        Box::new(move |body, state, start, epoch| {
            let plant =
                Self::from_body_fixed(environment.clone(), epoch.clone(), start, body, state)
                    .map_err(local_error)?;
            Ok(Box::new(plant))
        })
    }

    fn assemble(
        environment: RaketaEnvironment,
        epoch: SimEpoch,
        now: SimInstant,
        body: BodyEllipsoid,
        primary: NaifId,
        plant: ContinuousPlantState,
        pinned: Option<PinnedSurfaceState>,
    ) -> Result<Self, RaketaPlantError> {
        let rotation = environment
            .orientation
            .body_rotation(primary, epoch.tdb_at(now))?;
        let (fixed, _extras) = to_body_fixed(&plant, primary, &rotation)
            .map_err(|e| RaketaPlantError::Frame(Box::new(e)))?;
        let mut built = Self {
            environment,
            epoch,
            body,
            primary,
            now,
            plant,
            pinned,
            command: RaketaCommand::default(),
            fixed,
            q_inf_pa: 0.0,
            ground_height_m: 0.0,
        };
        built.refresh()?;
        Ok(built)
    }

    /// Sets the ellipsoidal height of the ground the vessel must stay above,
    /// m: the pad's for a vessel built on one, else the ellipsoid.
    pub const fn set_ground_height_m(&mut self, ground_height_m: f64) {
        self.ground_height_m = ground_height_m;
    }

    /// Replaces the standing command; it flies from the next step.
    pub fn set_command(&mut self, command: RaketaCommand) {
        self.command = command;
    }

    /// The standing command.
    #[must_use]
    pub const fn command(&self) -> RaketaCommand {
        self.command
    }

    /// The vessel's state in ICRF axes about its body.
    #[must_use]
    pub const fn inertial_state(&self) -> &ContinuousPlantState {
        &self.plant
    }

    /// Whether the vessel is still held on its pad.
    #[must_use]
    pub const fn is_on_pad(&self) -> bool {
        self.pinned.is_some()
    }

    /// Dynamic pressure at the last step, Pa.
    #[must_use]
    pub const fn dynamic_pressure_pa(&self) -> f64 {
        self.q_inf_pa
    }

    fn law(&self) -> RaketaStageLaw<'_> {
        RaketaStageLaw {
            commanded_main_accel_m_s2: self.command.main_accel_m_s2.max(0.0),
            desired_body_y_icrf: self.command.nose_icrf,
            rcs_cmd_unit_body: self.command.rcs_unit_body,
            fin_cmd_deg: self.command.fin_deg,
            is_landing_or_pad_mode: self.command.landing_mode,
            orientation: self.environment.orientation.as_ref(),
        }
    }

    /// Brings the body-fixed state and the air loads up to date with the
    /// inertial state.
    fn refresh(&mut self) -> Result<(), RaketaPlantError> {
        let at = self.epoch.tdb_at(self.now);
        let rotation = self
            .environment
            .orientation
            .body_rotation(self.primary, at)?;
        let (fixed, _extras) = to_body_fixed(&self.plant, self.primary, &rotation)
            .map_err(|e| RaketaPlantError::Frame(Box::new(e)))?;
        self.fixed = fixed;
        self.q_inf_pa = if self.pinned.is_some() {
            0.0
        } else {
            self.law().evaluate_detailed(at, &self.plant).2.q_inf_pa
        };
        Ok(())
    }

    fn step_inner(&mut self) -> Result<(), RaketaPlantError> {
        let next = self.now.advanced(RAKETA_PLANT_PERIOD);
        let (t0, t1) = (self.epoch.tdb_at(self.now), self.epoch.tdb_at(next));
        if self.command.main_accel_m_s2 > STANDARD_GRAVITY_G0_M_S2 {
            self.pinned = None;
        }
        self.plant = match self.pinned {
            Some(pinned) => pinned.to_icrf_plant_state(
                t1,
                self.plant.propellant_mass_kg,
                self.plant.h_rw_body_nms,
                self.environment.orientation.as_ref(),
            )?,
            None => {
                propagate_plant_interval(
                    &self.plant,
                    t0,
                    t1,
                    None,
                    &flight_tolerances(),
                    &raketa_mass_model(),
                    PropellantMode::InfiniteConstantMass,
                    raketa_default_rw_mode(),
                    &self.environment.force_model,
                    self.environment.ephemeris.as_ref(),
                    self.environment.orientation.as_ref(),
                    &self.law(),
                )?
                .final_state
            }
        };
        self.now = next;
        self.refresh()?;
        let height_m = self
            .body
            .fixed_to_geodetic(self.fixed.position_ecef)
            .height_m;
        if self.pinned.is_none() && height_m <= self.ground_height_m {
            return Err(RaketaPlantError::GroundImpact { height_m });
        }
        Ok(())
    }
}

impl LocalPlant for RaketaPlant {
    fn body(&self) -> BodyEllipsoid {
        self.body
    }

    fn period(&self) -> TickSpan {
        RAKETA_PLANT_PERIOD
    }

    fn now(&self) -> SimInstant {
        self.now
    }

    fn state(&self) -> VehicleState {
        self.fixed
    }

    fn orbital_body(&self) -> Option<OrbitalBody> {
        Some(OrbitalBody {
            mass_kg: RAKETA_MASS_KG,
            inertia_cg_body: DMat3::from_diagonal(DVec3::from_array(RAKETA_INERTIA_DIAGONAL_KG_M2)),
            bounding_radius_m: RAKETA_BOUNDING_RADIUS_M,
        })
    }

    fn srp_surface(&self) -> Option<SrpSurface> {
        Some(SrpSurface {
            area_m2: RAKETA_REF_AREA_M2,
            reflectivity_coefficient: 1.25,
        })
    }

    fn dynamic_pressure_pa(&self) -> f64 {
        self.q_inf_pa
    }

    fn is_ground_contact(&self) -> bool {
        self.pinned.is_some()
    }

    fn holds_local_tier(&self) -> bool {
        self.pinned.is_some()
            || self.command.main_accel_m_s2 > 0.0
            || self.command.rcs_unit_body != DVec3::ZERO
            || self.q_inf_pa > RAKETA_SENSIBLE_AIR_Q_PA
    }

    fn step(&mut self) -> Result<(), UniverseError> {
        self.step_inner().map_err(local_error)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

fn local_error(error: RaketaPlantError) -> UniverseError {
    UniverseError::LocalPlant(Box::new(error))
}

/// Tolerances for closed-loop powered and aerodynamic flight at
/// [`RAKETA_PLANT_PERIOD`].
fn flight_tolerances() -> Dop853Tolerances {
    Dop853Tolerances {
        rtol: 1.0e-8,
        atol_pos_m: 1.0e-4,
        atol_vel_m_s: 1.0e-6,
        atol_att_rad: 1.0e-7,
        atol_omega_rad_s: 1.0e-7,
        ..Dop853Tolerances::default()
    }
}
