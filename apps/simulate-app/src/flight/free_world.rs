//! Free flight as one vessel of a [`Universe`].
//!
//! The universe owns the free-flight [`SimSession`] as the ownship's local
//! plant, on the flight's master clock, so the vessels around it share one
//! timeline with it. An aircraft is flown by flight dynamics alone, so the
//! ownship stays in the local tier; the accessors still answer for a vessel
//! the universe has taken into orbit, which has no session to ask.

use std::any::Any;
use std::sync::Arc;

use sim_core::{
    ControlInputs, SensorExposureEvent, SessionError, SimInstant, SimSession, TickSpan,
    VehicleState,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarError, BuiltinSolarSystem2026_2028, Dop853Tolerances,
    EphemerisCatalog, ForceModel, GravityError, SolarSystemKernelPack, VesselId,
    apollo_1969_spk_bytes,
};
use sim_spacetime::SimEpoch;
use sim_universe::{
    LocalPlant, OrbitalBody, OrbitalContext, Universe, UniverseCheckpoint, UniverseError,
};

use super::plant_block::PlantBlock;
use crate::error::AppSimError;

/// The vessel the pilot flies.
const OWNSHIP: VesselId = VesselId(1);

/// The orbital tier integrates on a one-minute grid and samples densely in
/// between.
const ORBITAL_QUANTUM: TickSpan = TickSpan::from_seconds_and_subsec_ticks(60, 0);

/// Why the solar system the universe propagates in could not be set up.
#[derive(Debug, thiserror::Error)]
pub enum SolarSystemError {
    /// The built-in ephemeris, orientation or constants failed verification.
    #[error("the built-in solar-system kernels failed to load")]
    Kernels(#[from] BuiltinSolarError),
    /// The gravity model could not be built from the constants.
    #[error("the solar-system gravity model could not be built")]
    Gravity(#[from] GravityError),
}

/// The solar system every free flight's universe reads, loaded once.
pub(super) struct SolarSystem {
    ephemeris: Arc<dyn EphemerisCatalog>,
    orientation: Arc<dyn BodyOrientationCatalog>,
    force_model: ForceModel,
}

impl SolarSystem {
    pub(super) fn load() -> Result<Arc<Self>, SolarSystemError> {
        let apollo_spk = apollo_1969_spk_bytes();
        let spk_slices = [apollo_spk.as_slice()];
        let pack = SolarSystemKernelPack {
            spk_files: &spk_slices,
            earth_bpc_files: &[],
            moon_bpc_files: &[],
            allow_iau_outside_bpc: true,
        };
        let solar = BuiltinSolarSystem2026_2028::load_with_kernel_pack(&pack)?;
        let force_model = ForceModel::standard_solar_system(solar.constants)?;
        Ok(Arc::new(Self {
            ephemeris: Arc::new(solar.ephemeris),
            orientation: Arc::new(solar.orientation),
            force_model,
        }))
    }

    /// The same catalogs, as `Raketa`'s flight dynamics read them.
    pub(super) fn raketa_environment(&self) -> sim_raketa::RaketaEnvironment {
        sim_raketa::RaketaEnvironment {
            ephemeris: Arc::clone(&self.ephemeris),
            orientation: Arc::clone(&self.orientation),
            force_model: self.force_model.clone(),
        }
    }

    /// Unit direction from `observer_fixed_m`, in `observer_body`'s fixed axes, to
    /// `target`'s centre at `at`, as the dynamics see it; `None` outside the kernels'
    /// coverage.
    pub(super) fn direction_fixed(
        &self,
        target: sim_orbital::NaifId,
        observer_body: sim_orbital::NaifId,
        observer_fixed_m: glam::DVec3,
        at: sim_orbital::TdbEpoch,
    ) -> Option<glam::DVec3> {
        let from_body = self
            .ephemeris
            .geometric_state_icrf(
                target,
                observer_body,
                at,
                sim_orbital::DerivativeOrder::Position,
            )
            .ok()?
            .pos_m;
        let rotation = self.orientation.body_rotation(observer_body, at).ok()?;
        (rotation.r_icrf_to_fixed * from_body - observer_fixed_m).try_normalize()
    }

    /// The solar system as a universe's orbital tier reads it.
    pub(super) fn context(&self) -> OrbitalContext {
        OrbitalContext {
            ephemeris: Arc::clone(&self.ephemeris),
            orientation: Arc::clone(&self.orientation),
            force_model: self.force_model.clone(),
            quantum: ORBITAL_QUANTUM,
            tolerances: Dop853Tolerances::default(),
        }
    }

    pub(super) fn ephemeris(&self) -> &dyn EphemerisCatalog {
        self.ephemeris.as_ref()
    }

    pub(super) fn orientation(&self) -> &dyn BodyOrientationCatalog {
        self.orientation.as_ref()
    }
}

/// The free-flight session as the ownship's local plant.
pub(super) struct SessionPlant {
    session: SimSession<PlantBlock>,
}

impl LocalPlant for SessionPlant {
    fn body(&self) -> BodyEllipsoid {
        BodyEllipsoid::WGS84
    }

    fn period(&self) -> TickSpan {
        self.session.block().plant().period()
    }

    fn now(&self) -> SimInstant {
        self.session.physics_time()
    }

    fn state(&self) -> VehicleState {
        self.session.truth_state()
    }

    fn orbital_body(&self) -> Option<OrbitalBody> {
        None
    }

    fn holds_local_tier(&self) -> bool {
        true
    }

    fn step(&mut self) -> Result<(), UniverseError> {
        let period = self.period();
        self.session.advance(period).map(drop).map_err(local_error)
    }

    /// The session keeps the master time handed to it, which the display
    /// interpolates by, so it takes the whole advance rather than steps.
    fn advance_to(&mut self, until: SimInstant) -> Result<(), UniverseError> {
        self.session
            .advance_to(until)
            .map(drop)
            .map_err(local_error)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

fn local_error(error: SessionError) -> UniverseError {
    UniverseError::LocalPlant(Box::new(error))
}

/// A free flight's universe: the ownship's session, and the clock every
/// vessel in it shares.
pub(super) struct FreeWorld {
    universe: Universe,
    period: TickSpan,
    truth: VehicleState,
}

impl FreeWorld {
    /// A universe on `epoch`'s clock holding `session` as the ownship.
    pub(super) fn new(
        session: SimSession<PlantBlock>,
        epoch: SimEpoch,
        solar: &SolarSystem,
    ) -> Result<Self, AppSimError> {
        let period = session.block().plant().period();
        let truth = session.truth_state();
        let mut universe = Universe::new(epoch, solar.context());
        universe
            .insert_local(
                OWNSHIP,
                Box::new(SessionPlant { session }),
                Box::new(|_, _, _, _| Err(UniverseError::LocalOnly(OWNSHIP))),
            )
            .map_err(universe_error)?;
        Ok(Self {
            universe,
            period,
            truth,
        })
    }

    /// What each tick of this flight means in TDB, UTC and UT1.
    pub(super) const fn epoch(&self) -> &SimEpoch {
        self.universe.epoch()
    }

    /// The ownship's session while flight dynamics fly it.
    pub(super) fn session(&self) -> Option<&SimSession<PlantBlock>> {
        self.universe
            .local::<SessionPlant>(OWNSHIP)
            .map(|plant| &plant.session)
    }

    fn session_mut(&mut self) -> Option<&mut SimSession<PlantBlock>> {
        self.universe
            .local_mut::<SessionPlant>(OWNSHIP)
            .map(|plant| &mut plant.session)
    }

    /// The ownship's native step.
    pub(super) const fn period(&self) -> TickSpan {
        self.period
    }

    /// Master-clock time handed to the universe so far.
    pub(super) const fn now(&self) -> SimInstant {
        self.universe.now()
    }

    /// The instant the ownship's truth state describes.
    pub(super) fn physics_time(&self) -> SimInstant {
        self.session()
            .map_or(self.universe.now(), SimSession::physics_time)
    }

    /// The ownship's true state at [`Self::physics_time`].
    pub(super) const fn truth_state(&self) -> VehicleState {
        self.truth
    }

    /// The ownship as the window draws it, between its last two steps.
    pub(super) fn display_state(&self) -> VehicleState {
        self.session().map_or(self.truth, SimSession::display_state)
    }

    /// How far the master clock has run past the ownship's last step, in
    /// steps.
    pub(super) fn display_phase(&self) -> f64 {
        let Some(session) = self.session() else {
            return 0.0;
        };
        let period = self.period.ticks_u128();
        if period == 0 {
            return 0.0;
        }
        let ahead = session.now().duration_since(session.physics_time());
        ahead.ticks_u128() as f64 / period as f64
    }

    /// Advances every vessel by `elapsed` and returns how many steps the
    /// ownship took.
    pub(super) fn advance(&mut self, elapsed: TickSpan) -> Result<u64, AppSimError> {
        let before = self.physics_time();
        let until = self.universe.now().advanced(elapsed);
        if let Err(error) = self.universe.advance_to(until) {
            let fault = self.session().and_then(|s| s.block().take_fault());
            return Err(fault.unwrap_or_else(|| advance_error(error)));
        }
        self.refresh_truth()?;
        let period = self.period.ticks_u128();
        let stepped = self.physics_time().duration_since(before).ticks_u128();
        Ok(stepped
            .checked_div(period)
            .and_then(|steps| u64::try_from(steps).ok())
            .unwrap_or(0))
    }

    /// Queues `controls` for the ownship at `at`; returns the instant used.
    pub(super) fn submit_at(&mut self, at: SimInstant, controls: ControlInputs) -> SimInstant {
        self.session_mut()
            .map_or(at, |session| session.submit_at(at, controls))
    }

    /// Hands over the sensor exposures the ownship emitted since the last
    /// call.
    pub(super) fn take_exposures(&mut self) -> Vec<SensorExposureEvent> {
        self.session_mut()
            .map(SimSession::take_exposures)
            .unwrap_or_default()
    }

    /// Everything but the ownship's own plant, as it is now.
    pub(super) fn checkpoint(&self) -> UniverseCheckpoint {
        self.universe.checkpoint()
    }

    /// Puts the ownship back through `rewind_session`, then the rest of the
    /// universe back to `checkpoint`.
    pub(super) fn restore(
        &mut self,
        checkpoint: &UniverseCheckpoint,
        rewind_session: impl FnOnce(&mut SimSession<PlantBlock>) -> Result<(), AppSimError>,
    ) -> Result<(), AppSimError> {
        if let Some(session) = self.session_mut() {
            rewind_session(session)?;
        }
        self.universe.restore(checkpoint).map_err(universe_error)?;
        self.refresh_truth()
    }

    fn refresh_truth(&mut self) -> Result<(), AppSimError> {
        self.truth = self
            .universe
            .vehicle_state(OWNSHIP)
            .map_err(universe_error)?;
        Ok(())
    }
}

fn universe_error(error: UniverseError) -> AppSimError {
    AppSimError::Universe(Box::new(error))
}

/// A failed advance as the session's own error where that is what failed.
fn advance_error(error: UniverseError) -> AppSimError {
    match error {
        UniverseError::LocalPlant(inner) => match inner.downcast::<SessionError>() {
            Ok(session) => AppSimError::Session(session),
            Err(other) => universe_error(UniverseError::LocalPlant(other)),
        },
        other => universe_error(other),
    }
}
