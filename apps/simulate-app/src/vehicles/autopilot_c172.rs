//! The hand-flown Cessna 172 with its autopilot installed: the JSBSim aeroplane under
//! the Aviate flight controller and Pilotage's host core, the same chain that flies a
//! mission. The servos stay released, so the pilot's controls fly the aeroplane; the
//! flight controller runs on the aeroplane's simulated devices and the host waits for a
//! plan.

use std::fmt;
use std::sync::Arc;

use glam::DVec3;
use sim_atmosphere::GeoidStandardAtmosphere;
use sim_core::{
    Block, ControlInputs, EngineStartMode, JsbsimFdmBlock, SystemControls, TICKS_PER_SECOND,
    TickSpan, VehicleState,
};
use sim_flight_chain::{
    C172_PLANNING_BANK_DEG, C172_RATE_HZ, C172Airframe, C172Chain, C172ChainSnapshot, ChainRoute,
    HostChain, OwnshipFeed, SharedAir, TerminalHoldingPattern, VehicleClass, c172_board_on,
    c172_pilotage_airframe, c172_sensor_suite,
};
use sim_geo::{Geodetic, UtcTime, ecef_to_geodetic};
use sim_pack::LoadedSyspack;
use tracing::error;

use super::C172_GEAR_HEIGHT_M;
use super::jsbsim::{ParkedStance, build_jsbsim_fdm, build_parked_fdm, ground_under};
use crate::error::AppSimError;
use crate::world_surface::WorldSurfaceHandle;

/// Date of the geomagnetic field the magnetometer senses and the flight controller
/// refers its heading to. Both read the one field, so the date moves neither against the
/// other; the flight's own clock is not known where the aeroplane is built.
const MAGNETIC_EPOCH_UNIX_S: f64 = 1_781_525_040.0;
/// [`MAGNETIC_EPOCH_UNIX_S`] as a decimal year.
const MAGNETIC_DECIMAL_YEAR: f64 = 2026.45;
/// Seed of the device errors: every flight draws the same devices, so a flight repeats.
const DEVICE_SEED: u64 = 172;
/// Speed the host core would ask of guidance for a plan handed to it, m/s.
const CRUISE_M_S: f64 = 52.0;
/// Climb rate the host core would ask toward a fix above the aeroplane, m/s.
const CLIMB_M_S: f64 = 2.0;

/// The flight controller's cycle, which the aeroplane's flight model steps at.
fn cycle() -> TickSpan {
    TickSpan::from_ticks(TICKS_PER_SECOND / C172_RATE_HZ)
}

/// A hand-flown C172 on its own control chain.
pub struct AutopilotC172 {
    chain: C172Chain,
}

/// An [`AutopilotC172`] at one instant. Shared rather than copied: a rewind history holds
/// many, and the chain it captures carries the host core and the flight controller whole.
#[derive(Clone)]
pub struct AutopilotC172Snapshot(Option<Arc<C172ChainSnapshot>>);

impl fmt::Debug for AutopilotC172Snapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("AutopilotC172Snapshot")
            .field(&self.0.as_ref().map(|_| "chain"))
            .finish()
    }
}

impl AutopilotC172 {
    /// The aeroplane of `pkg` in flight at `initial` over a field at `field_elev_m`.
    pub(super) fn in_flight(
        pkg: &LoadedSyspack,
        initial: Geodetic,
        heading_rad: f64,
        speed_m_s: f64,
        field_elev_m: f64,
    ) -> Result<Self, AppSimError> {
        let (fdm, _) =
            build_jsbsim_fdm(pkg, initial, heading_rad, speed_m_s, field_elev_m, cycle())?;
        Self::around(fdm)
    }

    /// The aeroplane of `pkg` standing on its gear at `ground`, whose height is the
    /// terrain's, with the engine as `engine_start` says.
    pub(super) fn parked(
        pkg: &LoadedSyspack,
        ground: Geodetic,
        heading_rad: f64,
        engine_start: EngineStartMode,
    ) -> Result<Self, AppSimError> {
        let stance = ParkedStance {
            ground,
            heading_rad,
            engine_start,
            gear_height_m: C172_GEAR_HEIGHT_M,
            pitch_rad: 0.0,
        };
        Self::around(build_parked_fdm(pkg, &stance, cycle())?)
    }

    /// The chain around `fdm`, with the servos released and no plan.
    fn around(fdm: JsbsimFdmBlock) -> Result<Self, AppSimError> {
        let origin = ecef_to_geodetic(fdm.output().position_ecef);
        let utc = UtcTime::from_unix_seconds(MAGNETIC_EPOCH_UNIX_S);
        // Calm standard air: the hub states no wind, and the devices read the same air.
        let air = SharedAir::new(GeoidStandardAtmosphere::new());
        let mut airframe = C172Airframe::around(fdm, ControlInputs::default(), air, utc)?;
        airframe.set_servos_engaged(false);
        let suite = c172_sensor_suite(MAGNETIC_DECIMAL_YEAR, DEVICE_SEED);
        let board = c172_board_on(airframe, &suite, origin, MAGNETIC_DECIMAL_YEAR)?;
        let waiting = ChainRoute {
            fixes: Vec::new(),
            cruise_mps: CRUISE_M_S,
            route_bank_deg: C172_PLANNING_BANK_DEG,
            climb_rate_mps: CLIMB_M_S,
            max_horizontal_mps: CRUISE_M_S,
            ownship: OwnshipFeed::FlightController,
            class: VehicleClass::FixedWing,
            airframe: c172_pilotage_airframe()?,
            plan_loaded_at_s: 0.0,
            utc,
            terminal_hold: TerminalHoldingPattern::default(),
        };
        Ok(Self {
            chain: HostChain::new(board, &waiting)?,
        })
    }

    /// One cycle of the flight controller with `controls` in the pilot's hands and the
    /// gear on `surface`'s ground.
    pub(super) fn step(
        &mut self,
        controls: ControlInputs,
        surface: &WorldSurfaceHandle,
    ) -> Result<(), AppSimError> {
        let ground = ground_under(surface, self.chain.true_state().position_ecef);
        let airframe = self.chain.board_mut().airframe_mut();
        if let Some((ground_m, version)) = ground {
            airframe.sync_ground_versioned_m(ground_m, version)?;
        }
        airframe.set_pilot_controls(controls);
        self.chain.advance_tick()?;
        // Nothing reads the host's events here, and events left in the chain would be
        // copied into every snapshot.
        drop(self.chain.drain_events());
        Ok(())
    }

    /// The JSBSim flight model, for its articulation, engine and systems.
    pub(super) fn fdm(&self) -> &JsbsimFdmBlock {
        self.chain.board().airframe().fdm()
    }

    /// The flight controller's cycle.
    pub(super) fn period(&self) -> TickSpan {
        self.fdm().period()
    }

    /// The true vehicle state.
    pub(super) fn state(&self) -> VehicleState {
        self.chain.true_state()
    }

    /// Body FRD specific force of the last cycle, m/s², as the aeroplane's inertial unit
    /// meets it.
    pub(super) fn specific_force_frd_m_s2(&self) -> DVec3 {
        self.chain.board().airframe().specific_force_body_m_s2()
    }

    /// Latches the operator's system controls from the next cycle on.
    pub(super) fn set_system_controls(&mut self, controls: SystemControls) {
        self.chain
            .board_mut()
            .airframe_mut()
            .set_system_controls(controls);
    }

    /// The whole chain at this instant. A chain that refuses to be copied leaves a
    /// snapshot that [`Self::restore`] refuses in turn.
    pub(super) fn snapshot(&self) -> AutopilotC172Snapshot {
        match self.chain.snapshot() {
            Ok(snapshot) => AutopilotC172Snapshot(Some(Arc::new(snapshot))),
            Err(err) => {
                error!(error = %err, "the C172's flight chain refused a snapshot");
                AutopilotC172Snapshot(None)
            }
        }
    }

    /// Returns to `snapshot`, which this aeroplane took.
    pub(super) fn restore(&mut self, snapshot: &AutopilotC172Snapshot) -> Result<(), AppSimError> {
        let Some(saved) = &snapshot.0 else {
            return Err(sim_scenario::ScenarioError::Unsupported {
                detail: "the C172's flight chain refused the snapshot when it was taken",
            }
            .into());
        };
        Ok(self.chain.restore(saved)?)
    }

    /// The chain, to inspect its host and flight controller.
    #[cfg(test)]
    pub const fn chain(&self) -> &C172Chain {
        &self.chain
    }
}
