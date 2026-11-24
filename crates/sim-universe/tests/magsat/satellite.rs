//! A small rigid satellite that detumbles on the Earth's magnetic field: a
//! three-axis magnetometer, three magnetorquers, and a B-dot controller
//! between them.
//!
//! Every sample period the magnetometer reads the catalog field where the
//! satellite is, with its own bias, gain error and noise; the controller
//! differences two readings and commands the dipole `m = -k dB/dt`, each
//! axis limited to what a torquer can hold. In the orbital tier the plant
//! reports that dipole moment at each exchange and the integrator works out
//! `m × B` at every stage. In the local tier the plant itself applies the
//! torque of the held dipole over each of its steps.

use std::any::Any;
use std::sync::Arc;

use glam::{DMat3, DVec3};
use mbd_core::{ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use sim_atmosphere::VacuumAtmosphere;
use sim_bodies::{BodyDescriptor, BodyFields};
use sim_core::{
    Block, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, RigidBodySnapshot, SimInstant,
    ThrusterWrench, TickSpan, VehicleState,
};
use sim_devices::{Magnetometer, MagnetometerSpec};
use sim_geo::BodyEllipsoid;
use sim_orbital::{EnvironmentLoadSpec, EnvironmentLoads, GravityGradient};
use sim_spacetime::SimEpoch;
use sim_universe::{
    CarriedLoads, CarryContext, LocalPlant, OrbitalBody, PlantSnapshot, UniverseError,
};

pub const MASS_KG: f64 = 12.0;
/// Principal inertias of a 12U bus, kg·m².
pub const INERTIA_KG_M2: DVec3 = DVec3::new(0.15, 0.16, 0.08);
/// B-dot gain, A·m²·s/T.
pub const GAIN_A_M2_S_PER_T: f64 = 1.5e5;
/// What one torquer can hold, A·m². The detumble from 5°/s peaks near
/// 0.3 A·m², so the controller stays linear.
pub const MAX_DIPOLE_A_M2: f64 = 1.2;
const TESLA_PER_MICROTESLA: f64 = 1.0e-6;
const TESLA_PER_NANOTESLA: f64 = 1.0e-9;
const MICROTESLA_PER_NANOTESLA: f64 = 1.0e-3;
const BOUNDING_RADIUS_M: f64 = 0.3;

/// The exchange gave the magnetometer nothing to read.
#[derive(Debug, thiserror::Error)]
#[error("the body's magnetic field has no value at the vessel")]
struct NoFieldAtVessel;

/// How one satellite is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagsatSpec {
    /// Magnetometer and controller sample period; also the exchange period.
    pub sample_period: TickSpan,
    pub magnetometer: MagnetometerSpec,
    pub magnetometer_seed: u64,
    /// `false` leaves the torquers off.
    pub torquers_on: bool,
    /// Whether the orbital tier applies the gravity-gradient torque.
    pub gravity_gradient: bool,
}

/// The B-dot law and what it holds between samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BdotController {
    enabled: bool,
    /// The last reading, T, and when it was taken.
    previous: Option<(SimInstant, DVec3)>,
    /// The dipole commanded at the last sample, body axes, A·m².
    dipole_a_m2: DVec3,
}

impl BdotController {
    const fn new(enabled: bool) -> Self {
        Self {
            enabled,
            previous: None,
            dipole_a_m2: DVec3::ZERO,
        }
    }

    /// Takes `reading_t` at `at`. The first reading only primes the
    /// difference; a second one at the same instant changes nothing.
    fn sample(&mut self, at: SimInstant, reading_t: DVec3) {
        let Some((before, last_t)) = self.previous else {
            self.previous = Some((at, reading_t));
            return;
        };
        if at <= before {
            return;
        }
        let dt_s = at.duration_since(before).as_seconds();
        let rate_t_s = (reading_t - last_t) / dt_s;
        let limit = DVec3::splat(MAX_DIPOLE_A_M2);
        self.dipole_a_m2 = match self.enabled {
            true => (-GAIN_A_M2_S_PER_T * rate_t_s).clamp(-limit, limit),
            false => DVec3::ZERO,
        };
        self.previous = Some((at, reading_t));
    }
}

#[derive(Debug, Clone)]
struct MagsatSnapshot {
    block: RigidBodySnapshot,
    now: SimInstant,
    magnetometer: Magnetometer,
    controller: BdotController,
}

/// The satellite as a universe flies it in either tier.
pub struct Magsat {
    spec: MagsatSpec,
    block: RigidBody6DofBlock,
    body: BodyEllipsoid,
    fields: BodyFields,
    epoch: SimEpoch,
    now: SimInstant,
    magnetometer: Magnetometer,
    controller: BdotController,
}

fn tree() -> Result<ComponentTree, UniverseError> {
    ComponentTree::new([(
        ComponentSpec {
            name: "bus".to_owned(),
            placement: Placement::ROOT,
            dry_mass_kg: MASS_KG,
            dry_inertia_body: DMat3::from_diagonal(INERTIA_KG_M2),
            thermal_capacity_j_k: 1.0e4,
            nominal_power_w: 0.0,
            joint: JointSpec::FIXED,
        },
        ComponentState::at_temperature(290.0),
    )])
    .map_err(|e| UniverseError::LocalPlant(Box::new(e)))
}

fn block_at(
    earth: &BodyDescriptor,
    spec: &MagsatSpec,
    state: &VehicleState,
    at: SimInstant,
    epoch: &SimEpoch,
) -> Result<RigidBody6DofBlock, UniverseError> {
    Ok(RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: earth.ellipsoid,
            period: spec.sample_period,
            epoch: epoch.utc_at(at)?,
            initial: *state,
        },
        tree()?,
        Vec::new(),
        Box::new(VacuumAtmosphere),
        earth.fields().gravity,
    )?)
}

impl Magsat {
    /// The satellite at `state`, fixed to `earth`, at `start`: its dynamics
    /// and its magnetometer on that one catalog entry's fields.
    pub fn new(
        earth: &BodyDescriptor,
        spec: MagsatSpec,
        state: &VehicleState,
        start: SimInstant,
        epoch: &SimEpoch,
    ) -> Result<Self, UniverseError> {
        let magnetometer = Magnetometer::new(spec.magnetometer, spec.magnetometer_seed)
            .map_err(|e| UniverseError::LocalPlant(Box::new(e)))?;
        Ok(Self {
            spec,
            block: block_at(earth, &spec, state, start, epoch)?,
            body: earth.ellipsoid,
            fields: earth.fields(),
            epoch: epoch.clone(),
            now: start,
            magnetometer,
            controller: BdotController::new(spec.torquers_on),
        })
    }

    /// The fields the dynamics and the magnetometer were built on.
    pub const fn fields(&self) -> &BodyFields {
        &self.fields
    }

    /// The dipole the torquers hold, body axes, A·m².
    pub const fn dipole_a_m2(&self) -> DVec3 {
        self.controller.dipole_a_m2
    }

    /// Everything the plant holds besides its base motion, printed bit for
    /// bit, to compare two runs.
    pub fn internals_fingerprint(&self) -> String {
        let bits = |v: DVec3| v.to_array().map(f64::to_bits);
        format!(
            "{:?} {:?} {:?} {:?} {:?}",
            self.now,
            self.magnetometer,
            self.controller.enabled,
            self.controller.previous.map(|(at, b)| (at, bits(b))),
            bits(self.controller.dipole_a_m2),
        )
    }

    /// Reads the magnetometer in the field `field_fixed_nt` with the
    /// attitude of `state`, and lets the controller have the reading.
    fn sense_and_command(&mut self, at: SimInstant, state: &VehicleState, field_fixed_nt: DVec3) {
        if self.controller.previous.is_some_and(|(last, _)| at <= last) {
            return;
        }
        let truth_body_ut = state.attitude.inverse() * field_fixed_nt * MICROTESLA_PER_NANOTESLA;
        let reading_ut = self.magnetometer.sense(truth_body_ut);
        self.controller
            .sample(at, reading_ut * TESLA_PER_MICROTESLA);
    }

    fn orbital_properties(&self) -> OrbitalBody {
        let mass = self.block.mass_properties();
        OrbitalBody {
            mass_kg: mass.total_mass_kg,
            inertia_cg_body: mass.inertia_cg_body,
            bounding_radius_m: BOUNDING_RADIUS_M,
        }
    }
}

impl LocalPlant for Magsat {
    fn body(&self) -> BodyEllipsoid {
        self.body
    }

    fn period(&self) -> TickSpan {
        self.spec.sample_period
    }

    fn now(&self) -> SimInstant {
        self.now
    }

    fn state(&self) -> VehicleState {
        self.block.output()
    }

    fn orbital_body(&self) -> Option<OrbitalBody> {
        Some(self.orbital_properties())
    }

    fn holds_local_tier(&self) -> bool {
        false
    }

    fn environment_loads(&self) -> EnvironmentLoads {
        self.spec
            .gravity_gradient
            .then_some(EnvironmentLoadSpec::GravityGradient(GravityGradient))
            .into_iter()
            .collect()
    }

    /// One sample period in the local tier: a reading, a command, and the
    /// torque of the held dipole in the field at the start of the step.
    fn step(&mut self) -> Result<(), UniverseError> {
        let state = self.block.output();
        let year = self.epoch.tdb_at(self.now).approximate_decimal_year();
        let field_fixed_nt = self
            .fields
            .magnetic
            .field_fixed_nt(state.position_ecef, year)
            .map_err(|e| UniverseError::LocalPlant(Box::new(e)))?;
        self.sense_and_command(self.now, &state, field_fixed_nt);
        let field_body_t = state.attitude.inverse() * field_fixed_nt * TESLA_PER_NANOTESLA;
        let input = RigidBodyInput {
            thrusters: vec![ThrusterWrench {
                application_point_body_m: DVec3::ZERO,
                force_body_n: DVec3::ZERO,
                moment_body_nm: self.controller.dipole_a_m2.cross(field_body_t),
            }],
            ..RigidBodyInput::default()
        };
        self.block.step(&input)?;
        self.now = self.now.advanced(self.spec.sample_period);
        Ok(())
    }

    fn carry_period(&self) -> Option<TickSpan> {
        Some(self.spec.sample_period)
    }

    /// At an exchange the magnetometer reads the field the orbital tier's
    /// torque acts in, and the dipole the controller answers with is what
    /// that tier holds until the next one.
    fn carry(&mut self, context: &CarryContext) -> Result<CarriedLoads, UniverseError> {
        if context.body != self.body {
            return Err(UniverseError::UnmodelledBody);
        }
        let field_fixed_nt = context
            .magnetic_field_fixed_nt
            .ok_or_else(|| UniverseError::LocalPlant(Box::new(NoFieldAtVessel)))?;
        self.sense_and_command(context.at, &context.state, field_fixed_nt);
        self.now = context.at;
        Ok(CarriedLoads {
            force_body_n: DVec3::ZERO,
            torque_body_nm: DVec3::ZERO,
            dipole_body_a_m2: self.controller.dipole_a_m2,
            angular_impulse_body_nms: DVec3::ZERO,
            internal_momentum_body_nms: DVec3::ZERO,
            body: self.orbital_properties(),
        })
    }

    /// The rigid-body block cannot be handed a state, so the plant builds a
    /// new one there and keeps its magnetometer and controller.
    fn adopt_base(
        &mut self,
        described: &Arc<BodyDescriptor>,
        state: &VehicleState,
        at: SimInstant,
    ) -> Result<bool, UniverseError> {
        let body = described.ellipsoid;
        if body != self.body || at != self.now {
            return Ok(false);
        }
        let config = RigidBodyConfig {
            body,
            period: self.spec.sample_period,
            epoch: self.epoch.utc_at(at)?,
            initial: *state,
        };
        self.block = RigidBody6DofBlock::new(
            config,
            tree()?,
            Vec::new(),
            Box::new(VacuumAtmosphere),
            self.fields.gravity.clone(),
        )?;
        Ok(true)
    }

    fn snapshot_internals(&self) -> Option<PlantSnapshot> {
        Some(PlantSnapshot::new(MagsatSnapshot {
            block: self.block.snapshot(),
            now: self.now,
            magnetometer: self.magnetometer,
            controller: self.controller,
        }))
    }

    fn restore_internals(&mut self, snapshot: &PlantSnapshot) -> bool {
        let Some(saved) = snapshot.downcast_ref::<MagsatSnapshot>() else {
            return false;
        };
        if self.block.try_restore(&saved.block).is_err() {
            return false;
        }
        self.now = saved.now;
        self.magnetometer = saved.magnetometer;
        self.controller = saved.controller;
        true
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
