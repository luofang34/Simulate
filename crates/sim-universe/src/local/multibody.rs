//! A multibody vehicle whose internals stay alive in either tier.

use std::any::Any;

use glam::DVec3;
use sim_core::{
    Block, MultibodyVehicleBlock, MultibodyVehicleInput, MultibodyVehicleSnapshot, SimInstant,
    TickSpan, VehicleState,
};
use sim_geo::BodyEllipsoid;

use super::{LocalPlant, OrbitalBody};
use crate::UniverseError;
use crate::carry::{CarriedLoads, CarryContext, PlantSnapshot};

/// How often a [`MultibodyLocalPlant`] exchanges with the orbital tier unless
/// told otherwise, rounded up to whole block periods.
pub const DEFAULT_CARRY_PERIOD: TickSpan = TickSpan::from_seconds_and_subsec_ticks(10, 0);

/// Thrust acceleration, m/s², up to which a [`MultibodyLocalPlant`] lets the
/// orbital tier hold its thrust between exchanges: electric propulsion and
/// attitude jets, not an ascent or capture burn.
pub const DEFAULT_MAX_CARRIED_THRUST_ACCEL_M_S2: f64 = 1.0e-2;

/// A unified [`MultibodyVehicleBlock`] participating in [`crate::Universe`] across
/// planetary atmospheric flight, surface contact, powered ascent, and orbital promotion.
///
/// The same plant flies in both tiers. In the orbital tier its subsystems
/// step with the base prescribed ([`LocalPlant::carry`]) and its thrust is
/// what the orbital integrator applies, so tank levels, battery charge and
/// temperatures carry through a promotion and back.
pub struct MultibodyLocalPlant {
    block: MultibodyVehicleBlock,
    input: MultibodyVehicleInput,
    body: BodyEllipsoid,
    start: SimInstant,
    bounding_radius_m: f64,
    carry_period: Option<TickSpan>,
    max_carried_thrust_accel_m_s2: f64,
}

#[derive(Debug, Clone)]
struct MultibodyPlantSnapshot {
    block: MultibodyVehicleSnapshot,
    input: MultibodyVehicleInput,
    start: SimInstant,
}

impl MultibodyLocalPlant {
    /// Wraps `block` started at `start` inside bounding sphere radius `bounding_radius_m`.
    #[must_use]
    pub fn new(
        block: MultibodyVehicleBlock,
        body: BodyEllipsoid,
        start: SimInstant,
        bounding_radius_m: f64,
    ) -> Self {
        let carry_period = whole_periods(DEFAULT_CARRY_PERIOD, block.period());
        Self {
            block,
            input: MultibodyVehicleInput::default(),
            body,
            start,
            bounding_radius_m,
            carry_period,
            max_carried_thrust_accel_m_s2: DEFAULT_MAX_CARRIED_THRUST_ACCEL_M_S2,
        }
    }

    /// Replaces active per-step vehicle input.
    #[must_use]
    pub fn with_input(mut self, input: MultibodyVehicleInput) -> Self {
        self.input = input;
        self
    }

    /// Exchanges with the orbital tier every `period`, rounded up to whole
    /// block periods.
    #[must_use]
    pub fn with_carry_period(mut self, period: TickSpan) -> Self {
        self.carry_period = whole_periods(period, self.block.period());
        self
    }

    /// Makes the plant one the universe drops on promotion and rebuilds from
    /// its factory on the way back.
    #[must_use]
    pub fn without_carrying(mut self) -> Self {
        self.carry_period = None;
        self
    }

    /// Lets the orbital tier hold thrust up to `accel_m_s2` between exchanges
    /// in place of [`DEFAULT_MAX_CARRIED_THRUST_ACCEL_M_S2`].
    #[must_use]
    pub fn with_max_carried_thrust_accel(mut self, accel_m_s2: f64) -> Self {
        self.max_carried_thrust_accel_m_s2 = accel_m_s2;
        self
    }

    /// Mutable access to active vehicle step input.
    pub fn set_input(&mut self, input: MultibodyVehicleInput) {
        self.input = input;
    }

    /// The input the next step reads. While the plant is carried its solar
    /// fields are the ones the last exchange set.
    #[must_use]
    pub const fn input(&self) -> &MultibodyVehicleInput {
        &self.input
    }

    /// Immutable reference to underlying [`MultibodyVehicleBlock`].
    #[must_use]
    pub const fn block(&self) -> &MultibodyVehicleBlock {
        &self.block
    }

    /// Mutable reference to underlying [`MultibodyVehicleBlock`].
    pub fn block_mut(&mut self) -> &mut MultibodyVehicleBlock {
        &mut self.block
    }

    fn body_properties(&self) -> OrbitalBody {
        let mass = self.block.mass_properties();
        OrbitalBody {
            mass_kg: mass.total_mass_kg,
            inertia_cg_body: mass.inertia_cg_body,
            bounding_radius_m: self.bounding_radius_m,
        }
    }

    /// Whether the orbital tier can stand in for this thrust between
    /// exchanges: none at all, or a small one from a plant that reports it.
    fn thrust_can_be_carried(&self) -> bool {
        let thrust_n = self.block.telemetry().total_thrust_body_n.length();
        if thrust_n == 0.0 {
            return true;
        }
        let accel = thrust_n / self.block.mass_properties().total_mass_kg;
        self.carry_period.is_some() && accel <= self.max_carried_thrust_accel_m_s2
    }
}

/// `span` rounded up to a whole number of `period`s, at least one.
fn whole_periods(span: TickSpan, period: TickSpan) -> Option<TickSpan> {
    let period_ticks = period.ticks_u128();
    if period_ticks == 0 {
        return None;
    }
    let steps = span.ticks_u128().div_ceil(period_ticks).max(1);
    Some(TickSpan::from_ticks_u128(
        steps.saturating_mul(period_ticks),
    ))
}

impl LocalPlant for MultibodyLocalPlant {
    fn body(&self) -> BodyEllipsoid {
        self.body
    }

    fn period(&self) -> TickSpan {
        self.block.period()
    }

    fn now(&self) -> SimInstant {
        self.start
            .advanced(self.block.now().duration_since(SimInstant::START))
    }

    fn state(&self) -> VehicleState {
        self.block.output()
    }

    fn orbital_body(&self) -> Option<OrbitalBody> {
        Some(self.body_properties())
    }

    /// The plant stays in the local tier while the base and the internals
    /// act on each other faster than an exchange can pass on: in ground
    /// contact or pinned rest, splashed down, in sensible dynamic pressure,
    /// under an external wrench, while any joint is driven or moving, and
    /// under thrust beyond the carried limit. Small steady thrust does not
    /// hold it: [`LocalPlant::carry`] reports that to the orbital tier.
    fn holds_local_tier(&self) -> bool {
        let tel = self.block.telemetry();
        tel.in_ground_contact
            || tel.is_pinned_rest
            || tel.is_splashed_down
            || tel.dynamic_pressure_pa > 0.1
            || !self.input.external_wrenches.is_empty()
            || self.block.joints_in_motion(&self.input)
            || !self.thrust_can_be_carried()
    }

    fn dynamic_pressure_pa(&self) -> f64 {
        self.block.telemetry().dynamic_pressure_pa
    }

    fn is_ground_contact(&self) -> bool {
        let tel = self.block.telemetry();
        tel.in_ground_contact || tel.is_pinned_rest
    }

    fn carry_period(&self) -> Option<TickSpan> {
        self.carry_period
    }

    fn carry(&mut self, context: &CarryContext) -> Result<CarriedLoads, UniverseError> {
        if context.body != self.body {
            return Err(UniverseError::UnmodelledBody);
        }
        self.input.sun_dir_world = context.sun_direction_fixed;
        // The bus and thermal models read visibility as lit or not, so the
        // occulted fraction goes into the irradiance, once.
        self.input.sun_irradiance_w_m2 = context.solar_irradiance_w_m2 * context.solar_lit_fraction;
        self.input.sun_visibility = context.solar_lit_fraction;
        let mut force = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        let mut steps = 0_u32;
        while self.now().advanced(self.block.period()) <= context.at {
            self.block.step_carried(&self.input, &context.state)?;
            let tel = self.block.telemetry();
            force += tel.total_thrust_body_n;
            torque += tel.thrust_torque_cg_body_nm;
            steps = steps.wrapping_add(1);
        }
        let tel = self.block.telemetry();
        let (force_body_n, torque_body_nm) = if steps == 0 {
            (tel.total_thrust_body_n, tel.thrust_torque_cg_body_nm)
        } else {
            (force / f64::from(steps), torque / f64::from(steps))
        };
        Ok(CarriedLoads {
            force_body_n,
            torque_body_nm,
            dipole_body_a_m2: DVec3::ZERO,
            body: self.body_properties(),
        })
    }

    fn adopt_base(
        &mut self,
        body: BodyEllipsoid,
        state: &VehicleState,
        at: SimInstant,
    ) -> Result<bool, UniverseError> {
        if body != self.body || at != self.now() {
            return Ok(false);
        }
        self.block.adopt_base_state(state)?;
        Ok(true)
    }

    fn snapshot_internals(&self) -> Option<PlantSnapshot> {
        Some(PlantSnapshot::new(MultibodyPlantSnapshot {
            block: self.block.snapshot(),
            input: self.input.clone(),
            start: self.start,
        }))
    }

    fn restore_internals(&mut self, snapshot: &PlantSnapshot) -> bool {
        let Some(saved) = snapshot.downcast_ref::<MultibodyPlantSnapshot>() else {
            return false;
        };
        if self.block.try_restore(&saved.block).is_err() {
            return false;
        }
        self.input = saved.input.clone();
        self.start = saved.start;
        true
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self) -> Result<(), UniverseError> {
        Ok(self.block.step(&self.input)?)
    }
}
