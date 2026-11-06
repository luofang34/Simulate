use pilotage_adapter_api::{
    AdapterCapabilities, ApplyOutcome, Disposition, ExecutionMode, LinkLossEnactError,
    LinkLossPolicy, RejectReason, SIM_LIFECYCLE_SCOPE, StepBudget, StepOutcome, TelemetryBatch,
    TelemetrySample, VehicleAdapter, VehicleDescriptor, VideoSource, sim_lifecycle_descriptor,
};
use pilotage_protocol::{
    ControlAction, HeadingReference, ScopeId, ScopedControlFrame, VehicleClass, VehicleId,
};
use pilotage_timing::SimTick;
use sim_core::{
    Block, BlockError, EngineStartMode, JsbsimFdmBlock, JsbsimFdmConfig, KinematicInput,
    SimInstant, TickSpan, VehicleState,
};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};
use sim_pack::LoadedSyspack;

use crate::PilotageBridgeError;

mod capability;
mod checkpoint;
mod control;
mod telemetry;

pub use capability::vehicle_class;
use capability::{motion_scope_descriptor, supported_link_loss};
pub use checkpoint::AdapterCheckpoint;
use control::ControlLaw;
use telemetry::{AvionicsModel, SourceIdentity, body_to_ned};

#[cfg(test)]
mod tests;

/// Scope that flies the vehicle.
pub const MOTION_SCOPE: &str = "vehicle.motion";

const ADAPTER_VERSION: &str = env!("CARGO_PKG_VERSION");
const VEHICLE_DISPLAY_NAME: &str = "Simulate vehicle";

/// Identity of one adapter instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimulateAdapterConfig {
    /// The vehicle this adapter exposes.
    pub vehicle: VehicleId,
    /// Seed of the avionics noise, which with the commands fixes the run.
    pub seed: u64,
}

/// Where a package vehicle starts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PackageStart {
    /// Starting position.
    pub position: Geodetic,
    /// Heading from true north, radians.
    pub heading_rad: f64,
    /// True airspeed, m/s.
    pub speed_m_s: f64,
    /// Angle of attack, which is also the pitch attitude, radians.
    pub alpha_rad: f64,
    /// Integration period.
    pub period: TickSpan,
}

/// An Simulate vehicle behind Pilotage's [`VehicleAdapter`].
///
/// One Pilotage tick is one period of the dynamics block, so a budget of ticks
/// is a deterministic stretch of flight whatever the wall clock does. Control
/// arrives as typed body rates and a thrust setting and is flown through a rate
/// loop, because the block's stick means rate on one source and surface
/// deflection on another. A block failure ends stepping: the outcome reports
/// the ticks actually advanced and [`Self::fault`] says why.
pub struct SimulateVehicleAdapter<B: Block> {
    config: SimulateAdapterConfig,
    class: VehicleClass,
    block: B,
    initial: B::Snapshot,
    tick: SimTick,
    now: SimInstant,
    law: ControlLaw,
    armed: bool,
    lifecycle_latched: bool,
    avionics: AvionicsModel,
    latest: TelemetrySample,
    fault: Option<BlockError>,
    surface_elevation_m: Option<f64>,
}

impl SimulateVehicleAdapter<JsbsimFdmBlock> {
    /// The JSBSim vehicle in a verified package, described to Pilotage as what
    /// its [`sim_pack::SystemKind`] says. Refuses a kind Pilotage has no class
    /// for.
    pub fn from_package(
        config: SimulateAdapterConfig,
        package: &LoadedSyspack,
        start: &PackageStart,
    ) -> Result<Self, PilotageBridgeError> {
        let kind = package.manifest().kind;
        let class = vehicle_class(kind).ok_or(PilotageBridgeError::UnsupportedKind { kind })?;
        let files = package.build_jsbsim_resolver()?;
        let (aircraft_path, model) = package.jsbsim_model()?;
        let block = JsbsimFdmBlock::new(&JsbsimFdmConfig {
            resolver: &files,
            aircraft_path,
            model,
            period: start.period,
            initial: start.position,
            heading_rad: start.heading_rad,
            speed_m_s: start.speed_m_s,
            alpha_rad: start.alpha_rad,
            engine_start: EngineStartMode::Running,
        })?;
        Ok(Self {
            class,
            ..Self::new(config, block)
        })
    }
}

impl<B: Block<Input = KinematicInput, Output = VehicleState>> SimulateVehicleAdapter<B> {
    /// `block` behind the adapter API as a fixed-wing vehicle, its present
    /// state as the origin of the local frame telemetry reports in and the
    /// state a reset returns to.
    pub fn new(config: SimulateAdapterConfig, block: B) -> Self {
        let state = block.output();
        let origin = LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
        let identity = SourceIdentity::new(config.vehicle, config.seed);
        let mut avionics = AvionicsModel::new(config.seed, state.position_ecef, origin, identity);
        let tick = SimTick::new(0);
        let latest = avionics.sample(config.vehicle, tick, SimInstant::START, &state, true, None);
        Self {
            config,
            class: VehicleClass::FixedWing,
            initial: block.snapshot(),
            block,
            tick,
            now: SimInstant::START,
            law: ControlLaw::default(),
            armed: true,
            lifecycle_latched: false,
            avionics,
            latest,
            fault: None,
            surface_elevation_m: None,
        }
    }

    /// The instant the vehicle state describes.
    pub const fn now(&self) -> SimInstant {
        self.now
    }

    /// The true vehicle state.
    pub fn vehicle_state(&self) -> VehicleState {
        self.block.output()
    }

    /// Why stepping stopped, once it has.
    pub const fn fault(&self) -> Option<&BlockError> {
        self.fault.as_ref()
    }

    /// Sets the WGS84 ellipsoidal height, metres, of the terrain or runway
    /// under the vehicle, which its downward ranging and landed detector
    /// measure against; `None` reports neither. The caller owns the terrain
    /// model and updates this as the vehicle moves; it reaches telemetry from
    /// the next measurement on.
    pub fn set_surface_elevation_m(&mut self, elevation_m: Option<f64>) {
        self.surface_elevation_m = elevation_m;
    }

    fn refuse(&self, reason: RejectReason) -> ApplyOutcome {
        ApplyOutcome::new(self.tick, Disposition::Rejected(reason))
    }

    /// `sim.lifecycle` carries `SimReset` alone, under its own lease.
    fn apply_lifecycle(&mut self, frame: &ScopedControlFrame) -> ApplyOutcome {
        if self.lifecycle_latched {
            return self.refuse(RejectReason::LinkLossEngaged);
        }
        if frame.intent.is_some() || frame.actions != [ControlAction::SimReset] {
            let detail = "sim.lifecycle takes exactly one SimReset action".to_owned();
            return self.refuse(RejectReason::Other(detail));
        }
        self.reset();
        ApplyOutcome {
            tick: self.tick,
            disposition: Disposition::Accepted,
            action_results: vec![pilotage_adapter_api::ActionResult::accepted(
                ControlAction::SimReset,
            )],
        }
    }

    /// Back to the initial state with a neutral law and a fresh source epoch.
    /// The tick and clock keep running, so stamps stay monotonic across it.
    fn reset(&mut self) {
        self.block.restore(&self.initial);
        self.law.reset();
        self.armed = true;
        self.fault = None;
        self.avionics.restart(self.config.seed);
        self.publish();
    }

    fn publish(&mut self) {
        let state = self.block.output();
        self.latest = self.avionics.sample(
            self.config.vehicle,
            self.tick,
            self.now,
            &state,
            self.armed,
            self.surface_elevation_m,
        );
    }

    fn step_once(&mut self, period: TickSpan) -> Result<(), BlockError> {
        let state = self.block.output();
        // The origin's NED is the frame telemetry reports attitude in, so an
        // attitude command means what the host read back.
        let attitude_ned = body_to_ned(self.avionics.origin(), state.attitude);
        let controls = self.law.controls(
            state.angular_rate_body,
            attitude_ned,
            period.as_seconds(),
            self.armed,
        );
        self.block.step(&KinematicInput { controls })?;
        self.tick = self.tick.next();
        self.now = self.now.advanced(period);
        self.publish();
        Ok(())
    }
}

impl<B: Block<Input = KinematicInput, Output = VehicleState>> VehicleAdapter
    for SimulateVehicleAdapter<B>
{
    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            execution: ExecutionMode {
                real_time: false,
                stepped: true,
                accelerated: true,
                deterministic: true,
                // Frames render through sim-scenario, not this adapter.
                render_capable: false,
                physically_embodied: false,
            },
            vehicles: vec![VehicleDescriptor {
                id: self.config.vehicle,
                class: self.class,
                // Attitude is reported against the geographic NED frame.
                heading_reference: Some(HeadingReference::True),
                display_name: VEHICLE_DISPLAY_NAME.to_owned(),
                scopes: vec![motion_scope_descriptor(), sim_lifecycle_descriptor()],
                link_loss_actions: supported_link_loss(),
            }],
            adapter_version: ADAPTER_VERSION.to_owned(),
            control_feel: None,
        }
    }

    fn apply_control(&mut self, frame: &ScopedControlFrame) -> ApplyOutcome {
        if frame.vehicle != self.config.vehicle {
            return self.refuse(RejectReason::UnknownVehicle);
        }
        match frame.scope.as_str() {
            SIM_LIFECYCLE_SCOPE => self.apply_lifecycle(frame),
            MOTION_SCOPE => {
                let outcome = self.law.apply(frame, &mut self.armed);
                ApplyOutcome {
                    tick: self.tick,
                    ..outcome
                }
            }
            _ => self.refuse(RejectReason::UnknownScope),
        }
    }

    fn sample_telemetry(&mut self) -> TelemetryBatch {
        TelemetryBatch {
            samples: vec![self.latest.clone()],
        }
    }

    fn video_sources(&self) -> Vec<VideoSource> {
        Vec::new()
    }

    fn set_link_loss_policy(
        &mut self,
        vehicle: VehicleId,
        scope: &ScopeId,
        policy: Option<LinkLossPolicy>,
    ) -> Result<(), LinkLossEnactError> {
        if vehicle != self.config.vehicle {
            return Err(LinkLossEnactError::UnknownVehicle { vehicle });
        }
        match scope.as_str() {
            MOTION_SCOPE => self.law.set_link_loss(policy),
            SIM_LIFECYCLE_SCOPE => {
                self.lifecycle_latched = policy.is_some();
                Ok(())
            }
            other => Err(LinkLossEnactError::ChannelRejected {
                detail: format!("scope {other} has no actuation"),
            }),
        }
    }

    fn step(&mut self, budget: StepBudget) -> StepOutcome {
        let period = self.block.period();
        let mut advanced = 0;
        while advanced < budget.ticks && self.fault.is_none() {
            match self.step_once(period) {
                Ok(()) => advanced = advanced.wrapping_add(1),
                Err(error) => self.fault = Some(error),
            }
        }
        StepOutcome {
            advanced,
            now: self.tick,
        }
    }
}
