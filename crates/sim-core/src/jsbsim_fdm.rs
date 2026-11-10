use glam::{DQuat, DVec3};
use jsbsim::{Fdm, LoadOptions, NodeId, Resolver, Snapshot, TrimMode};
use sim_geo::Geodetic;

use crate::{
    Block, BlockError, ControlInputs, KinematicInput, TICKS_PER_SECOND, TickSpan, VehicleState,
};

mod articulation;
mod ground;
mod mass;
mod system;

use articulation::ArticulationNodes;
pub use articulation::{AirframeArticulation, PropellerVisualState};
use mass::MassNodes;
use system::SystemNodes;
pub use system::{EngineStartMode, EngineStatus, MagnetoSwitch, SystemControls};

#[cfg(test)]
mod tests;

const BLOCK: &str = "jsbsim_fdm";
const METRES_PER_FOOT: f64 = 0.3048;
const DEFAULT_RATE_HZ: u64 = 120;
// A rate that does not divide the master clock would drift, so it is a build
// error rather than a runtime fallback.
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(DEFAULT_RATE_HZ));

/// Default integration period, 120 Hz, JSBSim's own default rate.
pub const JSBSIM_DEFAULT_PERIOD: TickSpan =
    TickSpan::from_ticks(TICKS_PER_SECOND / DEFAULT_RATE_HZ);

/// What a [`JsbsimFdmBlock`] loads and how it starts.
pub struct JsbsimFdmConfig<'a> {
    /// Supplies the aircraft's XML documents.
    pub resolver: &'a dyn Resolver,
    /// Directory of aircraft inside the resolver.
    pub aircraft_path: &'a str,
    /// The aircraft's model name.
    pub model: &'a str,
    /// Integration period; [`JSBSIM_DEFAULT_PERIOD`] unless a test needs
    /// another.
    pub period: TickSpan,
    /// Starting position, on WGS84.
    pub initial: Geodetic,
    /// Starting heading from true north, radians.
    pub heading_rad: f64,
    /// Starting true airspeed, m/s.
    pub speed_m_s: f64,
    /// Starting angle of attack, radians, which is also the pitch attitude.
    pub alpha_rad: f64,
    /// Whether the engine starts running or shut down.
    pub engine_start: EngineStartMode,
}

/// The JSBSim properties the block touches each step, resolved once because
/// a path lookup per frame is what the core promises never to need.
#[derive(Debug, Clone, Copy)]
struct Nodes {
    aileron: NodeId,
    elevator: NodeId,
    rudder: NodeId,
    throttle: NodeId,
    velocity_body: [NodeId; 3],
    rate_body: [NodeId; 3],
    h_agl_ft: NodeId,
    terrain_elevation_asl_ft: NodeId,
    system: SystemNodes,
    articulation: ArticulationNodes,
    mass: MassNodes,
}

impl Nodes {
    fn resolve(fdm: &Fdm) -> Result<Self, BlockError> {
        let node = |path: &'static str| {
            fdm.node(path)
                .map_err(fdm_error)?
                .ok_or(BlockError::InputOutOfRange {
                    block: BLOCK,
                    input: "aircraft",
                    detail: format!("the loaded aircraft has no property {path}"),
                })
        };
        Ok(Self {
            aileron: node("fcs/aileron-cmd-norm")?,
            elevator: node("fcs/elevator-cmd-norm")?,
            rudder: node("fcs/rudder-cmd-norm")?,
            throttle: node("fcs/throttle-cmd-norm")?,
            velocity_body: [
                node("velocities/u-fps")?,
                node("velocities/v-fps")?,
                node("velocities/w-fps")?,
            ],
            rate_body: [
                node("velocities/p-rad_sec")?,
                node("velocities/q-rad_sec")?,
                node("velocities/r-rad_sec")?,
            ],
            h_agl_ft: node("position/h-agl-ft")?,
            terrain_elevation_asl_ft: node("position/terrain-elevation-asl-ft")?,
            system: SystemNodes::resolve(fdm)?,
            articulation: ArticulationNodes::resolve(fdm)?,
            mass: MassNodes::resolve(fdm)?,
        })
    }
}

/// Complete state of a [`JsbsimFdmBlock`].
#[derive(Debug, Clone, PartialEq)]
pub struct JsbsimFdmSnapshot {
    fdm: Snapshot,
    output: VehicleState,
    engine: EngineStatus,
    articulation: AirframeArticulation,
    system: SystemControls,
    ground_version: Option<u64>,
    cg_structural_in: DVec3,
}

/// A JSBSim aircraft as a flight dynamics [`Block`], with the same input and
/// output as [`crate::KinematicModel`] so either can fly a vehicle.
///
/// [`ControlInputs`] follow pilot sense: stick back is nose up and right pedal
/// is nose right. JSBSim's elevator and rudder commands run the other way, so
/// the block negates them.
///
/// The operator's [`SystemControls`] are latched rather than step inputs,
/// because they change far less often than the flight controls; the flight
/// controls stay the same [`KinematicInput`] for either dynamics source.
///
/// `Fdm::snapshot_into` needs exclusive access but [`Block::snapshot`] does
/// not have it, so each step keeps a snapshot of its result. That costs
/// microseconds, and it also lets a failed step put the executive back.
#[derive(Debug)]
pub struct JsbsimFdmBlock {
    fdm: Fdm,
    nodes: Nodes,
    period: TickSpan,
    latest: JsbsimFdmSnapshot,
    last_ground_sync_m: Option<f64>,
}

fn fdm_error(source: jsbsim::Error) -> BlockError {
    BlockError::Fdm {
        block: BLOCK,
        source,
    }
}

/// Body-to-ECEF rotation from JSBSim's ECEF-to-body quaternion.
///
/// JSBSim stores `[w, x, y, z]` and its transformation matrix is the transpose
/// of the textbook rotation matrix of those components, so the same components
/// read as a Hamilton quaternion already rotate body components into ECEF.
fn body_to_ecef(ecef_to_body: [f64; 4]) -> DQuat {
    let [w, x, y, z] = ecef_to_body;
    DQuat::from_xyzw(x, y, z, w).normalize()
}

impl JsbsimFdmBlock {
    /// Loads the aircraft strictly, so content the port would ignore fails,
    /// and applies the initial conditions with the engine as
    /// [`JsbsimFdmConfig::engine_start`] says. Fails on a zero period, a failed
    /// load, or an aircraft without the standard flight controls.
    ///
    /// The ground starts at the ellipsoid. An aircraft that starts on or near
    /// real terrain belongs in [`Self::new_on_terrain`]: moving the ground
    /// afterwards cannot undo the gear forces the initial conditions saw.
    pub fn new(config: &JsbsimFdmConfig<'_>) -> Result<Self, BlockError> {
        Self::build(config, None)
    }

    /// [`Self::new`] with the ground at `terrain_elevation_m` above the WGS84
    /// ellipsoid before the initial conditions run, so a parked aircraft
    /// starts at rest on its gear rather than buried or in free fall.
    pub fn new_on_terrain(
        config: &JsbsimFdmConfig<'_>,
        terrain_elevation_m: f64,
    ) -> Result<Self, BlockError> {
        if !terrain_elevation_m.is_finite() {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "terrain_elevation_m",
                detail: format!("{terrain_elevation_m} is not finite"),
            });
        }
        Self::build(config, Some(terrain_elevation_m))
    }

    fn build(
        config: &JsbsimFdmConfig<'_>,
        terrain_elevation_m: Option<f64>,
    ) -> Result<Self, BlockError> {
        if config.period == TickSpan::ZERO {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "period",
                detail: "a zero period never advances".to_owned(),
            });
        }
        let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true)).map_err(fdm_error)?;
        fdm.set_dt(config.period.as_seconds());
        fdm.load_model(config.resolver, config.aircraft_path, config.model)
            .map_err(fdm_error)?;
        let ic = [
            ("ic/lat-geod-deg", config.initial.lat_rad.to_degrees()),
            ("ic/long-gc-deg", config.initial.lon_rad.to_degrees()),
            ("ic/h-sl-ft", config.initial.height_m / METRES_PER_FOOT),
            ("ic/psi-true-deg", config.heading_rad.to_degrees()),
            ("ic/vt-fps", config.speed_m_s / METRES_PER_FOOT),
            ("ic/alpha-deg", config.alpha_rad.to_degrees()),
        ];
        for (path, value) in ic {
            fdm.set_property(path, value).map_err(fdm_error)?;
        }
        if config.speed_m_s.abs() < f64::EPSILON {
            // At rest there is no flight path for the angle of attack to be measured
            // from, so the attitude is set directly.
            fdm.set_property("ic/theta-deg", config.alpha_rad.to_degrees())
                .map_err(fdm_error)?;
        }
        if let Some(height_m) = terrain_elevation_m {
            // Sea-level altitude was set last, so the aircraft keeps its
            // height rather than its height above the moved ground.
            fdm.set_property("ic/terrain-elevation-ft", height_m / METRES_PER_FOOT)
                .map_err(fdm_error)?;
        }
        let nodes = Nodes::resolve(&fdm)?;
        let system = match config.engine_start {
            EngineStartMode::ColdAndDark => SystemControls::cold_and_dark(),
            EngineStartMode::Running => SystemControls::running_defaults(),
        };
        nodes.system.write_and_sync_gear(&mut fdm, system)?;
        if config.engine_start == EngineStartMode::Running {
            nodes.system.set_running(&mut fdm)?;
        }
        fdm.run_ic().map_err(fdm_error)?;
        nodes.system.sync_gear_pos(&mut fdm, system)?;
        let mut block = Self {
            fdm,
            nodes,
            period: config.period,
            latest: JsbsimFdmSnapshot {
                fdm: Snapshot::new(),
                output: VehicleState {
                    position_ecef: DVec3::ZERO,
                    velocity_ecef: DVec3::ZERO,
                    attitude: DQuat::IDENTITY,
                    angular_rate_body: DVec3::ZERO,
                },
                engine: EngineStatus {
                    running: false,
                    propeller_rpm: 0.0,
                },
                articulation: AirframeArticulation::default(),
                system,
                ground_version: None,
                cg_structural_in: DVec3::ZERO,
            },
            last_ground_sync_m: terrain_elevation_m,
        };
        block.capture(0.0)?;
        Ok(block)
    }

    /// The operator controls in force.
    pub fn system_controls(&self) -> SystemControls {
        self.latest.system
    }

    /// Latches `controls`, limited to their ranges, from the next step on.
    pub fn set_system_controls(&mut self, controls: SystemControls) {
        self.latest.system = controls.clamped();
    }

    /// The engine as of the latest step.
    pub fn engine_status(&self) -> EngineStatus {
        self.latest.engine
    }

    /// Full powerplant, propeller, control-surface, and landing-gear articulation as of the latest step.
    #[must_use]
    pub fn articulation(&self) -> AirframeArticulation {
        self.latest.articulation
    }

    /// Trims the aircraft for steady level flight at its initial conditions
    /// and returns the flight controls that hold it, to feed every later step.
    /// The aircraft must sum `fcs/pitch-trim-cmd-norm` into its elevator, which
    /// is where the pitch solution stays. On failure the aircraft is back at
    /// its initial conditions.
    pub fn trim_level_flight(&mut self) -> Result<ControlInputs, BlockError> {
        self.nodes
            .system
            .write_and_sync_gear(&mut self.fdm, self.latest.system)?;
        let trimmed = self.fdm.do_trim(TrimMode::Full).map(|_| ());
        self.nodes
            .system
            .sync_gear_pos(&mut self.fdm, self.latest.system)?;
        self.capture(0.0)?;
        trimmed.map_err(fdm_error)?;
        Ok(ControlInputs {
            roll: self.fdm.read_node(self.nodes.aileron).map_err(fdm_error)?,
            pitch: -self.fdm.read_node(self.nodes.elevator).map_err(fdm_error)?,
            yaw: -self.fdm.read_node(self.nodes.rudder).map_err(fdm_error)?,
            throttle: self.fdm.read_node(self.nodes.throttle).map_err(fdm_error)?,
        })
    }

    /// Restores `snapshot`, or reports that it does not fit this aircraft and
    /// leaves the block unchanged.
    pub fn try_restore(&mut self, snapshot: &JsbsimFdmSnapshot) -> Result<(), BlockError> {
        self.fdm.restore(&snapshot.fdm).map_err(fdm_error)?;
        self.latest.clone_from(snapshot);
        Ok(())
    }

    fn read_vector(&mut self, nodes: [NodeId; 3]) -> Result<DVec3, BlockError> {
        let [x, y, z] = nodes;
        Ok(DVec3::new(
            self.fdm.read_node(x).map_err(fdm_error)?,
            self.fdm.read_node(y).map_err(fdm_error)?,
            self.fdm.read_node(z).map_err(fdm_error)?,
        ))
    }

    fn write_controls(&mut self, controls: ControlInputs) -> Result<(), BlockError> {
        let commands = [
            (self.nodes.aileron, controls.roll),
            (self.nodes.elevator, -controls.pitch),
            (self.nodes.rudder, -controls.yaw),
            (self.nodes.throttle, controls.throttle),
        ];
        for (node, value) in commands {
            self.fdm.write_node(node, value).map_err(fdm_error)?;
        }
        self.nodes.system.write_step(
            &mut self.fdm,
            controls.throttle,
            self.latest.system,
            self.period.as_seconds(),
        )
    }

    /// Reads the output and takes the snapshot of this instant.
    fn capture(&mut self, dt_s: f64) -> Result<(), BlockError> {
        let position_ft = self.fdm.location().ecef();
        let attitude = body_to_ecef(self.fdm.quaternion_ecef().to_array());
        let velocity_body_ft_s = self.read_vector(self.nodes.velocity_body)?;
        let rate_body = self.read_vector(self.nodes.rate_body)?;
        let output = VehicleState {
            position_ecef: DVec3::from_array(position_ft.to_array()) * METRES_PER_FOOT,
            velocity_ecef: attitude * (velocity_body_ft_s * METRES_PER_FOOT),
            attitude,
            angular_rate_body: rate_body,
        };
        if !(output.position_ecef.is_finite()
            && output.velocity_ecef.is_finite()
            && output.attitude.is_finite()
            && output.angular_rate_body.is_finite())
        {
            return Err(BlockError::NonFiniteState {
                block: BLOCK,
                detail: format!("{output:?}"),
            });
        }
        self.fdm
            .snapshot_into(&mut self.latest.fdm)
            .map_err(fdm_error)?;
        self.latest.output = output;
        self.latest.cg_structural_in = self.nodes.mass.sample(&mut self.fdm)?;
        self.latest.engine = self.nodes.system.engine_status(&mut self.fdm)?;
        self.latest.articulation = self.nodes.articulation.sample(
            &mut self.fdm,
            &self.latest.articulation,
            self.latest.system,
            dt_s,
        )?;
        Ok(())
    }

    fn advance(&mut self, controls: ControlInputs) -> Result<(), BlockError> {
        self.write_controls(controls)?;
        if !self.fdm.run().map_err(fdm_error)? {
            return Err(BlockError::FdmTerminated { block: BLOCK });
        }
        self.capture(self.period.as_seconds())
    }
}

impl Block for JsbsimFdmBlock {
    type Input = KinematicInput;
    type Output = VehicleState;
    type Snapshot = JsbsimFdmSnapshot;

    fn period(&self) -> TickSpan {
        self.period
    }

    /// Advances one period. A step that fails leaves the aircraft where it
    /// was.
    fn step(&mut self, input: &KinematicInput) -> Result<(), BlockError> {
        let result = self.advance(input.controls.clamped());
        if result.is_err() {
            self.fdm.restore(&self.latest.fdm).map_err(fdm_error)?;
        }
        result
    }

    fn output(&self) -> VehicleState {
        self.latest.output
    }

    fn snapshot(&self) -> JsbsimFdmSnapshot {
        self.latest.clone()
    }

    /// Restores `snapshot`. One taken from a different aircraft is logged and
    /// ignored; [`JsbsimFdmBlock::try_restore`] reports it instead.
    fn restore(&mut self, snapshot: &JsbsimFdmSnapshot) {
        if let Err(error) = self.try_restore(snapshot) {
            tracing::error!(%error, "JSBSim snapshot does not fit this aircraft");
        }
    }
}
