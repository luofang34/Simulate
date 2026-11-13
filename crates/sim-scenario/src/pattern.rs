//! Closed-loop C172 airport traffic-pattern scenarios over OpenStreetMap aerodrome
//! diagrams (`47N` Central Jersey uncontrolled CTAF and `KEWR` Newark Liberty Class B
//! towered), coupled to a 5-channel `30 Hz` [`VirtualCameraRigBlock`].

use std::collections::BTreeSet;

use glam::DVec3;
use sim_apollo::{CockpitInstrumentPanel, CockpitInstrumentSnapshot, EarthVehicleSources};
use sim_core::{
    AirframeArticulation, Block, CameraFrameCapture, CameraMount, CameraPoseEcef, ControlInputs,
    EngineStartMode, EnuAnchor, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    KinematicInput, KinematicModel, RateAdapter, SimInstant, TickSpan, VehicleState,
    VirtualCameraRigBlock, VirtualCameraSpec,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_pack::builtin_c172_package;
use sim_scenery::{AerodromeId, AuthoritativeWorldSurface, OsmAerodromeDiagram, WorldSurfaceQuery};

use crate::corridor::ScenarioPitot;
use crate::{FlightDynamics, ScenarioError};

mod cameras;
mod guidance;
pub(crate) mod snapshot;

#[cfg(test)]
mod tests;

pub(crate) use cameras::pattern_cameras_on_datum;
pub use cameras::pattern_virtual_cameras;
use guidance::{
    PatternAutopilot, RunwayReading, advance_leg, command_for_leg, initial_upwind_geodetic,
};

const PERIOD: TickSpan = JSBSIM_DEFAULT_PERIOD;
const C172_START_SPEED_M_S: f64 = 39.5;
const C172_START_ALPHA_RAD: f64 = 0.055;
const SEA_LEVEL_MIXTURE_NORM: f64 = 0.88;
/// The C172's left-seat design eye in FRD body axes, metres from the drawn airframe's
/// origin: the one value the cockpit, SVS and pattern-scenario cameras share, and that the
/// FDM's `EYEPOINT` repeats in structural inches.
///
/// Height: the cabin is 48 in floor to headliner (POH Fig. 6-4), hung from the drawn roof
/// 3 cm above the headliner (floor at `-0.477 m`); a 10 in seat cushion compressed 3 cm
/// (assumed: no 172S seat height is published) and the ANSUR II male median sitting eye
/// height, 804 mm, on a 12° seat back put the eye at `+0.53 m`, the head top 3.8 in under
/// the headliner. Seen from there the cowl hides the ground above about 9° below the
/// horizon. Station and lateral offset sit over the left seat (POH Fig. 6-3, arm 37).
pub const C172_PILOT_EYE_BODY_M: DVec3 = DVec3::new(0.10, -0.26, -0.53);
const MAX_SINGLE_CIRCUIT_DURATION_S: f64 = 420.0;

/// Standard rectangular airport pattern turn direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrafficDirection {
    /// Standard left-hand traffic (`-90°` turns).
    Left,
    /// Right-hand traffic (`+90°` turns).
    Right,
}

impl TrafficDirection {
    pub(crate) const fn right_sign(self) -> f64 {
        match self {
            Self::Left => -1.0,
            Self::Right => 1.0,
        }
    }
}

/// Active leg of the 5-leg airport traffic pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PatternLeg {
    /// Initial / touch-and-go climbout along active runway centerline.
    Upwind,
    /// `90°` climb turn leg perpendicular to the departure end.
    Crosswind,
    /// Parallel opposite-direction leg at `1,000 ft` AGL (`1,350 m` abeam).
    Downwind,
    /// `90°` descending base turn with `20°` flaps.
    Base,
    /// Stabilized `3.0°` final approach along extended runway centerline with `30°` flaps.
    Final,
}

impl PatternLeg {
    /// All five legs in circuit order.
    pub const ALL: [Self; 5] = [
        Self::Upwind,
        Self::Crosswind,
        Self::Downwind,
        Self::Base,
        Self::Final,
    ];

    /// Short display title for HUD and window telemetry.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::Upwind => "Upwind",
            Self::Crosswind => "Crosswind",
            Self::Downwind => "Downwind",
            Self::Base => "Base",
            Self::Final => "Final",
        }
    }
}

/// Summary metrics recorded over an [`AirportPatternRun`].
#[derive(Debug, Clone, PartialEq)]
pub struct PatternReport {
    /// Distinct pattern legs flown so far.
    pub legs_visited: BTreeSet<PatternLeg>,
    /// Number of full circuits completed (`Final -> Upwind` transitions).
    pub completed_circuits: u32,
    /// Number of `30 Hz` multi-camera exposure batches emitted.
    pub virtual_camera_batches: u64,
    /// Minimum AGL altitude (`m`) observed within `±40 m` of the landing threshold on `Final`.
    pub min_threshold_agl_m: f64,
    /// Cross-track offset (`m`) from runway centerline when crossing the landing threshold on `Final`.
    pub threshold_cross_track_m: f64,
    /// Peak roll bank magnitude (`rad`) across the run.
    pub peak_bank_rad: f64,
    /// Maximum flap command (`0.0..=1.0`) commanded during the circuit.
    pub max_flap_norm: f64,
}

enum PatternPlant {
    Kinematic(KinematicModel),
    C172Jsbsim(Box<JsbsimFdmBlock>),
}

impl PatternPlant {
    fn new(
        dynamics: FlightDynamics,
        initial: Geodetic,
        heading_rad: f64,
        field_elevation_m: f64,
    ) -> Result<Self, ScenarioError> {
        Ok(match dynamics {
            FlightDynamics::Kinematic => Self::Kinematic(KinematicModel::new(
                VehicleState::level_flight(&initial, heading_rad, C172_START_SPEED_M_S),
            )),
            FlightDynamics::Jsbsim => {
                let pkg = builtin_c172_package()?;
                let files = pkg.build_jsbsim_resolver()?;
                let (aircraft_path, model) = pkg.jsbsim_model()?;
                let mut fdm = JsbsimFdmBlock::new_on_terrain(
                    &JsbsimFdmConfig {
                        resolver: &files,
                        aircraft_path,
                        model,
                        period: PERIOD,
                        initial,
                        heading_rad,
                        speed_m_s: C172_START_SPEED_M_S,
                        alpha_rad: C172_START_ALPHA_RAD,
                        engine_start: EngineStartMode::Running,
                    },
                    field_elevation_m,
                )?;
                let mut sys = fdm.system_controls();
                sys.mixture_norm = SEA_LEVEL_MIXTURE_NORM;
                sys.flaps_norm = 0.0;
                fdm.set_system_controls(sys);
                Self::C172Jsbsim(Box::new(fdm))
            }
        })
    }

    fn set_flaps(&mut self, flaps_norm: f64) {
        if let Self::C172Jsbsim(fdm) = self {
            let mut sys = fdm.system_controls();
            sys.flaps_norm = flaps_norm.clamp(0.0, 1.0);
            fdm.set_system_controls(sys);
        }
    }

    fn sync_ground_at(
        &mut self,
        position_ecef: DVec3,
        fallback_m: f64,
    ) -> Result<(), ScenarioError> {
        if let Self::C172Jsbsim(fdm) = self {
            let geo = ecef_to_geodetic(position_ecef);
            let ground_m = AuthoritativeWorldSurface::regional_earth_shared()?
                .sample_surface(&geo)?
                .map_or(fallback_m, |sample| sample.ellipsoidal_height_m.0);
            fdm.sync_ground_elevation_m(ground_m)?;
        }
        Ok(())
    }

    fn step(&mut self, input: &KinematicInput) -> Result<(), ScenarioError> {
        match self {
            Self::Kinematic(m) => m.step(input)?,
            Self::C172Jsbsim(m) => m.step(input)?,
        }
        Ok(())
    }

    fn output(&self) -> VehicleState {
        match self {
            Self::Kinematic(m) => m.output(),
            Self::C172Jsbsim(m) => m.output(),
        }
    }

    fn articulation(&self, elapsed_s: f64) -> AirframeArticulation {
        match self {
            Self::Kinematic(m) => AirframeArticulation::from_kinematic(
                &m.output(),
                ControlInputs {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                    throttle: 0.65,
                },
                elapsed_s,
            ),
            Self::C172Jsbsim(m) => m.articulation(),
        }
    }
}

/// Deterministic C172 closed-loop airport traffic-pattern simulation run.
pub struct AirportPatternRun {
    diagram: OsmAerodromeDiagram,
    direction: TrafficDirection,
    runway_bearing_rad: f64,
    crosswind_along_m: f64,
    threshold_ecef: DVec3,
    threshold_frame: LocalFrame,
    plant: PatternPlant,
    autopilot: PatternAutopilot,
    pitot: ScenarioPitot,
    last_controls: ControlInputs,
    leg: PatternLeg,
    camera_rig: VirtualCameraRigBlock,
    camera_steps: RateAdapter,
    panel: CockpitInstrumentPanel,
    now: SimInstant,
    max_circuits: Option<u32>,
    report: PatternReport,
}

impl AirportPatternRun {
    /// Starts a single-circuit C172 traffic pattern at `aerodrome` (`Left` traffic).
    pub fn new(aerodrome: AerodromeId, dynamics: FlightDynamics) -> Result<Self, ScenarioError> {
        Self::with_max_circuits(aerodrome, dynamics, Some(1))
    }

    /// Starts a C172 traffic pattern at `aerodrome` with optional circuit limit (`None` = continuous).
    pub fn with_max_circuits(
        aerodrome: AerodromeId,
        dynamics: FlightDynamics,
        max_circuits: Option<u32>,
    ) -> Result<Self, ScenarioError> {
        let diagram = OsmAerodromeDiagram::builtin(aerodrome)?;
        let rwy = diagram.primary_runway_end().ok_or_else(|| {
            sim_scenery::SceneryError::InvalidCatalog {
                detail: format!("{} has no primary runway end", diagram.id.code()),
            }
        })?;
        let runway_bearing_rad = rwy.true_bearing_rad;
        let crosswind_along_m = (rwy.length_m + 380.0).clamp(1_450.0, 2_350.0);
        let thresh_geo = Geodetic::from_degrees(
            rwy.threshold.latitude_deg,
            rwy.threshold.longitude_deg,
            diagram.field_elevation_m,
        );
        let threshold_ecef = geodetic_to_ecef(&thresh_geo);
        let threshold_frame = LocalFrame::at(&thresh_geo);
        let initial = initial_upwind_geodetic(
            rwy.threshold.latitude_deg,
            rwy.threshold.longitude_deg,
            diagram.field_elevation_m,
            runway_bearing_rad,
        );
        let plant = PatternPlant::new(
            dynamics,
            initial,
            runway_bearing_rad,
            diagram.field_elevation_m,
        )?;
        let specs = pattern_virtual_cameras(&diagram);
        let mut camera_rig = VirtualCameraRigBlock::at_default_rate(
            specs,
            EnuAnchor::new(&BodyEllipsoid::WGS84, &thresh_geo),
        )?;
        let camera_steps = RateAdapter::new(camera_rig.period());
        let vehicle = plant.output();
        camera_rig.step(&(vehicle, DVec3::ZERO))?;

        let mut panel = CockpitInstrumentPanel::new(SimInstant::START);
        panel.observe_earth(&EarthVehicleSources {
            timestamp: SimInstant::START,
            vehicle,
            height_m: ecef_to_geodetic(vehicle.position_ecef).height_m,
        });

        let mut legs_visited = BTreeSet::new();
        legs_visited.insert(PatternLeg::Upwind);

        Ok(Self {
            diagram,
            direction: TrafficDirection::Left,
            runway_bearing_rad,
            crosswind_along_m,
            threshold_ecef,
            threshold_frame,
            plant,
            autopilot: PatternAutopilot::default(),
            pitot: ScenarioPitot::new(0x5041_5454_4552_4E01)?,
            last_controls: ControlInputs::default(),
            leg: PatternLeg::Upwind,
            camera_rig,
            camera_steps,
            panel,
            now: SimInstant::START,
            max_circuits,
            report: PatternReport {
                legs_visited,
                completed_circuits: 0,
                virtual_camera_batches: 1,
                min_threshold_agl_m: f64::INFINITY,
                threshold_cross_track_m: f64::NAN,
                peak_bank_rad: 0.0,
                max_flap_norm: 0.0,
            },
        })
    }

    /// Underlying OpenStreetMap airport diagram.
    #[must_use]
    pub const fn diagram(&self) -> &OsmAerodromeDiagram {
        &self.diagram
    }

    /// Active leg in the traffic pattern.
    #[must_use]
    pub const fn leg(&self) -> PatternLeg {
        self.leg
    }

    /// Traffic pattern turn direction.
    #[must_use]
    pub const fn direction(&self) -> TrafficDirection {
        self.direction
    }

    /// Configured `30 Hz` virtual camera specifications.
    #[must_use]
    pub fn virtual_camera_specs(&self) -> &[VirtualCameraSpec] {
        self.camera_rig.specs()
    }

    /// Latest `30 Hz` virtual camera captures across all 5 channels.
    #[must_use]
    pub fn latest_virtual_captures(&self) -> Vec<CameraFrameCapture> {
        self.camera_rig.output()
    }

    /// Flight metrics accumulated so far.
    #[must_use]
    pub const fn report(&self) -> &PatternReport {
        &self.report
    }

    /// Master-clock step period (`120 Hz`).
    #[must_use]
    pub const fn period(&self) -> TickSpan {
        PERIOD
    }

    /// Current simulation instant.
    #[must_use]
    pub const fn now(&self) -> SimInstant {
        self.now
    }

    /// Current 6-DoF vehicle state in ECEF.
    #[must_use]
    pub fn vehicle(&self) -> VehicleState {
        self.plant.output()
    }

    /// Current engine, propeller, control-surface, and landing-gear articulation.
    #[must_use]
    pub fn articulation(&self) -> AirframeArticulation {
        let elapsed_s = self.now.duration_since(SimInstant::START).as_seconds();
        self.plant.articulation(elapsed_s)
    }

    /// The controls the autopilot last commanded; neutral before the first
    /// step.
    #[must_use]
    pub fn last_controls(&self) -> ControlInputs {
        self.last_controls
    }

    /// Cockpit instrument snapshot at [`Self::now`].
    #[must_use]
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        self.panel.snapshot()
    }

    /// Pilot eye pose in the C172 cabin at [`C172_PILOT_EYE_BODY_M`], angled `-3.0°` over the
    /// cowling.
    #[must_use]
    pub fn eye_pose(&self) -> CameraPoseEcef {
        CameraMount::oblique(C172_PILOT_EYE_BODY_M, 0.0, -3.0_f64.to_radians(), 0.0)
            .pose(&self.plant.output(), DVec3::ZERO)
    }

    /// Whether the requested circuit count (or safety duration limit) has finished.
    #[must_use]
    pub fn finished(&self) -> bool {
        if let Some(limit) = self.max_circuits
            && self.report.completed_circuits >= limit
        {
            return true;
        }
        self.max_circuits.is_some()
            && self.now.duration_since(SimInstant::START).as_seconds()
                >= MAX_SINGLE_CIRCUIT_DURATION_S
    }

    /// Advances the C172, pattern controller, and `30 Hz` virtual camera rig by one `120 Hz` tick.
    pub fn step(&mut self) -> Result<(), ScenarioError> {
        if self.finished() {
            return Ok(());
        }
        let dt_s = PERIOD.as_seconds();
        let state = self.plant.output();
        let sensed_airspeed_m_s = self.pitot.sense(&state, dt_s)?;
        let reading = RunwayReading::observe(
            &state,
            self.threshold_ecef,
            &self.threshold_frame,
            self.diagram.field_elevation_m,
            self.runway_bearing_rad,
            self.direction,
            sensed_airspeed_m_s,
        );
        if self.leg == PatternLeg::Final
            && reading.along_m.abs() <= 40.0
            && reading.agl_m < self.report.min_threshold_agl_m
        {
            self.report.min_threshold_agl_m = reading.agl_m;
            self.report.threshold_cross_track_m = reading.right_of_centerline_m;
        }
        if advance_leg(&mut self.leg, &reading, self.crosswind_along_m) {
            self.report.completed_circuits = self.report.completed_circuits.wrapping_add(1);
        }
        self.report.legs_visited.insert(self.leg);
        let cmd = command_for_leg(
            self.leg,
            &reading,
            self.runway_bearing_rad,
            self.direction,
            self.crosswind_along_m,
        );
        self.report.max_flap_norm = self.report.max_flap_norm.max(cmd.flaps_norm);
        self.report.peak_bank_rad = self.report.peak_bank_rad.max(reading.bank_rad.abs());
        self.plant.set_flaps(cmd.flaps_norm);
        self.plant
            .sync_ground_at(state.position_ecef, self.diagram.field_elevation_m)?;
        let controls = self.autopilot.step(&cmd, &reading, dt_s);
        self.last_controls = controls;
        self.plant.step(&KinematicInput { controls })?;
        self.now = self.now.advanced(PERIOD);

        let updated = self.plant.output();
        for _ in 0..self.camera_steps.steps_for(PERIOD) {
            self.camera_rig.step(&(updated, DVec3::ZERO))?;
            self.report.virtual_camera_batches = self.report.virtual_camera_batches.wrapping_add(1);
        }
        self.panel.observe_earth(&EarthVehicleSources {
            timestamp: self.now,
            vehicle: updated,
            height_m: ecef_to_geodetic(updated.position_ecef).height_m,
        });
        Ok(())
    }
}
