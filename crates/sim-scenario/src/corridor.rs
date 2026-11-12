use std::f64::consts::{FRAC_PI_2, PI};

use glam::{DVec2, DVec3};
use sim_apollo::{CockpitInstrumentPanel, CockpitInstrumentSnapshot, EarthVehicleSources};
use sim_core::ControlInputs;
use sim_core::{
    Block, CAMERA_DEFAULT_PERIOD, CameraFrameCapture, CameraMount, CameraPoseEcef,
    CameraSensorBlock, CameraSensorConfig, CameraSensorInput, EnuAnchor, FusionEligibility,
    JSBSIM_DEFAULT_PERIOD, KinematicInput, MapCell, NeesSummary, RateAdapter, SimInstant, TickSpan,
    TwoTierConfig, TwoTierVisualHarness, VehicleState, VisualFix,
};
use sim_geo::{
    BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics, ecef_to_geodetic, geodetic_to_ecef,
};
use sim_scenery::SyntheticCorridorSpec;

use crate::{ScenarioError, ScenarioProfile};

mod autopilot;
pub(crate) mod dynamics;
mod image_mode;
mod navigation;
mod report;
pub(crate) mod snapshot;
mod visual;

pub use autopilot::{Autopilot, FlightReading, Guidance, Steering};
pub(crate) use autopilot::{ScenarioPitot, angle_difference};
pub use dynamics::FlightDynamics;
use dynamics::{CorridorStart, Source};
use image_mode::VisualSource;
pub use image_mode::{ImageDrivenConfig, ImageNavigationReport};
pub use report::CorridorReport;
use report::horizon_in_view;
use visual::SyntheticVisualEstimator;
pub use visual::{CorridorVisualEstimator, RenderedSensorFrame, SensorObservation, VisualTiers};

#[cfg(test)]
mod tests;

/// Both sources step at this rate, so a run reads the same on either.
const PERIOD: TickSpan = JSBSIM_DEFAULT_PERIOD;
const ESTIMATOR_SEED: u64 = 0xC0_12_1D_0E;
/// A navigation tile's width in `Navigate`'s `NVKTILE1` banks, metres.
const NAV_TILE_M: f64 = 600.0;
/// Camera position, ahead of the centre of gravity, body axes, metres.
const CAMERA_OFFSET_BODY_M: DVec3 = DVec3::new(1.0, 0.0, 0.0);
/// `800 × 600` with about a 53° vertical field of view.
const CAMERA_INTRINSICS: SensorIntrinsics = SensorIntrinsics {
    width: 800,
    height: 600,
    fx: 600.0,
    fy: 600.0,
    cx: 399.5,
    cy: 299.5,
};

const CRUISE_HEIGHT_M: f64 = 900.0;
const CRUISE_SPEED_M_S: f64 = 55.0;
/// A point on the nominal track, east and north of the anchor, metres.
const TRACK_ORIGIN_M: DVec2 = DVec2::new(3_000.0, 1_000.0);
const TRACK_BEARING_RAD: f64 = 85.0 * PI / 180.0;
const OFF_SEED_LATERAL_M: f64 = 900.0;
const OFF_SEED_HEIGHT_M: f64 = 200.0;
const AGGRESSIVE_HEIGHT_M: f64 = 1_300.0;
const AGGRESSIVE_SPEED_M_S: f64 = 65.0;
/// The kinematic stand-in has no stall and tracks a slow orbit tightly.
const KINEMATIC_LOITER_SPEED_M_S: f64 = 32.0;
/// The C172 stalls at 57 KCAS in a 30 degree bank (POH Figure 5-3), so it orbits
/// at about 1.4 times its stall speed in the 35 degree bank this circle needs.
const C172_LOITER_SPEED_M_S: f64 = 42.0;
/// Small enough that the orbit stays inside one 500 m map cell.
const LOITER_RADIUS_M: f64 = 200.0;
/// The middle of a 500 m map cell, so the orbit stays inside it. North first,
/// as autopilot positions are.
const LOITER_CENTRE_M: DVec2 = DVec2::new(3_250.0, 2_250.0);

/// The anchor of the corridor's local frame.
fn anchor_position() -> Geodetic {
    SyntheticCorridorSpec::delaware().south_west
}

/// The WGS84 point `north_east_m` from the anchor, at `height_m`.
fn local_position(north_east_m: DVec2, height_m: f64) -> Geodetic {
    let anchor = anchor_position();
    let frame = LocalFrame::at(&anchor);
    let offset = frame.north() * north_east_m.x + frame.east() * north_east_m.y;
    Geodetic {
        height_m,
        ..ecef_to_geodetic(geodetic_to_ecef(&anchor) + offset)
    }
}

fn track_point(along_m: f64, right_m: f64) -> DVec2 {
    let (sin, cos) = TRACK_BEARING_RAD.sin_cos();
    TRACK_ORIGIN_M + DVec2::new(cos, sin) * along_m + DVec2::new(-sin, cos) * right_m
}

/// How a profile begins.
fn start_of(profile: ScenarioProfile, dynamics: FlightDynamics) -> CorridorStart {
    match profile {
        ScenarioProfile::CorridorOffSeedAcquisition => CorridorStart {
            position: local_position(
                track_point(0.0, OFF_SEED_LATERAL_M),
                CRUISE_HEIGHT_M + OFF_SEED_HEIGHT_M,
            ),
            heading_rad: TRACK_BEARING_RAD,
            speed_m_s: CRUISE_SPEED_M_S,
        },
        ScenarioProfile::CorridorAggressiveBank => CorridorStart {
            position: local_position(TRACK_ORIGIN_M, AGGRESSIVE_HEIGHT_M),
            heading_rad: TRACK_BEARING_RAD,
            speed_m_s: AGGRESSIVE_SPEED_M_S,
        },
        ScenarioProfile::CorridorSameCellLoiter => CorridorStart {
            position: local_position(
                LOITER_CENTRE_M + DVec2::new(LOITER_RADIUS_M, 0.0),
                CRUISE_HEIGHT_M,
            ),
            heading_rad: FRAC_PI_2,
            speed_m_s: loiter_speed_m_s(dynamics),
        },
        _ => CorridorStart {
            position: local_position(TRACK_ORIGIN_M, CRUISE_HEIGHT_M),
            heading_rad: TRACK_BEARING_RAD,
            speed_m_s: CRUISE_SPEED_M_S,
        },
    }
}

/// How long a profile's script lasts, seconds.
fn duration_s(profile: ScenarioProfile) -> f64 {
    match profile {
        ScenarioProfile::CorridorOffSeedAcquisition => 150.0,
        ScenarioProfile::CorridorAggressiveBank => 60.0,
        _ => 120.0,
    }
}

/// The nominal cruise's gentle turns: track offsets from the nominal bearing
/// by time.
fn cruise_track_rad(elapsed_s: f64) -> f64 {
    let offset_deg = match elapsed_s {
        t if t < 30.0 => 0.0,
        t if t < 60.0 => -15.0,
        t if t < 90.0 => 15.0,
        _ => 0.0,
    };
    TRACK_BEARING_RAD + offset_deg * PI / 180.0
}

/// The aggressive profile's attitudes by time: banks either way, a zoom that
/// puts the nose camera's view all in the sky, and a push that puts it all on
/// the ground.
fn aggressive_steering(elapsed_s: f64) -> Steering {
    let attitude = |bank_deg: f64, pitch_deg: f64| Steering::Attitude {
        bank_rad: bank_deg * PI / 180.0,
        pitch_rad: pitch_deg * PI / 180.0,
    };
    match elapsed_s {
        t if t < 6.0 => attitude(0.0, 3.0),
        t if t < 16.0 => attitude(75.0, 2.0),
        t if t < 26.0 => attitude(-75.0, 2.0),
        t if t < 34.0 => attitude(0.0, 42.0),
        t if t < 42.0 => attitude(0.0, -40.0),
        _ => Steering::Track(TRACK_BEARING_RAD),
    }
}

/// Orbit speed of the same-cell loiter on `dynamics`.
const fn loiter_speed_m_s(dynamics: FlightDynamics) -> f64 {
    match dynamics {
        FlightDynamics::Kinematic => KINEMATIC_LOITER_SPEED_M_S,
        FlightDynamics::Jsbsim => C172_LOITER_SPEED_M_S,
    }
}

fn guidance_of(profile: ScenarioProfile, dynamics: FlightDynamics, elapsed_s: f64) -> Guidance {
    let line = Steering::Line {
        through_m: TRACK_ORIGIN_M,
        bearing_rad: TRACK_BEARING_RAD,
    };
    match profile {
        ScenarioProfile::CorridorOffSeedAcquisition => Guidance {
            steering: line,
            height_m: CRUISE_HEIGHT_M,
            speed_m_s: CRUISE_SPEED_M_S,
        },
        ScenarioProfile::CorridorAggressiveBank => Guidance {
            steering: aggressive_steering(elapsed_s),
            height_m: AGGRESSIVE_HEIGHT_M,
            speed_m_s: AGGRESSIVE_SPEED_M_S,
        },
        ScenarioProfile::CorridorSameCellLoiter => Guidance {
            steering: Steering::Orbit {
                centre_m: LOITER_CENTRE_M,
                radius_m: LOITER_RADIUS_M,
            },
            height_m: CRUISE_HEIGHT_M,
            speed_m_s: loiter_speed_m_s(dynamics),
        },
        _ => Guidance {
            steering: Steering::Track(cruise_track_rad(elapsed_s)),
            height_m: CRUISE_HEIGHT_M,
            speed_m_s: CRUISE_SPEED_M_S,
        },
    }
}

/// One corridor flight, stepped on the master clock: an autopilot flies the
/// chosen dynamics source along the profile's script while a nose camera feeds
/// the two-tier visual harness.
pub struct CorridorRun {
    profile: ScenarioProfile,
    dynamics: FlightDynamics,
    source: Source,
    autopilot: Autopilot,
    pitot: ScenarioPitot,
    last_controls: ControlInputs,
    camera: CameraSensorBlock,
    camera_rate: RateAdapter,
    harness: TwoTierVisualHarness,
    visual: VisualSource,
    panel: CockpitInstrumentPanel,
    origin_ecef: DVec3,
    anchor_frame: LocalFrame,
    now: SimInstant,
    report: CorridorReport,
}

impl CorridorRun {
    /// `profile` on `dynamics`, at its first instant. `profile` must be a
    /// corridor profile; any other flies the nominal cruise.
    pub fn new(profile: ScenarioProfile, dynamics: FlightDynamics) -> Result<Self, ScenarioError> {
        Self::assemble(
            profile,
            dynamics,
            Source::new(dynamics, &start_of(profile, dynamics))?,
            CameraMount::forward_nose(CAMERA_OFFSET_BODY_M),
            CAMERA_INTRINSICS,
            TwoTierConfig::default(),
            VisualSource::Synthetic(SyntheticVisualEstimator::new(ESTIMATOR_SEED)),
        )
    }

    fn assemble(
        profile: ScenarioProfile,
        dynamics: FlightDynamics,
        source: Source,
        mount: CameraMount,
        intrinsics: SensorIntrinsics,
        harness: TwoTierConfig,
        visual: VisualSource,
    ) -> Result<Self, ScenarioError> {
        let anchor = anchor_position();
        let camera = CameraSensorBlock::new(CameraSensorConfig {
            mount,
            intrinsics,
            anchor: EnuAnchor::new(&BodyEllipsoid::WGS84, &anchor),
            period: CAMERA_DEFAULT_PERIOD,
        })?;
        let mut run = Self {
            profile,
            dynamics,
            source,
            autopilot: Autopilot::default(),
            pitot: ScenarioPitot::new(ESTIMATOR_SEED.wrapping_add(0x5049_544F))?,
            last_controls: ControlInputs::default(),
            camera,
            camera_rate: RateAdapter::new(CAMERA_DEFAULT_PERIOD),
            harness: TwoTierVisualHarness::new(harness)?,
            visual,
            panel: CockpitInstrumentPanel::new(SimInstant::START),
            origin_ecef: geodetic_to_ecef(&anchor),
            anchor_frame: LocalFrame::at(&anchor),
            now: SimInstant::START,
            report: CorridorReport::default(),
        };
        run.observe_panel();
        Ok(run)
    }

    /// The step period.
    pub fn period(&self) -> TickSpan {
        PERIOD
    }

    /// The instant the vehicle state describes.
    pub fn now(&self) -> SimInstant {
        self.now
    }

    /// The true vehicle state.
    pub fn vehicle(&self) -> VehicleState {
        self.source.output()
    }

    /// The controls the autopilot last commanded; neutral before the first
    /// step.
    pub fn last_controls(&self) -> ControlInputs {
        self.last_controls
    }

    /// The cockpit at [`Self::now`].
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        self.panel.snapshot()
    }

    /// The view from the nose camera at [`Self::now`].
    pub fn eye_pose(&self) -> CameraPoseEcef {
        CameraMount::forward_nose(CAMERA_OFFSET_BODY_M).pose(&self.source.output(), DVec3::ZERO)
    }

    /// The most recent exposure.
    pub fn latest_capture(&self) -> Option<CameraFrameCapture> {
        self.camera.output()
    }

    /// What the flight has shown so far.
    pub fn report(&self) -> &CorridorReport {
        &self.report
    }

    /// Position NEES over the delivered fixes of `eligibility`.
    pub fn nees(&self, eligibility: FusionEligibility) -> NeesSummary {
        self.harness.nees_summary(eligibility)
    }

    /// Whether the script has run out.
    pub fn finished(&self) -> bool {
        self.elapsed_s() >= duration_s(self.profile)
    }

    /// Advances one period. Past the end of the script nothing moves.
    pub fn step(&mut self) -> Result<(), ScenarioError> {
        self.step_navigated(None).map(drop)
    }

    /// Advances one period with the autopilot steering on `navigated` instead
    /// of the true state, and hands over the visual fixes delivered by the new
    /// instant. Camera, truth, and instruments stay on the true state, so a
    /// navigation error becomes a flight error. Past the end of the script
    /// nothing moves.
    pub fn step_navigated(
        &mut self,
        navigated: Option<VehicleState>,
    ) -> Result<Vec<VisualFix>, ScenarioError> {
        if self.finished() {
            return Ok(Vec::new());
        }
        let vehicle = self.source.output();
        if self.now == SimInstant::START || self.camera_rate.steps_for_u128(PERIOD) > 0 {
            self.expose(vehicle)?;
        }
        let dt_s = PERIOD.as_seconds();
        let sensed_airspeed_m_s = self.pitot.sense(&vehicle, dt_s)?;
        let reading = FlightReading::of(&vehicle, self.origin_ecef, &self.anchor_frame)
            .with_airspeed(sensed_airspeed_m_s);
        let navigated = navigated.or_else(|| match &self.visual {
            VisualSource::Image(mode) => {
                Some(mode.navigated(&vehicle, self.origin_ecef, &self.anchor_frame))
            }
            VisualSource::Synthetic(_) => None,
        });
        let steered_on = navigated.map_or(reading, |state| {
            FlightReading::of(&state, self.origin_ecef, &self.anchor_frame)
                .with_airspeed(sensed_airspeed_m_s)
        });
        let guidance = guidance_of(self.profile, self.dynamics, self.elapsed_s());
        let controls = self.autopilot.update(&guidance, &steered_on, dt_s);
        self.last_controls = controls;
        self.source.sync_ground_at(vehicle.position_ecef)?;
        self.source.step(&KinematicInput { controls })?;
        self.now = self.now.advanced(PERIOD);
        self.record_flight(&reading, &vehicle);
        let fixes = self.collect_fixes();
        if let VisualSource::Image(mode) = &mut self.visual {
            let truth = self.source.output();
            let truth_enu = self
                .anchor_frame
                .ecef_to_enu_vector(truth.position_ecef - self.origin_ecef);
            let dt = PERIOD.as_seconds();
            mode.navigate(&truth, truth_enu, &self.anchor_frame, dt, self.now, &fixes);
        }
        self.observe_panel();
        Ok(fixes)
    }

    fn elapsed_s(&self) -> f64 {
        self.now.duration_since(SimInstant::START).as_seconds()
    }

    fn expose(&mut self, vehicle: VehicleState) -> Result<(), ScenarioError> {
        self.camera.step(&CameraSensorInput {
            vehicle,
            cg_body_m: DVec3::ZERO,
        })?;
        let Some(capture) = self.camera.output() else {
            return Ok(());
        };
        let frame = match &mut self.visual {
            VisualSource::Synthetic(estimator) => Some(estimator.frame(capture)),
            VisualSource::Image(mode) => mode.observe(&capture)?,
        };
        if let Some(frame) = &frame {
            self.harness.submit(frame)?;
        }
        let report = &mut self.report;
        report.frames = report.frames.wrapping_add(1);
        report.nav_tiles_crossed.insert(MapCell::containing(
            DVec3::from_array(capture.eye_enu),
            NAV_TILE_M,
        ));
        if !horizon_in_view(&capture) {
            report.horizon_loss_frames = report.horizon_loss_frames.wrapping_add(1);
        }
        Ok(())
    }

    fn record_flight(&mut self, reading: &FlightReading, vehicle: &VehicleState) {
        let report = &mut self.report;
        report.peak_bank_rad = report.peak_bank_rad.max(reading.bank_rad.abs());
        report.peak_body_rate_rad_s = report
            .peak_body_rate_rad_s
            .max(vehicle.angular_rate_body.length());
        let (sin, cos) = TRACK_BEARING_RAD.sin_cos();
        let offset =
            DVec2::new(reading.position_enu_m.y, reading.position_enu_m.x) - TRACK_ORIGIN_M;
        report.cross_track_m = offset.y * cos - offset.x * sin;
        report.height_error_m = reading.height_m - CRUISE_HEIGHT_M;
    }

    fn collect_fixes(&mut self) -> Vec<VisualFix> {
        let report = &mut self.report;
        let fixes = self.harness.take_due(self.now);
        for fix in &fixes {
            match fix.eligibility {
                FusionEligibility::CorrelatedTrack => {
                    report.warm_fixes = report.warm_fixes.wrapping_add(1);
                }
                FusionEligibility::IndependentAnchor => {
                    report.independent_anchor_fixes =
                        report.independent_anchor_fixes.wrapping_add(1);
                }
                FusionEligibility::SharedMapCell => {
                    report.shared_map_cell_fixes = report.shared_map_cell_fixes.wrapping_add(1);
                }
            }
            if let Some(cell) = fix.map_cell {
                report.anchor_cells.insert(cell);
            }
        }
        fixes
    }

    fn observe_panel(&mut self) {
        let vehicle = self.source.output();
        self.panel.observe_earth(&EarthVehicleSources {
            timestamp: self.now,
            vehicle,
            height_m: ecef_to_geodetic(vehicle.position_ecef).height_m,
        });
    }
}
