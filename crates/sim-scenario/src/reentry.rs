use glam::{DMat3, DQuat, DVec3};
use sim_apollo::{
    CM_EMS, CM_IMU, CSM_SERVICE_MODULE, CockpitInstrumentPanel, CockpitInstrumentSnapshot,
    CommandModuleSources, CorridorIndication, EntryMonitorBlock, EntryMonitorConfig,
    InertialSiteSample, ThreeGimbalImuBlock, ThreeGimbalImuConfig, apollo_cm_aero,
    build_apollo_csm_tree, cm_analytic_trim_incidence_rad,
};
use sim_atmosphere::ReentryAtmosphere;
use sim_core::{
    Block, CameraMount, CameraPoseEcef, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput,
    SimInstant, TICKS_PER_SECOND, ThrusterWrench, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, UtcTime};

use crate::ScenarioError;

mod steering;

use steering::{EntryCondition, bank_moment_nm, bank_rad, lift_up_fraction};

#[cfg(test)]
mod tests;

const BODY: BodyEllipsoid = BodyEllipsoid::WGS84;
const STEP_HZ: u64 = 50;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(STEP_HZ));
const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / STEP_HZ);
/// 24 July 1969, 16:35 UTC, the morning of Apollo 11's entry.
const EPOCH_UNIX_S: f64 = -13_854_300.0;

/// Entry interface, 400,000 ft, near where Columbia met the atmosphere.
const ENTRY_LAT_DEG: f64 = -3.2;
const ENTRY_LON_DEG: f64 = 171.9;
const ENTRY_HEIGHT_M: f64 = 121_920.0;
const ENTRY_SPEED_M_S: f64 = 11_000.0;
const ENTRY_FLIGHT_PATH_DEG: f64 = 6.5;
const ENTRY_AZIMUTH_DEG: f64 = 50.0;
/// The ΔV counter's load, the crew's pad value at entry, m/s.
const EMS_INITIAL_DELTA_V_M_S: f64 = 11_000.0;
/// Below this speed the capsule is past its hypersonic trim and the entry
/// phase is over, m/s.
const END_SPEED_M_S: f64 = 300.0;
/// The deceleration the guidance holds, g.
const TARGET_LOAD_G: f64 = 4.0;
const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
/// Speed at which the capsule rolls across to steer back towards its
/// original ground track, m/s.
const BANK_REVERSAL_SPEED_M_S: f64 = 7_500.0;
/// Dynamic-pressure proxy above which a sample counts towards the trim
/// average: deceleration on the Entry Monitor, g.
const TRIM_MEASURE_MIN_G: f64 = 0.5;
/// Height above which a climb counts as skipping back out, metres.
const SKIP_OUT_HEIGHT_M: f64 = 80_000.0;
/// The view from the crew's side of the capsule, tilted from the heat shield
/// axis towards body `+z`, which faces the Earth while the lift is up.
const WINDOW_VIEW_PITCH_RAD: f64 = -65.0 * std::f64::consts::PI / 180.0;

/// What an entry has shown so far.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CapsuleReentryReport {
    /// Peak deceleration on the Entry Monitor, g.
    pub peak_g_load: f64,
    /// The Entry Monitor's corridor verdict.
    pub corridor: CorridorIndication,
    /// Height at the 0.05 g threshold, metres.
    pub threshold_height_m: Option<f64>,
    /// Mean incidence while the air is thick enough to trim, radians.
    pub mean_trim_incidence_rad: f64,
    /// The incidence the offset centre of gravity should trim at, radians.
    pub analytic_trim_incidence_rad: f64,
    /// Smallest and largest bank flown, radians.
    pub bank_range_rad: (f64, f64),
    /// Whether the capsule has rolled across for its reversal.
    pub bank_reversed: bool,
    /// Largest deceleration-weighted lift-to-drag ratio seen while trimmed.
    pub lift_to_drag: f64,
    /// Lowest height so far, metres.
    pub height_m: f64,
    /// The capsule has climbed back above the sensible atmosphere after
    /// entering it.
    pub skipped_out: bool,
    /// The `ΔV` counter, m/s.
    pub delta_v_counter_m_s: f64,
}

/// Columbia's Command Module from entry interface down through its hypersonic
/// flight, stepped on the master clock.
///
/// The capsule trims on its offset centre of gravity and steers only by
/// rolling that lift about the velocity on its reaction jets. Guidance holds
/// a constant deceleration, and a reversal at a set speed keeps the ground
/// track from walking away. The Entry Monitor is the crew's independent check
/// on all this: it steers nothing. Steering reads the true height and speed.
pub struct CapsuleReentryRun {
    vehicle: RigidBody6DofBlock,
    imu: ThreeGimbalImuBlock,
    ems: EntryMonitorBlock,
    panel: CockpitInstrumentPanel,
    wanted_bank_rad: f64,
    report: CapsuleReentryReport,
    trim_incidence_sum_rad: f64,
    trim_samples: u64,
    finished: bool,
}

/// The entry state: the capsule at its trim incidence with lift straight up.
fn entry_state(trim: f64) -> VehicleState {
    let geodetic = Geodetic::from_degrees(ENTRY_LAT_DEG, ENTRY_LON_DEG, ENTRY_HEIGHT_M);
    let position = BODY.geodetic_to_fixed(&geodetic);
    let up = position.normalize();
    let east = DVec3::Z.cross(up).normalize();
    let north = up.cross(east);
    let (sin_azimuth, cos_azimuth) = ENTRY_AZIMUTH_DEG.to_radians().sin_cos();
    let (sin_path, cos_path) = ENTRY_FLIGHT_PATH_DEG.to_radians().sin_cos();
    let horizontal = north * cos_azimuth + east * sin_azimuth;
    let along = horizontal * cos_path - up * sin_path;

    let lift_up = (up - along * up.dot(along)).normalize();
    let body_x = (along * trim.cos() + lift_up * trim.sin()).normalize();
    let body_z = (along * trim.sin() - lift_up * trim.cos()).normalize();
    VehicleState {
        position_ecef: position,
        velocity_ecef: along * ENTRY_SPEED_M_S,
        attitude: DQuat::from_mat3(&DMat3::from_cols(body_x, body_z.cross(body_x), body_z))
            .normalize(),
        angular_rate_body: DVec3::ZERO,
    }
}

fn utc_at(now: SimInstant) -> UtcTime {
    UtcTime::from_unix_seconds(EPOCH_UNIX_S)
        .advanced_by(now.duration_since(SimInstant::START).as_seconds())
}

impl CapsuleReentryRun {
    /// The capsule at entry interface, the Service Module already jettisoned.
    pub fn new() -> Result<Self, ScenarioError> {
        let trim = cm_analytic_trim_incidence_rad()?;
        let mut vehicle = RigidBody6DofBlock::new(
            RigidBodyConfig {
                body: BODY,
                period: PERIOD,
                epoch: UtcTime::from_unix_seconds(EPOCH_UNIX_S),
                initial: entry_state(trim),
            },
            build_apollo_csm_tree()?,
            Box::new(apollo_cm_aero()?),
            Box::new(ReentryAtmosphere),
        )?;
        vehicle.separate_stage(CSM_SERVICE_MODULE)?;
        let inertial = InertialTransform::earth_at(utc_at(SimInstant::START));
        let imu_site = InertialSiteSample::of(&vehicle, CM_IMU, &inertial)?;
        let mut run = Self {
            imu: ThreeGimbalImuBlock::new(ThreeGimbalImuConfig::nominal(
                PERIOD,
                imu_site.site_to_inertial,
            ))?,
            ems: EntryMonitorBlock::new(EntryMonitorConfig {
                period: PERIOD,
                sensing_axis_site: DVec3::X,
                initial_delta_v_m_s: EMS_INITIAL_DELTA_V_M_S,
            })?,
            panel: CockpitInstrumentPanel::new(SimInstant::START),
            wanted_bank_rad: 0.0,
            report: CapsuleReentryReport {
                peak_g_load: 0.0,
                corridor: CorridorIndication::Pending,
                threshold_height_m: None,
                mean_trim_incidence_rad: 0.0,
                analytic_trim_incidence_rad: trim,
                bank_range_rad: (0.0, 0.0),
                bank_reversed: false,
                lift_to_drag: 0.0,
                height_m: ENTRY_HEIGHT_M,
                skipped_out: false,
                delta_v_counter_m_s: EMS_INITIAL_DELTA_V_M_S,
            },
            trim_incidence_sum_rad: 0.0,
            trim_samples: 0,
            finished: false,
            vehicle,
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
        self.vehicle.now()
    }

    /// The true vehicle state.
    pub fn vehicle(&self) -> VehicleState {
        self.vehicle.output()
    }

    /// The cockpit at [`Self::now`].
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        self.panel.snapshot()
    }

    /// What the entry has shown so far.
    pub fn report(&self) -> CapsuleReentryReport {
        self.report
    }

    /// The bank the capsule is flying, radians.
    pub fn bank_rad(&self) -> f64 {
        let state = self.vehicle.output();
        bank_rad(state.attitude, state.position_ecef, state.velocity_ecef)
    }

    /// Whether the capsule has slowed through the end of the entry phase.
    pub fn finished(&self) -> bool {
        self.finished
    }

    /// A view of the Earth rushing up, past the edge of the heat shield.
    pub fn eye_pose(&self) -> CameraPoseEcef {
        CameraMount::oblique(DVec3::ZERO, 0.0, WINDOW_VIEW_PITCH_RAD, 0.0)
            .pose(&self.vehicle.output(), DVec3::ZERO)
    }

    /// Advances one period. Past the end of the entry nothing moves.
    pub fn step(&mut self) -> Result<(), ScenarioError> {
        if self.finished {
            return Ok(());
        }
        self.choose_bank();
        let state = self.vehicle.output();
        let moment = bank_moment_nm(
            state.attitude,
            state.position_ecef,
            state.velocity_ecef,
            self.vehicle.omega_inertial_body(),
            self.vehicle.mass_properties().inertia_cg_body,
            self.wanted_bank_rad,
        );
        self.vehicle.step(&RigidBodyInput {
            thrusters: vec![ThrusterWrench {
                application_point_body_m: self.vehicle.mass_properties().cg_body_m,
                force_body_n: DVec3::ZERO,
                moment_body_nm: moment,
            }],
            ..Default::default()
        })?;
        let inertial = InertialTransform::earth_at(utc_at(self.now()));
        self.imu
            .step(&InertialSiteSample::of(&self.vehicle, CM_IMU, &inertial)?)?;
        self.ems
            .step(&InertialSiteSample::of(&self.vehicle, CM_EMS, &inertial)?)?;
        self.observe_panel();
        self.record();
        Ok(())
    }

    /// The bank the drag-holding law calls for, on the side the reversal has
    /// reached.
    fn choose_bank(&mut self) {
        let state = self.vehicle.output();
        let speed = state.velocity_ecef.length();
        let up = state.position_ecef.normalize();
        let lift_up = lift_up_fraction(
            &EntryCondition {
                drag_m_s2: self.ems.output().g_load.max(0.0) * STANDARD_GRAVITY_M_S2,
                speed_m_s: speed,
                climb_m_s: state.velocity_ecef.dot(up),
                radius_m: state.position_ecef.length(),
                gravity_m_s2: BODY.gravity_fixed(state.position_ecef).length(),
            },
            TARGET_LOAD_G * STANDARD_GRAVITY_M_S2,
        );
        let reversed = speed < BANK_REVERSAL_SPEED_M_S;
        self.report.bank_reversed |= reversed;
        let bank = lift_up.acos();
        self.wanted_bank_rad = if reversed { -bank } else { bank };
    }

    fn observe_panel(&mut self) {
        let state = self.vehicle.output();
        self.panel.observe_command_module(&CommandModuleSources {
            timestamp: self.now(),
            imu: self.imu.output(),
            ems: self.ems.output(),
            height_m: BODY.fixed_to_geodetic(state.position_ecef).height_m,
            speed_m_s: state.velocity_ecef.length(),
        });
    }

    fn record(&mut self) {
        let state = self.vehicle.output();
        let ems = self.ems.output();
        let height = BODY.fixed_to_geodetic(state.position_ecef).height_m;
        let report = &mut self.report;
        report.peak_g_load = ems.peak_g_load;
        report.corridor = ems.corridor;
        report.delta_v_counter_m_s = ems.delta_v_counter_m_s;
        report.height_m = report.height_m.min(height);
        report.skipped_out |= report.height_m < SKIP_OUT_HEIGHT_M && height > SKIP_OUT_HEIGHT_M;
        if ems.threshold_reached && report.threshold_height_m.is_none() {
            report.threshold_height_m = Some(height);
        }
        let bank = bank_rad(state.attitude, state.position_ecef, state.velocity_ecef);
        let (low, high) = report.bank_range_rad;
        report.bank_range_rad = (low.min(bank), high.max(bank));
        if ems.g_load >= TRIM_MEASURE_MIN_G {
            let air = state.attitude.inverse() * state.velocity_ecef;
            self.trim_incidence_sum_rad += air.z.atan2(air.x);
            self.trim_samples = self.trim_samples.wrapping_add(1);
            report.mean_trim_incidence_rad = self.trim_incidence_sum_rad / self.trim_samples as f64;
            let force = state.attitude * self.vehicle.aero_wrench().force_body_n;
            let along = state.velocity_ecef.normalize();
            let drag = -force.dot(along);
            let lift = (force + along * drag).length();
            report.lift_to_drag = report.lift_to_drag.max(lift / drag.max(1.0));
        }
        self.finished = state.velocity_ecef.length() < END_SPEED_M_S;
    }
}
