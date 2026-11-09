use glam::{DQuat, DVec3};
use mbd_core::{ComponentTree, VacuumAero};
use sim_apollo::{
    CockpitInstrumentPanel, CockpitInstrumentSnapshot, GuidanceSource, InertialSiteSample, LM_ASA,
    LM_CDR_EYE, LM_DPS_EXHAUST_VELOCITY_M_S, LM_DPS_TANK, LM_DPS_THRUST_N, LM_DPS_THRUST_POINT_M,
    LM_IMU, LandingRadarBlock, LandingRadarConfig, LpdProjector, LpdReading,
    LunarContactProbeBlock, RadarSample, RateGyroAssemblyBlock, RateGyroConfig, StrapdownAsaBlock,
    StrapdownAsaConfig, SurfacePlane, ThreeGimbalImuBlock, ThreeGimbalImuConfig,
    build_apollo_lm_tree, site_in_fixed_frame,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    Block, CameraMount, CameraPoseEcef, PropellantBurn, RateAdapter, RigidBody6DofBlock,
    RigidBodyConfig, RigidBodyInput, SimInstant, TICKS_PER_SECOND, ThrusterWrench, TickSpan,
    VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, LocalFrame};
use sim_scenery::{ElevationSource, LunarTranquilityScenery, WEST_CRATER};
use sim_spacetime::SimEpoch;

use crate::{ScenarioError, ScenarioProfile};

mod guidance;
mod sensing;
pub(crate) mod snapshot;

use guidance::{
    ApproachTarget, approach_acceleration, attitude_moment_nm, roughness_m, thrust_attitude,
    touchdown_acceleration,
};

#[cfg(test)]
mod tests;

const BODY: BodyEllipsoid = BodyEllipsoid::MOON_IAU;
const STEP_HZ: u64 = 50;
const RADAR_HZ: u64 = 10;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(STEP_HZ));
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(RADAR_HZ));
const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / STEP_HZ);
const RADAR_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / RADAR_HZ);

/// Where the approach begins, inside the scenery's coverage: east of the site
/// and moving west, as Eagle did with the low Sun at its back.
const START_EAST_M: f64 = 3_300.0;
const START_NORTH_M: f64 = -150.0;
const START_HEIGHT_M: f64 = 1_150.0;
const START_WESTWARD_M_S: f64 = 70.0;
const START_SINK_M_S: f64 = 22.0;
/// DPS propellant left at the start of this last stretch, kg.
const START_DPS_PROPELLANT_KG: f64 = 2_000.0;
/// The auto-targeted point: the boulder field on West Crater's near rim.
const TARGET_EAST_M: f64 = WEST_CRATER.east_m - 105.0;
const TARGET_NORTH_M: f64 = 0.0;
/// Time from the start to the end of the approach, seconds.
const APPROACH_TIME_S: f64 = 95.0;
/// Height over the target where the approach hands over to the vertical
/// touchdown, metres.
const GATE_HEIGHT_M: f64 = 45.0;
const GATE_SINK_M_S: f64 = 2.5;
const TOUCHDOWN_SINK_M_S: f64 = 1.0;
/// When the Commander looks at the target, seconds from the start.
const REDESIGNATE_TIME_S: f64 = 55.0;
/// Extra time the longer flight to a redesignated target takes, seconds.
const REDESIGNATE_EXTENSION_S: f64 = 25.0;
/// Terrain spread beyond which a pad is not safe, metres.
const MAX_PAD_ROUGHNESS_M: f64 = 0.45;
/// Step of the search west along the approach for a clear pad, metres.
const PAD_SEARCH_STEP_M: f64 = 20.0;
const PAD_SEARCH_STEPS: u32 = 40;
/// Lowest thrust the DPS holds once lit, as a fraction of full.
const MIN_THROTTLE: f64 = 0.1;
/// The view out of the Commander's window: `+z`, tilted 25° down from it.
const WINDOW_VIEW_PITCH_RAD: f64 = -115.0 * std::f64::consts::PI / 180.0;

/// What may differ between lunar landing runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LunarLandingConfig {
    /// The navigation base's orientation in stable-member axes at the start.
    /// Identity puts the gimbals at zero, the best-placed alignment; a poor
    /// one lets the pitch-over walk the middle gimbal into lock.
    pub navbase_to_platform: DQuat,
}

impl Default for LunarLandingConfig {
    fn default() -> Self {
        Self {
            navbase_to_platform: DQuat::IDENTITY,
        }
    }
}

/// What a landing has shown so far.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LunarLandingReport {
    /// DPS propellant burned, kg.
    pub dps_burned_kg: f64,
    /// How far the centre of gravity has climbed the airframe, metres.
    pub cg_rise_m: f64,
    /// Radar samples inside limits.
    pub radar_valid_samples: u64,
    /// Radar samples beyond limits, or with no terrain under the beam.
    pub radar_invalid_samples: u64,
    /// The Commander moved the target off the boulders.
    pub redesignated: bool,
    /// The window graduation of the first target at the redesignation,
    /// degrees.
    pub lpd_first_target_deg: Option<f64>,
    /// The window graduation of the clear target at the redesignation,
    /// degrees.
    pub lpd_clear_target_deg: Option<f64>,
    /// Terrain spread at the first target, metres.
    pub first_target_roughness_m: f64,
    /// Terrain spread at the target flown to, metres.
    pub target_roughness_m: f64,
    /// Largest `|MGA|`, degrees.
    pub peak_mga_deg: f64,
    /// Distance across the surface from the target at contact, metres.
    pub contact_miss_m: Option<f64>,
    /// Descent rate at contact, m/s.
    pub contact_sink_m_s: Option<f64>,
    /// Speed across the surface at contact, m/s.
    pub contact_drift_m_s: Option<f64>,
}

/// Eagle's last 3 km to the Sea of Tranquility, stepped on the master clock.
///
/// Guidance flies the descent engine to the designated point on the active
/// source's attitude, so a transfer to AGS keeps flying the same law on the
/// backup sensor. Steering reads true position and velocity; only the attitude
/// is sensed. Touchdown itself is not modelled: the run ends when a probe
/// lights the contact light and the engine shuts down.
pub struct LunarLandingRun {
    vehicle: RigidBody6DofBlock,
    imu: ThreeGimbalImuBlock,
    asa: StrapdownAsaBlock,
    rate_gyros: RateGyroAssemblyBlock,
    radar: LandingRadarBlock,
    radar_rate: RateAdapter,
    radar_sample: Option<RadarSample>,
    probes: LunarContactProbeBlock,
    lpd: LpdProjector,
    panel: CockpitInstrumentPanel,
    scenery: LunarTranquilityScenery,
    epoch: SimEpoch,
    window_towards: DVec3,
    target_east_m: f64,
    target_north_m: f64,
    target_fixed_m: DVec3,
    approach_ends_s: f64,
    start_propellant_kg: f64,
    start_cg_x_m: f64,
    report: LunarLandingReport,
    finished: bool,
}

fn lm_tree() -> Result<ComponentTree, ScenarioError> {
    let mut tree = build_apollo_lm_tree()?;
    let mut tank = *tree.state(LM_DPS_TANK)?;
    tank.fluid_mass_kg = START_DPS_PROPELLANT_KG;
    tree.set_state(LM_DPS_TANK, tank)?;
    Ok(tree)
}

fn ground_at(scenery: &LunarTranquilityScenery, position: &Geodetic) -> Result<f64, ScenarioError> {
    scenery
        .ground_height_m(position)?
        .ok_or(ScenarioError::NoTerrain {
            lat_deg: position.lat_rad.to_degrees(),
            lon_deg: position.lon_rad.to_degrees(),
        })
}

/// The point at `east_m`, `north_m`, on the ground.
fn ground_point(
    scenery: &LunarTranquilityScenery,
    east_m: f64,
    north_m: f64,
) -> Result<DVec3, ScenarioError> {
    let mut position = scenery.at_local(east_m, north_m);
    position.height_m = ground_at(scenery, &position)?;
    Ok(BODY.geodetic_to_fixed(&position))
}

fn up_at(position_m: DVec3) -> DVec3 {
    LocalFrame::at(&BODY.fixed_to_geodetic(position_m)).up()
}

/// The thrust direction that gives acceleration `wanted` against gravity.
fn thrust_direction(position_m: DVec3, wanted: DVec3) -> DVec3 {
    (wanted - BODY.gravity_fixed(position_m)).normalize()
}

fn approach_target(target_fixed_m: DVec3) -> ApproachTarget {
    let up = up_at(target_fixed_m);
    ApproachTarget {
        position_m: target_fixed_m + up * GATE_HEIGHT_M,
        velocity_m_s: up * -GATE_SINK_M_S,
    }
}

impl LunarLandingRun {
    /// The approach at its first instant.
    pub fn new(config: &LunarLandingConfig) -> Result<Self, ScenarioError> {
        let epoch = ScenarioProfile::Apollo11LunarLanding.sim_epoch()?;
        let scenery = LunarTranquilityScenery::apollo_11_landing();
        let site_up = up_at(BODY.geodetic_to_fixed(&scenery.at_local(0.0, 0.0)));
        let west = -LocalFrame::at(&scenery.at_local(0.0, 0.0)).east();
        let target_fixed_m = ground_point(&scenery, TARGET_EAST_M, TARGET_NORTH_M)?;

        let mut start = scenery.at_local(START_EAST_M, START_NORTH_M);
        start.height_m = START_HEIGHT_M;
        let position_ecef = BODY.geodetic_to_fixed(&start);
        let velocity_ecef = west * START_WESTWARD_M_S - site_up * START_SINK_M_S;
        let wanted = approach_acceleration(
            position_ecef,
            velocity_ecef,
            &approach_target(target_fixed_m),
            APPROACH_TIME_S,
        );
        let attitude = thrust_attitude(thrust_direction(position_ecef, wanted), west);
        let vehicle = RigidBody6DofBlock::new(
            RigidBodyConfig {
                body: BODY,
                period: PERIOD,
                epoch: epoch.start_utc()?,
                initial: VehicleState {
                    position_ecef,
                    velocity_ecef,
                    attitude,
                    angular_rate_body: DVec3::ZERO,
                },
            },
            lm_tree()?,
            Box::new(VacuumAero),
            Box::new(VacuumAtmosphere),
        )?;
        let inertial = InertialTransform::moon_at(epoch.start_utc()?);
        let imu_site = InertialSiteSample::of(&vehicle, LM_IMU, &inertial)?;
        let asa_site = InertialSiteSample::of(&vehicle, LM_ASA, &inertial)?;
        let platform = imu_site.site_to_inertial * config.navbase_to_platform.inverse();
        let mut run = Self {
            imu: ThreeGimbalImuBlock::new(ThreeGimbalImuConfig::nominal(PERIOD, platform))?,
            asa: StrapdownAsaBlock::new(StrapdownAsaConfig::nominal(
                PERIOD,
                asa_site.site_to_inertial,
            ))?,
            rate_gyros: RateGyroAssemblyBlock::new(RateGyroConfig::nominal(PERIOD, DVec3::ZERO))?,
            radar: LandingRadarBlock::new(LandingRadarConfig::nominal(RADAR_PERIOD))?,
            radar_rate: RateAdapter::new(RADAR_PERIOD),
            radar_sample: None,
            probes: LunarContactProbeBlock::new(PERIOD)?,
            lpd: LpdProjector,
            panel: CockpitInstrumentPanel::new(SimInstant::START),
            scenery,
            epoch,
            window_towards: west,
            target_east_m: TARGET_EAST_M,
            target_north_m: TARGET_NORTH_M,
            target_fixed_m,
            approach_ends_s: APPROACH_TIME_S,
            start_propellant_kg: START_DPS_PROPELLANT_KG,
            start_cg_x_m: vehicle.mass_properties().cg_body_m.x,
            report: LunarLandingReport::default(),
            finished: false,
            vehicle,
        };
        run.sense()?;
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

    /// The true vehicle state, Moon-fixed.
    pub fn vehicle(&self) -> VehicleState {
        self.vehicle.output()
    }

    /// The cockpit at [`Self::now`].
    pub fn cockpit(&self) -> CockpitInstrumentSnapshot {
        self.panel.snapshot()
    }

    /// What the landing has shown so far.
    pub fn report(&self) -> LunarLandingReport {
        self.report
    }

    /// The point the guidance is flying to, Moon-fixed metres.
    pub fn target_fixed_m(&self) -> DVec3 {
        self.target_fixed_m
    }

    /// The target east and north of the site, metres.
    pub fn target_local_m(&self) -> (f64, f64) {
        (self.target_east_m, self.target_north_m)
    }

    /// The crew moves the guidance switch to `source`.
    pub fn crew_select(&mut self, source: GuidanceSource) {
        self.panel.crew_select(source);
    }

    /// Whether a probe has touched and the engine has shut down.
    pub fn finished(&self) -> bool {
        self.finished
    }

    /// The Commander's view through the window.
    pub fn eye_pose(&self) -> Result<CameraPoseEcef, ScenarioError> {
        let eye = site_in_fixed_frame(&self.vehicle, LM_CDR_EYE)?;
        let view = CameraMount::oblique(DVec3::ZERO, 0.0, WINDOW_VIEW_PITCH_RAD, 0.0);
        Ok(CameraPoseEcef {
            position_ecef: eye.position_m,
            ..view.pose(&self.vehicle.output(), DVec3::ZERO)
        })
    }

    /// Advances one period. After contact nothing moves.
    pub fn step(&mut self) -> Result<(), ScenarioError> {
        if self.finished {
            return Ok(());
        }
        if self.elapsed_s() >= REDESIGNATE_TIME_S && self.report.first_target_roughness_m == 0.0 {
            self.review_target()?;
        }
        let input = self.steer()?;
        self.vehicle.step(&input)?;
        self.sense()
    }

    fn elapsed_s(&self) -> f64 {
        self.now().duration_since(SimInstant::START).as_seconds()
    }

    /// The attitude being flown, Moon-fixed, from the cockpit's active source.
    fn sensed_attitude(&self) -> Result<DQuat, ScenarioError> {
        let inertial = InertialTransform::moon_at(self.epoch.utc_at(self.now())?);
        let flown = self.panel.snapshot().attitude.value;
        Ok((inertial.fixed_to_j2000_rotation().inverse() * flown).normalize())
    }

    /// What the guidance wants of the engine and jets this step.
    fn steer(&self) -> Result<RigidBodyInput, ScenarioError> {
        let state = self.vehicle.output();
        let position = state.position_ecef;
        let time_to_go = self.approach_ends_s - self.elapsed_s();
        let wanted = if time_to_go > 0.0 {
            approach_acceleration(
                position,
                state.velocity_ecef,
                &approach_target(self.target_fixed_m),
                time_to_go,
            )
        } else {
            touchdown_acceleration(
                position,
                state.velocity_ecef,
                self.target_fixed_m,
                up_at(self.target_fixed_m),
                TOUCHDOWN_SINK_M_S,
            )
        };
        let mass = self.vehicle.mass_properties();
        let direction = thrust_direction(position, wanted);
        let sensed = self.sensed_attitude()?;
        let thrust_n = ((wanted - BODY.gravity_fixed(position)).dot(sensed * DVec3::X)
            * mass.total_mass_kg)
            .clamp(MIN_THROTTLE * LM_DPS_THRUST_N, LM_DPS_THRUST_N);
        let moment = attitude_moment_nm(
            sensed,
            thrust_attitude(direction, self.window_towards),
            self.panel.snapshot().body_rate_rad_s.value,
            mass.inertia_cg_body,
        );
        let from_gimbal = (mass.cg_body_m - LM_DPS_THRUST_POINT_M).normalize();
        Ok(RigidBodyInput {
            thrusters: vec![ThrusterWrench {
                application_point_body_m: LM_DPS_THRUST_POINT_M,
                force_body_n: from_gimbal * thrust_n,
                moment_body_nm: moment,
            }],
            burns: vec![PropellantBurn {
                tank: LM_DPS_TANK,
                dm_dt_kg_s: -thrust_n / LM_DPS_EXHAUST_VELOCITY_M_S,
            }],
            ..Default::default()
        })
    }

    pub(super) fn lpd_reading(&self, target_fixed_m: DVec3) -> Result<LpdReading, ScenarioError> {
        let eye = site_in_fixed_frame(&self.vehicle, LM_CDR_EYE)?;
        Ok(self.lpd.project(
            eye.position_m,
            self.vehicle.output().attitude,
            target_fixed_m,
        ))
    }

    /// The first clear pad west of the target along the approach.
    fn clear_pad(&self) -> Option<(f64, f64)> {
        (1..=PAD_SEARCH_STEPS)
            .map(|step| {
                let east = self.target_east_m - PAD_SEARCH_STEP_M * f64::from(step);
                (east, self.target_north_m)
            })
            .find(|(east, north)| roughness_m(&self.scenery, *east, *north) <= MAX_PAD_ROUGHNESS_M)
    }

    /// Looks at the target through the window and, if it is rough, reads a
    /// clear pad off the scale and redesignates to it.
    fn review_target(&mut self) -> Result<(), ScenarioError> {
        let roughness = roughness_m(&self.scenery, self.target_east_m, self.target_north_m);
        self.report.first_target_roughness_m = roughness.max(f64::MIN_POSITIVE);
        self.report.target_roughness_m = roughness;
        self.report.lpd_first_target_deg = Some(
            self.lpd_reading(self.target_fixed_m)?
                .elevation_rad
                .to_degrees(),
        );
        if roughness <= MAX_PAD_ROUGHNESS_M {
            return Ok(());
        }
        let Some((east, north)) = self.clear_pad() else {
            return Ok(());
        };
        let pad = ground_point(&self.scenery, east, north)?;
        let reading = self.lpd_reading(pad)?;
        let surface = SurfacePlane {
            point_fixed_m: pad,
            normal_fixed: up_at(pad),
        };
        let eye = site_in_fixed_frame(&self.vehicle, LM_CDR_EYE)?;
        if let Some(point) = self.lpd.redesignate(
            eye.position_m,
            self.vehicle.output().attitude,
            &reading,
            &surface,
        ) {
            self.target_fixed_m = point;
            self.target_east_m = east;
            self.target_north_m = north;
            self.approach_ends_s += REDESIGNATE_EXTENSION_S;
            self.report.redesignated = true;
            self.report.lpd_clear_target_deg = Some(reading.elevation_rad.to_degrees());
            self.report.target_roughness_m = roughness_m(&self.scenery, east, north);
        }
        Ok(())
    }
}
