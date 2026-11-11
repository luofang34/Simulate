mod kernels;
#[cfg(test)]
mod tests;

use std::collections::VecDeque;

use aviate_core::control::VehicleController;
use aviate_core::kernel::DefaultAviateKernel;
use aviate_core::mixer::Mixer;
use aviate_core::state::{EstimateQuality, StateEstimate};
use aviate_hal_io::SystemCommand;
use aviate_xil_direct::{
    DirectActuatorCmd, DirectBoardConfig, DirectMeasuredKinematics, DirectNoiseTier,
    DirectTruthSample, DirectXilBoard,
};
use glam::{DQuat, DVec3};
use sim_core::{
    EndpointError, ExternalEndpoint, SimInstant, StampedResponse, TICKS_PER_SECOND, VehicleState,
};
use sim_devices::DeviceSamples;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use world_core::{DecimalYear, GeodeticPosition};

use crate::{AviateBridgeError, AviateStimulus, DirectInitStage};

pub use kernels::{
    DirectFixedWingKernel, DirectX500Kernel, build_direct_fixed_wing_kernel,
    build_direct_x500_kernel, direct_fixed_wing_controls,
};

const TICKS_PER_MICROSECOND: u64 = TICKS_PER_SECOND / 1_000_000;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(1_000_000));

/// Local north-east-down tangent frame anchored at a WGS84 origin for
/// [`DirectXilBoard`] truth synthesis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NedWorld {
    body: BodyEllipsoid,
    origin: Geodetic,
    origin_ecef: DVec3,
    frame: LocalFrame,
    ned_from_ecef: DQuat,
}

impl NedWorld {
    /// Constructs a local NED tangent world at `origin` on `body`.
    #[must_use]
    pub fn new(body: &BodyEllipsoid, origin: &Geodetic) -> Self {
        let frame = LocalFrame::at(origin);
        let ned_from_ecef = DQuat::from_mat3(&frame.ned_to_ecef().transpose()).normalize();
        Self {
            body: *body,
            origin: *origin,
            origin_ecef: body.geodetic_to_fixed(origin),
            frame,
            ned_from_ecef,
        }
    }

    /// The local tangent frame at the origin.
    #[must_use]
    pub const fn frame(&self) -> &LocalFrame {
        &self.frame
    }

    /// The origin in ECEF, metres.
    #[must_use]
    pub const fn origin_ecef(&self) -> DVec3 {
        self.origin_ecef
    }

    /// Converts the anchor origin into `world_core::GeodeticPosition`.
    #[must_use]
    pub fn world_geodetic(&self) -> GeodeticPosition {
        GeodeticPosition::wgs84(
            self.origin.lat_rad.to_degrees(),
            self.origin.lon_rad.to_degrees(),
            self.origin.height_m,
        )
    }

    /// Projects centre-of-gravity kinematics directly into a [`DirectTruthSample`]
    /// with zero heap allocations.
    pub fn truth_from_cg(
        &self,
        time_us: u64,
        reset_generation: u32,
        step: u64,
        vehicle: &VehicleState,
        specific_force_frd_m_s2: DVec3,
    ) -> Result<DirectTruthSample, AviateBridgeError> {
        check_vec3(vehicle.position_ecef, "vehicle.position_ecef")?;
        check_vec3(vehicle.velocity_ecef, "vehicle.velocity_ecef")?;
        check_vec3(vehicle.angular_rate_body, "vehicle.angular_rate_body")?;
        check_vec3(specific_force_frd_m_s2, "specific_force_frd_m_s2")?;
        if !vehicle.attitude.is_finite() {
            return Err(AviateBridgeError::DirectNonFinite {
                field: "vehicle.attitude",
            });
        }
        let pos_ned = self
            .frame
            .ecef_to_ned_vector(vehicle.position_ecef - self.origin_ecef);
        let vel_ned = self.frame.ecef_to_ned_vector(vehicle.velocity_ecef);
        let q = (self.ned_from_ecef * vehicle.attitude).normalize();
        // The gyroscopes sense the rate relative to inertial space.
        let w = vehicle.inertial_rate_body(&self.body);
        let f = specific_force_frd_m_s2;

        Ok(DirectTruthSample {
            time_us,
            reset_generation,
            step,
            position_ned_m: [pos_ned.x as f32, pos_ned.y as f32, pos_ned.z as f32],
            attitude_wxyz: [q.w as f32, q.x as f32, q.y as f32, q.z as f32],
            measured: Some(DirectMeasuredKinematics {
                velocity_ned_mps: [vel_ned.x as f32, vel_ned.y as f32, vel_ned.z as f32],
                body_rates_frd_rps: [w.x as f32, w.y as f32, w.z as f32],
                specific_force_frd_mps2: [f.x as f32, f.y as f32, f.z as f32],
            }),
        })
    }

    /// The vehicle state an onboard NED estimate describes, in ECEF.
    #[must_use]
    pub fn vehicle_from_estimate(&self, estimate: &StateEstimate) -> VehicleState {
        let [n, e, d] = estimate.position_ned.map(|m| f64::from(m.0));
        let [vn, ve, vd] = estimate.velocity_ned.map(|v| f64::from(v.0));
        let [p, q, r] = estimate.angular_velocity.map(|w| f64::from(w.0));
        let a = estimate.attitude;
        let body_to_ned = DQuat::from_xyzw(
            f64::from(a.x),
            f64::from(a.y),
            f64::from(a.z),
            f64::from(a.w),
        )
        .normalize();
        let ned_to_ecef = self.frame.ned_to_ecef();
        VehicleState {
            position_ecef: self.origin_ecef + ned_to_ecef * DVec3::new(n, e, d),
            velocity_ecef: ned_to_ecef * DVec3::new(vn, ve, vd),
            attitude: (self.ned_from_ecef.inverse() * body_to_ned).normalize(),
            angular_rate_body: DVec3::new(p, q, r),
        }
    }

    /// Projects a component-mounted [`AviateStimulus`] into a [`DirectTruthSample`].
    pub fn truth_from_site(
        &self,
        time_us: u64,
        reset_generation: u32,
        step: u64,
        stimulus: &AviateStimulus,
    ) -> Result<DirectTruthSample, AviateBridgeError> {
        let v = &stimulus.vehicle;
        check_vec3(v.position_ecef, "vehicle.position_ecef")?;
        check_vec3(v.velocity_ecef, "vehicle.velocity_ecef")?;
        check_vec3(v.angular_rate_body, "vehicle.angular_rate_body")?;
        check_vec3(
            stimulus.site.specific_force_site_m_s2,
            "site.specific_force_site_m_s2",
        )?;
        check_vec3(
            stimulus.site.angular_rate_site_rad_s,
            "site.angular_rate_site_rad_s",
        )?;
        if !v.attitude.is_finite() {
            return Err(AviateBridgeError::DirectNonFinite {
                field: "vehicle.attitude",
            });
        }
        let site_ecef = stimulus
            .site
            .in_ecef(v.position_ecef, v.velocity_ecef, v.attitude);
        let rate_ecef = v.attitude * v.angular_rate_body;
        let lever_ecef = site_ecef.position_m - v.position_ecef;
        let vel_ecef = v.velocity_ecef + rate_ecef.cross(lever_ecef);

        let pos_ned = self
            .frame
            .ecef_to_ned_vector(site_ecef.position_m - self.origin_ecef);
        let vel_ned = self.frame.ecef_to_ned_vector(vel_ecef);
        let q = (self.ned_from_ecef * site_ecef.attitude).normalize();
        let rate_site = stimulus.site.angular_rate_site_rad_s;
        let f_site = stimulus.site.specific_force_site_m_s2;

        Ok(DirectTruthSample {
            time_us,
            reset_generation,
            step,
            position_ned_m: [pos_ned.x as f32, pos_ned.y as f32, pos_ned.z as f32],
            attitude_wxyz: [q.w as f32, q.x as f32, q.y as f32, q.z as f32],
            measured: Some(DirectMeasuredKinematics {
                velocity_ned_mps: [vel_ned.x as f32, vel_ned.y as f32, vel_ned.z as f32],
                body_rates_frd_rps: [rate_site.x as f32, rate_site.y as f32, rate_site.z as f32],
                specific_force_frd_mps2: [f_site.x as f32, f_site.y as f32, f_site.z as f32],
            }),
        })
    }
}

/// Centre-of-gravity stimulus for [`AviateDirectRunner`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectCgStimulus {
    /// True centre-of-gravity vehicle state.
    pub vehicle: VehicleState,
    /// Specific force in body FRD axes (`m/s²`).
    pub specific_force_frd_m_s2: DVec3,
}

/// Configuration for [`AviateDirectRunner`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AviateDirectConfig {
    /// WGS84 anchor origin for local NED synthesis and geomagnetic/atmosphere models.
    pub origin: Geodetic,
    /// Decimal year for IGRF geomagnetic synthesis (e.g. `2026.5`).
    pub decimal_year: f64,
    /// Deterministic sensor noise tier (`Off`, `Consumer`, `Industrial`, `Tactical`).
    pub noise: DirectNoiseTier,
    /// Deterministic PRNG seed for sensor noise.
    pub noise_seed: u64,
    /// Flight controller MAVLink system ID.
    pub sys_id: u8,
    /// Flight controller MAVLink component ID.
    pub comp_id: u8,
}

impl AviateDirectConfig {
    /// Clean noise-free configuration anchored at `origin` for epoch `2026.5`.
    #[must_use]
    pub const fn ideal(origin: Geodetic) -> Self {
        Self {
            origin,
            decimal_year: 2026.5,
            noise: DirectNoiseTier::Off,
            noise_seed: 1,
            sys_id: 1,
            comp_id: 1,
        }
    }

    /// [`Self::ideal`] with industrial MEMS sensor noise (ICM-42688-P / BMI088 class) on
    /// the IMU, magnetometer, barometer, and GNSS the onboard EKF fuses.
    #[must_use]
    pub const fn industrial(origin: Geodetic) -> Self {
        Self {
            noise: DirectNoiseTier::Industrial,
            ..Self::ideal(origin)
        }
    }
}

/// Zero-overhead Tier-0 in-process flight-controller runner wrapping
/// [`DirectXilBoard<C, M>`].
pub struct AviateDirectRunner<C, M>
where
    C: VehicleController,
    M: Mixer,
{
    board: DirectXilBoard<C, M>,
    world: NedWorld,
    sim_step: u64,
    reset_generation: u32,
    inbox: VecDeque<StampedResponse<DirectActuatorCmd>>,
}

impl<C, M> AviateDirectRunner<C, M>
where
    C: VehicleController,
    M: Mixer,
{
    /// Initialises a Tier-0 direct runner around `kernel`.
    pub fn new(
        kernel: DefaultAviateKernel<C, M>,
        config: AviateDirectConfig,
    ) -> Result<Self, AviateBridgeError> {
        let world = NedWorld::new(&BodyEllipsoid::WGS84, &config.origin);
        let board_cfg = DirectBoardConfig {
            origin: world.world_geodetic(),
            epoch: DecimalYear(config.decimal_year),
            noise: config.noise,
            noise_seed: config.noise_seed,
            sys_id: config.sys_id,
            comp_id: config.comp_id,
        };
        let board = DirectXilBoard::new(kernel, board_cfg).map_err(|err| {
            AviateBridgeError::DirectInit {
                stage: DirectInitStage::from(err),
            }
        })?;
        Ok(Self {
            board,
            world,
            sim_step: 0,
            reset_generation: 0,
            inbox: VecDeque::new(),
        })
    }

    /// Steps the flight controller directly from CG truth (`0` syscalls, `0` heap allocs).
    pub fn step_cg(
        &mut self,
        at: SimInstant,
        vehicle: &VehicleState,
        specific_force_frd_m_s2: DVec3,
    ) -> Result<DirectActuatorCmd, AviateBridgeError> {
        let time_us = micros_since_start(at)?;
        let next_step = self.sim_step.wrapping_add(1);
        let truth = self.world.truth_from_cg(
            time_us,
            self.reset_generation,
            next_step,
            vehicle,
            specific_force_frd_m_s2,
        )?;
        self.sim_step = next_step;
        Ok(self.board.step_truth(&truth))
    }

    /// Steps the flight controller from a component-mounted [`AviateStimulus`].
    pub fn step_site(
        &mut self,
        at: SimInstant,
        stimulus: &AviateStimulus,
    ) -> Result<DirectActuatorCmd, AviateBridgeError> {
        let time_us = micros_since_start(at)?;
        let next_step = self.sim_step.wrapping_add(1);
        let truth =
            self.world
                .truth_from_site(time_us, self.reset_generation, next_step, stimulus)?;
        self.sim_step = next_step;
        Ok(self.board.step_truth(&truth))
    }

    /// Steps the flight controller on what the vehicle's devices sampled at
    /// `at`. The first call of a world generation only sets the board's
    /// time base.
    pub fn step_sensors(
        &mut self,
        at: SimInstant,
        samples: &DeviceSamples,
    ) -> Result<DirectActuatorCmd, AviateBridgeError> {
        let time_us = micros_since_start(at)?;
        self.sim_step = self.sim_step.wrapping_add(1);
        let sensors = self
            .world
            .host_sensors(time_us, self.reset_generation, samples);
        Ok(self.board.step_sensors(&sensors))
    }

    /// Resets the flight controller onto a fresh `kernel` and increments `reset_generation`.
    pub fn reset_world(&mut self, kernel: DefaultAviateKernel<C, M>) {
        self.reset_generation = self.reset_generation.wrapping_add(1);
        self.board.reset_with_kernel(kernel);
        self.inbox.clear();
    }

    /// Borrows the underlying [`DirectXilBoard`].
    #[must_use]
    pub const fn board(&self) -> &DirectXilBoard<C, M> {
        &self.board
    }

    /// Mutably borrows the underlying [`DirectXilBoard`].
    pub fn board_mut(&mut self) -> &mut DirectXilBoard<C, M> {
        &mut self.board
    }

    /// The onboard EKF's navigation solution in ECEF, or `None` while the estimator
    /// grades it unusable.
    #[must_use]
    pub fn estimated_state(&self) -> Option<VehicleState> {
        let estimate = self.board.kernel().state.estimator.get_estimate();
        (estimate.quality != EstimateQuality::Unusable)
            .then(|| self.navigation_world().vehicle_from_estimate(&estimate))
    }

    /// The tangent world the onboard estimate is in. The board moves its
    /// frame's origin to the vehicle as it flies, so after a move the
    /// configured origin no longer anchors the estimate's positions and axes.
    #[must_use]
    pub fn navigation_world(&self) -> NedWorld {
        let frame = self.board.navigation_frame();
        if frame.rebases == 0 {
            return self.world;
        }
        let origin = frame.origin;
        NedWorld::new(
            &BodyEllipsoid::WGS84,
            &Geodetic::from_degrees(
                origin.latitude_deg,
                origin.longitude_deg,
                origin.ellipsoidal_height_m,
            ),
        )
    }

    /// Whether the EKF and pre-arm gates report ready.
    #[must_use]
    pub fn is_ready(&self) -> bool {
        self.board.is_ready()
    }

    /// Whether actuator outputs are armed.
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.board.is_armed()
    }

    /// Enqueues a typed [`SystemCommand`] directly into the flight controller.
    pub fn send_command(&mut self, cmd: SystemCommand) {
        self.board.send_command(cmd);
    }

    /// Pushes raw MAVLink v2 bytes into the in-memory RX ring.
    pub fn push_mavlink_rx(&mut self, bytes: &[u8]) -> usize {
        self.board.push_mavlink_rx(bytes)
    }

    /// Pops outbound MAVLink v2 telemetry bytes from the in-memory TX ring.
    pub fn pop_mavlink_tx(&mut self, buf: &mut [u8]) -> usize {
        self.board.pop_mavlink_tx(buf)
    }
}

impl<C, M> ExternalEndpoint for AviateDirectRunner<C, M>
where
    C: VehicleController,
    M: Mixer,
{
    type Stimulus = DirectCgStimulus;
    type Response = DirectActuatorCmd;

    fn exchange_blocking(
        &mut self,
        at: SimInstant,
        stimulus: &Self::Stimulus,
    ) -> Result<Self::Response, EndpointError> {
        self.step_cg(at, &stimulus.vehicle, stimulus.specific_force_frd_m_s2)
            .map_err(|err| EndpointError {
                detail: err.to_string(),
            })
    }

    fn post(&mut self, at: SimInstant, stimulus: &Self::Stimulus) -> Result<(), EndpointError> {
        let response = self.exchange_blocking(at, stimulus)?;
        self.inbox.push_back(StampedResponse {
            deliver_at: at,
            response,
        });
        Ok(())
    }

    fn try_receive(&mut self) -> Result<Option<StampedResponse<Self::Response>>, EndpointError> {
        Ok(self.inbox.pop_front())
    }
}

fn micros_since_start(at: SimInstant) -> Result<u64, AviateBridgeError> {
    let (seconds, subsec_ticks) = at
        .duration_since(SimInstant::START)
        .seconds_and_subsec_ticks();
    seconds
        .checked_mul(1_000_000)
        .and_then(|us| us.checked_add(subsec_ticks / TICKS_PER_MICROSECOND))
        .ok_or(AviateBridgeError::TimeOutOfRange { seconds })
}

fn check_vec3(v: DVec3, field: &'static str) -> Result<(), AviateBridgeError> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(AviateBridgeError::DirectNonFinite { field })
    }
}
