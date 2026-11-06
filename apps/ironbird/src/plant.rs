use glam::{DMat3, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, LocalSiteKinematics, Placement,
    VacuumAero,
};
use sim_atmosphere::{AtmosphereField, AtmosphereSample, StandardAtmosphere};
use sim_aviate::RotorLayout;
use sim_core::{
    Block, BlockError, ControlInputs, EngineStartMode, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock,
    JsbsimFdmConfig, JsbsimFdmSnapshot, KinematicInput, RigidBody6DofBlock, RigidBodyConfig,
    RigidBodyInput, RigidBodySnapshot, TICKS_PER_SECOND, ThrusterWrench, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime, ecef_to_geodetic};
use sim_pack::{builtin_c172_package, builtin_light_trainer_package};
use sim_xil::XilStimulus;
use world_core::{GeodeticPosition, normal_gravity};

use crate::IronbirdError;

/// Built-in SHA-256-verified JSBSim fixed-wing aircraft packages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixedWingVehicle {
    /// Cessna 172 Skyhawk (`vehicles/c172`).
    C172,
    /// Simulate Light Trainer (`vehicles/light-trainer`).
    LightTrainer,
}

impl FixedWingVehicle {
    /// Nominal trimmed cruise true airspeed (`m/s`).
    #[must_use]
    pub const fn nominal_cruise_speed_m_s(self) -> f64 {
        match self {
            Self::C172 => 52.0,
            Self::LightTrainer => 48.0,
        }
    }
}

/// Closed-loop JSBSim fixed-wing plant tracking exact body-axis specific force.
pub struct JsbsimPlant {
    fdm: JsbsimFdmBlock,
    period: TickSpan,
    specific_force_frd_m_s2: DVec3,
    utc: UtcTime,
}

impl JsbsimPlant {
    /// Loads `vehicle` trimmed in running cruise at `initial` heading North.
    pub fn new_cruise(vehicle: FixedWingVehicle, initial: Geodetic) -> Result<Self, IronbirdError> {
        let package = match vehicle {
            FixedWingVehicle::C172 => builtin_c172_package()?,
            FixedWingVehicle::LightTrainer => builtin_light_trainer_package()?,
        };
        let resolver = package.build_jsbsim_resolver()?;
        let (aircraft_path, model) = package.jsbsim_model()?;
        let period = JSBSIM_DEFAULT_PERIOD;
        let fdm = JsbsimFdmBlock::new(&JsbsimFdmConfig {
            resolver: &resolver,
            aircraft_path,
            model,
            period,
            initial,
            heading_rad: 0.0,
            speed_m_s: vehicle.nominal_cruise_speed_m_s(),
            alpha_rad: 0.035,
            engine_start: EngineStartMode::Running,
        })?;
        let state = fdm.output();
        let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(state.position_ecef);
        let ecef_to_body = DMat3::from_quat(state.attitude.inverse());
        let specific_force_frd_m_s2 = -(ecef_to_body * g_ecef);
        Ok(Self {
            fdm,
            period,
            specific_force_frd_m_s2,
            utc: UtcTime::from_unix_seconds(0.0),
        })
    }

    /// Integration step period (`120 Hz` default).
    #[must_use]
    pub const fn period(&self) -> TickSpan {
        self.period
    }

    /// Current `VehicleState` in `f64` ECEF.
    #[must_use]
    pub fn state(&self) -> VehicleState {
        self.fdm.output()
    }

    /// Current body FRD specific force (`m/s²`).
    #[must_use]
    pub const fn specific_force_frd_m_s2(&self) -> DVec3 {
        self.specific_force_frd_m_s2
    }

    /// Advances JSBSim by one step under `controls` and updates body specific force.
    pub fn step(&mut self, controls: ControlInputs) -> Result<VehicleState, IronbirdError> {
        Block::step(self, &controls)?;
        Ok(self.fdm.output())
    }

    /// Builds a Tier-2 [`XilStimulus`] from the current state and standard atmosphere.
    pub fn xil_stimulus(&self) -> Result<XilStimulus, IronbirdError> {
        let state = self.fdm.output();
        let geo = ecef_to_geodetic(state.position_ecef);
        let atm: AtmosphereSample = StandardAtmosphere.sample(&geo, self.utc)?;
        Ok(XilStimulus::from_state_and_atmosphere(
            state,
            self.specific_force_frd_m_s2,
            &atm,
        )?)
    }
}

/// Complete state of a [`JsbsimPlant`], including the specific force and
/// atmosphere clock its sensors read.
#[derive(Debug, Clone, PartialEq)]
pub struct JsbsimPlantSnapshot {
    fdm: JsbsimFdmSnapshot,
    specific_force_frd_m_s2: DVec3,
    utc: UtcTime,
}

impl Block for JsbsimPlant {
    type Input = ControlInputs;
    type Output = VehicleState;
    type Snapshot = JsbsimPlantSnapshot;

    fn period(&self) -> TickSpan {
        self.period
    }

    fn step(&mut self, controls: &ControlInputs) -> Result<(), BlockError> {
        let prev = self.fdm.output();
        self.fdm.step(&KinematicInput {
            controls: *controls,
        })?;
        let next = self.fdm.output();
        let dt_s = self.period.as_seconds();
        let a_ecef = (next.velocity_ecef - prev.velocity_ecef) / dt_s;
        let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(next.position_ecef);
        let ecef_to_body = DMat3::from_quat(next.attitude.inverse());
        self.specific_force_frd_m_s2 = ecef_to_body * (a_ecef - g_ecef);
        self.utc = self.utc.advanced_by(dt_s);
        Ok(())
    }

    fn output(&self) -> VehicleState {
        self.fdm.output()
    }

    fn snapshot(&self) -> JsbsimPlantSnapshot {
        JsbsimPlantSnapshot {
            fdm: self.fdm.snapshot(),
            specific_force_frd_m_s2: self.specific_force_frd_m_s2,
            utc: self.utc,
        }
    }

    fn restore(&mut self, snapshot: &JsbsimPlantSnapshot) {
        self.fdm.restore(&snapshot.fdm);
        self.specific_force_frd_m_s2 = snapshot.specific_force_frd_m_s2;
        self.utc = snapshot.utc;
    }
}

/// Closed-loop 6-DoF rigid-body X500 quadrotor plant (`400 Hz` default).
pub struct X500RigidPlant {
    body: RigidBody6DofBlock,
    rotors: RotorLayout,
    period: TickSpan,
    utc: UtcTime,
}

impl X500RigidPlant {
    /// Creates an X500 quadrotor resting level at `origin` with `400 Hz` RK4 step.
    pub fn new_at(origin: Geodetic) -> Result<Self, IronbirdError> {
        let period = TickSpan::from_ticks(TICKS_PER_SECOND / 400);
        let g = normal_gravity(GeodeticPosition::wgs84(
            origin.lat_rad.to_degrees(),
            origin.lon_rad.to_degrees(),
            origin.height_m,
        ))
        .unwrap_or(9.806_65);
        let mass_kg = 20.25 / g;
        let spec = ComponentSpec {
            name: "X500 Airframe".to_owned(),
            placement: Placement::ROOT,
            dry_mass_kg: mass_kg,
            dry_inertia_body: DMat3::from_diagonal(DVec3::new(0.029, 0.029, 0.055)),
            thermal_capacity_j_k: 0.0,
            nominal_power_w: 0.0,
        };
        let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))])?;
        let cfg = RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period,
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: VehicleState::level_flight(&origin, 0.0, 0.0),
        };
        let body = RigidBody6DofBlock::new(
            cfg,
            tree,
            Box::new(VacuumAero),
            Box::new(StandardAtmosphere),
        )?;
        Ok(Self {
            body,
            rotors: RotorLayout::x500(),
            period,
            utc: UtcTime::from_unix_seconds(0.0),
        })
    }

    /// Integration period (`400 Hz`).
    #[must_use]
    pub const fn period(&self) -> TickSpan {
        self.period
    }

    /// Rotor geometry and thrust curves.
    #[must_use]
    pub const fn rotors(&self) -> &RotorLayout {
        &self.rotors
    }

    /// Current `VehicleState`.
    #[must_use]
    pub fn state(&self) -> VehicleState {
        self.body.output()
    }

    /// Current root-component [`LocalSiteKinematics`].
    pub fn site_kinematics(&self) -> Result<LocalSiteKinematics, IronbirdError> {
        Ok(self.body.site_kinematics(ComponentId(0))?)
    }

    /// Steps the 6-DoF RK4 plant under `thrusters`.
    pub fn step(&mut self, thrusters: Vec<ThrusterWrench>) -> Result<VehicleState, IronbirdError> {
        Block::step(self, &thrusters)?;
        Ok(self.body.output())
    }

    /// Builds a Tier-2 [`XilStimulus`] from the quadrotor's current state.
    pub fn xil_stimulus(&self) -> Result<XilStimulus, IronbirdError> {
        let state = self.body.output();
        let geo = ecef_to_geodetic(state.position_ecef);
        let atm = StandardAtmosphere.sample(&geo, self.utc)?;
        Ok(XilStimulus::from_state_and_atmosphere(
            state,
            self.body.specific_force_cg_body(),
            &atm,
        )?)
    }
}

/// Complete state of an [`X500RigidPlant`], including its atmosphere clock.
#[derive(Debug, Clone, PartialEq)]
pub struct X500PlantSnapshot {
    body: RigidBodySnapshot,
    utc: UtcTime,
}

impl Block for X500RigidPlant {
    type Input = Vec<ThrusterWrench>;
    type Output = VehicleState;
    type Snapshot = X500PlantSnapshot;

    fn period(&self) -> TickSpan {
        self.period
    }

    fn step(&mut self, thrusters: &Vec<ThrusterWrench>) -> Result<(), BlockError> {
        self.body.step(&RigidBodyInput {
            thrusters: thrusters.clone(),
            ..RigidBodyInput::default()
        })?;
        self.utc = self.utc.advanced_by(self.period.as_seconds());
        Ok(())
    }

    fn output(&self) -> VehicleState {
        self.body.output()
    }

    fn snapshot(&self) -> X500PlantSnapshot {
        X500PlantSnapshot {
            body: self.body.snapshot(),
            utc: self.utc,
        }
    }

    fn restore(&mut self, snapshot: &X500PlantSnapshot) {
        self.body.restore(&snapshot.body);
        self.utc = snapshot.utc;
    }
}
