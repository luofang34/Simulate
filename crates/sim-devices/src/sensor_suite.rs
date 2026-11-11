//! One vehicle's sensors, declared once. Every consumer aboard — the flight
//! controller, the navigator — takes the samples this suite produces, so
//! they see one realisation of each device, and nothing aboard sees the
//! truth the devices sense.

use glam::DVec3;
use sim_atmosphere::AtmosphereSample;
use sim_core::VehicleState;
use sim_geo::{LocalFrame, ecef_to_geodetic};

use crate::{
    AirDataSample, AirDataSystem, AirFlow, DeviceError, GnssErrorSpec, GnssReading, GnssReceiver,
    ImuErrorModel, ImuErrorSpec, InertialIncrement, MagneticEnvironment, Magnetometer,
    MagnetometerSpec, PitotStatic, PressureErrorSpec,
};

#[cfg(test)]
mod tests;

/// Offsets that give each device its own noise stream from one suite seed.
const IMU_STREAM: u64 = 0x1D5A_11CE;
const AIR_DATA_STREAM: u64 = 0x0A1D_DA7A;
const MAGNETOMETER_STREAM: u64 = 0x3A6F_1E1D;
const RECEIVER_STREAM: u64 = 0x6E55_F1C5;

/// What the world presents to a vehicle's devices at one instant: truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensedTruth {
    /// The vehicle's true state.
    pub vehicle: VehicleState,
    /// True specific force at the IMU, body forward-right-down axes, m/s².
    pub specific_force_body_m_s2: DVec3,
    /// True angular rate relative to inertial space, body axes, rad/s.
    pub angular_rate_body_rad_s: DVec3,
    /// The air at the vehicle.
    pub air: AtmosphereSample,
}

/// The samples a vehicle's devices produced in one cycle of its fastest
/// device. A device that produced none this cycle is `None`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DeviceSamples {
    /// What the IMU integrated over the cycle, body axes.
    pub imu: Option<InertialIncrement>,
    /// The air-data system's pressures.
    pub air_data: Option<AirDataSample>,
    /// The magnetometer's reading, body axes, microtesla.
    pub magnetic_field_body_ut: Option<DVec3>,
    /// The receiver's fix.
    pub gnss: Option<GnssReading>,
}

/// The devices a vehicle carries and how often each samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorSuiteSpec {
    /// The IMU, which samples every cycle.
    pub imu: ImuErrorSpec,
    /// The static port's pressure sensor.
    pub static_port: PressureErrorSpec,
    /// The pitot probe's pressure sensor and the body axis it looks along,
    /// on a vehicle that has one.
    pub pitot: Option<(PressureErrorSpec, DVec3)>,
    /// The magnetometer.
    pub magnetometer: MagnetometerSpec,
    /// The GNSS receiver.
    pub gnss: GnssErrorSpec,
    /// The magnetic field the vehicle flies in.
    pub magnetic: MagneticEnvironment,
    /// Cycles between air-data samples.
    pub air_data_every: u32,
    /// Cycles between magnetometer samples.
    pub magnetometer_every: u32,
    /// Cycles between GNSS fixes.
    pub gnss_every: u32,
    /// Seed of every device's errors.
    pub seed: u64,
}

/// A vehicle's devices, sensing truth cycle by cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorSuite {
    spec: SensorSuiteSpec,
    imu: ImuErrorModel,
    air_data: PitotStatic,
    magnetometer: Magnetometer,
    receiver: GnssReceiver,
    cycle: u64,
}

impl SensorSuite {
    /// The devices of `spec` at turn-on.
    pub fn new(spec: &SensorSuiteSpec) -> Result<Self, DeviceError> {
        let rates = [
            ("air_data_every", spec.air_data_every),
            ("magnetometer_every", spec.magnetometer_every),
            ("gnss_every", spec.gnss_every),
        ];
        if let Some(&(field, _)) = rates.iter().find(|(_, every)| *every == 0) {
            return Err(DeviceError::InvalidStatistic {
                device: "sensor suite",
                field,
                value: 0.0,
            });
        }
        let stream = |offset: u64| spec.seed.wrapping_add(offset);
        let air_data = match spec.pitot {
            Some((pitot, axis_body)) => PitotStatic::with_pitot(
                spec.static_port,
                pitot,
                axis_body,
                stream(AIR_DATA_STREAM),
            )?,
            None => PitotStatic::static_only(spec.static_port, stream(AIR_DATA_STREAM))?,
        };
        Ok(Self {
            spec: *spec,
            imu: ImuErrorModel::new(spec.imu, stream(IMU_STREAM))?,
            air_data,
            magnetometer: Magnetometer::new(spec.magnetometer, stream(MAGNETOMETER_STREAM))?,
            receiver: GnssReceiver::new(spec.gnss, stream(RECEIVER_STREAM))?,
            cycle: 0,
        })
    }

    /// What the suite was declared as.
    pub const fn spec(&self) -> &SensorSuiteSpec {
        &self.spec
    }

    /// The inertial unit, to read its biases.
    pub const fn imu(&self) -> &ImuErrorModel {
        &self.imu
    }

    /// The inertial unit, to read or set its biases.
    pub const fn imu_mut(&mut self) -> &mut ImuErrorModel {
        &mut self.imu
    }

    /// The GNSS receiver, to take the sky from it or displace its fixes.
    pub const fn receiver_mut(&mut self) -> &mut GnssReceiver {
        &mut self.receiver
    }

    /// One cycle of `cycle_s` seconds ending at `truth`: the IMU's increment
    /// over it, and a sample from each slower device whose turn it is.
    pub fn sense(
        &mut self,
        truth: &SensedTruth,
        cycle_s: f64,
    ) -> Result<DeviceSamples, DeviceError> {
        let spec = self.spec;
        let due = |every: u32| self.cycle.is_multiple_of(u64::from(every));
        let period = |every: u32| cycle_s * f64::from(every);
        let (air_due, magnetometer_due, gnss_due) = (
            due(spec.air_data_every),
            due(spec.magnetometer_every),
            due(spec.gnss_every),
        );
        self.cycle = self.cycle.wrapping_add(1);
        let imu = self.imu.sense(&InertialIncrement {
            dt_s: cycle_s,
            delta_theta_rad: truth.angular_rate_body_rad_s * cycle_s,
            delta_velocity_m_s: truth.specific_force_body_m_s2 * cycle_s,
        })?;
        let air_data = air_due.then(|| {
            let flow = AirFlow::of(&truth.air, &truth.vehicle);
            self.air_data.sense(&flow, period(spec.air_data_every))
        });
        let magnetic_field_body_ut = match magnetometer_due {
            true => Some(self.sense_field(&truth.vehicle)?),
            false => None,
        };
        let gnss = match gnss_due {
            true => self
                .receiver
                .measure(&truth.vehicle, period(spec.gnss_every)),
            false => None,
        };
        Ok(DeviceSamples {
            imu: Some(imu),
            air_data,
            magnetic_field_body_ut,
            gnss,
        })
    }

    fn sense_field(&mut self, vehicle: &VehicleState) -> Result<DVec3, DeviceError> {
        let position = ecef_to_geodetic(vehicle.position_ecef);
        let field_ned = self.spec.magnetic.field_ned_ut(&position)?;
        let field_ecef = LocalFrame::at(&position).ned_to_ecef() * field_ned;
        Ok(self
            .magnetometer
            .sense(vehicle.attitude.inverse() * field_ecef))
    }
}
