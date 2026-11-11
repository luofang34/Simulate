//! Simulated devices' samples as the flight controller's own HAL readings.
//!
//! The flight controller is given what a board's drivers would give it and
//! nothing else: no truth sample crosses here.

use aviate_hal_io::{
    GnssFix, RawAirspeedReading, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading,
};
use aviate_xil_direct::{DirectHostSensors, DirectSensorSample};
use glam::DVec3;
use sim_devices::{DeviceSamples, GnssReading};
use sim_geo::geodetic_to_ecef;

use crate::NedWorld;

#[cfg(test)]
mod tests;

/// Satellites a receiver with a fix reports. The receiver model has no
/// constellation, so this is the count of a healthy open-sky receiver.
const OPEN_SKY_SATELLITES: u8 = 12;
const KELVIN_AT_ZERO_CELSIUS: f64 = 273.15;

fn f32_array(value: DVec3) -> [f32; 3] {
    value.to_array().map(|component| component as f32)
}

impl NedWorld {
    /// `samples`, taken at `time_us`, as host sensor input. Every sample
    /// carries that time: a late sample is delivered late, not back-dated.
    #[must_use]
    pub fn host_sensors(
        &self,
        time_us: u64,
        reset_generation: u32,
        samples: &DeviceSamples,
    ) -> DirectHostSensors {
        let stamped = |reading| DirectSensorSample::new(time_us, reading);
        let air = samples.air_data;
        let celsius = |kelvin: f64| (kelvin - KELVIN_AT_ZERO_CELSIUS) as f32;
        DirectHostSensors {
            time_us,
            reset_generation,
            imu: samples.imu.map(|increment| {
                stamped(RawImuReading {
                    accel: f32_array(increment.delta_velocity_m_s / increment.dt_s),
                    gyro: f32_array(increment.delta_theta_rad / increment.dt_s),
                    temperature: None,
                })
            }),
            baro: air.map(|air| {
                DirectSensorSample::new(
                    time_us,
                    RawBaroReading {
                        pressure_pa: air.static_pressure_pa as f32,
                        temperature_c: celsius(air.static_temperature_k),
                        ..RawBaroReading::default()
                    },
                )
            }),
            mag: samples.magnetic_field_body_ut.map(|field| {
                DirectSensorSample::new(
                    time_us,
                    RawMagReading {
                        field_ut: f32_array(field),
                    },
                )
            }),
            gnss: samples
                .gnss
                .map(|fix| DirectSensorSample::new(time_us, self.gnss_reading(&fix))),
            airspeed: air.and_then(|air| {
                let differential = air.differential_pressure_pa?;
                Some(DirectSensorSample::new(
                    time_us,
                    RawAirspeedReading {
                        differential_pressure_pa: differential as f32,
                        static_pressure_pa: Some(air.static_pressure_pa as f32),
                        temperature_c: Some(celsius(air.static_temperature_k)),
                    },
                ))
            }),
            geomagnetic: None,
        }
    }

    /// A fix as the board's receiver driver reports it: geodetic, and in the
    /// flight controller's local frame about this world's origin. Height is
    /// ellipsoidal throughout, as the board's own synthesis has it.
    fn gnss_reading(&self, fix: &GnssReading) -> RawGnssReading {
        let position_ned = self
            .frame()
            .ecef_to_ned_vector(geodetic_to_ecef(&fix.position) - self.origin_ecef());
        let sigma = fix.position_sigma_ned_m;
        RawGnssReading {
            lat_deg: fix.position.lat_deg(),
            lon_deg: fix.position.lon_deg(),
            alt_m: fix.position.height_m as f32,
            position_ned: f32_array(position_ned),
            vel_ned: f32_array(fix.velocity_ned_m_s),
            fix: GnssFix::ThreeD,
            h_acc: sigma.x.hypot(sigma.y) as f32,
            v_acc: sigma.z as f32,
            satellites: OPEN_SKY_SATELLITES,
        }
    }
}
