use glam::{DQuat, DVec3};
use mbd_core::ComponentId;
use sim_core::{BlockError, CameraMount, SensorIntrinsics, TICKS_PER_SECOND, TickSpan};
use sim_devices::{
    Arinc429Bus, Arinc429Speed, Bmp390Block, Bmp390Config, Icm42688Block, Icm42688Config,
};

use super::LoadedSyspack;
use crate::{DeviceKindSpec, MountedDeviceSpec, PackError, SyspackError};

/// Largest departure of a camera rotation's squared norm from one, the same
/// bound the component tree holds rotations to.
const UNIT_QUATERNION_TOLERANCE: f64 = 1e-9;

/// An off-the-shelf device built from a manifest with no vehicle code.
#[derive(Debug, Clone, PartialEq)]
pub enum CotsDevice {
    /// An ICM-42688 IMU.
    Icm42688(Box<Icm42688Block>),
    /// A BMP390 barometer.
    Bmp390(Bmp390Block),
    /// An ARINC 429 channel.
    Arinc429(Arinc429Bus),
}

/// A built device and where it is mounted.
#[derive(Debug, Clone, PartialEq)]
pub struct MountedCotsDevice {
    /// The manifest's device id.
    pub id: String,
    /// The component it is mounted on.
    pub site: ComponentId,
    /// The device.
    pub device: CotsDevice,
}

/// A pinhole camera as a package mounts it.
#[derive(Debug, Clone, PartialEq)]
pub struct PackageCamera {
    /// The manifest's device id.
    pub id: String,
    /// The component it is mounted on.
    pub site: ComponentId,
    /// Lever arm and orientation on the airframe.
    pub mount: CameraMount,
    /// Pinhole intrinsics.
    pub intrinsics: SensorIntrinsics,
    /// Capture period on the master clock.
    pub period: TickSpan,
}

fn invalid(device: &MountedDeviceSpec, field: &'static str, detail: String) -> PackError {
    PackError::Syspack(SyspackError::InvalidDevice {
        device: device.id.clone(),
        field,
        detail,
    })
}

fn built<T>(device: &MountedDeviceSpec, result: Result<T, BlockError>) -> Result<T, PackError> {
    result.map_err(|source| PackError::DeviceBuild {
        device: device.id.clone(),
        source: Box::new(source),
    })
}

fn period_of(device: &MountedDeviceSpec, rate_hz: u64) -> Result<TickSpan, PackError> {
    if rate_hz == 0 || !TICKS_PER_SECOND.is_multiple_of(rate_hz) {
        let detail = format!("{rate_hz} Hz does not divide the master clock");
        return Err(invalid(device, "rate_hz", detail));
    }
    Ok(TickSpan::from_ticks(TICKS_PER_SECOND / rate_hz))
}

fn camera_rotation(device: &MountedDeviceSpec, [x, y, z, w]: [f64; 4]) -> Result<DQuat, PackError> {
    let rotation = DQuat::from_xyzw(x, y, z, w);
    if rotation.is_finite() && (rotation.length_squared() - 1.0).abs() <= UNIT_QUATERNION_TOLERANCE
    {
        return Ok(rotation);
    }
    let detail = format!("{rotation} is not a unit quaternion");
    Err(invalid(device, "body_to_camera_xyzw", detail))
}

impl LoadedSyspack {
    /// Builds every off-the-shelf sensor and bus in the manifest, in manifest
    /// order. Cameras and custom devices are not built here: see
    /// [`LoadedSyspack::camera_mounts`].
    pub fn instantiate_cots_devices(&self) -> Result<Vec<MountedCotsDevice>, PackError> {
        let mut devices = Vec::new();
        let mut channels = 0..=u8::MAX;
        for spec in &self.manifest().devices {
            let device = match spec.kind {
                DeviceKindSpec::Icm42688 { seed } => {
                    let block = Icm42688Block::new(Icm42688Config::nominal(seed));
                    CotsDevice::Icm42688(Box::new(built(spec, block)?))
                }
                DeviceKindSpec::Bmp390 { bus_id, i2c_addr } => {
                    let block = Bmp390Block::new(Bmp390Config {
                        bus_id,
                        address: i2c_addr,
                        ..Bmp390Config::default()
                    });
                    CotsDevice::Bmp390(built(spec, block)?)
                }
                DeviceKindSpec::Arinc429Channel {
                    high_speed,
                    gap_bits,
                } => {
                    let speed = if high_speed {
                        Arinc429Speed::High
                    } else {
                        Arinc429Speed::Low
                    };
                    let channel = channels.next().ok_or_else(|| {
                        invalid(spec, "kind", "more than 256 ARINC 429 channels".to_owned())
                    })?;
                    let bus = Arinc429Bus::new(channel, speed);
                    CotsDevice::Arinc429(bus.with_gap_bits(u64::from(gap_bits)))
                }
                // Pitot-static plumbing is an air-data source, not a COTS bus device.
                DeviceKindSpec::PinholeCamera { .. }
                | DeviceKindSpec::PitotStatic { .. }
                | DeviceKindSpec::Custom { .. } => continue,
            };
            devices.push(MountedCotsDevice {
                id: spec.id.clone(),
                site: self.component_id(&spec.site)?,
                device,
            });
        }
        Ok(devices)
    }

    /// Every pinhole camera in the manifest, in manifest order.
    pub fn camera_mounts(&self) -> Result<Vec<PackageCamera>, PackError> {
        let mut cameras = Vec::new();
        for spec in &self.manifest().devices {
            let DeviceKindSpec::PinholeCamera {
                offset_body_m,
                body_to_camera_xyzw,
                width,
                height,
                fx,
                fy,
                cx,
                cy,
                rate_hz,
            } = spec.kind
            else {
                continue;
            };
            let intrinsics = SensorIntrinsics {
                width,
                height,
                fx,
                fy,
                cx,
                cy,
            };
            if !intrinsics.is_usable() {
                return Err(invalid(spec, "intrinsics", format!("{intrinsics:?}")));
            }
            let offset_body_m = DVec3::from_array(offset_body_m);
            if !offset_body_m.is_finite() {
                let detail = format!("{offset_body_m} is not finite");
                return Err(invalid(spec, "offset_body_m", detail));
            }
            cameras.push(PackageCamera {
                id: spec.id.clone(),
                site: self.component_id(&spec.site)?,
                mount: CameraMount {
                    offset_body_m,
                    body_to_camera: camera_rotation(spec, body_to_camera_xyzw)?,
                },
                intrinsics,
                period: period_of(spec, rate_hz)?,
            });
        }
        Ok(cameras)
    }
}
