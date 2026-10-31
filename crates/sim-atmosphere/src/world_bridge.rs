//! Bridges `world` (`world-atmosphere` and `world-airmass`) into
//! [`AtmosphereField`] and [`CloudField`].

use glam::DVec3;
use sim_geo::{Geodetic, UtcTime};
use world_airmass::{AirmassStore, CloudGenus, WindModel, dry_air_speed_of_sound_m_s};
use world_atmosphere::{
    Altitude, AtmosphereError as WorldAtmosphereError, AtmosphereModel, AtmospherePoint, Isa,
};
use world_core::{DecimalYear, GeodeticPosition};

use crate::{
    AtmosphereError, AtmosphereField, AtmosphereSample, CloudField, CloudKind, CloudSample,
};

#[cfg(test)]
mod tests;

/// Converts [`UtcTime`] (`f64` Unix seconds) into [`DecimalYear`] with sub-second
/// linear interpolation across the enclosing whole-second boundary.
pub fn utc_to_decimal_year(utc: UtcTime) -> DecimalYear {
    let secs = utc.unix_seconds();
    if !secs.is_finite() {
        return DecimalYear(f64::NAN);
    }
    let whole = secs.floor();
    let frac = secs - whole;
    let base_i64 = whole as i64;
    let y0 = DecimalYear::from_unix_seconds(base_i64).value();
    let y1 = DecimalYear::from_unix_seconds(base_i64.saturating_add(1)).value();
    DecimalYear(y0 + (y1 - y0) * frac)
}

fn to_atmosphere_point(position: &Geodetic, utc: UtcTime) -> AtmospherePoint {
    AtmospherePoint {
        latitude_deg: position.lat_rad.to_degrees(),
        longitude_deg: position.lon_rad.to_degrees(),
        altitude: Altitude::GeometricMsl(position.height_m),
        at: utc_to_decimal_year(utc),
    }
}

fn map_world_error(err: WorldAtmosphereError) -> AtmosphereError {
    match err {
        WorldAtmosphereError::OutsideDomain {
            geopotential_m,
            lowest_m,
            highest_m,
        } => AtmosphereError::OutOfRange {
            geopotential_m,
            floor_m: lowest_m,
            ceiling_m: highest_m,
        },
    }
}

/// Adapts any [`world_atmosphere::AtmosphereModel`] into an [`AtmosphereField`]
/// with still air (`wind_ned_m_s = 0`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldAtmosphere<M>(pub M);

impl<M: AtmosphereModel> AtmosphereField for WorldAtmosphere<M> {
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let state = self
            .0
            .state(to_atmosphere_point(position, utc))
            .map_err(map_world_error)?;
        Ok(AtmosphereSample {
            temperature_k: state.temperature_k,
            pressure_pa: state.pressure_pa,
            density_kg_m3: state.density_kg_m3,
            speed_of_sound_m_s: dry_air_speed_of_sound_m_s(state.temperature_k),
            wind_ned_m_s: DVec3::ZERO,
        })
    }
}

impl AtmosphereField for Isa {
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        WorldAtmosphere(*self).sample(position, utc)
    }
}

impl AtmosphereField for AirmassStore {
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let sample =
            WindModel::sample(self, to_atmosphere_point(position, utc)).map_err(map_world_error)?;
        Ok(AtmosphereSample {
            temperature_k: sample.state.temperature_k,
            pressure_pa: sample.state.pressure_pa,
            density_kg_m3: sample.state.density_kg_m3,
            speed_of_sound_m_s: sample.speed_of_sound_m_s,
            wind_ned_m_s: DVec3::new(
                sample.wind_ned_m_s.north_m,
                sample.wind_ned_m_s.east_m,
                sample.wind_ned_m_s.down_m,
            ),
        })
    }
}

impl CloudField for AirmassStore {
    fn sample(&self, position: &Geodetic, utc: UtcTime) -> Option<CloudSample> {
        let pos = GeodeticPosition::wgs84(
            position.lat_rad.to_degrees(),
            position.lon_rad.to_degrees(),
            position.height_m,
        );
        let layer = self.cloud_at(pos, utc_to_decimal_year(utc))?;
        let kind = match layer.genus {
            CloudGenus::Stratiform => CloudKind::Stratiform,
            CloudGenus::Cumuliform => CloudKind::Cumuliform,
            CloudGenus::Convective => CloudKind::Convective,
            CloudGenus::Cirriform => CloudKind::Cirriform,
            _ => CloudKind::Stratiform,
        };
        Some(CloudSample {
            coverage: layer.coverage.clamp(0.0, 1.0) as f32,
            base_m: layer.base_msl_m as f32,
            top_m: layer.top_msl_m as f32,
            kind,
        })
    }
}
