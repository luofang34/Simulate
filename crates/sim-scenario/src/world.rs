use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_scenery::{LunarTranquilityScenery, SyntheticCorridorSpec};

#[cfg(feature = "render")]
mod scene;

#[cfg(feature = "render")]
pub use scene::WorldScene;

/// A fixed sun: roughly mid-morning over the Pacific coast of North America.
const EARTH_SUN_DIRECTION_ECEF: DVec3 = DVec3::new(-0.2, -0.9, 0.4);
/// Mid-morning over Delaware, degrees: clockwise from north, and elevation.
const DELAWARE_SUN_AZIMUTH_DEG: f64 = 120.0;
const DELAWARE_SUN_ELEVATION_DEG: f64 = 40.0;
/// Eagle's Sun, degrees: due east, and elevation.
const LUNAR_SUN_AZIMUTH_DEG: f64 = 90.0;
const LUNAR_SUN_ELEVATION_DEG: f64 = 10.8;

/// The terrain a scenario flies over.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ScenarioWorld {
    /// The bare WGS84 globe.
    EarthGlobe,
    /// The displaced, ortho-draped synthetic Delaware corridor on WGS84.
    DelawareCorridor,
    /// Central Jersey Regional Airport (`47N`, uncontrolled CTAF) graded plateau on WGS84.
    CentralJersey47N,
    /// Newark Liberty International Airport (`KEWR`, Class B towered) graded plateau on WGS84.
    NewarkKewr,
    /// The cratered Sea of Tranquility on the IAU Moon.
    LunarTranquility,
    /// The bare IAU Mars spheroid.
    MarsGlobe,
}

/// The direction of a Sun at `azimuth_deg`, `elevation_deg` above `site`.
fn sun_above(site: &Geodetic, azimuth_deg: f64, elevation_deg: f64) -> DVec3 {
    let frame = LocalFrame::at(site);
    let (sin_el, cos_el) = elevation_deg.to_radians().sin_cos();
    let (sin_az, cos_az) = azimuth_deg.to_radians().sin_cos();
    frame.up() * sin_el + (frame.north() * cos_az + frame.east() * sin_az) * cos_el
}

impl ScenarioWorld {
    /// The body the world is on.
    pub const fn body(self) -> BodyEllipsoid {
        match self {
            Self::EarthGlobe
            | Self::DelawareCorridor
            | Self::CentralJersey47N
            | Self::NewarkKewr => BodyEllipsoid::WGS84,
            Self::LunarTranquility => BodyEllipsoid::MOON_IAU,
            Self::MarsGlobe => BodyEllipsoid::MARS_IAU,
        }
    }

    /// Direction towards the Sun, body-fixed.
    pub fn sun_direction_ecef(self) -> DVec3 {
        match self {
            Self::EarthGlobe | Self::MarsGlobe => EARTH_SUN_DIRECTION_ECEF,
            Self::DelawareCorridor => sun_above(
                &SyntheticCorridorSpec::delaware().south_west,
                DELAWARE_SUN_AZIMUTH_DEG,
                DELAWARE_SUN_ELEVATION_DEG,
            ),
            Self::CentralJersey47N => sun_above(
                &SyntheticCorridorSpec::central_jersey_47n().south_west,
                DELAWARE_SUN_AZIMUTH_DEG,
                DELAWARE_SUN_ELEVATION_DEG,
            ),
            Self::NewarkKewr => sun_above(
                &SyntheticCorridorSpec::newark_kewr().south_west,
                DELAWARE_SUN_AZIMUTH_DEG,
                DELAWARE_SUN_ELEVATION_DEG,
            ),
            Self::LunarTranquility => sun_above(
                &LunarTranquilityScenery::apollo_11_landing().at_local(0.0, 0.0),
                LUNAR_SUN_AZIMUTH_DEG,
                LUNAR_SUN_ELEVATION_DEG,
            ),
        }
    }
}
