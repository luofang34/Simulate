//! Where the navigator the mission steers on places the aeroplane: the moving maps'
//! ownship on the real chain, with the uncertainty the navigator states for it.

use navigate_contract::NavigationSolution;

/// The steered navigator's position, ground velocity and stated uncertainty.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteeredFix {
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Ground velocity, `[east, north]` m/s.
    pub ground_velocity_en_m_s: [f64; 2],
    /// The horizontal one-sigma the navigator states, metres.
    pub sigma_horizontal_m: f64,
}

impl SteeredFix {
    /// The fix of `solution`.
    #[must_use]
    pub fn of(solution: &NavigationSolution) -> Self {
        let (north_m2, east_m2, _) = solution.position_cov.diagonal();
        Self {
            lat_deg: solution.position.latitude_rad.to_degrees(),
            lon_deg: solution.position.longitude_rad.to_degrees(),
            ground_velocity_en_m_s: [solution.velocity.east_mps, solution.velocity.north_mps],
            sigma_horizontal_m: (north_m2 + east_m2).sqrt(),
        }
    }

    /// The fix a fraction `phase` of the way to `next`, the navigator's following one:
    /// between two published instants the map's ownship moves as the truth beside it
    /// does, so the two stay comparable at any pace.
    #[must_use]
    pub fn toward(&self, next: &Self, phase: f64) -> Self {
        let lerp = |from: f64, to: f64| from + (to - from) * phase;
        Self {
            lat_deg: lerp(self.lat_deg, next.lat_deg),
            lon_deg: lerp(self.lon_deg, next.lon_deg),
            ground_velocity_en_m_s: self.ground_velocity_en_m_s,
            sigma_horizontal_m: lerp(self.sigma_horizontal_m, next.sigma_horizontal_m),
        }
    }
}
