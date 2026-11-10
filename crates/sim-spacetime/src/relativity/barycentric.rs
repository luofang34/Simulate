//! Proper time of a clock anywhere in the solar system, against Barycentric
//! Dynamical Time: on the Moon, on Mars, in a halo orbit, or beside the
//! clocks near Earth that [`super::ClockPotential`] rates against TT.
//!
//! Rates follow the IAU 2000 barycentric metric to order `1/c²`: a clock
//! runs slow by the potential of every body at its position and by its own
//! barycentric speed. TDB is scaled so that a clock on Earth's geoid keeps
//! it on average, which is what `L_B` restores.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use super::{ClockPotential, SPEED_OF_LIGHT_M_S};
use crate::catalogs::{DerivativeOrder, EphemerisCatalog, EphemerisError};
use crate::ids::NaifId;
use crate::scalar_math::sqrt;
use crate::time::TdbEpoch;

/// `1 - dTDB/dTCB` (IAU 2006 Resolution B3, a defining constant).
pub const L_B: f64 = 1.550_519_768e-8;

/// The part of `TDB - TT` that depends on where a clock is, seconds:
/// `v_E · (x - x_E) / c²`, from Earth's barycentric velocity and the clock's
/// position from the geocentre. Two clocks that agree in TT are this far
/// apart in TDB, about `2 µs` across Earth and `9 µs` across a GPS orbit, so
/// proper time integrated against TDB needs it at both ends before it can
/// be compared with proper time reckoned against TT.
#[must_use]
pub fn tdb_minus_tt_position_term_s(
    earth_velocity_ssb_m_s: DVec3,
    position_from_geocentre_m: DVec3,
) -> f64 {
    earth_velocity_ssb_m_s.dot(position_from_geocentre_m)
        / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S)
}

/// A body whose gravity slows clocks.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MassiveBody {
    /// The body, or the barycentre of a planet and its moons.
    pub body: NaifId,
    /// Gravitational parameter `GM`, m³/s².
    pub gm_m3_s2: f64,
}

/// The bodies whose potential a clock feels.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BarycentricClockModel {
    /// Bodies treated as point masses.
    pub bodies: Vec<MassiveBody>,
    /// Earth with its oblateness, which matters to clocks near it.
    pub earth: ClockPotential,
    /// Mars with its oblateness (`J2 = 1.95545e-3`), which shifts polar vs equatorial surface clocks.
    pub mars: ClockPotential,
    /// Mars's rotation pole in ICRF axes (unit vector). It stands 35.6° from the ICRF
    /// pole, so the oblateness term needs it; Earth's pole is the ICRF pole to within
    /// what the term can resolve.
    pub mars_pole_icrf: [f64; 3],
}

/// Mars's pole at J2000 from the IAU 2015 report's right ascension `317.269202°` and
/// declination `54.432516°` (`pck00011.tpc`, `BODY499_POLE_RA` and `BODY499_POLE_DEC`).
/// It precesses `0.1°` a century, far below what a clock rate shows.
const MARS_POLE_ICRF: [f64; 3] = [0.427259382168775, -0.394689045359322, 0.813430991432035];

impl BarycentricClockModel {
    /// The Sun, Earth, the Moon and the planets out to Saturn, with DE440
    /// masses: the bodies the standard force model attracts with. Uranus
    /// and Neptune would each add a nearly constant `2e-14`.
    #[must_use]
    pub fn solar_system() -> Self {
        let mass = |body, gm_m3_s2| MassiveBody { body, gm_m3_s2 };
        Self {
            bodies: vec![
                mass(NaifId::SUN, 1.327_124_400_412_794_2e20),
                mass(NaifId::MERCURY_BARYCENTER, 2.203_186_855_1e13),
                mass(NaifId::VENUS_BARYCENTER, 3.248_585_92e14),
                mass(NaifId::MOON, 4.902_800_118e12),
                mass(NaifId::JUPITER_BARYCENTER, 1.267_127_641e17),
                mass(NaifId::SATURN_BARYCENTER, 3.794_058_484_18e16),
            ],
            earth: ClockPotential::EARTH,
            mars: ClockPotential::MARS,
            mars_pole_icrf: MARS_POLE_ICRF,
        }
    }

    /// `dτ/dTDB - 1` for a clock at `position_ssb_m` moving at
    /// `velocity_ssb_m_s`, both barycentric in ICRF axes. Positive means the
    /// clock gains on TDB. A clock at a body's centre takes no potential
    /// from that body.
    pub fn proper_rate_offset(
        &self,
        position_ssb_m: DVec3,
        velocity_ssb_m_s: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<f64, EphemerisError> {
        let from = |body| {
            ephemeris
                .geometric_state_icrf(body, NaifId::SSB, epoch, DerivativeOrder::Position)
                .map(|state| position_ssb_m - state.pos_m)
        };
        let mut potential = self
            .earth
            .potential_m2_s2(from(NaifId::EARTH)?)
            .unwrap_or(0.0)
            + self
                .mars
                .potential_m2_s2(about_pole(
                    from(NaifId::MARS)?,
                    DVec3::from_array(self.mars_pole_icrf),
                ))
                .unwrap_or(0.0);
        for mass in &self.bodies {
            let distance_m = sqrt(from(mass.body)?.length_squared());
            if distance_m > 0.0 {
                potential += mass.gm_m3_s2 / distance_m;
            }
        }
        let kinetic = 0.5 * velocity_ssb_m_s.length_squared();
        Ok(L_B - (potential + kinetic) / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S))
    }
}

/// `offset_m` re-expressed with `z` along `pole`: the oblateness term reads only the
/// distance and the height above the equator, so the direction round the pole is free.
fn about_pole(offset_m: DVec3, pole: DVec3) -> DVec3 {
    let along_m = offset_m.dot(pole);
    let across_m = sqrt((offset_m.length_squared() - along_m * along_m).max(0.0));
    DVec3::new(across_m, 0.0, along_m)
}
