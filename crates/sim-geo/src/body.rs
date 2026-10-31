use glam::DVec3;

use crate::{Geodetic, WGS84_A, WGS84_F};

#[cfg(test)]
mod tests;

/// Horizontal distance from the polar axis below which a point is treated as
/// on the axis, where longitude is undefined and the closed form divides by
/// zero.
const POLAR_AXIS_EPSILON_M: f64 = 1e-6;

/// Earth's second zonal harmonic, the WGS84 dynamic form factor.
pub const WGS84_J2: f64 = 1.082_629_821e-3;

/// A celestial body's reference ellipsoid and central gravity field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyEllipsoid {
    /// Equatorial radius, metres.
    pub semi_major_m: f64,
    /// Flattening; exactly `0.0` for a sphere.
    pub flattening: f64,
    /// Standard gravitational parameter, m³/s².
    pub mu_m3_s2: f64,
    /// Sidereal rotation rate, rad/s.
    pub omega_rad_s: f64,
}

impl BodyEllipsoid {
    /// Earth on the WGS84 ellipsoid.
    pub const WGS84: Self = Self {
        semi_major_m: WGS84_A,
        flattening: WGS84_F,
        mu_m3_s2: 3.986_004_418e14,
        omega_rad_s: 7.292_115e-5,
    };

    /// The Moon as the IAU mean sphere.
    pub const MOON_IAU: Self = Self {
        semi_major_m: 1_737_400.0,
        flattening: 0.0,
        mu_m3_s2: 4.904_869_5e12,
        omega_rad_s: 2.661_699_5e-6,
    };

    /// Polar radius, metres.
    pub const fn semi_minor_m(&self) -> f64 {
        self.semi_major_m * (1.0 - self.flattening)
    }

    /// First eccentricity squared.
    pub const fn eccentricity_squared(&self) -> f64 {
        self.flattening * (2.0 - self.flattening)
    }

    /// Converts a geodetic position to body-fixed metres.
    pub fn geodetic_to_fixed(&self, position: &Geodetic) -> DVec3 {
        let e2 = self.eccentricity_squared();
        let (sin_lat, cos_lat) = position.lat_rad.sin_cos();
        let (sin_lon, cos_lon) = position.lon_rad.sin_cos();
        let prime_vertical = self.semi_major_m / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        let horizontal = (prime_vertical + position.height_m) * cos_lat;
        DVec3::new(
            horizontal * cos_lon,
            horizontal * sin_lon,
            (prime_vertical * (1.0 - e2) + position.height_m) * sin_lat,
        )
    }

    /// Converts body-fixed metres to a geodetic position. Spheres are solved
    /// directly; spheroids use Heikkinen's closed form, which is
    /// millimetre-exact from the body's core to well beyond orbit and needs no
    /// iteration.
    pub fn fixed_to_geodetic(&self, fixed: DVec3) -> Geodetic {
        let p = fixed.x.hypot(fixed.y);
        if self.flattening == 0.0 {
            return Geodetic {
                lat_rad: fixed.z.atan2(p),
                lon_rad: fixed.y.atan2(fixed.x),
                height_m: fixed.length() - self.semi_major_m,
            };
        }
        if p < POLAR_AXIS_EPSILON_M {
            let lat_rad = if fixed.z >= 0.0 {
                std::f64::consts::FRAC_PI_2
            } else {
                -std::f64::consts::FRAC_PI_2
            };
            return Geodetic {
                lat_rad,
                lon_rad: 0.0,
                height_m: fixed.z.abs() - self.semi_minor_m(),
            };
        }
        self.heikkinen(fixed, p)
    }

    /// Heikkinen's closed form for a spheroid, off the polar axis.
    fn heikkinen(&self, fixed: DVec3, p: f64) -> Geodetic {
        let a = self.semi_major_m;
        let b = self.semi_minor_m();
        let z = fixed.z;
        let a2 = a * a;
        let b2 = b * b;
        let e2 = self.eccentricity_squared();
        let ep2 = (a2 - b2) / b2;
        let z2 = z * z;
        let p2 = p * p;

        let f = 54.0 * b2 * z2;
        let g = p2 + (1.0 - e2) * z2 - e2 * (a2 - b2);
        let c = e2 * e2 * f * p2 / (g * g * g);
        let s = (1.0 + c + (c * c + 2.0 * c).sqrt()).cbrt();
        let k = s + 1.0 + 1.0 / s;
        let big_p = f / (3.0 * k * k * g * g);
        let q = (1.0 + 2.0 * e2 * e2 * big_p).sqrt();
        let r0_radicand = 0.5 * a2 * (1.0 + 1.0 / q)
            - big_p * (1.0 - e2) * z2 / (q * (1.0 + q))
            - 0.5 * big_p * p2;
        let r0 = -(big_p * e2 * p) / (1.0 + q) + r0_radicand.max(0.0).sqrt();
        let dp = p - e2 * r0;
        let u = (dp * dp + z2).sqrt();
        let v = (dp * dp + (1.0 - e2) * z2).sqrt();
        let z0 = b2 * z / (a * v);

        Geodetic {
            lat_rad: (z + ep2 * z0).atan2(p),
            lon_rad: fixed.y.atan2(fixed.x),
            height_m: u * (1.0 - b2 / (a * v)),
        }
    }

    /// Central-body gravitational acceleration `-μ r / |r|³`, m/s², in the
    /// body-fixed frame. Excludes the centrifugal term of the rotating frame;
    /// the body's centre, where the field is singular, returns zero.
    pub fn gravity_fixed(&self, fixed: DVec3) -> DVec3 {
        let r2 = fixed.length_squared();
        if r2 == 0.0 {
            return DVec3::ZERO;
        }
        fixed * (-self.mu_m3_s2 / (r2 * r2.sqrt()))
    }

    /// [`Self::gravity_fixed`] plus the oblateness perturbation of the
    /// second zonal harmonic `j2`, such as [`WGS84_J2`].
    pub fn gravity_fixed_j2(&self, fixed: DVec3, j2: f64) -> DVec3 {
        let r2 = fixed.length_squared();
        if r2 == 0.0 {
            return DVec3::ZERO;
        }
        let oblateness = 1.5 * j2 * self.semi_major_m * self.semi_major_m / r2;
        let sin2_lat = fixed.z * fixed.z / r2;
        let equatorial = 1.0 - oblateness * (5.0 * sin2_lat - 1.0);
        let axial = 1.0 - oblateness * (5.0 * sin2_lat - 3.0);
        self.gravity_fixed(fixed) * DVec3::new(equatorial, equatorial, axial)
    }
}
