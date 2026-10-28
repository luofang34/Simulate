use glam::DVec3;

#[cfg(test)]
mod tests;

/// WGS84 semi-major axis, metres.
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS84 semi-minor axis, metres.
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);
/// WGS84 first eccentricity squared.
pub const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

/// Horizontal distance from the polar axis below which a point is treated as
/// on the axis, where longitude is undefined and the closed form divides by
/// zero.
const POLAR_AXIS_EPSILON_M: f64 = 1e-6;

/// A geodetic position on WGS84 with height above the ellipsoid (not MSL).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geodetic {
    /// Geodetic latitude, radians, positive north.
    pub lat_rad: f64,
    /// Longitude, radians, positive east.
    pub lon_rad: f64,
    /// Height above the WGS84 ellipsoid, metres.
    pub height_m: f64,
}

impl Geodetic {
    /// Builds a position from degrees and ellipsoidal height in metres.
    pub fn from_degrees(lat_deg: f64, lon_deg: f64, height_m: f64) -> Self {
        Self {
            lat_rad: lat_deg.to_radians(),
            lon_rad: lon_deg.to_radians(),
            height_m,
        }
    }

    /// Latitude in degrees.
    pub fn lat_deg(&self) -> f64 {
        self.lat_rad.to_degrees()
    }

    /// Longitude in degrees.
    pub fn lon_deg(&self) -> f64 {
        self.lon_rad.to_degrees()
    }
}

/// Converts a geodetic position to ECEF metres.
pub fn geodetic_to_ecef(position: &Geodetic) -> DVec3 {
    let (sin_lat, cos_lat) = position.lat_rad.sin_cos();
    let (sin_lon, cos_lon) = position.lon_rad.sin_cos();
    let prime_vertical = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let horizontal = (prime_vertical + position.height_m) * cos_lat;
    DVec3::new(
        horizontal * cos_lon,
        horizontal * sin_lon,
        (prime_vertical * (1.0 - WGS84_E2) + position.height_m) * sin_lat,
    )
}

/// Converts ECEF metres to a geodetic position with Heikkinen's closed form,
/// which is millimetre-exact from the Earth's core to well beyond orbit and
/// needs no iteration.
pub fn ecef_to_geodetic(ecef: DVec3) -> Geodetic {
    let p = ecef.x.hypot(ecef.y);
    let z = ecef.z;
    if p < POLAR_AXIS_EPSILON_M {
        let lat_rad = if z >= 0.0 {
            std::f64::consts::FRAC_PI_2
        } else {
            -std::f64::consts::FRAC_PI_2
        };
        return Geodetic {
            lat_rad,
            lon_rad: 0.0,
            height_m: z.abs() - WGS84_B,
        };
    }

    let a2 = WGS84_A * WGS84_A;
    let b2 = WGS84_B * WGS84_B;
    let e2 = WGS84_E2;
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
    let r0_radicand =
        0.5 * a2 * (1.0 + 1.0 / q) - big_p * (1.0 - e2) * z2 / (q * (1.0 + q)) - 0.5 * big_p * p2;
    let r0 = -(big_p * e2 * p) / (1.0 + q) + r0_radicand.max(0.0).sqrt();
    let dp = p - e2 * r0;
    let u = (dp * dp + z2).sqrt();
    let v = (dp * dp + (1.0 - e2) * z2).sqrt();
    let z0 = b2 * z / (WGS84_A * v);

    Geodetic {
        lat_rad: (z + ep2 * z0).atan2(p),
        lon_rad: ecef.y.atan2(ecef.x),
        height_m: u * (1.0 - b2 / (WGS84_A * v)),
    }
}
