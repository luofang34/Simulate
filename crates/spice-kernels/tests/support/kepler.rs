//! An analytic two-body orbit: the truth the interpolation tests sample and
//! compare against. Transcendentals go through `libm` so the reference is the
//! same on every platform.

/// Earth's gravitational parameter, km^3/s^2.
const GM_EARTH: f64 = 398_600.441_8;

/// A Keplerian ellipse about the Earth.
pub struct Orbit {
    /// Semi-major axis, km.
    pub semi_major_axis: f64,
    /// Eccentricity.
    pub eccentricity: f64,
    /// Inclination, radians.
    pub inclination: f64,
    /// Right ascension of the ascending node, radians.
    pub node: f64,
    /// Argument of periapsis, radians.
    pub periapsis: f64,
    /// Epoch of periapsis passage, TDB seconds past J2000.
    pub epoch: f64,
}

impl Orbit {
    /// Position (km) and velocity (km/s) at `et`.
    pub fn state(&self, et: f64) -> [f64; 6] {
        let (a, e) = (self.semi_major_axis, self.eccentricity);
        let mean_motion = libm::sqrt(GM_EARTH / libm::pow(a, 3.0));
        let mean_anomaly = mean_motion * (et - self.epoch);
        // Newton's iteration on Kepler's equation; a fixed count keeps the
        // result independent of any convergence test.
        let mut anomaly = mean_anomaly;
        for _ in 0..40 {
            let residual = anomaly - e * libm::sin(anomaly) - mean_anomaly;
            anomaly -= residual / (1.0 - e * libm::cos(anomaly));
        }
        let (sin_e, cos_e) = (libm::sin(anomaly), libm::cos(anomaly));
        let minor = libm::sqrt(1.0 - e * e);
        let radius = a * (1.0 - e * cos_e);
        let speed = libm::sqrt(GM_EARTH * a) / radius;
        let in_plane = [a * (cos_e - e), a * minor * sin_e];
        let in_plane_rate = [-speed * sin_e, speed * minor * cos_e];
        let [x, y, z] = self.to_inertial(in_plane);
        let [vx, vy, vz] = self.to_inertial(in_plane_rate);
        [x, y, z, vx, vy, vz]
    }

    /// Rotates a perifocal in-plane vector into the inertial frame.
    fn to_inertial(&self, [p, q]: [f64; 2]) -> [f64; 3] {
        let (sin_w, cos_w) = (libm::sin(self.periapsis), libm::cos(self.periapsis));
        let (sin_i, cos_i) = (libm::sin(self.inclination), libm::cos(self.inclination));
        let (sin_n, cos_n) = (libm::sin(self.node), libm::cos(self.node));
        let (u, v) = (cos_w * p - sin_w * q, sin_w * p + cos_w * q);
        [
            cos_n * u - sin_n * cos_i * v,
            sin_n * u + cos_n * cos_i * v,
            sin_i * v,
        ]
    }

    /// States at `count` epochs from `start`, the steps cycling through
    /// `steps` so that the spacing is unequal.
    pub fn sample(&self, start: f64, steps: &[f64], count: usize) -> (Vec<f64>, Vec<[f64; 6]>) {
        let mut epochs = Vec::with_capacity(count);
        let mut et = start;
        for step in steps.iter().cycle().take(count) {
            epochs.push(et);
            et += step;
        }
        let states = epochs.iter().map(|&epoch| self.state(epoch)).collect();
        (epochs, states)
    }
}

/// A low Earth orbit with every element away from a special case.
pub fn low_earth_orbit() -> Orbit {
    Orbit {
        semi_major_axis: 7_000.0,
        eccentricity: 0.05,
        inclination: 0.9,
        node: 1.1,
        periapsis: 0.4,
        epoch: 8.0e8,
    }
}

/// Length of the position difference (km) and of the velocity difference
/// (km/s) between two states.
pub fn difference(a: &[f64; 6], b: &[f64; 6]) -> (f64, f64) {
    let [ax, ay, az, avx, avy, avz] = *a;
    let [bx, by, bz, bvx, bvy, bvz] = *b;
    (
        norm([ax - bx, ay - by, az - bz]),
        norm([avx - bvx, avy - bvy, avz - bvz]),
    )
}

fn norm([x, y, z]: [f64; 3]) -> f64 {
    libm::sqrt(x * x + y * y + z * z)
}
