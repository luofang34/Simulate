//! Text-kernel constants and the Mars IAU rotation evaluated from them.

use glam::DMat3;

/// Concatenated `\begindata` blocks of a text kernel.
#[cfg(test)]
pub fn data_blocks(text: &str) -> String {
    let mut out = String::new();
    let mut in_data = false;
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("\\begindata") {
            in_data = true;
        } else if trimmed.starts_with("\\begintext") {
            in_data = false;
        } else if in_data {
            out.push_str(line);
            out.push('\n');
        }
    }
    out
}

/// Numeric value list assigned to `key` in kernel data text.
#[cfg(test)]
pub fn kernel_values(data: &str, key: &str) -> Vec<f64> {
    let mut from = 0;
    while let Some(offset) = data[from..].find(key) {
        let at = from + offset;
        let rest = data[at + key.len()..].trim_start();
        let starts_name = at == 0 || data[..at].ends_with(char::is_whitespace);
        if starts_name && let Some(value) = rest.strip_prefix('=') {
            let value = value.trim_start();
            let body = match value.strip_prefix('(') {
                Some(list) => list.split(')').next().unwrap_or(""),
                None => value.split_whitespace().next().unwrap_or(""),
            };
            return body
                .split_whitespace()
                .map(|token| {
                    token
                        .replace(['D', 'd'], "E")
                        .parse()
                        .expect("kernel number")
                })
                .collect();
        }
        from = at + key.len();
    }
    panic!("{key} is not assigned in the text kernel");
}

/// Ellipsoid `(a, e^2)` from a `BODY<id>_RADII` entry in km.
#[cfg(test)]
pub fn pck_ellipsoid(data: &str, key: &str) -> (f64, f64) {
    let radii = kernel_values(data, key);
    let (a, c) = (radii[0] * 1.0e3, radii[2] * 1.0e3);
    let f = (a - c) / a;
    (a, f * (2.0 - f))
}

#[cfg(test)]
pub struct MarsIau {
    ra: Vec<f64>,
    dec: Vec<f64>,
    pm: Vec<f64>,
    ra_nut: Vec<f64>,
    dec_nut: Vec<f64>,
    pm_nut: Vec<f64>,
    angles: Vec<f64>,
}

#[cfg(test)]
impl MarsIau {
    pub fn from_data(data: &str) -> Self {
        let values = |key| kernel_values(data, key);
        Self {
            ra: values("BODY499_POLE_RA"),
            dec: values("BODY499_POLE_DEC"),
            pm: values("BODY499_PM"),
            ra_nut: values("BODY499_NUT_PREC_RA"),
            dec_nut: values("BODY499_NUT_PREC_DEC"),
            pm_nut: values("BODY499_NUT_PREC_PM"),
            angles: values("BODY4_NUT_PREC_ANGLES"),
        }
    }

    /// Sum of `coeff[k] * trig(theta_k)` over the phase angles at `t_cy` centuries.
    pub fn nutation(&self, coeff: &[f64], t_cy: f64, trig: fn(f64) -> f64) -> f64 {
        coeff
            .iter()
            .zip(self.angles.chunks(3))
            .map(|(c, a)| c * trig((a[0] + a[1] * t_cy + a[2] * t_cy * t_cy).to_radians()))
            .sum()
    }

    /// Pole right ascension, declination and prime meridian angle (radians) at `et`.
    pub fn pole_and_meridian(&self, et: f64) -> (f64, f64, f64) {
        let d = et / 86_400.0;
        let t = d / 36_525.0;
        let poly = |c: &[f64], x: f64| c[0] + c[1] * x + c[2] * x * x;
        let ra = poly(&self.ra, t) + self.nutation(&self.ra_nut, t, f64::sin);
        let dec = poly(&self.dec, t) + self.nutation(&self.dec_nut, t, f64::cos);
        let w = poly(&self.pm, d) + self.nutation(&self.pm_nut, t, f64::sin);
        (ra.to_radians(), dec.to_radians(), w.to_radians())
    }

    /// Rotation taking J2000 vectors to Mars-fixed: `Rz(W) Rx(pi/2 - dec) Rz(ra + pi/2)`
    /// as frame rotations, which are the transposes of glam's active rotations.
    pub fn j2000_to_fixed(&self, et: f64) -> DMat3 {
        let (ra, dec, w) = self.pole_and_meridian(et);
        let half_pi = core::f64::consts::FRAC_PI_2;
        DMat3::from_rotation_z(-w)
            * DMat3::from_rotation_x(-(half_pi - dec))
            * DMat3::from_rotation_z(-(ra + half_pi))
    }
}
