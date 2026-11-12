//! A GNSS receiver in the position domain: each fix is the true antenna
//! position plus white noise plus a first-order Gauss–Markov error per
//! north-east-down axis, and the true velocity plus white noise. No
//! constellation, pseudorange, geometry or multipath is modelled.

use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};

use crate::{DeviceError, WhiteNoise};

#[cfg(test)]
mod tests;

/// Error statistics of a receiver.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssErrorSpec {
    /// White position noise per horizontal axis, metres.
    pub horizontal_white_sigma_m: f64,
    /// White position noise, vertical, metres.
    pub vertical_white_sigma_m: f64,
    /// Correlated position error per horizontal axis, metres.
    pub horizontal_correlated_sigma_m: f64,
    /// Correlated position error, vertical, metres.
    pub vertical_correlated_sigma_m: f64,
    /// Correlation time of the correlated error, seconds.
    pub correlation_time_s: f64,
    /// White velocity noise per axis, m/s.
    pub velocity_sigma_m_s: f64,
}

impl GnssErrorSpec {
    /// A single-frequency code receiver in open sky: decimetre-level
    /// receiver noise under metre-level slowly varying ionosphere, orbit and
    /// clock residuals, and centimetre-per-second Doppler velocity.
    pub const OPEN_SKY: Self = Self {
        horizontal_white_sigma_m: 0.6,
        vertical_white_sigma_m: 1.0,
        horizontal_correlated_sigma_m: 1.2,
        vertical_correlated_sigma_m: 2.0,
        correlation_time_s: 300.0,
        velocity_sigma_m_s: 0.05,
    };

    fn white_sigma_ned_m(&self) -> DVec3 {
        let h = self.horizontal_white_sigma_m;
        DVec3::new(h, h, self.vertical_white_sigma_m)
    }

    fn correlated_sigma_ned_m(&self) -> DVec3 {
        let h = self.horizontal_correlated_sigma_m;
        DVec3::new(h, h, self.vertical_correlated_sigma_m)
    }
}

/// One fix, with the spread the receiver declares for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssReading {
    /// Measured position of the antenna, which sits at the centre of
    /// gravity.
    pub position: Geodetic,
    /// Measured velocity, north-east-down, m/s.
    pub velocity_ned_m_s: DVec3,
    /// Declared 1-sigma of the position per north-east-down axis, metres:
    /// the white and correlated errors together.
    pub position_sigma_ned_m: DVec3,
    /// Declared 1-sigma of the velocity per axis, m/s.
    pub velocity_sigma_m_s: f64,
}

/// A spoofer that has captured the receiver and drags its fixes away: each
/// fix time it holds the receiver, the reported position leaves the true one
/// a little further at a constant rate, and the reported velocity carries
/// the same rate, so position and Doppler agree with each other. The
/// receiver declares the same spread as under the true sky.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssSpoof {
    /// The rate the reported position leaves the true one, north-east-down,
    /// m/s.
    pub drag_ned_m_s: DVec3,
}

impl GnssSpoof {
    /// A level drag of `rate_m_s` toward `bearing_rad` from true north.
    pub fn toward(rate_m_s: f64, bearing_rad: f64) -> Self {
        Self {
            drag_ned_m_s: DVec3::new(bearing_rad.cos(), bearing_rad.sin(), 0.0) * rate_m_s,
        }
    }
}

/// A receiver and whether it has the sky.
///
/// Its errors advance at every fix time whether or not it is tracking, so a
/// receiver that loses and regains the sky meets the error sequence a
/// receiver that never lost it would.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssReceiver {
    spec: GnssErrorSpec,
    noise: WhiteNoise,
    correlated_m: Option<DVec3>,
    displacement_ned_m: DVec3,
    /// The spoofer holding the receiver and how far it has dragged the
    /// fixes, north-east-down metres.
    spoof: Option<(GnssSpoof, DVec3)>,
    tracking: bool,
}

impl GnssReceiver {
    /// A tracking receiver drawing its errors from `seed`.
    pub fn new(spec: GnssErrorSpec, seed: u64) -> Result<Self, DeviceError> {
        let fields = [
            ("horizontal_white_sigma_m", spec.horizontal_white_sigma_m),
            ("vertical_white_sigma_m", spec.vertical_white_sigma_m),
            (
                "horizontal_correlated_sigma_m",
                spec.horizontal_correlated_sigma_m,
            ),
            (
                "vertical_correlated_sigma_m",
                spec.vertical_correlated_sigma_m,
            ),
            ("velocity_sigma_m_s", spec.velocity_sigma_m_s),
        ];
        let invalid = |field, value| DeviceError::InvalidStatistic {
            device: "GNSS receiver",
            field,
            value,
        };
        if let Some(&(field, value)) = fields.iter().find(|(_, v)| !(v.is_finite() && *v >= 0.0)) {
            return Err(invalid(field, value));
        }
        if !(spec.correlation_time_s.is_finite() && spec.correlation_time_s > 0.0) {
            return Err(invalid("correlation_time_s", spec.correlation_time_s));
        }
        Ok(Self {
            spec,
            noise: WhiteNoise::from_seed(seed),
            correlated_m: None,
            displacement_ned_m: DVec3::ZERO,
            spoof: None,
            tracking: true,
        })
    }

    /// The error statistics the receiver draws from, for a host declaring
    /// them to a filter.
    pub const fn spec(&self) -> GnssErrorSpec {
        self.spec
    }

    /// Whether the receiver has the sky.
    pub const fn is_tracking(&self) -> bool {
        self.tracking
    }

    /// Takes the sky from the receiver or gives it back: without it the
    /// receiver produces no fix.
    pub const fn set_tracking(&mut self, tracking: bool) {
        self.tracking = tracking;
    }

    /// Displaces every later fix by `offset_ned_m` on top of the receiver's
    /// own errors: a spoofed or mis-surveyed position, which the receiver
    /// does not declare.
    pub const fn displace(&mut self, offset_ned_m: DVec3) {
        self.displacement_ned_m = offset_ned_m;
    }

    /// Hands the receiver to `spoof`, or back to the true sky with `None`.
    /// A spoofer drags from no offset; the one already holding the receiver
    /// keeps what it has dragged, and released, the next fix is back at the
    /// true position.
    pub fn set_spoof(&mut self, spoof: Option<GnssSpoof>) {
        self.spoof = match (spoof, self.spoof) {
            (Some(new), Some((held, dragged_m))) if new == held => Some((held, dragged_m)),
            (Some(new), _) => Some((new, DVec3::ZERO)),
            (None, _) => None,
        };
    }

    /// The spoofer holding the receiver and how far it has dragged the
    /// fixes from the truth, north-east-down metres.
    pub const fn spoof(&self) -> Option<(GnssSpoof, DVec3)> {
        self.spoof
    }

    /// The fix for the vehicle in `truth`, `period_s` after the last fix
    /// time, or `None` while the receiver has no sky.
    pub fn measure(&mut self, truth: &VehicleState, period_s: f64) -> Option<GnssReading> {
        let spec = self.spec;
        let draw = self.noise.next_vec3(spec.correlated_sigma_ned_m());
        let correlated = match self.correlated_m {
            None => draw,
            Some(previous) => {
                let decay = (-period_s / spec.correlation_time_s).exp();
                previous * decay + draw * (1.0 - decay * decay).sqrt()
            }
        };
        self.correlated_m = Some(correlated);
        let white_sigma = spec.white_sigma_ned_m();
        // The spoofer drags on through a fix time without the sky and draws
        // no noise, so the receiver's own errors are those of the same
        // flight unspoofed.
        let (drag_ned_m_s, dragged_ned_m) = match self.spoof.as_mut() {
            Some((spoof, dragged_m)) => {
                *dragged_m += spoof.drag_ned_m_s * period_s;
                (spoof.drag_ned_m_s, *dragged_m)
            }
            None => (DVec3::ZERO, DVec3::ZERO),
        };
        let error_ned = correlated
            + self.noise.next_vec3(white_sigma)
            + self.displacement_ned_m
            + dragged_ned_m;
        let velocity_error =
            self.noise.next_vec3(DVec3::splat(spec.velocity_sigma_m_s)) + drag_ned_m_s;
        if !self.tracking {
            return None;
        }
        let frame = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
        let correlated_sigma = spec.correlated_sigma_ned_m();
        let variance = white_sigma * white_sigma + correlated_sigma * correlated_sigma;
        Some(GnssReading {
            position: ecef_to_geodetic(truth.position_ecef + frame.ned_to_ecef() * error_ned),
            velocity_ned_m_s: frame.ecef_to_ned_vector(truth.velocity_ecef) + velocity_error,
            position_sigma_ned_m: DVec3::new(
                variance.x.sqrt(),
                variance.y.sqrt(),
                variance.z.sqrt(),
            ),
            velocity_sigma_m_s: spec.velocity_sigma_m_s,
        })
    }
}
