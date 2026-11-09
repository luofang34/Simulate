//! Sky state for rendering: Sun, Moon, eclipse obscuration, Earth's shadow,
//! and the bright stars, from Navigate's kernel-free analytic ephemeris.
//!
//! Directions are unit vectors in the observer's body-fixed axes (ECEF on the
//! Earth, IAU mean-Earth/polar-axis axes on the Moon) so the renderer needs no
//! inertial frame of its own.

mod camera_stars;
mod earth;
mod frame;
mod moon;
mod stars;
mod time;

pub use camera_stars::{camera_stars, star_camera_epoch};
pub use frame::{CelestialFrame, EarthShadowSample, MoonDiscSample, StarSkySample};

use glam::DVec3;
use navigate_celestial::Epoch;
use sim_geo::BodyEllipsoid;

/// Faintest catalogue star handed to the renderer, Johnson V. About 5000
/// stars, which is roughly the naked-eye sky under a dark site.
pub const FAINTEST_STAR_V_MAGNITUDE: f64 = 6.0;

/// Minimum UTC change between star uploads. Stars drift 0.004° per second of
/// Earth rotation, far below a pixel at typical fields of view in 2 s.
const STAR_REFRESH_S: f64 = 2.0;

/// Which clock the sky follows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SkyTimeMode {
    /// The scenario's start time plus simulated time, else live UTC.
    #[default]
    Auto,
    /// The host's wall-clock UTC.
    LiveUtc,
    /// 06:05 local mean solar time.
    Dawn,
    /// 12:00 local mean solar time.
    Noon,
    /// 18:10 local mean solar time.
    Dusk,
    /// 23:30 local mean solar time.
    Night,
}

impl SkyTimeMode {
    /// Every mode in cycling order.
    pub const ALL: [Self; 6] = [
        Self::Auto,
        Self::LiveUtc,
        Self::Dawn,
        Self::Noon,
        Self::Dusk,
        Self::Night,
    ];

    /// Short human-readable name.
    pub const fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto",
            Self::LiveUtc => "Live UTC",
            Self::Dawn => "Dawn",
            Self::Noon => "Noon",
            Self::Dusk => "Dusk",
            Self::Night => "Night",
        }
    }

    /// The following mode, wrapping from the last to the first.
    pub const fn next(self) -> Self {
        match self {
            Self::Auto => Self::LiveUtc,
            Self::LiveUtc => Self::Dawn,
            Self::Dawn => Self::Noon,
            Self::Noon => Self::Dusk,
            Self::Dusk => Self::Night,
            Self::Night => Self::Auto,
        }
    }

    /// The preceding mode, wrapping from the first to the last.
    pub const fn prev(self) -> Self {
        match self {
            Self::Auto => Self::Night,
            Self::LiveUtc => Self::Auto,
            Self::Dawn => Self::LiveUtc,
            Self::Noon => Self::Dawn,
            Self::Dusk => Self::Noon,
            Self::Night => Self::Dusk,
        }
    }
}

/// The body whose fixed axes the observer position and outputs use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReferenceBody {
    /// WGS84 Earth, ECEF axes.
    #[default]
    Earth,
    /// The IAU mean lunar sphere, Moon-fixed axes.
    Moon,
}

impl ReferenceBody {
    /// The body's reference ellipsoid.
    pub const fn ellipsoid(self) -> BodyEllipsoid {
        match self {
            Self::Earth => BodyEllipsoid::WGS84,
            Self::Moon => BodyEllipsoid::MOON_IAU,
        }
    }
}

/// Evaluates the sky each frame and throttles star uploads.
///
/// Holds the last successful frame so an ephemeris refusal (non-finite input
/// or an epoch outside -2000..3000) repeats the last sky instead of failing a
/// render frame.
#[derive(Debug, Clone, Default)]
pub struct CelestialClock {
    last_star_utc_s: Option<f64>,
    last_star_body: Option<ReferenceBody>,
    last_frame: Option<CelestialFrame>,
}

impl CelestialClock {
    /// A clock that has emitted no stars yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// The sky at the instant `mode` selects, seen from `observer_fixed_m`
    /// (body-fixed metres on `body`).
    ///
    /// `scenario_base_utc_s` is the scenario's start as Unix seconds, if it has
    /// one; `live_wall_utc_s` is the host's wall clock, used by
    /// [`SkyTimeMode::LiveUtc`], by [`SkyTimeMode::Auto`] without a scenario
    /// time, and for the calendar date of the local-time presets.
    pub fn evaluate(
        &mut self,
        mode: SkyTimeMode,
        body: ReferenceBody,
        observer_fixed_m: DVec3,
        sim_elapsed_s: f64,
        scenario_base_utc_s: Option<f64>,
        live_wall_utc_s: f64,
    ) -> CelestialFrame {
        let observer = if observer_fixed_m.is_finite() {
            observer_fixed_m
        } else {
            DVec3::ZERO
        };
        let utc = time::utc_for_mode(
            mode,
            body,
            observer,
            sim_elapsed_s,
            scenario_base_utc_s,
            live_wall_utc_s,
        );
        let refresh_stars = self.stars_due(utc, body);
        let computed = time::finite(utc).and_then(|utc| {
            let epoch = Epoch::from_unix_seconds(utc);
            let mut frame = match body {
                ReferenceBody::Earth => earth::frame(epoch, observer)?,
                ReferenceBody::Moon => moon::frame(epoch, observer)?,
            };
            frame.utc_unix_s = utc;
            if refresh_stars {
                frame.stars_update = Some(match body {
                    ReferenceBody::Earth => stars::earth_stars(epoch)?,
                    ReferenceBody::Moon => stars::moon_stars(epoch)?,
                });
            }
            Some(frame)
        });
        match computed {
            Some(frame) => {
                if frame.stars_update.is_some() {
                    self.last_star_utc_s = Some(frame.utc_unix_s);
                    self.last_star_body = Some(body);
                }
                let mut cached = frame.clone();
                cached.stars_update = None;
                self.last_frame = Some(cached);
                frame
            }
            None => self
                .last_frame
                .clone()
                .unwrap_or_else(|| CelestialFrame::fallback(utc)),
        }
    }

    fn stars_due(&self, utc: f64, body: ReferenceBody) -> bool {
        match (self.last_star_utc_s, self.last_star_body) {
            (Some(last), Some(last_body)) if last_body == body => {
                (utc - last).abs() >= STAR_REFRESH_S
            }
            _ => true,
        }
    }
}

#[cfg(test)]
mod tests;
