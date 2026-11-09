//! Live astronomical sky from `Navigate`'s analytic ephemeris: Sun, Moon, eclipses, Earth's
//! shadow, twilight, and the bright starfield, evaluated each frame for the truth scene, the
//! instrument maps' relief lighting, and the fallback `GlobeScene`.

use glam::DVec3;
use sim_core::SimInstant;
use sim_geo::BodyEllipsoid;
use sim_maplibre::{
    CelestialDisc, CelestialEarthShadow, CelestialSkyInput, CelestialStar, CelestialTwilight,
};
use sim_navigate::{CelestialClock, CelestialFrame, ReferenceBody, SkyTimeMode, StarSkySample};
use sim_scenario::ScenarioProfile;

use crate::esc_menu::Step;
use crate::flight::Flight;

/// One frame's sky for the renderers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkyFrame<'a> {
    /// Unit direction to the Sun in the observer body's fixed axes.
    pub sun_direction_ecef: DVec3,
    /// The full sky for the truth scene, or `None` to light it by the Sun alone.
    pub celestial: Option<CelestialSkyInput<'a>>,
}

#[cfg(any(test, not(target_arch = "wasm32")))]
impl SkyFrame<'static> {
    /// A sky lit only by a fixed Sun direction, for deterministic headless renders.
    #[must_use]
    pub const fn fixed_sun(sun_direction_ecef: DVec3) -> Self {
        Self {
            sun_direction_ecef,
            celestial: None,
        }
    }
}

/// Minimum interval (`s`) and observer displacement (`m²`) before recomputing ephemeris.
const EPHEMERIS_MIN_DT_S: f64 = 0.10;
const EPHEMERIS_MAX_MOVE_M2: f64 = 500.0 * 500.0;

#[derive(Debug, Clone)]
struct CachedSkyEval {
    mode: SkyTimeMode,
    body: ReferenceBody,
    profile: Option<ScenarioProfile>,
    base_utc_s: Option<f64>,
    observer_fixed_m: DVec3,
    elapsed_s: f64,
    live_wall_utc_s: f64,
    frame: CelestialFrame,
}

/// Owns the ephemeris clock, the selected [`SkyTimeMode`], and the last uploaded starfield.
#[derive(Debug, Default)]
pub struct SkyRig {
    clock: CelestialClock,
    mode: SkyTimeMode,
    stars: Vec<CelestialStar>,
    cached: Option<CachedSkyEval>,
}

impl SkyRig {
    /// A rig following [`SkyTimeMode::Auto`].
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The active time mode.
    #[must_use]
    pub const fn mode(&self) -> SkyTimeMode {
        self.mode
    }

    /// Cycles the time mode (`Auto → Live UTC → Dawn → Noon → Dusk → Night`).
    pub fn cycle(&mut self, step: Step) -> SkyTimeMode {
        self.mode = match step {
            Step::Next => self.mode.next(),
            Step::Prev => self.mode.prev(),
        };
        self.mode
    }

    /// The sky over `observer_fixed_m` at the flight's instant, with stars attached only on
    /// the frames the clock refreshes them (every two seconds of sky time).
    pub fn evaluate(
        &mut self,
        flight: &Flight,
        observer_fixed_m: DVec3,
        live_wall_utc_s: f64,
    ) -> SkyFrame<'_> {
        let body = if flight.world().body() == BodyEllipsoid::MOON_IAU {
            ReferenceBody::Moon
        } else {
            ReferenceBody::Earth
        };
        let profile = flight.profile();
        let base_utc_s = flight
            .sim_epoch()
            .start_utc()
            .ok()
            .map(|utc| utc.unix_seconds());
        let elapsed_s = flight.now().duration_since(SimInstant::START).as_seconds();
        if let Some(cached) = self.cached.as_ref().filter(|c| {
            c.mode == self.mode
                && c.body == body
                && c.profile == profile
                && c.base_utc_s == base_utc_s
                && (elapsed_s - c.elapsed_s).abs() < EPHEMERIS_MIN_DT_S
                && (live_wall_utc_s - c.live_wall_utc_s).abs() < EPHEMERIS_MIN_DT_S
                && observer_fixed_m.distance_squared(c.observer_fixed_m) < EPHEMERIS_MAX_MOVE_M2
        }) {
            return SkyFrame {
                sun_direction_ecef: cached.frame.sun_direction_fixed,
                celestial: Some(sky_input(&cached.frame, None)),
            };
        }
        let frame = self.clock.evaluate(
            self.mode,
            body,
            observer_fixed_m,
            elapsed_s,
            base_utc_s,
            live_wall_utc_s,
        );
        let refreshed = frame.stars_update.as_deref().map(|stars| {
            self.stars.clear();
            self.stars.extend(stars.iter().map(celestial_star));
        });
        let stars = refreshed.map(|()| self.stars.as_slice());
        let out_sun = frame.sun_direction_fixed;
        self.cached = Some(CachedSkyEval {
            mode: self.mode,
            body,
            profile,
            base_utc_s,
            observer_fixed_m,
            elapsed_s,
            live_wall_utc_s,
            frame,
        });
        let celestial = self.cached.as_ref().map(|c| sky_input(&c.frame, stars));
        SkyFrame {
            sun_direction_ecef: out_sun,
            celestial,
        }
    }
}

/// The host's wall-clock UTC, Unix seconds (`0` if the clock reads before 1970).
#[must_use]
pub fn wall_clock_utc_s() -> f64 {
    web_time::SystemTime::now()
        .duration_since(web_time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// The scenario's historical start time, if it has one; other flights follow live UTC.
fn sky_input<'a>(
    frame: &CelestialFrame,
    stars: Option<&'a [CelestialStar]>,
) -> CelestialSkyInput<'a> {
    CelestialSkyInput {
        sun_direction_ecef: frame.sun_direction_fixed,
        sun_angular_radius_deg: frame.sun_angular_radius_deg,
        sun_visible_fraction: frame.sun_visible_fraction,
        moon: frame.moon_disc.map(|moon| CelestialDisc {
            direction_ecef: moon.direction_ecef,
            angular_radius_deg: moon.angular_radius_deg,
        }),
        earth_shadow: frame.earth_shadow.map(|shadow| CelestialEarthShadow {
            axis_ecef: shadow.direction_ecef,
            umbra_deg: shadow.umbra_radius_deg,
            penumbra_deg: shadow.penumbra_radius_deg,
            danjon: shadow.danjon,
        }),
        twilight: CelestialTwilight {
            night_end_deg: frame.twilight_night_end_deg,
            day_start_deg: frame.twilight_day_start_deg,
            night_brightness: frame.night_brightness,
        },
        stars,
    }
}

/// A catalogue star with its linear colour gamma-encoded to the sRGB `maplibre` expects.
fn celestial_star(star: &StarSkySample) -> CelestialStar {
    CelestialStar {
        direction_ecef: star.direction_ecef,
        v_mag: star.v_magnitude,
        srgb: star.linear_rgb.map(srgb_encode),
    }
}

/// The sRGB transfer function, clamped to `0..=1` (non-finite input encodes as black).
#[must_use]
pub fn srgb_encode(linear: f32) -> f32 {
    let c = if linear.is_finite() { linear } else { 0.0 };
    let encoded = if c <= 0.003_130_8 {
        12.92 * c
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    encoded.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn srgb_encoding_matches_the_transfer_function_and_clamps() {
        assert_eq!(srgb_encode(0.0), 0.0);
        assert!((srgb_encode(0.002) - 0.025_84).abs() < 1e-5);
        assert!((srgb_encode(0.214_041) - 0.5).abs() < 1e-4);
        assert!((srgb_encode(1.0) - 1.0).abs() < 1e-6);
        assert_eq!(srgb_encode(4.0), 1.0);
        assert_eq!(srgb_encode(-1.0), 0.0);
        assert_eq!(srgb_encode(f32::NAN), 0.0);
    }

    #[test]
    fn sky_follows_the_flight_and_refreshes_stars_on_the_first_frame() {
        let flight = Flight::new().expect("free flight");
        let mut rig = SkyRig::new();
        assert_eq!(rig.cycle(Step::Next), SkyTimeMode::LiveUtc);
        assert_eq!(rig.cycle(Step::Prev), SkyTimeMode::Auto);
        let observer = flight.vehicle_state().position_ecef;
        let first = rig.evaluate(&flight, observer, 1_782_864_000.0);
        assert!((first.sun_direction_ecef.length() - 1.0).abs() < 1e-6);
        let stars = first.celestial.and_then(|c| c.stars).map_or(0, <[_]>::len);
        assert!(stars > 1_000, "naked-eye starfield, got {stars}");
        let second = rig.evaluate(&flight, observer, 1_782_864_000.5);
        assert!(second.celestial.is_some_and(|c| c.stars.is_none()));
    }
}
