//! Live astronomical sky evaluated from the dynamics ephemeris first and falling back to
//! `Navigate`'s analytic ephemeris outside SPK coverage: Sun, Moon, eclipses, Earth's
//! shadow, twilight, and the bright starfield, evaluated each frame for the truth scene,
//! the instrument maps' relief lighting, and the fallback `GlobeScene`.

use glam::DVec3;
use sim_core::SimInstant;
use sim_geo::BodyEllipsoid;
use sim_maplibre::{CelestialDisc, CelestialEarthShadow, CelestialStar};
use sim_navigate::{CelestialClock, CelestialFrame, ReferenceBody, SkyTimeMode, StarSkySample};
use sim_render::EnvironmentFrame;
use sim_scenario::ScenarioProfile;

use crate::environment::environment_frame;
use crate::esc_menu::Step;
use crate::flight::Flight;

/// One frame's sky for the renderers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkyFrame<'a> {
    /// Sun, twilight, atmosphere, and fog: what every renderer lights and fogs the world by.
    pub environment: EnvironmentFrame,
    /// What only the truth scene draws on top, or `None` to light it by the Sun alone.
    pub bodies: Option<SkyBodies<'a>>,
}

/// The Moon, Earth's shadow, and the stars for the truth scene.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkyBodies<'a> {
    /// Lunar disc, when the Moon is drawn.
    pub moon: Option<CelestialDisc>,
    /// Earth's shadow, during a lunar eclipse.
    pub earth_shadow: Option<CelestialEarthShadow>,
    /// Replacement star list; `None` keeps the stars already uploaded.
    pub stars: Option<&'a [CelestialStar]>,
    /// Whether the sky evaluation fell back to `Navigate`'s analytic ephemeris outside SPK coverage.
    pub used_analytic_fallback: bool,
}

impl SkyFrame<'static> {
    /// A sky over `body` lit only by the Sun in a given direction.
    #[must_use]
    pub fn fixed_sun(body: BodyEllipsoid, sun_direction_ecef: DVec3) -> Self {
        Self {
            environment: environment_frame(
                &CelestialFrame::with_sun(sun_direction_ecef, 0.0),
                body,
            ),
            bodies: None,
        }
    }
}

#[cfg(any(test, not(target_arch = "wasm32")))]
impl SkyFrame<'static> {
    /// [`Self::fixed_sun`] with the scenario world's body and Sun.
    #[must_use]
    pub fn of_world(world: sim_scenario::ScenarioWorld) -> Self {
        Self::fixed_sun(world.body(), world.sun_direction_ecef())
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
        let world_body = flight.world().body();
        // The sky's own ephemeris stands on Earth or the Moon. Anywhere else a flight
        // goes, the Sun is where the dynamics have it (or the world's fixed Sun outside
        // kernel coverage).
        if world_body != BodyEllipsoid::WGS84 && world_body != BodyEllipsoid::MOON_IAU {
            let sun = flight
                .dynamics_direction_fixed(sim_orbital::NaifId::SUN, observer_fixed_m)
                .unwrap_or_else(|| flight.world().sun_direction_ecef());
            return SkyFrame::fixed_sun(world_body, sun);
        }
        let body = if world_body == BodyEllipsoid::MOON_IAU {
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
                environment: environment_frame(&cached.frame, world_body),
                bodies: Some(sky_bodies(&cached.frame, None)),
            };
        }
        let frame = self.clock.evaluate_with_ephemeris(
            &flight.celestial_observer_adapter(),
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
        let environment = environment_frame(&frame, world_body);
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
        SkyFrame {
            environment,
            bodies: self.cached.as_ref().map(|c| sky_bodies(&c.frame, stars)),
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

/// What the truth scene draws beyond the Sun, from the ephemeris frame.
fn sky_bodies<'a>(frame: &CelestialFrame, stars: Option<&'a [CelestialStar]>) -> SkyBodies<'a> {
    SkyBodies {
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
        stars,
        used_analytic_fallback: frame.used_analytic_fallback,
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
    fn the_sky_draws_the_sun_and_moon_where_the_dynamics_have_them() {
        use sim_orbital::NaifId;

        let flight = Flight::new().expect("flight");
        let observer = flight.vehicle_state().position_ecef;
        let strict = CelestialClock::evaluate_strict_with_ephemeris(
            &flight.celestial_observer_adapter(),
            ReferenceBody::Earth,
            observer,
            1_782_864_000.0,
        )
        .expect("SPK sight evaluation on Earth ITRF93 succeeds without fallback");
        assert!(!strict.used_analytic_fallback);

        let mut rig = SkyRig::new();
        let sky = rig.evaluate(&flight, observer, 1_782_864_000.0);
        let dynamics = |body| {
            flight
                .dynamics_direction_fixed(body, observer)
                .expect("inside the built-in kernels")
        };

        // The sky shows the Sun where its light arrives from, 20 arc-seconds behind where
        // the ephemeris puts it by the aberration of light.
        let sun_apart = dynamics(NaifId::SUN).angle_between(sky.environment.sun.direction_fixed);
        assert!(
            sun_apart.to_degrees() < 0.01,
            "the sky's Sun is {} degrees from the dynamics'",
            sun_apart.to_degrees()
        );
        let moon = sky.bodies.and_then(|bodies| bodies.moon).expect("a Moon");
        let moon_apart = dynamics(NaifId::MOON).angle_between(moon.direction_ecef);
        assert!(
            moon_apart.to_degrees() < 0.001,
            "the sky's Moon is {} degrees from the dynamics'",
            moon_apart.to_degrees()
        );
    }

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
        assert!((first.environment.sun.direction_fixed.length() - 1.0).abs() < 1e-6);
        let stars = first.bodies.and_then(|c| c.stars).map_or(0, <[_]>::len);
        assert!(stars > 1_000, "naked-eye starfield, got {stars}");
        let second = rig.evaluate(&flight, observer, 1_782_864_000.5);
        assert!(second.bodies.is_some_and(|c| c.stars.is_none()));
    }

    #[test]
    fn apollo_1969_loads_a_kernel_pack_for_1969_lunar_and_reentry_epochs() {
        use sim_orbital::NaifId;

        let mut flight = Flight::new().expect("flight");
        flight
            .select(Some(ScenarioProfile::Apollo11LunarLanding))
            .expect("apollo 11 landing");
        let observer = flight.vehicle_state().position_ecef;
        let sun_from_moon = flight
            .dynamics_direction_fixed(NaifId::SUN, observer)
            .expect("1969 kernel pack covers Apollo 11 landing");
        let earth_from_moon = flight
            .dynamics_direction_fixed(NaifId::EARTH, observer)
            .expect("1969 kernel pack covers Earth from Tranquility Base");
        assert!((sun_from_moon.length() - 1.0).abs() < 1e-12);
        assert!((earth_from_moon.length() - 1.0).abs() < 1e-12);

        let mut rig = SkyRig::new();
        let sky = rig.evaluate(&flight, observer, 0.0);
        let sun_apart = sun_from_moon.angle_between(sky.environment.sun.direction_fixed);
        assert!(
            sun_apart.to_degrees() < 0.01,
            "the 1969 lunar Sun is {} degrees from the sky's",
            sun_apart.to_degrees()
        );

        flight
            .select(Some(ScenarioProfile::Apollo11CapsuleReentry))
            .expect("apollo 11 reentry");
        let reentry_obs = flight.vehicle_state().position_ecef;
        assert!(
            flight
                .dynamics_direction_fixed(NaifId::SUN, reentry_obs)
                .is_some()
        );
    }

    #[test]
    fn out_of_coverage_epoch_returns_explicit_error_and_triggers_analytic_fallback() {
        let flight = Flight::new().expect("flight");
        let observer = flight.vehicle_state().position_ecef;
        let out_of_coverage_utc_s = 500_000_000.0; // 1985-11-05 UTC (outside 1969 & 2026-2027 kernels)

        let strict_err = CelestialClock::evaluate_strict_with_ephemeris(
            &flight.celestial_observer_adapter(),
            ReferenceBody::Earth,
            observer,
            out_of_coverage_utc_s,
        )
        .expect_err("1985 epoch must be rejected by strict SPK sight evaluation");
        assert!(
            format!("{strict_err:?}").contains("OutOfCoverage"),
            "expected OutOfCoverage, got {strict_err:?}"
        );

        let mut clock = CelestialClock::new();
        let fallback_frame = clock.evaluate_with_ephemeris(
            &flight.celestial_observer_adapter(),
            SkyTimeMode::LiveUtc,
            ReferenceBody::Earth,
            observer,
            0.0,
            None,
            out_of_coverage_utc_s,
        );
        assert!(
            fallback_frame.used_analytic_fallback,
            "out-of-coverage epoch must explicitly record analytic fallback"
        );
        assert!((fallback_frame.sun_direction_fixed.length() - 1.0).abs() < 1e-6);
    }
}
