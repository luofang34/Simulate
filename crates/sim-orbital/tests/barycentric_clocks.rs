//! Barycentric proper-time rates against the built-in DE440 ephemeris: a clock on Earth's
//! geoid keeps TDB on average, a GPS clock gains its factory offset on it, and clocks on
//! the Moon and on Mars gain what the published four-body studies give.

use glam::DVec3;
use sim_orbital::{
    BuiltinSolarSystem2026_2028, DerivativeOrder, EphemerisCatalog, NaifId, TdbEpoch,
};
use sim_spacetime::{BarycentricClockModel, ClockPotential, tdb_minus_tt_position_term_s};

const DAY_S: f64 = 86_400.0;
/// Earth's polar radius, m: a clock at the pole sits on the geoid and does not move
/// with the planet's spin.
const EARTH_POLAR_RADIUS_M: f64 = 6_356_752.314;

/// `2026-01-02T12:00:00 TDB`, a day inside the built-in kernels' coverage.
#[cfg(test)]
fn start() -> TdbEpoch {
    TdbEpoch::new(820_497_600 + 86_400, 0)
}

#[cfg(test)]
struct Clocks {
    solar: BuiltinSolarSystem2026_2028,
    model: BarycentricClockModel,
}

#[cfg(test)]
impl Clocks {
    fn new() -> Self {
        Self {
            solar: BuiltinSolarSystem2026_2028::load_verified().expect("solar"),
            model: BarycentricClockModel::solar_system(),
        }
    }

    fn body(&self, body: NaifId, at: TdbEpoch) -> (DVec3, DVec3) {
        let state = self
            .solar
            .ephemeris
            .geometric_state_icrf(body, NaifId::SSB, at, DerivativeOrder::Velocity)
            .expect("body state");
        (state.pos_m, state.vel_m_s)
    }

    /// `dτ/dTDB - 1` of a clock at `offset_m` from `body`, moving with it plus
    /// `relative_velocity_m_s`.
    fn rate(
        &self,
        body: NaifId,
        offset_m: DVec3,
        relative_velocity_m_s: DVec3,
        at: TdbEpoch,
    ) -> f64 {
        let (position, velocity) = self.body(body, at);
        self.model
            .proper_rate_offset(
                position + offset_m,
                velocity + relative_velocity_m_s,
                at,
                &self.solar.ephemeris,
            )
            .expect("rate")
    }

    fn geoid(&self, at: TdbEpoch) -> f64 {
        self.rate(
            NaifId::EARTH,
            DVec3::new(0.0, 0.0, EARTH_POLAR_RADIUS_M),
            DVec3::ZERO,
            at,
        )
    }

    /// Mean of `rate` over `days`, sampled every `step_s`.
    fn mean(&self, days: f64, step_s: f64, rate: impl Fn(TdbEpoch) -> f64) -> f64 {
        let samples = (days * DAY_S / step_s).round() as u64;
        let total: f64 = (0..samples)
            .map(|i| rate(start().advanced_seconds(i as f64 * step_s)))
            .sum();
        total / samples as f64
    }
}

#[test]
fn a_geoid_clock_keeps_tdb_over_a_year_and_swings_within_it() {
    let clocks = Clocks::new();
    let year = 365.256_36;
    let mean = clocks.mean(year, 3_600.0, |at| clocks.geoid(at));
    assert!(mean.abs() < 2e-12, "a geoid clock drifts {mean:e} from TDB");

    // TDB - TT swings by 1.66 ms each way over the year: the rate by its derivative.
    let mut accumulated = 0.0_f64;
    let (mut lowest, mut highest) = (0.0_f64, 0.0_f64);
    for hour in 0..(year * 24.0) as u64 {
        let at = start().advanced_seconds(hour as f64 * 3_600.0);
        accumulated += (clocks.geoid(at) - mean) * 3_600.0;
        lowest = lowest.min(accumulated);
        highest = highest.max(accumulated);
    }
    let swing_ms = (highest - lowest) * 1e3;
    assert!((3.2..3.4).contains(&swing_ms), "swing of {swing_ms} ms");
}

#[test]
fn a_gps_clock_gains_its_factory_offset_on_a_geoid_clock() {
    let clocks = Clocks::new();
    let (a, mu) = (26_561_750.0_f64, 3.986_004_418e14_f64);
    let mean_motion = (mu / (a * a * a)).sqrt();
    let inclination = 55.0_f64.to_radians();
    let (radial, along) = (
        DVec3::X,
        DVec3::new(0.0, inclination.cos(), inclination.sin()),
    );
    let orbit = |at: TdbEpoch| {
        let phase = mean_motion * at.signed_offset_seconds_from(start());
        (
            (radial * phase.cos() + along * phase.sin()) * a,
            (along * phase.cos() - radial * phase.sin()) * (a * mean_motion),
        )
    };
    let pole = DVec3::new(0.0, 0.0, EARTH_POLAR_RADIUS_M);
    // Whole revolutions, so the terms that only shuffle time around the orbit average out.
    let span_s = 10.0 * std::f64::consts::TAU / mean_motion;
    let against_tdb = clocks.mean(span_s / DAY_S, 30.0, |at| {
        let (offset, velocity) = orbit(at);
        clocks.rate(NaifId::EARTH, offset, velocity, at) - clocks.geoid(at)
    });
    // Clocks that agree in TT disagree in TDB by where they are; Earth's velocity turns
    // through the span, so that disagreement changes between its ends.
    let apart_in_tdb = |at: TdbEpoch| {
        let (_, earth_velocity) = clocks.body(NaifId::EARTH, at);
        tdb_minus_tt_position_term_s(earth_velocity, orbit(at).0 - pole)
    };
    let end = start().advanced_seconds(span_s);
    let gain = against_tdb + (apart_in_tdb(end) - apart_in_tdb(start())) / span_s;

    // The geocentric formula rates both clocks against TT; the difference between them
    // does not care which coordinate time it was measured in.
    let earth = ClockPotential::EARTH;
    let speed = DVec3::new(0.0, a * mean_motion, 0.0);
    let geocentric = earth
        .proper_rate_offset(DVec3::new(a, 0.0, 0.0), speed)
        .expect("orbit")
        - earth.proper_rate_offset(pole, DVec3::ZERO).expect("pole");
    assert!(
        (gain - geocentric).abs() < 2e-14,
        "barycentric {gain:e} against geocentric {geocentric:e}"
    );
    assert!(
        (gain - 4.4647e-10).abs() < 1e-12,
        "fractional gain {gain:e}"
    );
}

#[test]
fn clocks_on_the_moon_and_on_mars_gain_what_the_four_body_studies_give() {
    let clocks = Clocks::new();
    let moon_clock = |at: TdbEpoch| {
        let (moon, _) = clocks.body(NaifId::MOON, at);
        let (earth, _) = clocks.body(NaifId::EARTH, at);
        let towards_earth = (earth - moon).normalize();
        clocks.rate(NaifId::MOON, towards_earth * 1_737_400.0, DVec3::ZERO, at)
    };
    let lunar_months = 13.0 * 27.321_66;
    let moon_us_day = clocks.mean(lunar_months, 3_600.0, |at| {
        moon_clock(at) - clocks.geoid(at)
    }) * DAY_S
        * 1e6;

    let mars_clock = |at: TdbEpoch| {
        clocks.rate(
            NaifId::MARS,
            DVec3::from_array(clocks.model.mars_pole_icrf) * 3_376_200.0,
            DVec3::ZERO,
            at,
        )
    };
    // Each planet's own year, so its annual swing averages out of its own mean.
    let mars_mean = clocks.mean(686.98, 3_600.0, mars_clock);
    let earth_mean = clocks.mean(365.256_36, 3_600.0, |at| clocks.geoid(at));
    let mars_time_mean_us_day = (mars_mean - earth_mean) * DAY_S * 1e6;
    // Ashby and Patla (2024) give 56.02 µs/day for the Moon.
    assert!(
        (55.8..56.2).contains(&moon_us_day),
        "the Moon gains {moon_us_day} µs/day"
    );
    // Ashby and Patla (2025, arXiv:2507.21388) give two numbers for Mars against Earth's
    // geoid. Their closed-form estimate from the constants (Eq. 12) is 488.08 µs/day, and
    // a clock's mean rate over time is what that estimates.
    assert!(
        (487.0..489.0).contains(&mars_time_mean_us_day),
        "Mars's time-mean rate {mars_time_mean_us_day} µs/day against Eq. 12's 488.08"
    );

    // Their headline 477.60 µs/day is read from one 15.8-year cycle of the DE440 series
    // (Fig. 1) without a stated weighting. The value midway between the fastest and the
    // slowest rate of a Martian year lands on it; the time mean above does not, because
    // Mars lingers near aphelion where its clock runs fastest.
    let samples = (686.98 * DAY_S / 3_600.0).round() as u64;
    let (mut slowest_us_day, mut fastest_us_day) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut synodic_min_us_day, mut synodic_max_us_day) = (f64::INFINITY, f64::NEG_INFINITY);
    for i in 0..samples {
        let at = start().advanced_seconds(i as f64 * 3_600.0);
        let rate_us_day = (mars_clock(at) - earth_mean) * DAY_S * 1e6;
        slowest_us_day = slowest_us_day.min(rate_us_day);
        fastest_us_day = fastest_us_day.max(rate_us_day);

        let synodic_us_day = (mars_clock(at) - clocks.geoid(at)) * DAY_S * 1e6;
        synodic_min_us_day = synodic_min_us_day.min(synodic_us_day);
        synodic_max_us_day = synodic_max_us_day.max(synodic_us_day);
    }
    let mars_midrange_us_day = 0.5 * (slowest_us_day + fastest_us_day);
    assert!(
        (476.5..479.0).contains(&mars_midrange_us_day),
        "midway between Mars's extreme rates: {mars_midrange_us_day} µs/day against 477.60"
    );
    assert!(
        mars_time_mean_us_day - mars_midrange_us_day > 8.0,
        "the time mean {mars_time_mean_us_day} must stand above the midrange {mars_midrange_us_day}"
    );
    // The same figure gives 226.79 µs/day for the swing over a Martian year.
    let mars_ecc_swing_us_day = synodic_max_us_day - synodic_min_us_day;
    assert!(
        (210.0..245.0).contains(&mars_ecc_swing_us_day),
        "Mars–Earth synodic/eccentricity rate swing {mars_ecc_swing_us_day} µs/day"
    );
}

#[test]
fn marss_oblateness_is_read_about_marss_own_pole() {
    let clocks = Clocks::new();
    let pole = DVec3::from_array(clocks.model.mars_pole_icrf);
    assert!((pole.length() - 1.0).abs() < 1e-12);
    assert!(pole.angle_between(DVec3::Z).to_degrees() > 35.0);
    let mars = clocks.model.mars;
    let (polar_m, equatorial_m) = (3_376_200.0, 3_396_190.0);
    let across = pole.cross(DVec3::Z).normalize();
    let at = start();
    let on_pole = clocks.rate(NaifId::MARS, pole * polar_m, DVec3::ZERO, at);
    let on_equator = clocks.rate(NaifId::MARS, across * equatorial_m, DVec3::ZERO, at);
    let c2 = sim_spacetime::SPEED_OF_LIGHT_M_S * sim_spacetime::SPEED_OF_LIGHT_M_S;
    let ratio2 = |r_m: f64| mars.reference_radius_m * mars.reference_radius_m / (r_m * r_m);
    let pole_potential = mars.gm_m3_s2 / polar_m * (1.0 - mars.j2 * ratio2(polar_m));
    let equator_potential =
        mars.gm_m3_s2 / equatorial_m * (1.0 + 0.5 * mars.j2 * ratio2(equatorial_m));
    // The two points are 3 400 km apart, so the Sun's potential differs between them by
    // more than the oblateness does; it is taken out by the same model without Mars.
    let sun_only = |offset: DVec3| {
        let (mars_at, _) = clocks.body(NaifId::MARS, at);
        let (sun_at, _) = clocks.body(NaifId::SUN, at);
        1.327_124_400_412_794_2e20 / (mars_at + offset - sun_at).length()
    };
    let expected = -((pole_potential - equator_potential)
        + (sun_only(pole * polar_m) - sun_only(across * equatorial_m)))
        / c2;
    assert!(
        ((on_pole - on_equator) - expected).abs() < 2e-15,
        "pole minus equator {:e}, expected {expected:e}",
        on_pole - on_equator
    );
}
