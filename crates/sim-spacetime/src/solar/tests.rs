use glam::DVec3;

use super::{
    ASTRONOMICAL_UNIT_M, AtmosphericTransmission, OccultingSphere, SOLAR_IRRADIANCE_1AU_W_M2,
    SOLAR_RADIUS_M, SolarFluxModel,
};
use crate::catalogs::{DerivativeOrder, EphemerisCatalog, EphemerisError, KinematicStateIcrf};
use crate::hash::ContentHash;
use crate::ids::NaifId;
use crate::time::{TdbEpoch, TdbSpanNs};

const EARTH_RADIUS_M: f64 = 6_371_008.8;
const MOON_DISTANCE_M: f64 = 384_400_000.0;

/// The Sun at the origin, Earth one astronomical unit along `+X`, and the
/// Moon between them.
struct FixedLayout;

impl EphemerisCatalog for FixedLayout {
    fn content_hash(&self) -> ContentHash {
        ContentHash::of_bytes(b"solar-tests-fixed-layout")
    }

    fn coverage(&self, _: NaifId, _: NaifId, _: DerivativeOrder) -> Vec<TdbSpanNs> {
        Vec::new()
    }

    fn geometric_state_icrf(
        &self,
        target: NaifId,
        _center: NaifId,
        _epoch: TdbEpoch,
        _order: DerivativeOrder,
    ) -> Result<KinematicStateIcrf, EphemerisError> {
        let pos_m = match target {
            NaifId::SUN => DVec3::ZERO,
            NaifId::EARTH => DVec3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0),
            NaifId::MOON => DVec3::new(ASTRONOMICAL_UNIT_M - MOON_DISTANCE_M, 0.0, 0.0),
            _ => DVec3::new(0.0, 5.0 * ASTRONOMICAL_UNIT_M, 0.0),
        };
        Ok(KinematicStateIcrf {
            pos_m,
            vel_m_s: DVec3::ZERO,
            acc_m_s2: DVec3::ZERO,
        })
    }

    fn segment_knots_in(&self, _: NaifId, _: NaifId, _: TdbEpoch, _: TdbEpoch) -> Vec<TdbEpoch> {
        Vec::new()
    }
}

fn earth_only() -> SolarFluxModel {
    SolarFluxModel {
        irradiance_1au_w_m2: SOLAR_IRRADIANCE_1AU_W_M2,
        occulters: vec![OccultingSphere {
            body: NaifId::EARTH,
            radius_m: EARTH_RADIUS_M,
        }],
    }
}

fn sample_at(model: &SolarFluxModel, position: DVec3) -> super::SolarFluxSample {
    model
        .sample(position, TdbEpoch::new(0, 0), &FixedLayout)
        .unwrap()
}

#[test]
fn irradiance_falls_with_the_square_of_distance_and_carries_its_momentum() {
    let model = earth_only();
    let at_1au = sample_at(&model, DVec3::new(0.0, ASTRONOMICAL_UNIT_M, 0.0));
    assert!((at_1au.irradiance_w_m2() - 1361.0).abs() < 1e-9);
    assert_eq!(at_1au.lit_fraction, 1.0);
    assert!((at_1au.to_sun_icrf - DVec3::NEG_Y).length() < 1e-12);
    let pressure = at_1au.radiation_pressure_n_m2();
    assert!((pressure - 4.539_8e-6).abs() < 1e-9, "{pressure} N/m²");

    let at_mars = sample_at(&model, DVec3::new(0.0, 1.523_7 * ASTRONOMICAL_UNIT_M, 0.0));
    let expected = 1361.0 / (1.523_7 * 1.523_7);
    assert!((at_mars.irradiance_w_m2() - expected).abs() < 1e-9);
    assert!((at_mars.irradiance_w_m2() - 586.2).abs() < 0.1);
}

#[test]
fn a_surface_collects_by_the_cosine_and_nothing_from_behind() {
    let sample = sample_at(&earth_only(), DVec3::new(0.0, ASTRONOMICAL_UNIT_M, 0.0));
    let tilted = DVec3::new(0.0, -0.5, 0.75_f64.sqrt());
    assert!((sample.irradiance_on_w_m2(tilted) - 680.5).abs() < 1e-9);
    assert_eq!(sample.irradiance_on_w_m2(DVec3::Y), 0.0);
}

#[test]
fn earths_shadow_has_an_umbra_a_penumbra_and_an_edge() {
    let model = earth_only();
    let behind = |along_m: f64, across_m: f64| {
        sample_at(
            &model,
            DVec3::new(ASTRONOMICAL_UNIT_M + along_m, across_m, 0.0),
        )
    };
    assert_eq!(
        behind(7_000_000.0, 0.0).lit_fraction,
        0.0,
        "low orbit at midnight"
    );
    assert_eq!(behind(7_000_000.0, 0.0).irradiance_w_m2(), 0.0);
    assert_eq!(behind(-7_000_000.0, 0.0).lit_fraction, 1.0, "the day side");
    assert_eq!(
        behind(42_164_000.0, 20_000_000.0).lit_fraction,
        1.0,
        "well clear"
    );

    // Across the shadow's edge at geostationary distance the disc comes into view
    // steadily, half of it when the limb of the Earth crosses the Sun's centre.
    let mut previous = 0.0;
    for step in 0..=40 {
        let across = 6_000_000.0 + 20_000.0 * f64::from(step);
        let lit = behind(42_164_000.0, across).lit_fraction;
        assert!(lit >= previous, "lit fraction fell at {across} m");
        previous = lit;
    }
    assert_eq!(behind(42_164_000.0, 6_000_000.0).lit_fraction, 0.0);
    assert_eq!(behind(42_164_000.0, 6_800_000.0).lit_fraction, 1.0);
    // Earth's limb, far larger than the Sun's disc, is nearly a straight edge across it.
    // Find where that edge passes through the Sun's centre.
    let limb_past_centre = |across_m: f64| {
        let here = DVec3::new(ASTRONOMICAL_UNIT_M + 42_164_000.0, across_m, 0.0);
        let to_earth = DVec3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0) - here;
        let separation = to_earth.angle_between(-here);
        separation - (EARTH_RADIUS_M / to_earth.length()).asin()
    };
    let (mut inside, mut outside) = (6_000_000.0, 6_800_000.0);
    for _ in 0..60 {
        let mid = 0.5 * (inside + outside);
        if limb_past_centre(mid) < 0.0 {
            inside = mid;
        } else {
            outside = mid;
        }
    }
    let half = behind(42_164_000.0, inside).lit_fraction;
    assert!((half - 0.5).abs() < 0.02, "half the disc, got {half}");
}

#[test]
fn a_small_body_far_away_leaves_a_ring_of_sun() {
    // From Earth's centre line the Moon at apogee-like distance does not cover the
    // disc: an annular eclipse leaves the area the Moon does not span.
    let model = SolarFluxModel {
        irradiance_1au_w_m2: SOLAR_IRRADIANCE_1AU_W_M2,
        occulters: vec![OccultingSphere {
            body: NaifId::MOON,
            radius_m: 1_600_000.0,
        }],
    };
    let sample = sample_at(&model, DVec3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0));
    let sun = (SOLAR_RADIUS_M / ASTRONOMICAL_UNIT_M).asin();
    let moon = (1_600_000.0_f64 / MOON_DISTANCE_M).asin();
    assert!(moon < sun);
    let ring = 1.0 - (moon * moon) / (sun * sun);
    assert!((sample.lit_fraction - ring).abs() < 1e-12);
    assert!(sample.lit_fraction > 0.0 && sample.lit_fraction < 0.25);
}

#[test]
fn an_atmosphere_passes_less_of_a_low_sun_and_none_of_a_set_one() {
    let earth = AtmosphericTransmission::EARTH_CLEAR_SKY;
    let overhead = earth.direct_beam_fraction(1.0);
    assert!((overhead - (-0.30_f64).exp()).abs() < 1e-3, "{overhead}");
    let mut previous = overhead;
    for degrees in [30.0_f64, 60.0, 80.0, 89.0] {
        let fraction = earth.direct_beam_fraction(degrees.to_radians().cos());
        assert!(fraction < previous && fraction > 0.0);
        previous = fraction;
    }
    assert_eq!(earth.direct_beam_fraction(0.0), 0.0);
    assert_eq!(earth.direct_beam_fraction(-0.2), 0.0);
    assert_eq!(
        AtmosphericTransmission::VACUUM.direct_beam_fraction(0.3),
        1.0
    );
    assert!(
        AtmosphericTransmission::MARS_CLEAR_SKY.direct_beam_fraction(1.0) < overhead,
        "dustier than a clear Earth sky"
    );
}
