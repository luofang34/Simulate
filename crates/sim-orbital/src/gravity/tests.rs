use glam::{DQuat, DVec3};
use sim_spacetime::{CollinearLagrangePoint, SynodicFrame};

use super::{DynamicalMode, ForceModel, HARMONIC_FAR_FIELD_ACCEL_M_S2};
use crate::{
    BuiltinSolarSystem2026_2028, CoastControlLaw, ContinuousPlantState, Dop853Tolerances,
    MassPropertiesModel, NaifId, PropellantMode, ReactionWheelMode, TdbEpoch,
    propagate_plant_interval,
};

const HARMONIC_BODIES: [NaifId; 3] = [NaifId::EARTH, NaifId::MOON, NaifId::MARS];

fn epoch() -> TdbEpoch {
    TdbEpoch::new(845_000_000, 0)
}

/// The standard model with only `body`'s field, so a comparison sees that field alone.
fn single_field_model(solar: &BuiltinSolarSystem2026_2028, body: NaifId) -> ForceModel {
    let standard = ForceModel::standard_solar_system(solar.constants.clone()).expect("model");
    ForceModel::new(
        solar.constants.clone(),
        standard.attracting_bodies().to_vec(),
        vec![body],
        DynamicalMode::PrescribedEphemeris,
        standard.max_harmonic_degree(),
    )
    .expect("model")
}

fn directions() -> [DVec3; 5] {
    [
        DVec3::X,
        DVec3::Z,
        DVec3::new(0.3, -0.8, 0.5).normalize(),
        DVec3::new(-0.6, 0.1, -0.79).normalize(),
        DVec3::new(0.0, 0.6, -0.8),
    ]
}

fn accel(
    model: &ForceModel,
    solar: &BuiltinSolarSystem2026_2028,
    primary: NaifId,
    pos: DVec3,
) -> DVec3 {
    model
        .relative_gravitational_accel_icrf(
            primary,
            pos,
            epoch(),
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("accel")
}

#[test]
fn reaches_keep_earth_whole_about_l2_and_drop_moon_and_mars_there() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let model = ForceModel::standard_solar_system(solar.constants.clone()).expect("model");
    let reach = |body| model.harmonic_reach_of_m(body).expect("reach");
    // An orbit about Sun–Earth L2 stays within about 2.1e9 m of Earth and beyond
    // 1.0e9 m of the Moon.
    assert!(reach(NaifId::EARTH) > 3.0e9, "{}", reach(NaifId::EARTH));
    assert!(reach(NaifId::MOON) < 1.0e9, "{}", reach(NaifId::MOON));
    assert!(reach(NaifId::MOON) > 4.1e8, "{}", reach(NaifId::MOON));
    assert!(reach(NaifId::MARS) < 5.0e10, "{}", reach(NaifId::MARS));
}

#[test]
fn far_field_threshold_is_part_of_the_hash() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let model = ForceModel::standard_solar_system(solar.constants.clone()).expect("model");
    let whole = model.clone().with_harmonic_far_field_accel_m_s2(0.0);
    assert_ne!(model.content_hash(), whole.content_hash());
    assert_eq!(
        model.content_hash(),
        whole
            .with_harmonic_far_field_accel_m_s2(HARMONIC_FAR_FIELD_ACCEL_M_S2)
            .content_hash()
    );
}

#[test]
fn inside_its_reach_a_field_is_evaluated_bit_for_bit() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    for body in HARMONIC_BODIES {
        let model = single_field_model(&solar, body);
        let whole = model.clone().with_harmonic_far_field_accel_m_s2(0.0);
        let reach = model.harmonic_reach_of_m(body).expect("reach");
        let radius = solar.constants.get(body).expect("spec").equatorial_radius_m;
        for r in [
            radius + 300_000.0,
            4.0 * radius,
            0.1 * reach,
            0.5 * reach,
            0.999 * reach,
        ] {
            for dir in directions() {
                let pos = dir * r;
                let cut = accel(&model, &solar, body, pos).to_array();
                let full = accel(&whole, &solar, body, pos).to_array();
                assert_eq!(
                    cut.map(f64::to_bits),
                    full.map(f64::to_bits),
                    "{body:?} at {r} m"
                );
            }
        }
    }
}

#[test]
fn just_beyond_its_reach_a_skipped_field_is_below_the_threshold() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    for body in HARMONIC_BODIES {
        let model = single_field_model(&solar, body);
        let whole = model.clone().with_harmonic_far_field_accel_m_s2(0.0);
        let reach = model.harmonic_reach_of_m(body).expect("reach");
        let mut largest = 0.0_f64;
        for r in [1.000_001 * reach, 1.01 * reach, 2.0 * reach] {
            for dir in directions() {
                let pos = dir * r;
                let error =
                    (accel(&model, &solar, body, pos) - accel(&whole, &solar, body, pos)).length();
                largest = largest.max(error);
                assert!(
                    error <= HARMONIC_FAR_FIELD_ACCEL_M_S2,
                    "{body:?} at {r} m: {error} m/s²"
                );
            }
        }
        assert!(largest > 0.0, "{body:?}'s field was not skipped");
    }
}

/// A spacecraft placed at Sun–Earth L2 at rest in the rotating frame, flown 30 days.
fn l2_coast(solar: &BuiltinSolarSystem2026_2028, model: &ForceModel) -> DVec3 {
    let t0 = epoch();
    let frame = SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, t0, &solar.ephemeris)
        .expect("frame");
    let mu = |body| solar.constants.require_mu(body).expect("mu");
    let l2 = frame.lagrange_point_synodic_m(
        mu(NaifId::SUN),
        mu(NaifId::EARTH),
        CollinearLagrangePoint::L2,
    );
    let (pos, vel) =
        frame.synodic_to_icrf_rel_secondary(l2 + DVec3::new(0.0, 2.0e8, -1.0e8), DVec3::ZERO);
    let state = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: pos,
        vel_rel_icrf_m_s: vel,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let out = propagate_plant_interval(
        &state,
        t0,
        t0.advanced_seconds(30.0 * 86_400.0),
        None,
        &Dop853Tolerances::default(),
        &MassPropertiesModel::diagonal(6_200.0, [45_000.0, 45_000.0, 25_000.0], [0.0; 3]),
        PropellantMode::InfiniteConstantMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        model,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("coast");
    out.final_state.pos_rel_icrf_m
}

#[test]
fn thirty_days_about_l2_end_within_a_centimetre_of_the_whole_field() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let model = ForceModel::standard_solar_system(solar.constants.clone()).expect("model");
    let whole = model.clone().with_harmonic_far_field_accel_m_s2(0.0);
    let apart_m = (l2_coast(&solar, &model) - l2_coast(&solar, &whole)).length();
    assert!(apart_m < 0.01, "{apart_m} m");
}
