//! Solar radiation pressure from the shared solar-flux model: its size and direction at
//! Earth's distance, its absence in Earth's shadow, and the drift it gives a vessel near
//! the Sun-Earth L2 distance over a day of full-ephemeris propagation.

use glam::{DQuat, DVec3};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, CoastControlLaw, ContinuousControlLaw, ContinuousPlantState,
    DerivativeOrder, Dop853Tolerances, EphemerisCatalog, ForceModel, MassPropertiesModel, NaifId,
    PropellantMode, ReactionWheelMode, SolarRadiationPressureLaw, SrpSurface, TdbEpoch,
    propagate_plant_interval,
};
use sim_spacetime::{SPEED_OF_LIGHT_M_S, SolarFluxModel};

/// A large sunshield on a light bus.
const SURFACE: SrpSurface = SrpSurface {
    area_m2: 100.0,
    reflectivity_coefficient: 1.5,
};
const MASS_KG: f64 = 6_200.0;

/// `2027-03-01T00:00:00 TDB`.
#[cfg(test)]
fn epoch() -> TdbEpoch {
    TdbEpoch::new(820_497_600 + 424 * 86_400, 0)
}

#[cfg(test)]
fn sun_from_earth(solar: &BuiltinSolarSystem2026_2028, at: TdbEpoch) -> DVec3 {
    solar
        .ephemeris
        .geometric_state_icrf(NaifId::SUN, NaifId::EARTH, at, DerivativeOrder::Position)
        .expect("sun")
        .pos_m
}

#[cfg(test)]
fn at_rest_about_earth(position_m: DVec3) -> ContinuousPlantState {
    ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: position_m,
        vel_rel_icrf_m_s: DVec3::ZERO,
        q_body_to_icrf: DQuat::from_rotation_z(0.7),
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    }
}

#[test]
fn sunlight_pushes_away_from_the_sun_and_not_at_all_in_earths_shadow() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let flux = SolarFluxModel::standard();
    let law = SolarRadiationPressureLaw {
        inner: &CoastControlLaw,
        surface: SURFACE,
        flux: &flux,
        ephemeris: &solar.ephemeris,
    };
    let to_sun = sun_from_earth(&solar, epoch());
    let sun_distance = to_sun.length();
    let sunward = to_sun / sun_distance;

    let across = sunward.cross(DVec3::Z).normalize();
    let lit = at_rest_about_earth(across * 42_164_000.0);
    let wrench = law
        .evaluate_stage_checked(0.0, epoch(), &lit)
        .expect("lit stage");
    let force_icrf = lit.q_body_to_icrf * wrench.env_force_body_n;
    let au = 149_597_870_700.0_f64;
    let here = (to_sun - lit.pos_rel_icrf_m).length();
    let expected = 1361.0 * (au / here).powi(2) / SPEED_OF_LIGHT_M_S * 1.5 * 100.0;
    assert!(
        (force_icrf.length() - expected).abs() < 1e-9 * expected,
        "{} N, expected {expected} N",
        force_icrf.length()
    );
    assert!(
        (6.0e-4..7.5e-4).contains(&expected),
        "{expected} N at Earth"
    );
    let away = -(to_sun - lit.pos_rel_icrf_m).normalize();
    assert!(force_icrf.normalize().dot(away) > 1.0 - 1e-12);

    let midnight = at_rest_about_earth(-sunward * 7_000_000.0);
    let dark = law
        .evaluate_stage_checked(0.0, epoch(), &midnight)
        .expect("shadow stage");
    assert_eq!(dark.env_force_body_n, DVec3::ZERO);
}

#[test]
fn force_model_cannonball_srp_agrees_with_solar_radiation_pressure_law_and_eclipses() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let flux = SolarFluxModel::standard();
    let law = SolarRadiationPressureLaw {
        inner: &CoastControlLaw,
        surface: SURFACE,
        flux: &flux,
        ephemeris: &solar.ephemeris,
    };
    let sunward = sun_from_earth(&solar, epoch()).normalize();
    let across = sunward.cross(DVec3::Z).normalize();
    let lit = at_rest_about_earth(across * 42_164_000.0);
    let wrench = law
        .evaluate_stage_checked(0.0, epoch(), &lit)
        .expect("lit stage");
    let force_icrf = lit.q_body_to_icrf * wrench.env_force_body_n;
    let midnight = at_rest_about_earth(-sunward * 7_000_000.0);

    let cr_area_over_mass = SURFACE.reflectivity_coefficient * SURFACE.area_m2 / MASS_KG;
    let fm_no_srp = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let fm_with_srp = fm_no_srp.clone().with_cannonball_srp(cr_area_over_mass);
    let a_lit_no_srp = fm_no_srp
        .relative_gravitational_accel_icrf(
            NaifId::EARTH,
            lit.pos_rel_icrf_m,
            epoch(),
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("lit no srp");
    let a_lit_with_srp = fm_with_srp
        .relative_gravitational_accel_icrf(
            NaifId::EARTH,
            lit.pos_rel_icrf_m,
            epoch(),
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("lit with srp");
    let fm_srp_accel = a_lit_with_srp - a_lit_no_srp;
    assert!(
        (fm_srp_accel - force_icrf / MASS_KG).length() < 1.0e-15,
        "ForceModel::with_cannonball_srp must agree with SolarRadiationPressureLaw: {fm_srp_accel:?} vs {:?}",
        force_icrf / MASS_KG
    );

    let a_dark_no_srp = fm_no_srp
        .relative_gravitational_accel_icrf(
            NaifId::EARTH,
            midnight.pos_rel_icrf_m,
            epoch(),
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("dark no srp");
    let a_dark_with_srp = fm_with_srp
        .relative_gravitational_accel_icrf(
            NaifId::EARTH,
            midnight.pos_rel_icrf_m,
            epoch(),
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("dark with srp");
    assert_eq!(
        a_dark_with_srp, a_dark_no_srp,
        "ForceModel::with_cannonball_srp must respect eclipse umbra via SolarFluxModel"
    );
}

#[test]
fn a_day_of_sunlight_moves_a_vessel_by_half_a_t_squared() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).expect("gravity");
    let flux = SolarFluxModel::standard();
    let start = epoch();
    let day_s = 86_400.0;
    let end = start.advanced_seconds(day_s);
    let sunward = sun_from_earth(&solar, start).normalize();
    // Beyond Earth on the Sun-Earth line, at the L2 distance, outside Earth's shadow cone.
    let across = sunward.cross(DVec3::Z).normalize();
    let initial = at_rest_about_earth(-sunward * 1.5e9 + across * 4.0e8);
    let mass = MassPropertiesModel::diagonal(MASS_KG, [9_000.0, 9_000.0, 9_000.0], [0.0; 3]);
    let fly = |law: &dyn ContinuousControlLaw| {
        propagate_plant_interval(
            &initial,
            start,
            end,
            None,
            &Dop853Tolerances::default(),
            &mass,
            PropellantMode::InfiniteConstantMass,
            ReactionWheelMode::ConserveMomentum {
                h_max_nms: 0.0,
                tau_max_nm: 0.0,
            },
            &force_model,
            &solar.ephemeris,
            &solar.orientation,
            law,
        )
        .expect("propagation")
        .final_state
    };
    let coasting = fly(&CoastControlLaw);
    let pushed = fly(&SolarRadiationPressureLaw {
        inner: &CoastControlLaw,
        surface: SURFACE,
        flux: &flux,
        ephemeris: &solar.ephemeris,
    });

    let drift = pushed.pos_rel_icrf_m - coasting.pos_rel_icrf_m;
    let accel = 1361.0 / SPEED_OF_LIGHT_M_S * 1.5 * 100.0 / MASS_KG;
    let expected = 0.5 * accel * day_s * day_s;
    assert!(
        (drift.length() - expected).abs() < 0.05 * expected,
        "drifted {} m, expected about {expected} m",
        drift.length()
    );
    assert!(
        drift.normalize().dot(-sunward) > 0.999,
        "the drift is away from the Sun"
    );
}
