//! Multi-orbit LEO orbital decay comparison driven by `Nrlmsise00Atmosphere` and
//! time-varying `SpaceWeatherTable` space weather indices.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_atmosphere::{
    CalendarDate, IndexQuality, Nrlmsise00Atmosphere, SpaceWeatherDay, SpaceWeatherSource,
    SpaceWeatherTable,
};
use sim_bodies::{BodyCatalog, BodyDescriptor};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    BodyGravitySpec, BuiltinSolarSystem2026_2028, CoastControlLaw, ContinuousPlantState,
    Dop853Tolerances, DragSurface, DynamicalMode, EnvironmentContext, EnvironmentLaw,
    EnvironmentLoadSpec, EnvironmentLoads, ForceModel, GravityConstantsCatalog,
    MassPropertiesModel, NaifId, PropellantMode, ReactionWheelMode, TdbEpoch,
    propagate_plant_interval,
};

const MU: f64 = 3.986_004_418e14;
const RADIUS_M: f64 = 6_378_137.0;
const GLOBE: BodyEllipsoid = BodyEllipsoid {
    semi_major_m: RADIUS_M,
    flattening: 0.0,
    mu_m3_s2: MU,
    omega_rad_s: 0.0,
    j2: 0.0,
};

fn build_weather_table(
    anchor_unix_s: i64,
    f107: f64,
    ap: f64,
) -> Result<SpaceWeatherTable, Box<dyn std::error::Error>> {
    let (center, _) = CalendarDate::from_unix_seconds(anchor_unix_s)?;
    let days: Vec<_> = (-5_i64..=5_i64)
        .map(|delta| {
            Ok::<_, Box<dyn std::error::Error>>(SpaceWeatherDay {
                date: center.add_days(delta)?,
                ap: [Some(ap); 8],
                daily_ap: Some(ap),
                f107_observed: Some(f107),
                f107_adjusted: Some(f107),
                f107_observed_center81: Some(f107),
                f107_adjusted_center81: Some(f107),
                quality: IndexQuality::Definitive,
                source: SpaceWeatherSource::Gfz,
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(SpaceWeatherTable::from_days(days)?)
}

fn fly_leo_decay(
    solar: &BuiltinSolarSystem2026_2028,
    epoch: TdbEpoch,
    atmo_id: &str,
    atmosphere: Nrlmsise00Atmosphere,
    span_s: f64,
) -> Result<(ContinuousPlantState, f64), Box<dyn std::error::Error>> {
    let body = BodyDescriptor::new(NaifId::EARTH, "globe", GLOBE)?
        .with_atmosphere(atmo_id, Arc::new(atmosphere));
    let catalog = Arc::new(BodyCatalog::new().with(body));
    let constants = GravityConstantsCatalog::from_specs([BodyGravitySpec {
        naif_id: NaifId::EARTH,
        mu_m3_s2: MU,
        equatorial_radius_m: RADIUS_M,
        polar_radius_m: RADIUS_M,
        harmonics: None,
    }])
    .with_body_catalog(catalog);
    let model = ForceModel::new(
        constants,
        vec![NaifId::EARTH],
        Vec::new(),
        DynamicalMode::SelfConsistentNBody,
        0,
    )?;
    let a0 = RADIUS_M + 350_000.0;
    let start = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(a0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, (MU / a0).sqrt(), 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let loads: EnvironmentLoads = [EnvironmentLoadSpec::Drag(DragSurface {
        ballistic_coefficient_kg_m2: 45.0,
    })]
    .into_iter()
    .collect();
    let mass = MassPropertiesModel::diagonal(450.0, [80.0, 80.0, 80.0], [0.0; 3]);
    let law = EnvironmentLaw {
        inner: &CoastControlLaw,
        loads: &loads,
        context: EnvironmentContext::of(&model, &solar.ephemeris, &solar.orientation, &mass),
        held_dipole_body_a_m2: DVec3::ZERO,
    };
    let out = propagate_plant_interval(
        &start,
        epoch,
        epoch.advanced_seconds(span_s),
        None,
        &Dop853Tolerances::default(),
        &mass,
        PropellantMode::InfiniteConstantMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        &model,
        &solar.ephemeris,
        &solar.orientation,
        &law,
    )?;
    let (r, v) = (
        out.final_state.pos_rel_icrf_m,
        out.final_state.vel_rel_icrf_m_s,
    );
    let a_final = -MU / (v.length_squared() - 2.0 * MU / r.length());
    Ok((out.final_state, a0 - a_final))
}

#[test]
fn nrlmsise00_leo_decay_is_deterministic_and_tracks_solar_storm_activity()
-> Result<(), Box<dyn std::error::Error>> {
    let solar = BuiltinSolarSystem2026_2028::load_verified()?;
    let epoch = TdbEpoch::new(845_000_000, 0);
    let utc = solar.time_scales.tdb_to_utc(epoch)?;
    let anchor_unix_s = utc.unix_seconds().floor() as i64;
    let six_orbits_s = 6.0 * 5_490.0;

    let quiet =
        Nrlmsise00Atmosphere::from_space_weather(build_weather_table(anchor_unix_s, 70.0, 4.0)?);
    let storm =
        Nrlmsise00Atmosphere::from_space_weather(build_weather_table(anchor_unix_s, 220.0, 120.0)?);

    let (quiet_a, decay_quiet) =
        fly_leo_decay(&solar, epoch, "msis-quiet", quiet.clone(), six_orbits_s)?;
    let (quiet_b, decay_quiet_b) = fly_leo_decay(&solar, epoch, "msis-quiet", quiet, six_orbits_s)?;
    let (_storm_state, decay_storm) =
        fly_leo_decay(&solar, epoch, "msis-storm", storm, six_orbits_s)?;

    assert_eq!(quiet_a.pos_rel_icrf_m, quiet_b.pos_rel_icrf_m);
    assert_eq!(quiet_a.vel_rel_icrf_m_s, quiet_b.vel_rel_icrf_m_s);
    assert_eq!(decay_quiet, decay_quiet_b);
    assert!(
        decay_quiet > 0.5,
        "quiet decay should exceed 0.5 m, got {decay_quiet}"
    );
    assert!(
        decay_storm > decay_quiet * 4.0,
        "storm/high-F10.7 decay ({decay_storm} m) should greatly exceed quiet decay ({decay_quiet} m)"
    );
    Ok(())
}
