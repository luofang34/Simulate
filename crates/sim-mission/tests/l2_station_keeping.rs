//! A JWST-like spacecraft about Sun–Earth L2 under the full ephemeris and solar radiation
//! pressure: left alone it leaves within months; with a small burn every three weeks it
//! stays for a year on a few metres per second.

use sim_mission::{
    L2StationKeepingSpec, StationKeepingError, fly_l2_station_keeping, plan_assisted_l2_bounded_arc,
};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, DynamicalMode, ForceModel, L2BoundedArcTargetSpec, NaifId,
    SrpSurface, TdbEpoch,
};
use sim_spacetime::SolarFluxModel;

const DAY_S: f64 = 86_400.0;

#[cfg(test)]
struct Setup {
    solar: BuiltinSolarSystem2026_2028,
    force_model: ForceModel,
    flux: SolarFluxModel,
    spec: L2StationKeepingSpec,
}

#[cfg(test)]
impl Setup {
    /// A spacecraft with JWST's mass and a sunshield-sized surface, started on an arc
    /// the assisted planner targets about L2.
    fn new() -> Self {
        let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
        // Point masses: at 1.5 million km no body's shape is felt.
        let force_model = ForceModel::new(
            solar.constants.clone(),
            vec![
                NaifId::SUN,
                NaifId::EARTH,
                NaifId::MOON,
                NaifId::JUPITER_BARYCENTER,
            ],
            Vec::new(),
            DynamicalMode::PrescribedEphemeris,
            0,
        )
        .expect("gravity");
        // 2026-03-01T12:00:00 TDB.
        let start = TdbEpoch::new(820_497_600 + 59 * 86_400, 0);
        let arc = plan_assisted_l2_bounded_arc(
            &L2BoundedArcTargetSpec::sun_earth_southern(start, 60.0 * DAY_S, 2.5e8),
            &force_model,
            &solar.ephemeris,
            &solar.orientation,
            1_000.0,
            0.1,
        )
        .expect("planned arc")
        .targeted_arc;
        let spec = L2StationKeepingSpec {
            primary: NaifId::SUN,
            secondary: NaifId::EARTH,
            start_epoch: start,
            initial_pos_syn_m: arc.initial_pos_syn_m,
            initial_vel_syn_m_s: arc.initial_vel_syn_m_s,
            duration_s: 365.0 * DAY_S,
            maintenance_interval_s: 21.0 * DAY_S,
            crossings_ahead: 2,
            lookahead_s: 240.0 * DAY_S,
            x_bound_m: 6.0e8,
            max_burn_m_s: 30.0,
            min_burn_m_s: 1.0e-3,
            mass_kg: 6_200.0,
            srp: Some(SrpSurface {
                area_m2: 160.0,
                reflectivity_coefficient: 1.6,
            }),
        };
        Self {
            solar,
            force_model,
            flux: SolarFluxModel::standard(),
            spec,
        }
    }

    fn fly(
        &self,
        spec: &L2StationKeepingSpec,
    ) -> Result<sim_mission::L2StationKeepingReport, StationKeepingError> {
        fly_l2_station_keeping(
            spec,
            &self.force_model,
            &self.flux,
            &self.solar.ephemeris,
            &self.solar.orientation,
        )
    }
}

#[test]
fn a_spacecraft_is_held_about_l2_for_a_year_on_a_few_metres_per_second() {
    let setup = Setup::new();
    let report = setup.fly(&setup.spec).expect("held");
    assert_eq!(report.synodic_l2_samples_m.len(), 365);
    assert!(
        report.max_x_excursion_m < 4.0e8 && report.max_l2_distance_m < 9.0e8,
        "strayed {} m along the line and {} m in all",
        report.max_x_excursion_m,
        report.max_l2_distance_m
    );
    // The planner's two-month arc is not yet a bounded orbit: the first burn puts the
    // spacecraft on one, and the rest only keep it there.
    let insertion = report.burns.first().expect("an insertion burn");
    assert_eq!(insertion.epoch, setup.spec.start_epoch);
    assert!((3.0..8.0).contains(&insertion.delta_v_m_s.abs()));
    let upkeep = report.total_delta_v_m_s - insertion.delta_v_m_s.abs();
    assert!(
        upkeep < 4.0,
        "a year's upkeep took {upkeep} m/s in {} burns",
        report.burns.len() - 1
    );
}

#[test]
fn left_alone_the_same_spacecraft_leaves_within_seven_months() {
    let setup = Setup::new();
    let unattended = L2StationKeepingSpec {
        // One maintenance epoch at the start that allows no burn worth the name.
        maintenance_interval_s: 400.0 * DAY_S,
        lookahead_s: DAY_S,
        duration_s: 200.0 * DAY_S,
        ..setup.spec
    };
    let report = setup.fly(&unattended).expect("free flight");
    assert!(report.burns.is_empty());
    assert!(
        report.max_x_excursion_m > 2.0 * setup.spec.x_bound_m,
        "unattended, it strayed only {} m",
        report.max_x_excursion_m
    );
}
