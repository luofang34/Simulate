use glam::{DQuat, DVec3};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, ContinuousPlantState, ForceModel, NaifId, TdbEpoch,
};

use super::Voyage;

#[test]
fn a_write_that_bypasses_the_propagators_is_counted_and_an_origin_change_is_not() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let leo = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_563_000.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_793.0, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let mut voyage = Voyage::new(&solar, &fm, TdbEpoch::new(835_000_000, 0), leo);

    voyage.coast(60.0).expect("coast");
    voyage.move_origin_to(NaifId::MOON).expect("origin change");
    voyage.coast(60.0).expect("coast");
    assert_eq!(voyage.state_reinitializations, 0);
    let change = voyage.origin_changes.first().expect("one origin change");
    assert!((change.ssb_pos_after_m - change.ssb_pos_before_m).length() < 1.0e-3);
    assert!((change.ssb_vel_after_m_s - change.ssb_vel_before_m_s).length() < 1.0e-6);

    voyage.state.vel_rel_icrf_m_s.x += 1.0;
    voyage.coast(60.0).expect("coast");
    assert_eq!(voyage.state_reinitializations, 1);

    voyage.epoch = voyage.epoch.advanced_seconds(1.0);
    voyage.coast(60.0).expect("coast");
    assert_eq!(voyage.state_reinitializations, 2);
}
