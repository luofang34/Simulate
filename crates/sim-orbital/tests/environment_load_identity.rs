//! A vessel's environment loads are part of what it is: they are in its checkpoint's
//! hash and survive a save.

use glam::{DQuat, DVec3};
use sim_orbital::{
    ContinuousPlantState, DragSurface, EnvironmentLoadSpec, GravityGradient, NaifId,
    ResidualDipole, SrpSurface, VesselCheckpoint, VesselId,
};
use sim_spacetime::SimInstant;

/// Two vessels that differ in any load fly differently and must not share a hash.
#[test]
fn the_checkpoint_hash_covers_the_environment_loads() {
    let state = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(7_000_000.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_546.0, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let bare = VesselCheckpoint::new(VesselId(1), SimInstant::START, state, 0);
    let drag = |beta| {
        bare.clone()
            .with_environment_load(EnvironmentLoadSpec::Drag(DragSurface {
                ballistic_coefficient_kg_m2: beta,
            }))
    };
    let hashes = [
        bare.checkpoint_hash(),
        drag(50.0).checkpoint_hash(),
        drag(51.0).checkpoint_hash(),
        bare.clone()
            .with_environment_load(EnvironmentLoadSpec::GravityGradient(GravityGradient))
            .checkpoint_hash(),
        bare.clone()
            .with_environment_load(EnvironmentLoadSpec::SolarRadiationPressure(SrpSurface {
                area_m2: 50.0,
                reflectivity_coefficient: 1.0,
            }))
            .checkpoint_hash(),
        bare.clone()
            .with_environment_load(EnvironmentLoadSpec::MagneticDipole(ResidualDipole {
                moment_body_a_m2: [50.0, 0.0, 0.0],
            }))
            .checkpoint_hash(),
    ];
    for (i, a) in hashes.iter().enumerate() {
        for b in hashes.iter().skip(i + 1) {
            assert_ne!(a, b);
        }
    }
    assert_eq!(drag(50.0).checkpoint_hash(), drag(50.0).checkpoint_hash());
    let json = serde_json::to_string(&drag(50.0)).expect("serialize");
    let back: VesselCheckpoint = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(back, drag(50.0));
}
