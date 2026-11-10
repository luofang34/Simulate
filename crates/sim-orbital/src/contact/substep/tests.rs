//! Pinning must not move the vessel: the pinned body-fixed pose has to equal the unpinned state
//! at the epoch the state was advanced to.

use glam::{DQuat, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_spacetime::{
    CollisionGroundSample, CollisionTerrain, ElevationSlab, TICKS_PER_SECOND, TickSpan,
};

use super::step_one_contact_substep;
use crate::{
    BristleFootState, BuiltinSolarSystem2026_2028, CoastControlLaw, ContactFootSpec,
    ContinuousPlantState, ForceModel, MassPropertiesModel, NaifId, PinnedSurfaceState,
    PropellantMode, ReactionWheelMode, TdbEpoch, VesselContactState,
};

/// Smooth ellipsoid at height zero with stock contact parameters.
struct SmoothBody(BodyEllipsoid);

impl CollisionTerrain for SmoothBody {
    fn ellipsoid(&self) -> BodyEllipsoid {
        self.0
    }

    fn content_hash(&self) -> [u8; 32] {
        [0; 32]
    }

    fn sample_ground(&self, position: &Geodetic) -> CollisionGroundSample {
        let frame = LocalFrame::at(position);
        CollisionGroundSample {
            ellipsoidal_height_m: 0.0,
            normal_enu: DVec3::Z,
            normal_fixed: (frame.enu_to_ecef() * DVec3::Z).normalize(),
            mu_static: 0.8,
            mu_kinetic: 0.6,
            normal_stiffness_n_m: 1.8e5,
            normal_damping_n_s_m: 6.5e4,
            tangential_stiffness_n_m: 1.5e5,
            tangential_damping_n_s_m: 1.0e4,
        }
    }

    fn elevation_slab(&self, _: f64, _: f64, _: f64, _: f64) -> ElevationSlab {
        ElevationSlab {
            min_m: 0.0,
            max_m: 0.0,
        }
    }
}

/// Settles a lander on `body` and returns (position error m, attitude error rad) between the
/// pinned reconstruction and the unpinned state at the epoch of pinning.
fn settle_and_measure(body: NaifId, ellipsoid: BodyEllipsoid) -> (f64, f64) {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("force model");
    let terrain = SmoothBody(ellipsoid);
    let feet = ContactFootSpec::raketa_four_fin_pads(4.2);
    let mass = MassPropertiesModel::diagonal(12_000.0, [85_000.0, 18_000.0, 85_000.0], [0.0; 3]);
    let rw = ReactionWheelMode::ConserveMomentum {
        h_max_nms: 5.0e6,
        tau_max_nm: 5.0e4,
    };
    let t0 = TdbEpoch::new(835_000_000, 0);
    let geo = Geodetic {
        lat_rad: 0.3,
        lon_rad: 0.7,
        height_m: 4.2 - 0.0369 + 0.03,
    };
    let up = LocalFrame::at(&geo).enu_to_ecef() * DVec3::Z;
    let pose = PinnedSurfaceState {
        body,
        pos_fixed_m: ellipsoid.geodetic_to_fixed(&geo).to_array(),
        q_body_to_fixed_xyzw: DQuat::from_rotation_arc(DVec3::Y, up).to_array(),
    };
    let mut state: ContinuousPlantState = pose
        .to_icrf_plant_state(t0, 0.0, DVec3::ZERO, &solar.orientation)
        .expect("initial state");
    let mut contact = VesselContactState {
        feet: vec![BristleFootState::default(); feet.len()],
        ..VesselContactState::default()
    };

    let dt = 1.0e-3;
    let ticks = (dt * TICKS_PER_SECOND as f64).round() as u128;
    for k in 0..40_000_u128 {
        let epoch = t0.add_span(TickSpan::from_ticks_u128(k * ticks));
        step_one_contact_substep(
            &mut state,
            &mut contact,
            &feet,
            0.0,
            epoch,
            dt,
            &mass,
            PropellantMode::InfiniteConstantMass,
            rw,
            &fm,
            &terrain,
            &solar.ephemeris,
            &solar.orientation,
            &CoastControlLaw,
        )
        .expect("substep");
        if let Some(pinned) = contact.pinned {
            let after = epoch.add_span(TickSpan::from_ticks_u128(ticks));
            let rebuilt = pinned
                .to_icrf_plant_state(after, 0.0, DVec3::ZERO, &solar.orientation)
                .expect("rebuild");
            let dq = rebuilt.q_body_to_icrf * state.q_body_to_icrf.conjugate();
            let att = 2.0 * dq.xyz().length();
            return (
                (rebuilt.pos_rel_icrf_m - state.pos_rel_icrf_m).length(),
                att,
            );
        }
    }
    panic!("vessel never pinned on body {body:?}");
}

#[test]
fn pinning_preserves_pose_on_mars_and_moon() {
    let cases = [
        (NaifId::MARS, BodyEllipsoid::MARS_IAU),
        (NaifId::MOON, BodyEllipsoid::MOON_IAU),
    ];
    let measured: Vec<_> = cases
        .iter()
        .map(|(body, ell)| (*body, settle_and_measure(*body, *ell)))
        .collect();
    for (body, (dp, dq)) in measured {
        assert!(dp < 1.0e-4, "{body:?} position jump {dp} m");
        assert!(dq < 1.0e-7, "{body:?} attitude jump {dq} rad");
    }
}
