//! A vessel pinned at rest reports the surface proper acceleration of its own body, not an
//! Earth constant.

use glam::{DQuat, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, CoastControlLaw, ContactFootSpec, ForceModel, MassPropertiesModel,
    NaifId, PinnedSurfaceState, PropellantMode, ReactionWheelMode, TdbEpoch, VesselContactState,
    integrate_contact_interval,
};
use sim_spacetime::{
    CollisionGroundSample, CollisionTerrain, ElevationSlab, TICKS_PER_SECOND, TickSpan,
};

/// Smooth ellipsoid at height zero.
#[cfg(test)]
struct SmoothBody(BodyEllipsoid);

#[cfg(test)]
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

#[cfg(test)]
/// Point-mass gravitation minus the centripetal term at `p_fixed`, from published body
/// constants, independent of the library's force model.
fn reference_surface_gravity(gm_m3_s2: f64, spin_rad_s: f64, p_fixed: DVec3) -> f64 {
    let omega = DVec3::new(0.0, 0.0, spin_rad_s);
    let gravitation = -gm_m3_s2 * p_fixed / p_fixed.length().powi(3);
    (gravitation - omega.cross(omega.cross(p_fixed))).length()
}

#[cfg(test)]
fn pinned_report(body: NaifId, ellipsoid: BodyEllipsoid, lat_rad: f64) -> (f64, f64, DVec3) {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("force model");
    let feet = ContactFootSpec::raketa_four_fin_pads(4.2);
    let mass = MassPropertiesModel::diagonal(12_000.0, [85_000.0, 18_000.0, 85_000.0], [0.0; 3]);
    let rw = ReactionWheelMode::ConserveMomentum {
        h_max_nms: 5.0e6,
        tau_max_nm: 5.0e4,
    };
    let t0 = TdbEpoch::new(835_000_000, 0);
    let geo = Geodetic {
        lat_rad,
        lon_rad: 0.7,
        height_m: 4.2 - 0.0369,
    };
    let up = LocalFrame::at(&geo).enu_to_ecef() * DVec3::Z;
    let p_fixed = ellipsoid.geodetic_to_fixed(&geo);
    let pose = PinnedSurfaceState {
        body,
        pos_fixed_m: p_fixed.to_array(),
        q_body_to_fixed_xyzw: DQuat::from_rotation_arc(DVec3::Y, up).to_array(),
    };
    let state = pose
        .to_icrf_plant_state(t0, 0.0, DVec3::ZERO, &solar.orientation)
        .expect("state");
    let contact = VesselContactState {
        pinned: Some(pose),
        ..VesselContactState::default()
    };
    let out = integrate_contact_interval(
        &state,
        contact,
        &feet,
        t0,
        t0.add_span(TickSpan::from_ticks(TICKS_PER_SECOND)),
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        &fm,
        &SmoothBody(ellipsoid),
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("pinned interval");
    assert!(out.contact.pinned.is_some(), "{body:?} must stay pinned");
    (
        out.peak_proper_accel_m_s2,
        out.min_foot_clearance_m,
        p_fixed,
    )
}

#[test]
fn pinned_lander_reports_each_bodys_own_surface_gravity() {
    let cases = [
        (
            NaifId::MOON,
            BodyEllipsoid::MOON_IAU,
            4.902_800_1e12,
            2.661_707e-6,
            0.3,
        ),
        (
            NaifId::MARS,
            BodyEllipsoid::MARS_IAU,
            4.282_837_4e13,
            7.088_218e-5,
            0.3,
        ),
        (
            NaifId::MARS,
            BodyEllipsoid::MARS_IAU,
            4.282_837_4e13,
            7.088_218e-5,
            1.2,
        ),
    ];
    for (body, ellipsoid, gm, spin, lat) in cases {
        let (peak, clearance, p_fixed) = pinned_report(body, ellipsoid, lat);
        let expected = reference_surface_gravity(gm, spin, p_fixed);
        assert!(
            (peak - expected).abs() <= 0.01 * expected,
            "{body:?} lat {lat}: peak {peak} vs surface gravity {expected}"
        );
        assert!(
            peak < 9.0,
            "{body:?} peak {peak} must not be the Earth constant"
        );
        assert_eq!(clearance, 0.0);
    }
}
