//! Integration tests for `sim-spacetime` frame contracts, time conversions,
//! sensor mount velocity transport, data provenance, and sampled trajectories.

use glam::{DQuat, DVec3};
use mbd_core::Placement;
use sim_geo::UtcTime;
use sim_spacetime::{
    AttitudeTrajectorySource, CollisionTerrain, ContentHash, DataProvenance,
    DataProvenanceWorldExt, DiscontinuousStepPatch, DynamicalModelIdentity, EarthEopEntry,
    EarthEopTable, FutureUtcPolicy, KinematicTrajectoryNode, LeapSecondTable, NaifId,
    OutOfEopPolicy, PatchedEllipsoidTerrain, SampledKinematicTrajectory, SensorMount,
    SensorMountPlacementExt, TdbEpoch, TimeScaleCatalog, TranslationalTrajectorySource,
    ValidEpochInterval, ValidEpochIntervalWorldExt,
};
use world_core::DatasetId;

#[test]
fn sensor_mount_placement_round_trips_and_transports_angular_velocity_lever_arm() {
    let placement = Placement {
        parent: None,
        translation_m: DVec3::new(2.0, 0.0, 0.0),
        rotation: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
    };
    let mount = SensorMount::from_placement(placement);
    let recovered = mount.to_placement();
    assert!((recovered.translation_m - placement.translation_m).length() < 1e-12);
    assert!((recovered.rotation.dot(placement.rotation).abs() - 1.0).abs() < 1e-12);

    // Body at origin in world frame, rotating around +Z at 2 rad/s; lever arm at +2 m along +X.
    // v_mount = v_cg + omega x r = (0, 0, 0) + (0, 0, 2) x (2, 0, 0) = (0, 4, 0) m/s.
    let kin = mount.evaluate_world_kinematics(
        DVec3::ZERO,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(0.0, 0.0, 2.0),
        DVec3::ZERO,
    );
    assert!((kin.position_world_m - DVec3::new(2.0, 0.0, 0.0)).length() < 1e-12);
    assert!((kin.velocity_world_m_s - DVec3::new(0.0, 4.0, 0.0)).length() < 1e-12);
}

#[test]
fn time_scale_catalog_converts_utc_and_tdb_to_decimal_year_and_world_validity_window() {
    let lsk_text = b"\\begindata\n\
        DELTET/DELTA_T_A = 32.184\n\
        DELTET/K = 1.657D-3\n\
        DELTET/EB = 1.671D-2\n\
        DELTET/M = ( 6.239996D0 1.99096871D-7 )\n\
        DELTET/DELTA_AT = ( 10, @1972-JAN-1\n\
                            32, @1999-JAN-1\n\
                            37, @2017-JAN-1 )\n\
        \\begintext\n";
    let ls = LeapSecondTable::from_naif_lsk_bytes(lsk_text).unwrap();
    let eop = EarthEopTable::new(vec![
        EarthEopEntry {
            utc_unix_s: 946_684_800.0,
            ut1_minus_utc_s: 0.1,
            xp_arcsec: 0.0,
            yp_arcsec: 0.0,
        },
        EarthEopEntry {
            utc_unix_s: 946_771_200.0,
            ut1_minus_utc_s: 0.1,
            xp_arcsec: 0.0,
            yp_arcsec: 0.0,
        },
    ]);
    let catalog = TimeScaleCatalog::new(
        ls,
        FutureUtcPolicy::RejectAfterExpiration {
            expires_tai_j2000_s: 2_000_000_000.0,
        },
        eop,
        OutOfEopPolicy::ZeroDeltaUt1Fallback,
    );

    let utc = UtcTime::from_unix_seconds(946_728_000.0);
    let dy_utc = TimeScaleCatalog::utc_to_decimal_year(utc);
    assert!((dy_utc.value() - 2000.0).abs() < 0.01);

    let t0 = TdbEpoch::new(0, 0);
    let t1 = TdbEpoch::new(86_400, 0);
    let interval = ValidEpochInterval::new(t0, t1);
    let window = interval.to_world_validity_window(&catalog).unwrap();
    assert!(window.until.value() > window.from.value());

    let prov = DataProvenance::from_dataset_id(
        DatasetId {
            name: "de440s",
            edition: "2020",
        },
        interval,
        ContentHash([0_u8; 32]),
    );
    assert_eq!(prov.dataset_name, "de440s");
    assert_eq!(prov.dataset_edition, "2020");
}

#[test]
fn sampled_kinematic_trajectory_and_patched_ellipsoid_terrain_satisfy_spacetime_traits() {
    let t0 = TdbEpoch::new(0, 0);
    let t1 = TdbEpoch::new(10, 0);

    let traj = SampledKinematicTrajectory::from_icrf_nodes(
        42,
        NaifId::EARTH,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight {
            body: NaifId::EARTH,
        },
        vec![
            KinematicTrajectoryNode {
                epoch: t0,
                pos_m: DVec3::new(6_378_137.0, 0.0, 0.0),
                vel_m_s: DVec3::new(0.0, 100.0, 0.0),
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
            KinematicTrajectoryNode {
                epoch: t1,
                pos_m: DVec3::new(6_378_137.0, 1_000.0, 0.0),
                vel_m_s: DVec3::new(0.0, 100.0, 0.0),
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            },
        ],
    )
    .unwrap();

    let mid = TdbEpoch::new(5, 0);
    let state_mid = traj.sample_translational_icrf(mid).unwrap();
    assert!((state_mid.pos.y - 500.0).abs() < 1e-9);
    let att_mid = traj.sample_attitude_icrf(mid).unwrap();
    assert!((att_mid.q_body_to_icrf.dot(DQuat::IDENTITY).abs() - 1.0).abs() < 1e-12);

    let terrain = PatchedEllipsoidTerrain::earth_wgs84(
        0.0,
        vec![DiscontinuousStepPatch::flat_pad(0.0, 0.0, 100.0, 120.0)],
    );
    let (_geo, sample) = terrain.sample_fixed(DVec3::new(6_378_137.0, 0.0, 0.0));
    assert!((sample.ellipsoidal_height_m - 120.0).abs() < 1e-9);
}
