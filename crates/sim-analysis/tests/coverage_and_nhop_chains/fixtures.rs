//! Shared orbital and RF fixtures for geodesic coverage and N-hop chain tests.

use glam::{DQuat, DVec3};
use sim_analysis::{
    AntennaPointing, AntennaSpec, GroundStationModel, OccultationEnvironmentModel,
    OccultingBodySpec, OccultingShape, ParabolicDishSpec, ReceiverNoise, RfLinkSpec,
    RfReceiverSpec, RfTransmitterSpec, SensorFovModel,
};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::BuiltinSolarSystem2026_2028;
use sim_spacetime::{
    BodyOrientationCatalog, DerivativeOrder, DynamicalModelIdentity, EphemerisCatalog,
    KinematicTrajectoryNode, NaifId, SampledKinematicTrajectory, SensorMount, TdbEpoch,
};

pub const MU_EARTH_M3_S2: f64 = 3.986_004_418e14;
pub const MU_MARS_M3_S2: f64 = 4.282_837_362_069_909e13;

pub fn mars_opposition_epoch() -> TdbEpoch {
    TdbEpoch::new(820_497_600 + 424 * 86_400, 0)
}

pub fn earth_fixed_stationary_trajectory(
    id: u64,
    lat_deg: f64,
    lon_deg: f64,
    radius_m: f64,
    start: TdbEpoch,
    duration_s: f64,
    solar: &BuiltinSolarSystem2026_2028,
) -> SampledKinematicTrajectory {
    let dir = Geodetic::from_degrees(lat_deg, lon_deg, 0.0);
    let unit = DVec3::new(
        dir.lat_rad.cos() * dir.lon_rad.cos(),
        dir.lat_rad.cos() * dir.lon_rad.sin(),
        dir.lat_rad.sin(),
    );
    let pos_fixed = unit * radius_m;
    let step_s = 60.0_f64;
    let n_steps = (duration_s / step_s).ceil() as u32;
    let mut nodes = Vec::new();
    for k in 0..=n_steps.max(1) {
        let ep = start.advanced_seconds(f64::from(k) * step_s);
        nodes.push(KinematicTrajectoryNode {
            epoch: ep,
            pos_m: pos_fixed,
            vel_m_s: DVec3::ZERO,
            q_body_to_axes: DQuat::IDENTITY,
            omega_body_rad_s: DVec3::ZERO,
        });
    }
    SampledKinematicTrajectory::from_body_fixed_samples(
        id,
        NaifId::EARTH,
        &nodes,
        &solar.orientation,
    )
    .expect("earth fixed trajectory")
}

pub fn circular_polar_earth_orbit(
    id: u64,
    altitude_m: f64,
    raan_rad: f64,
    phase0_rad: f64,
    start: TdbEpoch,
    duration_s: f64,
) -> SampledKinematicTrajectory {
    let r = BodyEllipsoid::WGS84.semi_major_m + altitude_m;
    let omega = (MU_EARTH_M3_S2 / (r * r * r)).sqrt();
    let v = r * omega;
    let (s_raan, c_raan) = raan_rad.sin_cos();
    let step_s = 30.0_f64;
    let n_steps = (duration_s / step_s).ceil() as u32;
    let nodes = (0..=n_steps.max(1))
        .map(|k| {
            let t = f64::from(k) * step_s;
            let u = phase0_rad + omega * t;
            let (su, cu) = u.sin_cos();
            KinematicTrajectoryNode {
                epoch: start.advanced_seconds(t),
                pos_m: DVec3::new(r * cu * c_raan, r * cu * s_raan, r * su),
                vel_m_s: DVec3::new(-v * su * c_raan, -v * su * s_raan, v * cu),
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            }
        })
        .collect();
    SampledKinematicTrajectory::from_icrf_nodes(
        id,
        NaifId::EARTH,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight {
            body: NaifId::EARTH,
        },
        nodes,
    )
    .expect("polar orbit")
}

pub fn circular_orbit_passing_zenith(
    id: u64,
    body: NaifId,
    mu_m3_s2: f64,
    radius_m: f64,
    zenith_dir_icrf: DVec3,
    t_zenith: TdbEpoch,
    span: sim_spacetime::ValidEpochInterval,
) -> SampledKinematicTrajectory {
    let e_r = zenith_dir_icrf.normalize();
    let ref_up = if e_r.z.abs() < 0.9 {
        DVec3::Z
    } else {
        DVec3::X
    };
    let e_t = ref_up.cross(e_r).normalize();
    let omega = (mu_m3_s2 / (radius_m * radius_m * radius_m)).sqrt();
    let v = radius_m * omega;
    let step_s = 30.0_f64;
    let n_steps = (span.duration_s() / step_s).ceil() as u32;
    let nodes = (0..=n_steps.max(1))
        .map(|k| {
            let ep = span.start_epoch.advanced_seconds(f64::from(k) * step_s);
            let dt = ep.signed_offset_seconds_from(t_zenith);
            let (s, c) = (omega * dt).sin_cos();
            KinematicTrajectoryNode {
                epoch: ep,
                pos_m: (e_r * c + e_t * s) * radius_m,
                vel_m_s: (-e_r * s + e_t * c) * v,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            }
        })
        .collect();
    SampledKinematicTrajectory::from_icrf_nodes(
        id,
        body,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight { body },
        nodes,
    )
    .expect("zenith orbit")
}

pub fn circular_equatorial_orbit(
    id: u64,
    body: NaifId,
    mu_m3_s2: f64,
    radius_m: f64,
    phase0_rad: f64,
    span: sim_spacetime::ValidEpochInterval,
) -> SampledKinematicTrajectory {
    let omega = (mu_m3_s2 / (radius_m * radius_m * radius_m)).sqrt();
    let v = radius_m * omega;
    let step_s = 60.0_f64;
    let n_steps = (span.duration_s() / step_s).ceil() as u32;
    let nodes = (0..=n_steps.max(1))
        .map(|k| {
            let t = f64::from(k) * step_s;
            let theta = phase0_rad + omega * t;
            let (s, c) = theta.sin_cos();
            KinematicTrajectoryNode {
                epoch: span.start_epoch.advanced_seconds(t),
                pos_m: DVec3::new(radius_m * c, radius_m * s, 0.0),
                vel_m_s: DVec3::new(-v * s, v * c, 0.0),
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            }
        })
        .collect();
    SampledKinematicTrajectory::from_icrf_nodes(
        id,
        body,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight { body },
        nodes,
    )
    .expect("equatorial orbit")
}

pub fn static_ssb_offset_trajectory(
    id: u64,
    pos_ssb_m: DVec3,
    vel_ssb_m_s: DVec3,
    start: TdbEpoch,
    duration_s: f64,
) -> SampledKinematicTrajectory {
    let step_s = 120.0_f64;
    let n_steps = (duration_s / step_s).ceil() as u32;
    let nodes = (0..=n_steps.max(1))
        .map(|k| {
            let t = f64::from(k) * step_s;
            KinematicTrajectoryNode {
                epoch: start.advanced_seconds(t),
                pos_m: pos_ssb_m + vel_ssb_m_s * t,
                vel_m_s: vel_ssb_m_s,
                q_body_to_axes: DQuat::IDENTITY,
                omega_body_rad_s: DVec3::ZERO,
            }
        })
        .collect();
    SampledKinematicTrajectory::from_icrf_nodes(
        id,
        NaifId::SSB,
        DynamicalModelIdentity::PlanetaryKinematicOrFlight { body: NaifId::SSB },
        nodes,
    )
    .expect("ssb offset trajectory")
}

pub fn solar_system_occultation_env() -> OccultationEnvironmentModel {
    OccultationEnvironmentModel {
        bodies: vec![
            OccultingBodySpec {
                body: NaifId::EARTH,
                shape: OccultingShape::Ellipsoid(BodyEllipsoid::WGS84),
            },
            OccultingBodySpec {
                body: NaifId::MARS,
                shape: OccultingShape::Ellipsoid(BodyEllipsoid::MARS_IAU),
            },
            OccultingBodySpec {
                body: NaifId::MOON,
                shape: OccultingShape::Sphere {
                    radius_m: 1_737_400.0,
                },
            },
            OccultingBodySpec {
                body: NaifId::SUN,
                shape: OccultingShape::Sphere {
                    radius_m: 695_700_000.0,
                },
            },
        ],
    }
}

pub fn rf_link(
    freq_hz: f64,
    tx_power_w: f64,
    dish_diam_m: f64,
    data_rate_bps: f64,
    required_eb_n0_db: f64,
) -> RfLinkSpec {
    RfLinkSpec {
        carrier_frequency_hz: freq_hz,
        bandwidth_hz: data_rate_bps * 2.0,
        data_rate_bps,
        required_eb_n0_db,
        polarization_mismatch_loss_db: 0.2,
        implementation_loss_db: 0.5,
        transmitter: RfTransmitterSpec {
            transmit_power_w: tx_power_w,
            antenna: AntennaSpec::ParabolicDish(ParabolicDishSpec {
                diameter_m: dish_diam_m,
                aperture_efficiency: 0.65,
                sidelobe_floor_below_peak_db: 30.0,
            }),
            pointing: AntennaPointing::TrackingTarget {
                pointing_error_rad: 0.0,
            },
            line_loss_db: 0.5,
        },
        receiver: RfReceiverSpec {
            antenna: AntennaSpec::ParabolicDish(ParabolicDishSpec {
                diameter_m: dish_diam_m,
                aperture_efficiency: 0.65,
                sidelobe_floor_below_peak_db: 30.0,
            }),
            pointing: AntennaPointing::TrackingTarget {
                pointing_error_rad: 0.0,
            },
            line_loss_db: 0.5,
            noise: ReceiverNoise::SystemTemperature {
                system_noise_temperature_k: 120.0,
            },
        },
        propagation: Vec::new(),
    }
}

pub fn sub_solar_or_mars_facing_station(
    solar: &BuiltinSolarSystem2026_2028,
    epoch: TdbEpoch,
    target_dir_icrf: DVec3,
    station_id: u32,
    name: &str,
    body: NaifId,
    ellipsoid: BodyEllipsoid,
) -> GroundStationModel {
    let rot = solar.orientation.body_rotation(body, epoch).expect("rot");
    let fixed_dir = rot.r_icrf_to_fixed * target_dir_icrf.normalize();
    let geo = ellipsoid.fixed_to_geodetic(fixed_dir * ellipsoid.semi_major_m);
    GroundStationModel {
        station_id,
        name: name.to_owned(),
        body,
        geodetic: Geodetic {
            lat_rad: geo.lat_rad,
            lon_rad: geo.lon_rad,
            height_m: 100.0,
        },
        ellipsoid,
        mount: SensorMount::IDENTITY,
        fov: SensorFovModel::OmnidirectionalHemisphere,
        min_elevation_rad: 5.0_f64.to_radians(),
    }
}

pub fn sun_earth_l2_relay_state(
    solar: &BuiltinSolarSystem2026_2028,
    epoch: TdbEpoch,
) -> (DVec3, DVec3) {
    let earth = solar
        .ephemeris
        .geometric_state_icrf(NaifId::EARTH, NaifId::SSB, epoch, DerivativeOrder::Velocity)
        .expect("earth");
    let sun = solar
        .ephemeris
        .geometric_state_icrf(NaifId::SUN, NaifId::SSB, epoch, DerivativeOrder::Velocity)
        .expect("sun");
    let anti_sun = (earth.pos_m - sun.pos_m).normalize();
    // Place L2 relay ~1.5M km anti-sunward plus a 120,000 km out-of-ecliptic offset so Earth
    // does not sit directly between L2 and an Earth-orbiting GEO relay.
    let offset = anti_sun * 1.5e9 + DVec3::new(0.0, 0.0, 1.2e8);
    (earth.pos_m + offset, earth.vel_m_s)
}

pub fn ssb_pos(solar: &BuiltinSolarSystem2026_2028, body: NaifId, epoch: TdbEpoch) -> DVec3 {
    solar
        .ephemeris
        .geometric_state_icrf(body, NaifId::SSB, epoch, DerivativeOrder::Position)
        .expect("ephemeris")
        .pos_m
}
