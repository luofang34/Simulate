//! Scripted ADS-B traffic trajectories for built-in NY/NJ flight-deck scenarios.

use crate::error::FlightDeckError;
use crate::planet::PlanetaryReference;
use crate::telemetry::{FEET_TO_METRES, FPM_TO_MPS, KNOTS_TO_MPS, OwnshipState};
use crate::traffic::TrafficCoordinator;
use world::surveillance::{
    AddressNamespace, AirGroundState, Band, Callsign, DeliveryPath, EmitterCategory, FieldQuality,
    ObservationOrigin, ObservationTime, PositionObservation, SourceRef, TimeBasis, TrackKey,
    TrafficObservation, VelocityObservation, VerticalRateSource, Wgs84Position,
};
use world_core::{LocalTangentPlane, Ned};

pub(super) struct ScriptedTargetSpec {
    pub icao: u32,
    pub callsign: &'static str,
    pub east0_m: f64,
    pub north0_m: f64,
    pub rel_alt_ft: f64,
    pub ve_mps: f64,
    pub vn_mps: f64,
    pub climb_fpm: f64,
    pub category: EmitterCategory,
}

pub(super) fn ingest_scripted_targets(
    ownship: &OwnshipState,
    planet: &PlanetaryReference,
    t_s: f64,
    now_ms: u64,
    specs: &[ScriptedTargetSpec],
    traffic: &mut TrafficCoordinator,
) -> Result<(), FlightDeckError> {
    let Some(own_geo) = ownship.valid_earth_wgs84_position(planet) else {
        return Ok(());
    };
    let own_alt_m = ownship
        .best_altitude_m()
        .unwrap_or(own_geo.ellipsoidal_height_m);
    let Some(flat_pos) =
        planet.earth_wgs84_position(own_geo.latitude_deg, own_geo.longitude_deg, 0.0)
    else {
        return Ok(());
    };
    let Ok(plane) = LocalTangentPlane::new(flat_pos) else {
        return Ok(());
    };
    let now_us = now_ms.saturating_mul(1_000);

    for spec in specs {
        let east_m = spec.east0_m + spec.ve_mps * t_s;
        let north_m = spec.north0_m + spec.vn_mps * t_s;
        let vd_mps = -spec.climb_fpm * FPM_TO_MPS;
        let alt_m = own_alt_m + spec.rel_alt_ft * FEET_TO_METRES - vd_mps * t_s;
        let pos = plane.to_geodetic(Ned {
            north_m,
            east_m,
            down_m: 0.0,
        });
        let obs =
            build_scripted_observation(spec, pos.latitude_deg, pos.longitude_deg, alt_m, now_us);
        traffic.ingest_observation(obs)?;
    }
    Ok(())
}

fn build_scripted_observation(
    spec: &ScriptedTargetSpec,
    latitude_deg: f64,
    longitude_deg: f64,
    alt_m: f64,
    now_us: u64,
) -> TrafficObservation {
    let ground_speed_kt = libm::hypot(spec.ve_mps, spec.vn_mps) / KNOTS_TO_MPS;
    let raw_deg = libm::atan2(spec.ve_mps, spec.vn_mps)
        .to_degrees()
        .rem_euclid(360.0);
    let track_deg = if (0.0..360.0).contains(&raw_deg) {
        raw_deg
    } else {
        0.0
    };
    let mut velocity = VelocityObservation::default();
    velocity.ground_speed_kt = Some(ground_speed_kt);
    velocity.track_angle_deg_true = Some(track_deg);
    velocity.vertical_rate_fpm = Some(spec.climb_fpm.round() as i32);
    velocity.vertical_rate_source = Some(VerticalRateSource::Barometric);

    let mut obs = TrafficObservation::new(
        ObservationTime {
            received_at_micros: now_us,
            observed_at_micros: now_us,
            basis: TimeBasis::LocalIngress,
        },
        SourceRef::new(1, 0, DeliveryPath::Simulator),
        ObservationOrigin::AdsbDirect {
            band: Band::Adsb1090,
        },
        TrackKey::new(AddressNamespace::Icao, spec.icao),
    );
    obs.position = Some(PositionObservation::Decoded(Wgs84Position {
        latitude_deg,
        longitude_deg,
    }));
    obs.geometric_altitude_ft = Some((alt_m.max(20.0) / FEET_TO_METRES).round() as i32);
    obs.velocity = Some(velocity);
    obs.callsign = Some(Callsign::new(spec.callsign));
    obs.emitter_category = Some(spec.category);
    obs.air_ground = Some(AirGroundState::Airborne);
    obs.quality = FieldQuality::new(Some(9), Some(9), None, Some(3));
    obs
}

pub(super) const CRUISE_TARGETS: [ScriptedTargetSpec; 4] = [
    ScriptedTargetSpec {
        icao: 0xA8_FCAA,
        callsign: "JBU1990",
        east0_m: 4_800.0,
        north0_m: 5_200.0,
        rel_alt_ft: 700.0,
        ve_mps: -42.0,
        vn_mps: -35.0,
        climb_fpm: -450.0,
        category: EmitterCategory::LargeAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xAA_430C,
        callsign: "DWI2363",
        east0_m: 7_400.0,
        north0_m: 1_900.0,
        rel_alt_ft: 1_100.0,
        ve_mps: -25.0,
        vn_mps: 38.0,
        climb_fpm: 550.0,
        category: EmitterCategory::LargeAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xA4_E211,
        callsign: "N412SP",
        east0_m: -1_800.0,
        north0_m: 3_100.0,
        rel_alt_ft: -300.0,
        ve_mps: 34.0,
        vn_mps: -18.0,
        climb_fpm: 0.0,
        category: EmitterCategory::LightAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xEA_60A3,
        callsign: "ATN3345",
        east0_m: 9_200.0,
        north0_m: 6_800.0,
        rel_alt_ft: 1_800.0,
        ve_mps: -55.0,
        vn_mps: -48.0,
        climb_fpm: -600.0,
        category: EmitterCategory::HeavyAircraft,
    },
];

pub(super) const CPA_TARGETS: [ScriptedTargetSpec; 4] = [
    ScriptedTargetSpec {
        icao: 0xAB_8420,
        callsign: "UAL842",
        east0_m: 2_350.0,
        north0_m: 1_650.0,
        rel_alt_ft: 200.0,
        ve_mps: -78.0,
        vn_mps: -54.0,
        climb_fpm: -350.0,
        category: EmitterCategory::LargeAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xA9_7390,
        callsign: "N739TX",
        east0_m: 1_400.0,
        north0_m: -950.0,
        rel_alt_ft: -150.0,
        ve_mps: -36.0,
        vn_mps: 34.0,
        climb_fpm: 320.0,
        category: EmitterCategory::LightAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xA3_1184,
        callsign: "DAL1184",
        east0_m: 5_800.0,
        north0_m: 3_600.0,
        rel_alt_ft: 900.0,
        ve_mps: -60.0,
        vn_mps: -30.0,
        climb_fpm: 800.0,
        category: EmitterCategory::LargeAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xA8_FCAA,
        callsign: "JBU1990",
        east0_m: -3_900.0,
        north0_m: 4_100.0,
        rel_alt_ft: 1_400.0,
        ve_mps: 52.0,
        vn_mps: -22.0,
        climb_fpm: 0.0,
        category: EmitterCategory::LargeAircraft,
    },
];

pub(super) const APPROACH_TARGETS: [ScriptedTargetSpec; 3] = [
    ScriptedTargetSpec {
        icao: 0xA4_09AA,
        callsign: "AAL409",
        east0_m: -2_400.0,
        north0_m: -4_200.0,
        rel_alt_ft: -650.0,
        ve_mps: -32.0,
        vn_mps: -56.0,
        climb_fpm: -650.0,
        category: EmitterCategory::LargeAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xAC_3812,
        callsign: "SKW3812",
        east0_m: 2_900.0,
        north0_m: -1_800.0,
        rel_alt_ft: 500.0,
        ve_mps: -30.0,
        vn_mps: -52.0,
        climb_fpm: -500.0,
        category: EmitterCategory::SmallAircraft,
    },
    ScriptedTargetSpec {
        icao: 0xA1_5520,
        callsign: "N52HE",
        east0_m: 3_600.0,
        north0_m: 2_200.0,
        rel_alt_ft: -200.0,
        ve_mps: -18.0,
        vn_mps: 24.0,
        climb_fpm: 0.0,
        category: EmitterCategory::Rotorcraft,
    },
];
