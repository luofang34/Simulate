use flightbag::{
    AirportSurfaceGraph, FacilityCode, ParkingKind, ParkingSpot, RunwayDesignator, RunwayEndSpec,
};
use sim_core::SimInstant;
use sim_geo::{Geodetic, UtcTime, geodetic_to_ecef};
use world::notices::{
    ClosureStatus, FacilityKey, NoticeId, RunwayKey, SurfaceClosureNotice, SurfaceElement,
};
use world::surveillance::{
    AddressNamespace, Callsign, DeliveryPath, EmitterCategory, IntegrityQuality, ObservationSource,
    TrackKey, TrackObservation,
};
use world::{DecimalYear, GeodeticPosition, LocalTangentPlane, Ned, ValidityWindow};

use crate::{
    OperatorViewportMode, RadioBeliefPolicy, RunwaySpawnError, SimSupervisor, SimTimeScale,
};

fn sample_airport() -> AirportSurfaceGraph {
    let kdov = FacilityCode::parse("KDOV").unwrap();
    let rwy01 = RunwayDesignator::parse("01").unwrap();
    AirportSurfaceGraph {
        facility: kdov,
        reference_point: GeodeticPosition::wgs84(39.1295, -75.4659, 8.5),
        runway_ends: vec![RunwayEndSpec {
            designator: rwy01,
            threshold: GeodeticPosition::wgs84(39.1170, -75.4680, 8.5),
            true_bearing_rad: 0.12,
            length_m: 2_926.0,
            width_m: 45.7,
        }],
        taxi_edges: vec![],
        hold_short_nodes: vec![],
        parking_spots: vec![ParkingSpot {
            label: "RAMP 1".to_owned(),
            kind: ParkingKind::GeneralAviationRamp,
            position: GeodeticPosition::wgs84(39.1280, -75.4620, 8.5),
            heading_rad: 1.57,
            connected_node: None,
        }],
    }
}

#[test]
fn radio_belief_filters_out_of_range_traffic_while_preserving_truth() {
    let mut sup = SimSupervisor::new();
    sup.set_time_scale(SimTimeScale::Warp(4.0));
    assert_eq!(sup.time_scale().factor(), 4.0);
    sup.set_viewport_mode(OperatorViewportMode::VolumetricScene);
    sup.set_radio_policy(RadioBeliefPolicy {
        adsb_range_m: 50_000.0,
        short_range_radio_m: 5_000.0,
        receive_weather: true,
        receive_notices: true,
    });

    let own = GeodeticPosition::wgs84(39.13, -75.47, 500.0);
    let plane = LocalTangentPlane::new(own).unwrap();

    // Near drone at 2 km (within 5 km Remote ID range) and far drone at 12 km (beyond 5 km).
    let near_pos = plane.to_geodetic(Ned {
        north_m: 2_000.0,
        east_m: 0.0,
        down_m: 0.0,
    });
    let far_pos = plane.to_geodetic(Ned {
        north_m: 12_000.0,
        east_m: 0.0,
        down_m: 0.0,
    });
    for (addr, pos) in [(1u64, near_pos), (2u64, far_pos)] {
        sup.truth_world_mut()
            .surveillance
            .ingest(
                TrackObservation {
                    key: TrackKey {
                        namespace: AddressNamespace::RemoteId,
                        address: addr,
                    },
                    source: ObservationSource::RemoteId,
                    delivery: DeliveryPath::Simulator,
                    observed_at_us: 0,
                    position: pos,
                    velocity_ned_m_s: Ned {
                        north_m: -10.0,
                        east_m: 0.0,
                        down_m: 0.0,
                    },
                    heading_rad: None,
                    callsign: Callsign::parse("UAS1"),
                    category: EmitterCategory::Uav,
                    on_ground: false,
                    quality: IntegrityQuality::default(),
                },
                0,
            )
            .unwrap();
    }

    let own_ecef = geodetic_to_ecef(&Geodetic::from_degrees(39.13, -75.47, 500.0));
    sup.sync_belief(
        own_ecef,
        SimInstant::START,
        UtcTime::from_unix_seconds(1_780_000_000.0),
    );

    // Truth sees both targets; single-copy operator belief sees only the in-range target.
    assert_eq!(sup.truth_world().surveillance.tracks().len(), 2);
    let op = sup.operator_view();
    assert_eq!(op.viewport_mode, OperatorViewportMode::VolumetricScene);
    assert_eq!(op.belief_world.surveillance.tracks().len(), 1);
    assert_eq!(op.belief_world.surveillance.tracks()[0].key.address, 1);
}

#[test]
fn flightbag_runway_and_parking_spawn_respect_closure_notices() {
    let mut sup = SimSupervisor::new();
    let airport = sample_airport();
    let rwy01 = RunwayDesignator::parse("01").unwrap();
    let at = DecimalYear(2026.5);

    let spawned = sup.spawn_on_runway(&airport, rwy01, 0.0, 30.0, at).unwrap();
    assert!(spawned.velocity_ecef.length() > 29.0);

    let parked = sup.spawn_at_parking(&airport, "RAMP 1").unwrap();
    assert_eq!(parked.velocity_ecef.length(), 0.0);

    // Close runway 01 via world-notices and verify spawn refusal.
    sup.truth_world_mut()
        .notices
        .upsert_surface_closure(SurfaceClosureNotice {
            id: NoticeId::parse("DOV 10/099").unwrap(),
            facility: FacilityKey::parse("KDOV").unwrap(),
            element: SurfaceElement::Runway(RunwayKey::parse("01").unwrap()),
            status: ClosureStatus::Closed,
            valid: ValidityWindow {
                from: DecimalYear(2026.0),
                until: DecimalYear(2027.0),
            },
            cancelled: false,
        })
        .unwrap();

    let refused = sup.spawn_on_runway(&airport, rwy01, 0.0, 30.0, at);
    assert_eq!(refused, Err(RunwaySpawnError::ClosedByNotice));
}
