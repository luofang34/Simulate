//! Truth traffic around the active airport and the radio-filtered belief the avionics
//! see, as scene entities.

use glam::DVec3;
use sim_core::SimInstant;
use sim_geo::{Geodetic, UtcTime, egm96_undulation_m};
use sim_maplibre::{
    AltitudeReference, DynamicEntity3D, ViewSlotMask, entities_from_surveillance_belief,
};
use sim_scenario::ScenarioProfile;
use sim_scenery::CatalogVerticalDatum;
use world::surveillance::SurveillanceStore;
use world::surveillance::{
    AddressNamespace, Callsign, DeliveryPath, EmitterCategory, IntegrityQuality, ObservationSource,
    TrackKey, TrackObservation,
};
use world::{GeodeticPosition, LocalTangentPlane, Ned};

use super::{
    DELAWARE_RUNWAY_BEARING_RAD, DELAWARE_RUNWAY_HEIGHT_M, DELAWARE_RUNWAY_LAT_DEG,
    DELAWARE_RUNWAY_LON_DEG, SupervisorRig,
};

impl SupervisorRig {
    /// Updates omniscient truth traffic kinematics at `now` and projects radio-visible tracks
    /// into [`sim_scenario::SimSupervisor::operator_view`]'s `belief_world`, on a flight
    /// that began at `start_utc`.
    pub fn sync(
        &mut self,
        ownship_ecef: DVec3,
        now: SimInstant,
        start_utc: UtcTime,
        profile: Option<ScenarioProfile>,
    ) {
        let t_s = now.duration_since(SimInstant::START).as_seconds().max(0.0);
        let now_us = (t_s * 1_000_000.0) as u64;
        let anchor = self.traffic_anchor(profile);
        self.ingest_truth_traffic(anchor, t_s, now_us);
        self.supervisor.sync_belief(ownship_ecef, now, start_utc);
    }

    /// Current revision of the operator's `belief_world` surveillance store.
    #[must_use]
    pub fn belief_surveillance_revision(&self) -> u64 {
        self.supervisor
            .operator_view()
            .belief_world
            .surveillance
            .revision()
    }

    /// Avionics-belief 3D traffic entities derived strictly from `OperatorBeliefView::belief_world`.
    ///
    /// Airborne `AboveMsl` heights are orthometric (`H = h − N`) in
    /// [`CatalogVerticalDatum::OrthometricMslEgm96`] and ellipsoidal in
    /// [`CatalogVerticalDatum::EllipsoidWgs84`], matching the scene's ownship placement and
    /// `ConformalFrameStamp::geoid_undulation_m`.
    #[must_use]
    pub fn belief_entities(&self, visibility: ViewSlotMask) -> Vec<DynamicEntity3D> {
        let store = &self.supervisor.operator_view().belief_world.surveillance;
        entities_for_datum(store, self.datum, visibility)
    }

    /// Omniscient physical-truth 3D traffic entities (rendered only in `TruthSceneMap`), in
    /// the same vertical datum as [`Self::belief_entities`].
    #[must_use]
    pub fn truth_entities(&self, visibility: ViewSlotMask) -> Vec<DynamicEntity3D> {
        entities_for_datum(
            &self.supervisor.truth_world().surveillance,
            self.datum,
            visibility,
        )
    }

    /// Active airport reference point in orthometric metres above EGM96 mean sea level.
    #[must_use]
    pub fn active_anchor_msl(&self, profile: Option<ScenarioProfile>) -> Geodetic {
        let anchor = self.traffic_anchor(profile);
        let msl_m = match self.datum {
            CatalogVerticalDatum::OrthometricMslEgm96 => {
                anchor.ellipsoidal_height_m
                    - egm96_undulation_m(
                        anchor.latitude_deg.to_radians(),
                        anchor.longitude_deg.to_radians(),
                    )
            }
            CatalogVerticalDatum::EllipsoidWgs84 => anchor.ellipsoidal_height_m,
        };
        Geodetic::from_degrees(anchor.latitude_deg, anchor.longitude_deg, msl_m)
    }

    /// Publishes active `transponder == true` NPCs into `truth_world().surveillance`.
    pub fn ingest_npc_transponders(
        &mut self,
        npcs: &[crate::npc_fleet::EvaluatedNpc],
        now: SimInstant,
    ) {
        let t_s = now.duration_since(SimInstant::START).as_seconds().max(0.0);
        let now_us = (t_s * 1_000_000.0) as u64;
        for (idx, npc) in npcs.iter().enumerate() {
            if !npc.transponder {
                continue;
            }
            let lat = npc.placement.position.latitude;
            let lon = npc.placement.position.longitude;
            let undul_m = match self.datum {
                CatalogVerticalDatum::OrthometricMslEgm96 => {
                    egm96_undulation_m(lat.to_radians(), lon.to_radians())
                }
                CatalogVerticalDatum::EllipsoidWgs84 => 0.0,
            };
            let key = TrackKey {
                namespace: AddressNamespace::Icao24,
                address: 0x00FE_0000_u64.wrapping_add(idx as u64),
            };
            let obs = TrackObservation {
                key,
                source: ObservationSource::Adsb1090,
                delivery: DeliveryPath::Simulator,
                observed_at_us: now_us,
                position: GeodeticPosition::wgs84(
                    lat,
                    lon,
                    npc.placement.altitude_meters + undul_m,
                ),
                velocity_ned_m_s: Ned {
                    north_m: npc.velocity_ned_mps[0],
                    east_m: npc.velocity_ned_mps[1],
                    down_m: npc.velocity_ned_mps[2],
                },
                heading_rad: Some(npc.heading_rad),
                callsign: Callsign::parse(&npc.callsign),
                category: EmitterCategory::Heavy,
                on_ground: npc.ground_height_msl_m.is_some(),
                quality: IntegrityQuality::default(),
            };
            let store = &mut self.supervisor.truth_world_mut().surveillance;
            let _outcome = store.ingest(obs, now_us);
            if let Some(track) = store.tracks().iter().find(|t| t.key == key) {
                self.npc_track_ids.insert(track.id.0);
            }
        }
    }

    /// Whether surveillance track `id` originates from a simulated [`crate::npc_fleet::NpcFleet`] entry.
    #[must_use]
    pub fn is_npc_track(&self, id: u64) -> bool {
        self.npc_track_ids.contains(&id)
    }

    fn ingest_truth_traffic(&mut self, anchor: GeodeticPosition, t_s: f64, now_us: u64) {
        let Ok(plane) = LocalTangentPlane::new(anchor) else {
            return;
        };
        let near_pos = plane.to_geodetic(Ned {
            north_m: 850.0 + 12.0 * (t_s * 0.25).sin() * 20.0,
            east_m: 2_800.0 + 46.0 * t_s,
            down_m: -420.0,
        });
        let _near_outcome = self.supervisor.truth_world_mut().surveillance.ingest(
            TrackObservation {
                key: TrackKey {
                    namespace: AddressNamespace::Icao24,
                    address: 0xA1_B2_C3,
                },
                source: ObservationSource::Adsb1090,
                delivery: DeliveryPath::Simulator,
                observed_at_us: now_us,
                position: near_pos,
                velocity_ned_m_s: Ned {
                    north_m: 3.0,
                    east_m: 46.0,
                    down_m: 0.0,
                },
                heading_rad: Some(DELAWARE_RUNWAY_BEARING_RAD),
                callsign: Callsign::parse("N172SP"),
                category: EmitterCategory::Light,
                on_ground: false,
                quality: IntegrityQuality::default(),
            },
            now_us,
        );

        let far_pos = plane.to_geodetic(Ned {
            north_m: 1_200.0,
            east_m: 45_000.0,
            down_m: -120.0,
        });
        let _far_outcome = self.supervisor.truth_world_mut().surveillance.ingest(
            TrackObservation {
                key: TrackKey {
                    namespace: AddressNamespace::RemoteId,
                    address: 0xD0_00_99,
                },
                source: ObservationSource::RemoteId,
                delivery: DeliveryPath::Simulator,
                observed_at_us: now_us,
                position: far_pos,
                velocity_ned_m_s: Ned {
                    north_m: 0.0,
                    east_m: 8.0,
                    down_m: 0.0,
                },
                heading_rad: Some(1.57),
                callsign: Callsign::parse("UAS99"),
                category: EmitterCategory::Uav,
                on_ground: false,
                quality: IntegrityQuality::default(),
            },
            now_us,
        );
    }

    /// The point traffic circulates around: the active airport's reference point at its
    /// true ellipsoidal height in [`Self::vertical_datum`].
    fn traffic_anchor(&self, profile: Option<ScenarioProfile>) -> GeodeticPosition {
        if let Some(record) = self.active_airport_record(profile) {
            let arp = record.arp_geodetic(self.datum);
            return GeodeticPosition::wgs84(
                arp.lat_rad.to_degrees(),
                arp.lon_rad.to_degrees(),
                arp.height_m,
            );
        }
        self.diagram(self.effective_aerodrome(profile)).map_or_else(
            || {
                GeodeticPosition::wgs84(
                    DELAWARE_RUNWAY_LAT_DEG,
                    DELAWARE_RUNWAY_LON_DEG,
                    DELAWARE_RUNWAY_HEIGHT_M,
                )
            },
            |diagram| {
                diagram
                    .to_flightbag_graph_for_datum(self.datum)
                    .reference_point
            },
        )
    }
}

fn entities_for_datum(
    store: &SurveillanceStore,
    datum: CatalogVerticalDatum,
    visibility: ViewSlotMask,
) -> Vec<DynamicEntity3D> {
    let mut entities = entities_from_surveillance_belief(store, 0.0);
    for entity in &mut entities {
        entity.placement.view_mask = visibility;
        if datum == CatalogVerticalDatum::OrthometricMslEgm96
            && entity.placement.altitude_reference == AltitudeReference::AboveMsl
        {
            let lat_rad = entity.placement.position.latitude.to_radians();
            let lon_rad = entity.placement.position.longitude.to_radians();
            entity.placement.altitude_meters -= egm96_undulation_m(lat_rad, lon_rad);
        }
    }
    entities
}
