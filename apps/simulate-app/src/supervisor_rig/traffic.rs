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
use world::surveillance::{
    AddressNamespace, AirGroundState, Band, Callsign, DeliveryPath, EmitterCategory,
    ObservationOrigin, ObservationTime, PositionObservation, SourceRef, SurveillanceEngine,
    TimeBasis, TrackKey, TrafficObservation, VelocityObservation, Wgs84Position,
};
use world::{GeodeticPosition, LocalTangentPlane, Ned};

use super::{
    DELAWARE_RUNWAY_BEARING_RAD, DELAWARE_RUNWAY_HEIGHT_M, DELAWARE_RUNWAY_LAT_DEG,
    DELAWARE_RUNWAY_LON_DEG, SupervisorRig,
};

const METRES_PER_FOOT: f64 = 0.3048;
const KNOTS_TO_MPS: f64 = 1_852.0 / 3_600.0;
const FPM_TO_MPS: f64 = METRES_PER_FOOT / 60.0;

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
    pub const fn belief_surveillance_revision(&self) -> u64 {
        self.supervisor.belief_surveillance_revision()
    }

    /// Avionics-belief 3D traffic entities derived strictly from `OperatorBeliefView::belief_world`.
    ///
    /// Airborne `AboveMsl` heights are orthometric (`H = h − N`) in
    /// [`CatalogVerticalDatum::OrthometricMslEgm96`] and ellipsoidal in
    /// [`CatalogVerticalDatum::EllipsoidWgs84`], matching the scene's ownship placement and
    /// `ConformalFrameStamp::geoid_undulation_m`.
    #[must_use]
    pub fn belief_entities(&self, visibility: ViewSlotMask) -> Vec<DynamicEntity3D> {
        let engine = &self.supervisor.operator_view().belief_world.surveillance;
        entities_for_datum(engine, self.datum, visibility)
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
            let key = TrackKey::new(
                AddressNamespace::Icao,
                0x00FE_0000_u32.wrapping_add(idx as u32),
            );
            let alt_m = npc.placement.altitude_meters + undul_m;
            let vel = velocity_from_ned(
                npc.velocity_ned_mps[0],
                npc.velocity_ned_mps[1],
                npc.velocity_ned_mps[2],
                Some(npc.heading_rad),
            );
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
                key,
            );
            obs.position = Some(PositionObservation::Decoded(Wgs84Position {
                latitude_deg: lat,
                longitude_deg: lon,
            }));
            obs.geometric_altitude_ft = Some((alt_m / METRES_PER_FOOT).round() as i32);
            obs.velocity = Some(vel);
            obs.callsign =
                (!npc.callsign.trim().is_empty()).then(|| Callsign::new(npc.callsign.trim()));
            obs.emitter_category = Some(EmitterCategory::HeavyAircraft);
            obs.air_ground = Some(if npc.ground_height_msl_m.is_some() {
                AirGroundState::Ground
            } else {
                AirGroundState::Airborne
            });
            let engine = &mut self.supervisor.truth_world_mut().surveillance;
            let _outcome = engine.ingest(obs, |_| {});
            if let Some(track) = engine.snapshot(key) {
                self.npc_track_ids.insert(track.id.get());
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
        let near_obs = build_traffic_obs(
            TrackKey::new(AddressNamespace::Icao, 0xA1_B2_C3),
            ObservationOrigin::AdsbDirect {
                band: Band::Adsb1090,
            },
            near_pos,
            velocity_from_ned(3.0, 46.0, 0.0, Some(DELAWARE_RUNWAY_BEARING_RAD)),
            "N172SP",
            EmitterCategory::LightAircraft,
            now_us,
        );
        let _near_outcome = self
            .supervisor
            .truth_world_mut()
            .surveillance
            .ingest(near_obs, |_| {});

        let far_pos = plane.to_geodetic(Ned {
            north_m: 1_200.0,
            east_m: 45_000.0,
            down_m: -120.0,
        });
        let far_obs = build_traffic_obs(
            TrackKey::new(AddressNamespace::SelfAssigned, 0xD0_00_99),
            ObservationOrigin::Unknown,
            far_pos,
            velocity_from_ned(0.0, 8.0, 0.0, Some(1.57)),
            "UAS99",
            EmitterCategory::UncrewedAircraft,
            now_us,
        );
        let _far_outcome = self
            .supervisor
            .truth_world_mut()
            .surveillance
            .ingest(far_obs, |_| {});
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

fn build_traffic_obs(
    key: TrackKey,
    origin: ObservationOrigin,
    pos: GeodeticPosition,
    vel: VelocityObservation,
    callsign: &str,
    category: EmitterCategory,
    now_us: u64,
) -> TrafficObservation {
    let mut obs = TrafficObservation::new(
        ObservationTime {
            received_at_micros: now_us,
            observed_at_micros: now_us,
            basis: TimeBasis::LocalIngress,
        },
        SourceRef::new(1, 0, DeliveryPath::Simulator),
        origin,
        key,
    );
    obs.position = Some(PositionObservation::Decoded(Wgs84Position {
        latitude_deg: pos.latitude_deg,
        longitude_deg: pos.longitude_deg,
    }));
    obs.geometric_altitude_ft = Some((pos.ellipsoidal_height_m / METRES_PER_FOOT).round() as i32);
    obs.velocity = Some(vel);
    obs.callsign = Some(Callsign::new(callsign));
    obs.emitter_category = Some(category);
    obs.air_ground = Some(AirGroundState::Airborne);
    obs
}

fn velocity_from_ned(vn: f64, ve: f64, vd: f64, heading_rad: Option<f64>) -> VelocityObservation {
    let gs_mps = vn.hypot(ve);
    let trk_deg = normalize_deg_360(ve.atan2(vn).to_degrees());
    let mut vel = VelocityObservation::default();
    vel.ground_speed_kt = Some(gs_mps / KNOTS_TO_MPS);
    vel.track_angle_deg_true = trk_deg;
    vel.heading_deg = heading_rad.and_then(|r| normalize_deg_360(r.to_degrees()));
    vel.vertical_rate_fpm = Some(((-vd) / FPM_TO_MPS).round() as i32);
    vel
}

fn normalize_deg_360(deg: f64) -> Option<f64> {
    if !deg.is_finite() {
        return None;
    }
    let wrapped = deg.rem_euclid(360.0);
    if !(0.0..360.0).contains(&wrapped) {
        Some(0.0)
    } else {
        Some(wrapped)
    }
}

fn entities_for_datum(
    store: &SurveillanceEngine,
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
