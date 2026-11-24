//! ADS-B / TIS-B multi-source traffic fusion, callsign labeling, CPA threat classification,
//! and `Indicate` HUD `TargetPicture` projection (`world::surveillance` + `world::traffic`).

mod projection;

use crate::connext::nmea::ConnextTrafficTarget;
use crate::error::FlightDeckError;
use crate::gdl90::Gdl90ParticipantReport;
use crate::planet::PlanetaryReference;
use crate::telemetry::{FEET_TO_METRES, FPM_TO_MPS, KNOTS_TO_MPS, OwnshipState};
use indicate_instrument_hud::{MAX_CONTACTS, TargetPicture};
use projection::{build_display_target, to_hud_contact};
use world::surveillance::{
    AddressNamespace, AirGroundState, Band, Callsign, DeliveryPath, EngineConfig, FieldQuality,
    ObservationOrigin, ObservationTime, PositionObservation, ProducerInstanceId, SourceRef,
    SurveillanceEngine, TimeBasis, TrackKey, TrafficObservation, VelocityObservation,
    VerticalRateSource, Wgs84Position,
};
use world::traffic::{Ownship, relative_to_ownship};
use world_core::{GeodeticPosition, LocalTangentPlane, Ned};

const METRES_PER_NM: f64 = 1_852.0;

/// Cockpit traffic threat tier derived from ownship-relative range, altitude, and CPA closure.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum TrafficThreat {
    /// Non-threatening traffic outside proximate bounds (open cyan/white diamond).
    Other = 0,
    /// Within `6 NM` horizontally and `±1,200 ft` vertically (filled cyan/white diamond).
    Proximate = 1,
    /// Traffic Advisory (`TA`): closing CPA `< 45 s` within `1.2 NM` / `±800 ft` or `< 0.55 NM` (amber circle).
    TrafficAdvisory = 2,
    /// Resolution / Collision Warning (`RA`): closing CPA `< 25 s` within `0.45 NM` / `±600 ft` (red square).
    ResolutionAdvisory = 3,
}

/// 3D traffic contact with resolved callsign and relative-altitude placard for XR rendering.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficDisplayTarget {
    /// Stable track ID assigned by [`SurveillanceEngine`].
    pub track_id: u64,
    /// 24-bit ICAO or track address formatted as 6 uppercase hex digits (e.g. `"A8FCAA"`).
    pub address_hex: String,
    /// Display callsign (flight ID such as `"JBU1990"`, or `"A8FCAA"` when callsign not yet broadcast).
    pub callsign: String,
    /// Formatted XR placard string (e.g. `"JBU1990 +12↑ 2.4NM"`).
    pub placard: String,
    /// Offset from ownship in local East-North-Up metres (`[east_m, north_m, up_m]`).
    pub offset_enu_m: [f32; 3],
    /// Velocity in local East-North-Up metres per second (`[ve, vn, vu]`).
    pub velocity_enu_mps: [f32; 3],
    /// True ground track or heading in degrees `[0, 360)` when known.
    pub track_true_deg: Option<f32>,
    /// Slant range from ownship in nautical miles.
    pub slant_range_nm: f32,
    /// Relative altitude (`target - ownship`) rounded to hundreds of feet (`+12` = `+1,200 ft`).
    pub rel_altitude_hundreds_ft: i32,
    /// Vertical rate trend (`+1` climbing `>= +300 fpm`, `-1` descending `<= -300 fpm`, `0` level).
    pub vertical_trend: i8,
    /// Time to horizontal closest point of approach in seconds (`0` if diverging).
    pub time_to_cpa_s: f32,
    /// Horizontal separation at CPA in metres.
    pub cpa_horizontal_m: f32,
    /// Evaluated cockpit threat level.
    pub threat: TrafficThreat,
    /// Whether the target is on the airport surface (`true`) or airborne (`false`).
    pub on_ground: bool,
    /// Whether the track is actively updated (`true`) or coasting (`false`).
    pub active: bool,
}

/// Stateful coordinator wrapping [`SurveillanceEngine`] for Connext, GDL-90, and simulated traffic.
#[derive(Debug)]
pub struct TrafficCoordinator {
    engine: SurveillanceEngine,
    producer_seq: u64,
}

impl TrafficCoordinator {
    /// Creates an empty [`TrafficCoordinator`].
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Surveillance`] if engine configuration validation fails.
    pub fn new() -> Result<Self, FlightDeckError> {
        let producer_seq = 1;
        let engine = SurveillanceEngine::new(
            EngineConfig::default(),
            ProducerInstanceId::new(producer_seq),
        )
        .map_err(|e| FlightDeckError::Surveillance {
            reason: e.to_string(),
        })?;
        Ok(Self {
            engine,
            producer_seq,
        })
    }

    /// Resets all tracked traffic (used when switching telemetry sources).
    pub fn clear(&mut self) {
        let next_seq = self.producer_seq.wrapping_add(1);
        if let Ok(fresh) =
            SurveillanceEngine::new(EngineConfig::default(), ProducerInstanceId::new(next_seq))
        {
            self.producer_seq = next_seq;
            self.engine = fresh;
        }
    }

    /// Underlying [`SurveillanceEngine`] reference.
    #[must_use]
    pub const fn engine(&self) -> &SurveillanceEngine {
        &self.engine
    }

    /// Updates ownship Mode-S address so ownship ADS-B/TIS-B echoes are tagged `ownship_shadow`.
    pub fn set_ownship_mode_s(&mut self, mode_s_code: Option<u32>) {
        if let Some(code) = mode_s_code {
            let key = TrackKey::new(AddressNamespace::Icao, code & 0x00FF_FFFF);
            let _res = self
                .engine
                .set_ownship_identities(core::slice::from_ref(&key), |_| {});
        } else {
            let _res = self.engine.set_ownship_identities(&[], |_| {});
        }
    }

    /// Ingests one Connext service `0x30` traffic target at monotonic `now_ms`.
    ///
    /// When consecutive positions arrive without explicit velocity fields (as in Connext's
    /// 31-byte slot table), finite-difference horizontal/vertical velocity is estimated from the
    /// previous fix so CPA vectors and HUD target velocity leaders stay live.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Surveillance`] if coordinates are out of WGS-84 range.
    pub fn ingest_connext_target(
        &mut self,
        target: &ConnextTrafficTarget,
        delivery: DeliveryPath,
        now_ms: u64,
    ) -> Result<(), FlightDeckError> {
        let now_us = now_ms.saturating_mul(1_000);
        let key = TrackKey::new(AddressNamespace::Icao, target.raw_id & 0x00FF_FFFF);
        let alt_m = target
            .pressure_altitude_ft
            .map_or(0.0, |ft| f64::from(ft) * FEET_TO_METRES);
        let new_pos = GeodeticPosition::wgs84(target.latitude_deg, target.longitude_deg, alt_m);
        let velocity = self.estimate_connext_velocity(key, new_pos, now_us);

        let mut obs = TrafficObservation::new(
            ObservationTime {
                received_at_micros: now_us,
                observed_at_micros: now_us,
                basis: TimeBasis::LocalIngress,
            },
            SourceRef::new(1, 0, delivery),
            ObservationOrigin::AdsbDirect {
                band: Band::Adsb1090,
            },
            key,
        );
        obs.position = Some(PositionObservation::Decoded(Wgs84Position {
            latitude_deg: target.latitude_deg,
            longitude_deg: target.longitude_deg,
        }));
        obs.pressure_altitude_ft = target.pressure_altitude_ft;
        obs.velocity = velocity;
        obs.callsign = target.callsign.as_deref().and_then(sanitize_callsign);
        obs.air_ground = Some(AirGroundState::Airborne);
        self.ingest_observation(obs)
    }

    /// Ingests one GDL-90 `0x14` traffic participant report at monotonic `now_ms`.
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Surveillance`] if coordinates or velocities are malformed.
    pub fn ingest_gdl90_target(
        &mut self,
        report: &Gdl90ParticipantReport,
        now_ms: u64,
    ) -> Result<(), FlightDeckError> {
        let now_us = now_ms.saturating_mul(1_000);
        let mut obs = TrafficObservation::new(
            ObservationTime {
                received_at_micros: now_us,
                observed_at_micros: now_us,
                basis: TimeBasis::LocalIngress,
            },
            SourceRef::new(1, 0, DeliveryPath::LocalReceiver),
            ObservationOrigin::AdsbDirect {
                band: Band::Adsb1090,
            },
            TrackKey::new(report.namespace, report.address),
        );
        obs.position = Some(PositionObservation::Decoded(Wgs84Position {
            latitude_deg: report.latitude_deg,
            longitude_deg: report.longitude_deg,
        }));
        obs.pressure_altitude_ft = report.pressure_altitude_ft;
        obs.velocity = build_gdl90_velocity(report);
        obs.callsign = report.callsign.as_deref().and_then(sanitize_callsign);
        obs.emitter_category = report.emitter_category;
        obs.air_ground = Some(if report.airborne {
            AirGroundState::Airborne
        } else {
            AirGroundState::Ground
        });
        obs.quality = FieldQuality::new(
            (report.nic <= 11).then_some(report.nic),
            (report.nac_p <= 11).then_some(report.nac_p),
            None,
            None,
        );
        self.ingest_observation(obs)
    }

    /// Ingests a direct [`TrafficObservation`] (used by deterministic traffic/CPA scenarios).
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Surveillance`] if rejected by [`SurveillanceEngine`].
    pub fn ingest_observation(&mut self, obs: TrafficObservation) -> Result<(), FlightDeckError> {
        self.engine
            .ingest(obs, |_| {})
            .map(|_| ())
            .map_err(|e| FlightDeckError::Surveillance {
                reason: e.to_string(),
            })
    }

    /// Advances track aging and computes 3D local ENU display targets + HUD [`TargetPicture`]
    /// when `planet` is `WGS-84` Earth.
    #[must_use]
    pub fn snapshot(
        &mut self,
        ownship: &OwnshipState,
        planet: &PlanetaryReference,
        designated_track_id: Option<u64>,
        now_ms: u64,
    ) -> (Vec<TrafficDisplayTarget>, TargetPicture) {
        let now_us = now_ms.saturating_mul(1_000);
        let _adv = self.engine.advance_time(now_us, |_| {});
        self.set_ownship_mode_s(ownship.identity.as_ref().map(|id| id.mode_s_code));

        let Some(mut own_geo) = ownship.valid_earth_wgs84_position(planet) else {
            return (Vec::new(), TargetPicture::default());
        };
        if let Some(best_alt_m) = ownship.best_altitude_m() {
            own_geo.ellipsoidal_height_m = best_alt_m;
        }
        let own_vel_ned = ownship_velocity_ned(ownship);
        let own_world = Ownship {
            position: own_geo,
            velocity_ned_m_s: own_vel_ned,
            pressure_altitude_ft: ownship.pressure_altitude_ft,
        };

        let mut display_targets = Vec::with_capacity(self.engine.track_count());
        let mut picture = TargetPicture::default();
        let mut designated_hud_idx: Option<usize> = None;

        for handle in self.engine.snapshots() {
            let track = handle.snapshot();
            let Ok(rel) = relative_to_ownship(&own_world, track, now_us) else {
                continue;
            };
            let target = build_display_target(track, &rel);
            if picture.contacts().len() < MAX_CONTACTS {
                let contact = to_hud_contact(track, &rel, &own_vel_ned);
                if let Some(idx) = picture.push(contact)
                    && (designated_track_id == Some(target.track_id)
                        || (designated_hud_idx.is_none()
                            && target.threat >= TrafficThreat::TrafficAdvisory))
                {
                    designated_hud_idx = Some(idx);
                }
            }
            display_targets.push(target);
        }
        picture.designate(designated_hud_idx);
        (display_targets, picture)
    }

    fn estimate_connext_velocity(
        &self,
        key: TrackKey,
        new_pos: GeodeticPosition,
        now_us: u64,
    ) -> Option<VelocityObservation> {
        let prev = self.engine.snapshot(key)?;
        let prev_pos = prev.position.as_ref()?;
        let dt_us = now_us.saturating_sub(prev.last_observed_at_micros);
        if !(200_000..=15_000_000).contains(&dt_us) {
            return prev.velocity.map(|v| v.value);
        }
        let prev_alt_m = prev
            .pressure_altitude_ft
            .or(prev.geometric_altitude_ft)
            .map_or(0.0, |a| f64::from(a.value) * FEET_TO_METRES);
        let prev_geo = GeodeticPosition::wgs84(
            prev_pos.value.latitude_deg,
            prev_pos.value.longitude_deg,
            prev_alt_m,
        );
        let plane = LocalTangentPlane::new(prev_geo).ok()?;
        let delta = plane.to_ned(new_pos).ok()?;
        let dt_s = dt_us as f64 * 1e-6;
        let vn = (delta.north_m / dt_s).clamp(-400.0, 400.0);
        let ve = (delta.east_m / dt_s).clamp(-400.0, 400.0);
        let vd = (delta.down_m / dt_s).clamp(-80.0, 80.0);
        let gs_mps = libm::hypot(vn, ve);
        let mut vel = VelocityObservation::default();
        vel.ground_speed_kt = Some(gs_mps / KNOTS_TO_MPS);
        if gs_mps > 2.0 {
            vel.track_angle_deg_true = normalize_deg_360(libm::atan2(ve, vn).to_degrees());
        } else {
            vel.track_angle_deg_true = prev.velocity.and_then(|v| v.value.track_angle_deg_true);
        }
        vel.vertical_rate_fpm = Some(((-vd) / FPM_TO_MPS).round() as i32);
        vel.vertical_rate_source = Some(VerticalRateSource::Barometric);
        Some(vel)
    }
}

fn build_gdl90_velocity(report: &Gdl90ParticipantReport) -> Option<VelocityObservation> {
    if report.horizontal_speed_kt.is_none()
        && report.track_or_heading_deg.is_none()
        && report.vertical_speed_fpm.is_none()
    {
        return None;
    }
    let mut vel = VelocityObservation::default();
    vel.ground_speed_kt = report
        .horizontal_speed_kt
        .filter(|v| v.is_finite() && *v >= 0.0);
    vel.track_angle_deg_true = report.track_or_heading_deg.and_then(normalize_deg_360);
    vel.vertical_rate_fpm = report.vertical_speed_fpm.map(|f| f.round() as i32);
    vel.vertical_rate_source = vel
        .vertical_rate_fpm
        .map(|_| VerticalRateSource::Barometric);
    Some(vel)
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

fn ownship_velocity_ned(ownship: &OwnshipState) -> Ned {
    let (vn, ve) = if ownship.ground_vector_valid {
        if let (Some(trk_deg), Some(gs_kt)) =
            (ownship.ground_track_true_deg, ownship.ground_speed_kt)
        {
            let spd = gs_kt * KNOTS_TO_MPS;
            let rad = trk_deg.to_radians();
            (spd * libm::cos(rad), spd * libm::sin(rad))
        } else {
            (0.0, 0.0)
        }
    } else {
        (0.0, 0.0)
    };
    let vd = -ownship.vertical_speed_fpm.unwrap_or(0.0) * FPM_TO_MPS;
    Ned {
        north_m: vn,
        east_m: ve,
        down_m: vd,
    }
}

fn sanitize_callsign(raw: &str) -> Option<Callsign> {
    let cleaned: String = raw
        .trim()
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || *c == '-' || *c == '_')
        .take(8)
        .collect();
    (!cleaned.is_empty()).then(|| Callsign::new(cleaned))
}
