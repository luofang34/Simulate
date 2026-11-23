//! ADS-B / TIS-B multi-source traffic fusion, callsign labeling, CPA threat classification,
//! and `Indicate` HUD `TargetPicture` projection (`world-surveillance`).

use crate::connext::nmea::ConnextTrafficTarget;
use crate::error::FlightDeckError;
use crate::gdl90::Gdl90ParticipantReport;
use crate::planet::PlanetaryReference;
use crate::telemetry::{FEET_TO_METRES, FPM_TO_MPS, KNOTS_TO_MPS, OwnshipState};
use indicate_instrument_hud::{Contact, Domain, Identity, MAX_CONTACTS, TargetPicture};
use indicate_instrument_state::SignalStatus;
use world_core::{GeodeticPosition, Ned};
use world_surveillance::{
    AddressNamespace, Callsign, DeliveryPath, EmitterCategory, IntegrityQuality, ObservationSource,
    SurveillanceStore, TrackKey, TrackObservation, TrackPhase,
};

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
    /// Stable track ID assigned by [`SurveillanceStore`].
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

/// Stateful coordinator wrapping [`SurveillanceStore`] for Connext, GDL-90, and simulated traffic.
#[derive(Clone, Debug, Default)]
pub struct TrafficCoordinator {
    store: SurveillanceStore,
}

impl TrafficCoordinator {
    /// Creates an empty [`TrafficCoordinator`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            store: SurveillanceStore::default(),
        }
    }

    /// Resets all tracked traffic (used when switching telemetry sources).
    pub fn clear(&mut self) {
        self.store = SurveillanceStore::default();
    }

    /// Underlying [`SurveillanceStore`] reference.
    #[must_use]
    pub const fn store(&self) -> &SurveillanceStore {
        &self.store
    }

    /// Updates ownship Mode-S address so ownship ADS-B/TIS-B echoes are tagged `ownship_shadow`.
    pub fn set_ownship_mode_s(&mut self, mode_s_code: Option<u32>) {
        if let Some(code) = mode_s_code {
            let key = TrackKey {
                namespace: AddressNamespace::Icao24,
                address: u64::from(code & 0x00FF_FFFF),
            };
            self.store.set_ownship_keys(&[key]);
        } else {
            self.store.set_ownship_keys(&[]);
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
        let key = TrackKey {
            namespace: AddressNamespace::Icao24,
            address: u64::from(target.raw_id & 0x00FF_FFFF),
        };
        let alt_m = target
            .pressure_altitude_ft
            .map_or(0.0, |ft| f64::from(ft) * FEET_TO_METRES);
        let position = GeodeticPosition::wgs84(target.latitude_deg, target.longitude_deg, alt_m);

        let (vel_ned, heading_rad) = self
            .estimate_connext_velocity(key, position, now_us)
            .unwrap_or((
                Ned {
                    north_m: 0.0,
                    east_m: 0.0,
                    down_m: 0.0,
                },
                None,
            ));

        let obs = TrackObservation {
            key,
            source: ObservationSource::Adsb1090,
            delivery,
            observed_at_us: now_us,
            position,
            velocity_ned_m_s: vel_ned,
            heading_rad,
            callsign: target.callsign.as_deref().and_then(sanitize_callsign),
            category: EmitterCategory::Unknown,
            on_ground: false,
            quality: IntegrityQuality::default(),
        };
        self.store
            .ingest(obs, now_us)
            .map(|_| ())
            .map_err(|e| FlightDeckError::Surveillance {
                reason: e.to_string(),
            })
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
        let alt_m = report
            .pressure_altitude_ft
            .map_or(0.0, |ft| f64::from(ft) * FEET_TO_METRES);
        let position = GeodeticPosition::wgs84(report.latitude_deg, report.longitude_deg, alt_m);
        let trk_rad = report.track_or_heading_deg.map(f64::to_radians);
        let spd_mps = report.horizontal_speed_kt.unwrap_or(0.0) * KNOTS_TO_MPS;
        let (vn, ve) = trk_rad.map_or((0.0, 0.0), |r| {
            (spd_mps * libm::cos(r), spd_mps * libm::sin(r))
        });
        let vd = -report.vertical_speed_fpm.unwrap_or(0.0) * FPM_TO_MPS;

        let obs = TrackObservation {
            key: TrackKey {
                namespace: report.namespace,
                address: u64::from(report.address),
            },
            source: ObservationSource::Adsb1090,
            delivery: DeliveryPath::LocalReceiver,
            observed_at_us: now_us,
            position,
            velocity_ned_m_s: Ned {
                north_m: vn,
                east_m: ve,
                down_m: vd,
            },
            heading_rad: trk_rad,
            callsign: report.callsign.as_deref().and_then(sanitize_callsign),
            category: report.emitter_category,
            on_ground: !report.airborne,
            quality: IntegrityQuality {
                nic: report.nic,
                nac_p: report.nac_p,
                sil: 0,
                horizontal_1sigma_m: None,
            },
        };
        self.store
            .ingest(obs, now_us)
            .map(|_| ())
            .map_err(|e| FlightDeckError::Surveillance {
                reason: e.to_string(),
            })
    }

    /// Ingests a direct [`TrackObservation`] (used by deterministic traffic/CPA scenarios).
    ///
    /// # Errors
    ///
    /// Returns [`FlightDeckError::Surveillance`] if rejected by [`SurveillanceStore`].
    pub fn ingest_observation(
        &mut self,
        obs: TrackObservation,
        now_ms: u64,
    ) -> Result<(), FlightDeckError> {
        self.store
            .ingest(obs, now_ms.saturating_mul(1_000))
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
        self.store.advance_time(now_us);
        self.set_ownship_mode_s(ownship.identity.as_ref().map(|id| id.mode_s_code));

        let Some(mut own_geo) = ownship.valid_earth_wgs84_position(planet) else {
            return (Vec::new(), TargetPicture::default());
        };
        if let Some(best_alt_m) = ownship.best_altitude_m() {
            own_geo.ellipsoidal_height_m = best_alt_m;
        }
        let own_vel_ned = ownship_velocity_ned(ownship);
        let relatives = self.store.relative_to_ownship(own_geo, own_vel_ned, now_us);

        let mut display_targets = Vec::with_capacity(relatives.len());
        let mut picture = TargetPicture::default();
        let mut designated_hud_idx: Option<usize> = None;

        for rel in relatives {
            let Some(track) = self.store.track(rel.id) else {
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
    ) -> Option<(Ned, Option<f64>)> {
        let prev = self.store.track_by_key(key)?;
        let dt_us = now_us.saturating_sub(prev.observed_at_us);
        if !(200_000..=15_000_000).contains(&dt_us) {
            return Some((prev.velocity_ned_m_s, prev.heading_rad));
        }
        let plane = world_core::LocalTangentPlane::new(prev.position).ok()?;
        let delta = plane.to_ned(new_pos).ok()?;
        let dt_s = dt_us as f64 * 1e-6;
        let vn = (delta.north_m / dt_s).clamp(-400.0, 400.0);
        let ve = (delta.east_m / dt_s).clamp(-400.0, 400.0);
        let vd = (delta.down_m / dt_s).clamp(-80.0, 80.0);
        let horiz = libm::hypot(vn, ve);
        let heading = (horiz > 2.0)
            .then(|| libm::atan2(ve, vn).rem_euclid(core::f64::consts::TAU))
            .or(prev.heading_rad);
        Some((
            Ned {
                north_m: vn,
                east_m: ve,
                down_m: vd,
            },
            heading,
        ))
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

fn build_display_target(
    track: &world_surveillance::TrackSnapshot,
    rel: &world_surveillance::RelativeTarget,
) -> TrafficDisplayTarget {
    let address_hex = format!("{:06X}", track.key.address & 0x00FF_FFFF);
    let callsign = track
        .callsign
        .map_or_else(|| address_hex.clone(), |cs| cs.as_str().to_owned());
    let slant_range_nm = (rel.slant_range_m / METRES_PER_NM) as f32;
    let rel_alt_ft = rel.relative_altitude_m / FEET_TO_METRES;
    let rel_altitude_hundreds_ft = (rel_alt_ft / 100.0).round() as i32;
    let climb_fpm = (-track.velocity_ned_m_s.down_m) / FPM_TO_MPS;
    let vertical_trend = if climb_fpm >= 300.0 {
        1
    } else if climb_fpm <= -300.0 {
        -1
    } else {
        0
    };
    let trend_arrow = match vertical_trend {
        1 => "↑",
        -1 => "↓",
        _ => "",
    };
    let placard = format!(
        "{callsign} {:+03}{trend_arrow} {:.1}NM",
        rel_altitude_hundreds_ft, slant_range_nm
    );
    let threat = classify_threat(rel);
    TrafficDisplayTarget {
        track_id: track.id.0,
        address_hex,
        callsign,
        placard,
        offset_enu_m: [
            rel.offset_ned_m.east_m as f32,
            rel.offset_ned_m.north_m as f32,
            (-rel.offset_ned_m.down_m) as f32,
        ],
        velocity_enu_mps: [
            track.velocity_ned_m_s.east_m as f32,
            track.velocity_ned_m_s.north_m as f32,
            (-track.velocity_ned_m_s.down_m) as f32,
        ],
        track_true_deg: track
            .heading_rad
            .map(|r| (r.to_degrees().rem_euclid(360.0)) as f32),
        slant_range_nm,
        rel_altitude_hundreds_ft,
        vertical_trend,
        time_to_cpa_s: rel.time_to_cpa_s as f32,
        cpa_horizontal_m: rel.cpa_horizontal_m as f32,
        threat,
        on_ground: track.on_ground,
        active: track.phase == TrackPhase::Active,
    }
}

fn classify_threat(rel: &world_surveillance::RelativeTarget) -> TrafficThreat {
    let horiz_nm = rel.horizontal_range_m / METRES_PER_NM;
    let abs_alt_ft = (rel.relative_altitude_m / FEET_TO_METRES).abs();
    let cpa_nm = rel.cpa_horizontal_m / METRES_PER_NM;

    if (horiz_nm <= 0.45 && abs_alt_ft <= 600.0)
        || (rel.time_to_cpa_s > 0.0
            && rel.time_to_cpa_s <= 25.0
            && cpa_nm <= 0.45
            && abs_alt_ft <= 600.0)
    {
        TrafficThreat::ResolutionAdvisory
    } else if (horiz_nm <= 0.80 && abs_alt_ft <= 850.0)
        || (rel.time_to_cpa_s > 0.0
            && rel.time_to_cpa_s <= 45.0
            && cpa_nm <= 1.20
            && abs_alt_ft <= 850.0)
    {
        TrafficThreat::TrafficAdvisory
    } else if horiz_nm <= 6.0 && abs_alt_ft <= 1_200.0 {
        TrafficThreat::Proximate
    } else {
        TrafficThreat::Other
    }
}

fn to_hud_contact(
    track: &world_surveillance::TrackSnapshot,
    rel: &world_surveillance::RelativeTarget,
    own_vel_ned: &Ned,
) -> Contact {
    let threat = classify_threat(rel);
    Contact {
        relative_position_m: [
            rel.offset_ned_m.north_m as f32,
            rel.offset_ned_m.east_m as f32,
            rel.offset_ned_m.down_m as f32,
        ],
        relative_velocity_mps: [
            (track.velocity_ned_m_s.north_m - own_vel_ned.north_m) as f32,
            (track.velocity_ned_m_s.east_m - own_vel_ned.east_m) as f32,
            (track.velocity_ned_m_s.down_m - own_vel_ned.down_m) as f32,
        ],
        identity: if threat >= TrafficThreat::TrafficAdvisory {
            Identity::Hostile
        } else {
            Identity::Friendly
        },
        domain: if track.on_ground {
            Domain::Surface
        } else {
            Domain::Air
        },
        status: match track.phase {
            TrackPhase::Active => SignalStatus::Valid,
            TrackPhase::Coasting => SignalStatus::Stale,
        },
    }
}

fn sanitize_callsign(raw: &str) -> Option<Callsign> {
    let cleaned: String = raw
        .trim()
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || *c == '-' || *c == '_')
        .take(8)
        .collect();
    Callsign::parse(&cleaned)
}
