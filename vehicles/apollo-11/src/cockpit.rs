use glam::{DQuat, DVec3};
use sim_core::{SimInstant, SourceValidity, StampedReading, TickSpan, VehicleState};

use crate::{
    AsaOutput, EntryMonitorOutput, GuidanceHandover, GuidanceSource, ImuOutput, LandingRadarOutput,
    LpdReading, LunarContactOutput, RadarValidity, RateGyroOutput,
};

#[cfg(test)]
mod tests;

/// Longest the radar may go unsampled before its data reads stale.
const RADAR_MAX_AGE: TickSpan = TickSpan::SECOND;

/// A landing radar sample and when it was taken, which may be well before the
/// panel looks at it because the radar runs slower than the display.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadarSample {
    /// The radar's output.
    pub output: LandingRadarOutput,
    /// When the radar produced it.
    pub timestamp: SimInstant,
}

/// What an Earth aircraft offers the panel: one attitude source, no guidance
/// computer, and none of the Apollo aids.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EarthVehicleSources {
    /// The instant being displayed.
    pub timestamp: SimInstant,
    /// The vehicle.
    pub vehicle: VehicleState,
    /// Height above the reference ellipsoid, metres.
    pub height_m: f64,
}

/// What the Lunar Module offers the panel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LunarModuleSources {
    /// The instant being displayed.
    pub timestamp: SimInstant,
    /// The gimballed platform.
    pub imu: ImuOutput,
    /// The strapdown abort sensor.
    pub asa: AsaOutput,
    /// The rate gyros.
    pub rate_gyros: RateGyroOutput,
    /// The landing radar, or `None` while it is off.
    pub radar: Option<RadarSample>,
    /// The contact probes.
    pub contact: LunarContactOutput,
    /// The designated target against the window scale, if there is one.
    pub lpd: Option<LpdReading>,
    /// Height above the local surface, metres.
    pub height_m: f64,
    /// Speed over the surface, m/s.
    pub speed_m_s: f64,
}

/// What the Command Module offers the panel during entry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandModuleSources {
    /// The instant being displayed.
    pub timestamp: SimInstant,
    /// The gimballed platform.
    pub imu: ImuOutput,
    /// The Entry Monitor System.
    pub ems: EntryMonitorOutput,
    /// Height above the reference ellipsoid, metres.
    pub height_m: f64,
    /// Speed relative to the Earth, m/s.
    pub speed_m_s: f64,
}

/// Every cockpit reading at one instant, each with its source's health.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CockpitInstrumentSnapshot {
    /// The instant displayed.
    pub timestamp: SimInstant,
    /// The system in control, or `None` on a vehicle with no guidance
    /// computer.
    pub guidance_source: Option<GuidanceSource>,
    /// The attitude being flown: the active source's, body to J2000 on Apollo
    /// and body to ECEF on an Earth aircraft.
    pub attitude: StampedReading<DQuat>,
    /// The primary (PGNS) attitude, [`SourceValidity::Failed`] once the
    /// platform has lost its reference.
    pub pgns_attitude: StampedReading<DQuat>,
    /// The backup (AGS) attitude.
    pub ags_attitude: StampedReading<DQuat>,
    /// `|MGA|` is at or past the 70° caution.
    pub gimbal_lock_caution: bool,
    /// The platform has been captured at 85°.
    pub gimbal_lock_captured: bool,
    /// Body angular rate from the rate gyros, rad/s.
    pub body_rate_rad_s: StampedReading<DVec3>,
    /// Landing radar slant range, metres.
    pub radar_slant_range_m: StampedReading<f64>,
    /// Landing radar Doppler velocity over the surface, antenna axes, m/s.
    pub radar_velocity_m_s: StampedReading<DVec3>,
    /// The blue `CONTACT LIGHT`.
    pub contact_light: bool,
    /// The designated target's graduation on the window, degrees, while it
    /// is on the scale.
    pub lpd_elevation_deg: Option<f64>,
    /// Sensed deceleration on the Entry Monitor, g.
    pub ems_g_load: Option<f64>,
    /// Height, metres.
    pub height_m: StampedReading<f64>,
    /// Speed, m/s.
    pub speed_m_s: StampedReading<f64>,
}

impl CockpitInstrumentSnapshot {
    /// A panel with nothing heard from any source yet.
    pub fn blank(timestamp: SimInstant) -> Self {
        let missing = SourceValidity::Missing;
        let quaternion = StampedReading::new(DQuat::IDENTITY, timestamp, missing);
        let vector = StampedReading::new(DVec3::ZERO, timestamp, missing);
        let scalar = StampedReading::new(0.0, timestamp, missing);
        Self {
            timestamp,
            guidance_source: None,
            attitude: quaternion,
            pgns_attitude: quaternion,
            ags_attitude: quaternion,
            gimbal_lock_caution: false,
            gimbal_lock_captured: false,
            body_rate_rad_s: vector,
            radar_slant_range_m: scalar,
            radar_velocity_m_s: vector,
            contact_light: false,
            lpd_elevation_deg: None,
            ems_g_load: None,
            height_m: scalar,
            speed_m_s: scalar,
        }
    }

    /// What a crew would notice between `previous` and this snapshot, in a
    /// fixed order.
    pub fn changes_since(&self, previous: &Self) -> Vec<InstrumentChange> {
        let mut changes = Vec::new();
        if self.guidance_source != previous.guidance_source {
            changes.push(InstrumentChange::GuidanceSource(self.guidance_source));
        }
        if self.attitude.validity != previous.attitude.validity {
            changes.push(InstrumentChange::Attitude(self.attitude.validity));
        }
        if self.pgns_attitude.validity != previous.pgns_attitude.validity {
            changes.push(InstrumentChange::PgnsAttitude(self.pgns_attitude.validity));
        }
        if self.gimbal_lock_caution != previous.gimbal_lock_caution {
            changes.push(InstrumentChange::GimbalLockCaution(
                self.gimbal_lock_caution,
            ));
        }
        if self.gimbal_lock_captured != previous.gimbal_lock_captured {
            changes.push(InstrumentChange::GimbalLockCaptured(
                self.gimbal_lock_captured,
            ));
        }
        if self.radar_slant_range_m.validity != previous.radar_slant_range_m.validity {
            changes.push(InstrumentChange::Radar(self.radar_slant_range_m.validity));
        }
        if self.contact_light != previous.contact_light {
            changes.push(InstrumentChange::ContactLight(self.contact_light));
        }
        changes
    }
}

/// A change on the panel worth reporting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InstrumentChange {
    /// A different system took control.
    GuidanceSource(Option<GuidanceSource>),
    /// The flown attitude's health changed.
    Attitude(SourceValidity),
    /// The primary attitude's health changed.
    PgnsAttitude(SourceValidity),
    /// The gimbal lock caution changed.
    GimbalLockCaution(bool),
    /// The gimbal capture changed.
    GimbalLockCaptured(bool),
    /// The landing radar's health changed.
    Radar(SourceValidity),
    /// The contact light changed.
    ContactLight(bool),
}

/// Monitors a vehicle's instrument sources and owns its guidance switch.
///
/// Attitude keeps coming on the panel through a PGNS-to-AGS transfer: the
/// flown attitude stays usable, [`SourceValidity::Degraded`] because it is the
/// backup, while the PGNS channel reads [`SourceValidity::Failed`].
#[derive(Debug, Clone, PartialEq)]
pub struct CockpitInstrumentPanel {
    handover: GuidanceHandover,
    latest: CockpitInstrumentSnapshot,
}

impl CockpitInstrumentPanel {
    /// A panel on PGNS with nothing heard yet.
    pub fn new(timestamp: SimInstant) -> Self {
        Self {
            handover: GuidanceHandover::default(),
            latest: CockpitInstrumentSnapshot::blank(timestamp),
        }
    }

    /// The most recent snapshot.
    pub fn snapshot(&self) -> CockpitInstrumentSnapshot {
        self.latest
    }

    /// The crew moves the guidance switch to `source`.
    pub fn crew_select(&mut self, source: GuidanceSource) {
        self.handover.crew_select(source);
    }

    /// Reads an Earth vehicle.
    pub fn observe_earth(&mut self, sources: &EarthVehicleSources) -> CockpitInstrumentSnapshot {
        let at = sources.timestamp;
        let valid = SourceValidity::Valid;
        self.latest = CockpitInstrumentSnapshot {
            attitude: StampedReading::new(sources.vehicle.attitude, at, valid),
            body_rate_rad_s: StampedReading::new(sources.vehicle.angular_rate_body, at, valid),
            height_m: StampedReading::new(sources.height_m, at, valid),
            speed_m_s: StampedReading::new(sources.vehicle.velocity_ecef.length(), at, valid),
            ..CockpitInstrumentSnapshot::blank(at)
        };
        self.latest
    }

    /// Reads the Lunar Module, failing over from PGNS if its platform has
    /// lost its reference.
    pub fn observe_lunar_module(
        &mut self,
        sources: &LunarModuleSources,
    ) -> CockpitInstrumentSnapshot {
        let at = sources.timestamp;
        let valid = SourceValidity::Valid;
        let flown = self.handover.attitude(&sources.imu, &sources.asa);
        let (range, velocity) = self.radar_readings(at, sources.radar);
        let rate_validity = if sources.rate_gyros.saturated.any() {
            SourceValidity::Degraded
        } else {
            valid
        };
        self.latest = CockpitInstrumentSnapshot {
            guidance_source: Some(self.handover.active()),
            attitude: StampedReading::new(flown, at, self.flown_validity()),
            pgns_attitude: pgns_attitude(at, &sources.imu),
            ags_attitude: StampedReading::new(sources.asa.site_to_inertial, at, valid),
            gimbal_lock_caution: sources.imu.gimbal_lock_warning,
            gimbal_lock_captured: sources.imu.gimbal_locked,
            body_rate_rad_s: StampedReading::new(sources.rate_gyros.rate_rad_s, at, rate_validity),
            radar_slant_range_m: range,
            radar_velocity_m_s: velocity,
            contact_light: sources.contact.lunar_contact,
            lpd_elevation_deg: sources
                .lpd
                .filter(|reading| reading.on_scale)
                .map(|reading| reading.elevation_rad.to_degrees()),
            height_m: StampedReading::new(sources.height_m, at, valid),
            speed_m_s: StampedReading::new(sources.speed_m_s, at, valid),
            ..CockpitInstrumentSnapshot::blank(at)
        };
        self.latest
    }

    /// Reads the Command Module, which has no abort guidance to fail over to:
    /// a captured platform leaves the crew with the Entry Monitor alone.
    pub fn observe_command_module(
        &mut self,
        sources: &CommandModuleSources,
    ) -> CockpitInstrumentSnapshot {
        let at = sources.timestamp;
        let pgns = pgns_attitude(at, &sources.imu);
        self.latest = CockpitInstrumentSnapshot {
            guidance_source: Some(GuidanceSource::Pgns),
            attitude: pgns,
            pgns_attitude: pgns,
            gimbal_lock_caution: sources.imu.gimbal_lock_warning,
            gimbal_lock_captured: sources.imu.gimbal_locked,
            ems_g_load: Some(sources.ems.g_load),
            height_m: StampedReading::new(sources.height_m, at, SourceValidity::Valid),
            speed_m_s: StampedReading::new(sources.speed_m_s, at, SourceValidity::Valid),
            ..CockpitInstrumentSnapshot::blank(at)
        };
        self.latest
    }

    fn flown_validity(&self) -> SourceValidity {
        match self.handover.active() {
            GuidanceSource::Pgns => SourceValidity::Valid,
            GuidanceSource::Ags => SourceValidity::Degraded,
        }
    }

    /// Range and velocity from `sample`. Out of limits, or with the radar off,
    /// both hold their last good value and read missing.
    fn radar_readings(
        &self,
        now: SimInstant,
        sample: Option<RadarSample>,
    ) -> (StampedReading<f64>, StampedReading<DVec3>) {
        let held = (
            self.latest.radar_slant_range_m,
            self.latest.radar_velocity_m_s,
        );
        let missing = |mut reading: (StampedReading<f64>, StampedReading<DVec3>)| {
            reading.0.validity = SourceValidity::Missing;
            reading.1.validity = SourceValidity::Missing;
            reading
        };
        let Some(RadarSample { output, timestamp }) = sample else {
            return missing(held);
        };
        if output.validity == RadarValidity::OutOfLimits {
            return missing(held);
        }
        let valid = SourceValidity::Valid;
        (
            StampedReading::new(output.slant_range_m, timestamp, valid).aged(now, RADAR_MAX_AGE),
            StampedReading::new(output.velocity_site_m_s, timestamp, valid)
                .aged(now, RADAR_MAX_AGE),
        )
    }
}

fn pgns_attitude(at: SimInstant, imu: &ImuOutput) -> StampedReading<DQuat> {
    let validity = if imu.platform_valid {
        SourceValidity::Valid
    } else {
        SourceValidity::Failed
    };
    StampedReading::new(imu.indicated_navbase_to_inertial, at, validity)
}
