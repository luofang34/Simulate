use glam::DVec3;
use sim_apollo::{
    InertialSiteSample, LM_ASA, LM_DPS_TANK, LM_IMU, LM_LANDING_RADAR, LM_PROBE_TIPS, LM_RGA,
    LandingRadarAntenna, LandingRadarInput, LunarContactInput, LunarModuleSources, RadarSample,
    RadarValidity, SurfacePlane, site_in_fixed_frame,
};
use sim_core::{Block, SimInstant};
use sim_geo::InertialTransform;
use sim_scenery::ElevationSource;

use super::{BODY, LunarLandingRun, PERIOD, up_at};
use crate::ScenarioError;

impl LunarLandingRun {
    /// Samples every sensor at the vehicle's current state and shows it on
    /// the panel.
    pub(super) fn sense(&mut self) -> Result<(), ScenarioError> {
        let now = self.now();
        if now != SimInstant::START {
            let inertial = InertialTransform::moon_at(self.epoch.utc_at(now)?);
            self.imu
                .step(&InertialSiteSample::of(&self.vehicle, LM_IMU, &inertial)?)?;
            self.asa
                .step(&InertialSiteSample::of(&self.vehicle, LM_ASA, &inertial)?)?;
            self.rate_gyros
                .step(&InertialSiteSample::of(&self.vehicle, LM_RGA, &inertial)?)?;
        }
        let state = self.vehicle.output();
        let geodetic = BODY.fixed_to_geodetic(state.position_ecef);
        let ground = self.scenery.ground_height_m(&geodetic)?;
        let surface =
            ground.map(|height| SurfacePlane::level_beneath(&BODY, state.position_ecef, height));
        if now == SimInstant::START || self.radar_rate.steps_for_u128(PERIOD) > 0 {
            self.sample_radar(surface)?;
        }
        self.sense_contact(surface)?;
        let imu = self.imu.output();
        self.panel.observe_lunar_module(&LunarModuleSources {
            timestamp: now,
            imu,
            asa: self.asa.output(),
            rate_gyros: self.rate_gyros.output(),
            radar: self.radar_sample,
            contact: self.probes.output(),
            lpd: Some(self.lpd_reading(self.target_fixed_m)?),
            height_m: ground.map_or(geodetic.height_m, |height| geodetic.height_m - height),
            speed_m_s: state.velocity_ecef.length(),
        });
        self.report.peak_mga_deg = self.report.peak_mga_deg.max(imu.mga_rad.abs().to_degrees());
        self.record_mass()?;
        Ok(())
    }

    fn sample_radar(&mut self, surface: Option<SurfacePlane>) -> Result<(), ScenarioError> {
        let Some(surface) = surface else {
            self.radar_sample = None;
            self.report.radar_invalid_samples = self.report.radar_invalid_samples.wrapping_add(1);
            return Ok(());
        };
        let site = site_in_fixed_frame(&self.vehicle, LM_LANDING_RADAR)?;
        // The crew drive the antenna to its hover position at high gate,
        // before this stretch of the descent begins.
        self.radar.step(&LandingRadarInput {
            site,
            surface,
            antenna: LandingRadarAntenna::Hover,
        })?;
        let output = self.radar.output();
        self.radar_sample = Some(RadarSample {
            output,
            timestamp: self.now(),
        });
        let report = &mut self.report;
        if output.validity == RadarValidity::Valid {
            report.radar_valid_samples = report.radar_valid_samples.wrapping_add(1);
        } else {
            report.radar_invalid_samples = report.radar_invalid_samples.wrapping_add(1);
        }
        Ok(())
    }

    fn sense_contact(&mut self, surface: Option<SurfacePlane>) -> Result<(), ScenarioError> {
        let Some(surface) = surface else {
            return Ok(());
        };
        let [first, second, third] = LM_PROBE_TIPS;
        self.probes.step(&LunarContactInput {
            tips_fixed_m: [
                site_in_fixed_frame(&self.vehicle, first)?.position_m,
                site_in_fixed_frame(&self.vehicle, second)?.position_m,
                site_in_fixed_frame(&self.vehicle, third)?.position_m,
            ],
            surface,
        })?;
        if self.probes.output().lunar_contact && !self.finished {
            self.finished = true;
            let state = self.vehicle.output();
            let up = up_at(state.position_ecef);
            let sink = -state.velocity_ecef.dot(up);
            let across = |v: DVec3| v - up * v.dot(up);
            self.report.contact_sink_m_s = Some(sink);
            self.report.contact_drift_m_s = Some(across(state.velocity_ecef).length());
            self.report.contact_miss_m =
                Some(across(state.position_ecef - self.target_fixed_m).length());
        }
        Ok(())
    }

    fn record_mass(&mut self) -> Result<(), ScenarioError> {
        let fuel = self.vehicle.tree().state(LM_DPS_TANK)?.fluid_mass_kg;
        self.report.dps_burned_kg = self.start_propellant_kg - fuel;
        self.report.cg_rise_m = self.vehicle.mass_properties().cg_body_m.x - self.start_cg_x_m;
        Ok(())
    }
}
