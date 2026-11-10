//! Per-vessel spacecraft bus devices (`SolarArrayBlock`, `OscillatorBlock`, and
//! `SrpSurface`) driven by the shared solar-flux and relativistic clock models.

use glam::DVec3;
use sim_core::{Block, SimInstant, TickSpan};
use sim_devices::{
    OscillatorBlock, OscillatorConfig, OscillatorSnapshot, SolarArrayBlock, SolarArrayConfig,
    SolarArrayInput,
};
use sim_orbital::{DerivativeOrder, EphemerisCatalog, NaifId, SrpSurface, VesselId};
use sim_spacetime::{BarycentricClockModel, SimEpoch, SolarFluxModel};

use crate::UniverseError;
use crate::handoff::{InertialExtras, to_inertial};
use crate::universe::{Flight, Universe};

/// Configuration for a vessel's mounted solar array, oscillator, and radiation-pressure surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VesselBusConfig {
    /// Photovoltaic solar array configuration.
    pub solar_array: SolarArrayConfig,
    /// Relativistic proper-time oscillator configuration.
    pub oscillator: OscillatorConfig,
    /// Optional cannonball solar radiation pressure surface (`SolarRadiationPressureLaw`).
    pub srp_surface: Option<SrpSurface>,
}

impl Default for VesselBusConfig {
    fn default() -> Self {
        Self {
            solar_array: SolarArrayConfig {
                area_m2: 24.0,
                efficiency: 0.30,
                temperature_coefficient_per_k: -0.004,
                reference_temperature_k: 298.15,
                period: TickSpan::SECOND,
            },
            oscillator: OscillatorConfig {
                fractional_frequency_offset: 0.0,
                period: TickSpan::SECOND,
            },
            srp_surface: None,
        }
    }
}

/// Live spacecraft bus telemetry sampled at the universe's current instant.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VesselBusTelemetry {
    /// Electrical power produced by the vessel's [`SolarArrayBlock`] (`W`).
    pub solar_power_w: f64,
    /// Incident solar irradiance after eclipse occultation (`W/m²`).
    pub solar_irradiance_w_m2: f64,
    /// Unocculted fraction of the solar disc (`[0, 1]`).
    pub solar_lit_fraction: f64,
    /// Instantaneous relativistic proper-time rate offset $d\tau / d\text{TDB} - 1$.
    pub clock_rate_offset: f64,
    /// Accumulated oscillator proper time minus coordinate time ($\tau - \text{TDB}$, `s`).
    pub oscillator_tau_minus_tdb_s: f64,
}

/// Checkpointable state of a vessel's bus blocks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct VesselBusSnapshot {
    solar_power_w: f64,
    oscillator: OscillatorSnapshot,
    last_instant: SimInstant,
    last_sample_instant: Option<SimInstant>,
    telemetry: VesselBusTelemetry,
}

pub(crate) struct VesselBusState {
    config: VesselBusConfig,
    solar_array: SolarArrayBlock,
    oscillator: OscillatorBlock,
    last_instant: SimInstant,
    last_sample_instant: Option<SimInstant>,
    telemetry: VesselBusTelemetry,
}

impl VesselBusState {
    pub(crate) fn new(config: VesselBusConfig, now: SimInstant) -> Self {
        Self {
            config,
            solar_array: SolarArrayBlock::new(config.solar_array),
            oscillator: OscillatorBlock::new(config.oscillator, 0.0, 0.0),
            last_instant: now,
            last_sample_instant: None,
            telemetry: VesselBusTelemetry::default(),
        }
    }

    pub(crate) const fn srp_surface(&self) -> Option<SrpSurface> {
        self.config.srp_surface
    }

    pub(crate) const fn telemetry(&self) -> VesselBusTelemetry {
        self.telemetry
    }

    pub(crate) fn set_config(&mut self, config: VesselBusConfig) {
        self.config = config;
        self.solar_array = SolarArrayBlock::new(config.solar_array);
        let snap = self.oscillator.snapshot();
        self.oscillator = OscillatorBlock::new(config.oscillator, 0.0, 0.0);
        self.oscillator.restore(&snap);
        self.last_sample_instant = None;
    }

    pub(crate) fn snapshot(&self) -> VesselBusSnapshot {
        VesselBusSnapshot {
            solar_power_w: self.solar_array.snapshot(),
            oscillator: self.oscillator.snapshot(),
            last_instant: self.last_instant,
            last_sample_instant: self.last_sample_instant,
            telemetry: self.telemetry,
        }
    }

    pub(crate) fn restore(&mut self, snapshot: &VesselBusSnapshot) {
        self.solar_array.restore(&snapshot.solar_power_w);
        self.oscillator.restore(&snapshot.oscillator);
        self.last_instant = snapshot.last_instant;
        self.last_sample_instant = snapshot.last_sample_instant;
        self.telemetry = snapshot.telemetry;
    }

    pub(crate) fn needs_ephemeris_sample(&self, instant: SimInstant) -> bool {
        let Some(prev) = self.last_sample_instant else {
            return true;
        };
        instant < prev
            || instant.duration_since(prev)
                >= TickSpan::from_seconds_and_subsec_ticks(0, 100_000_000)
    }

    pub(crate) fn advance_oscillator_only(&mut self, instant: SimInstant) {
        if instant > self.last_instant {
            let elapsed = instant.duration_since(self.last_instant);
            let step_osc = OscillatorConfig {
                fractional_frequency_offset: self.config.oscillator.fractional_frequency_offset,
                period: elapsed,
            };
            let prev_tau = self.oscillator.output();
            let rate = self.telemetry.clock_rate_offset;
            let mut span_osc = OscillatorBlock::new(step_osc, prev_tau, rate);
            if span_osc.step(&rate).is_ok() {
                self.oscillator.restore(&span_osc.snapshot());
            }
            self.last_instant = instant;
            self.telemetry.oscillator_tau_minus_tdb_s = self.oscillator.output();
        }
    }

    pub(crate) fn refresh_at(
        &mut self,
        instant: SimInstant,
        pos_ssb_m: DVec3,
        vel_ssb_m_s: DVec3,
        direct_beam_fraction: f64,
        epoch: &SimEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) {
        let tdb = epoch.tdb_at(instant);
        let flux_model = SolarFluxModel::standard();
        let Ok(sunlight) = flux_model.sample(pos_ssb_m, tdb, ephemeris) else {
            return;
        };
        let normal_icrf = if sunlight.to_sun_icrf.length_squared() > 0.0 {
            sunlight.to_sun_icrf
        } else {
            DVec3::X
        };
        self.solar_array
            .step(&SolarArrayInput {
                sunlight,
                normal_icrf,
                direct_beam_fraction: direct_beam_fraction.clamp(0.0, 1.0),
                cell_temperature_k: self.config.solar_array.reference_temperature_k,
            })
            .ok();

        let clock_model = BarycentricClockModel::solar_system();
        let rate_offset = clock_model
            .proper_rate_offset(pos_ssb_m, vel_ssb_m_s, tdb, ephemeris)
            .unwrap_or(0.0);

        if instant > self.last_instant {
            let elapsed = instant.duration_since(self.last_instant);
            let step_osc = OscillatorConfig {
                fractional_frequency_offset: self.config.oscillator.fractional_frequency_offset,
                period: elapsed,
            };
            let prev_tau = self.oscillator.output();
            let prev_rate = if self.last_sample_instant.is_none() {
                rate_offset
            } else {
                self.telemetry.clock_rate_offset
            };
            let mut span_osc = OscillatorBlock::new(step_osc, prev_tau, prev_rate);
            if span_osc.step(&rate_offset).is_ok() {
                self.oscillator.restore(&span_osc.snapshot());
            }
            self.last_instant = instant;
        } else if self.last_sample_instant.is_none() {
            self.oscillator = OscillatorBlock::new(self.config.oscillator, 0.0, rate_offset);
        }
        self.last_sample_instant = Some(instant);

        self.telemetry = VesselBusTelemetry {
            solar_power_w: self.solar_array.output(),
            solar_irradiance_w_m2: sunlight.irradiance_w_m2(),
            solar_lit_fraction: sunlight.lit_fraction,
            clock_rate_offset: rate_offset,
            oscillator_tau_minus_tdb_s: self.oscillator.output(),
        };
    }
}

impl Universe {
    /// Returns live spacecraft bus telemetry (`SolarArrayBlock` power, `SolarFluxSample`
    /// irradiance and lit fraction, and `OscillatorBlock` relativistic proper-time offset) for `id`.
    pub fn bus_telemetry(&self, id: VesselId) -> Result<VesselBusTelemetry, UniverseError> {
        let vessel = self
            .vessels
            .get(&id)
            .ok_or(UniverseError::UnknownVessel(id))?;
        Ok(vessel.bus.telemetry())
    }

    /// Updates `id`'s spacecraft bus configuration (`SolarArrayConfig`, `OscillatorConfig`, `SrpSurface`)
    /// and refreshes its telemetry at [`Self::now`].
    pub fn set_vessel_bus_config(
        &mut self,
        id: VesselId,
        config: VesselBusConfig,
    ) -> Result<(), UniverseError> {
        let vessel = self
            .vessels
            .get_mut(&id)
            .ok_or(UniverseError::UnknownVessel(id))?;
        vessel.bus.set_config(config);
        self.refresh_vessel_bus(id);
        Ok(())
    }

    pub(crate) fn refresh_all_buses(&mut self) {
        let ids: Vec<VesselId> = self.vessels.keys().copied().collect();
        for id in ids {
            self.refresh_vessel_bus(id);
        }
    }

    pub(crate) fn refresh_vessel_bus(&mut self, id: VesselId) {
        let now = self.now();
        if let Some(vessel) = self.vessels.get_mut(&id)
            && !vessel.bus.needs_ephemeris_sample(now)
        {
            vessel.bus.advance_oscillator_only(now);
            return;
        }
        let Some((pos_ssb_m, vel_ssb_m_s)) = self.vessel_barycentric_state(id, now) else {
            return;
        };
        let epoch = self.epoch().clone();
        let ephemeris = self.ephemeris.clone();
        if let Some(vessel) = self.vessels.get_mut(&id) {
            vessel
                .bus
                .refresh_at(now, pos_ssb_m, vel_ssb_m_s, 1.0, &epoch, ephemeris.as_ref());
        }
    }

    fn vessel_barycentric_state(&self, id: VesselId, at: SimInstant) -> Option<(DVec3, DVec3)> {
        let vessel = self.vessels.get(&id)?;
        match &vessel.flight {
            Flight::Orbital => {
                let st = self.orbital().sample_vessel_barycentric(id, at).ok()?;
                Some((st.pos_m, st.vel_m_s))
            }
            Flight::Local(plant) => {
                let body = plant.body();
                let (naif, rotation) = self.rotation(body, at).ok()?;
                let rel = to_inertial(&plant.state(), naif, &rotation, InertialExtras::default());
                let tdb = self.epoch().tdb_at(at);
                let body_ssb = self
                    .ephemeris
                    .geometric_state_icrf(naif, NaifId::SSB, tdb, DerivativeOrder::Velocity)
                    .ok()?;
                Some((
                    body_ssb.pos_m + rel.pos_rel_icrf_m,
                    body_ssb.vel_m_s + rel.vel_rel_icrf_m_s,
                ))
            }
        }
    }
}
