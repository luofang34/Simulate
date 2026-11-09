//! Plant readouts the cockpit, avionics, and hangar show: specific force, aircraft
//! systems, engine, articulation, airspeed envelope, and Aviate arming.

use std::sync::OnceLock;

use glam::DVec3;
use sim_core::{AirframeArticulation, EngineStatus, SystemControls};
use sim_pack::{
    AirspeedEnvelope, LoadedSyspack, PitotStaticCalibration, ResolvedAirspeedEnvelope,
    builtin_c172_package, builtin_p38g_package,
};
use sim_scenario::ScenarioProfile;

use super::Flight;
use super::mode::Mode;
use super::rewind::seconds;
use crate::vehicles::FreeFlightVehicle;

struct CachedAeroSpecs {
    envelope: Option<(AirspeedEnvelope, f64)>,
    pec: Option<PitotStaticCalibration>,
}

static C172_SPECS: OnceLock<CachedAeroSpecs> = OnceLock::new();
static P38_SPECS: OnceLock<CachedAeroSpecs> = OnceLock::new();

fn package_aero_specs(pkg: Result<LoadedSyspack, sim_pack::PackError>) -> CachedAeroSpecs {
    let Ok(pkg) = pkg else {
        return CachedAeroSpecs {
            envelope: None,
            pec: None,
        };
    };
    let envelope = pkg.airspeed_envelope().cloned().map(|env| {
        let mass_kg = pkg
            .tree()
            .mass_properties()
            .map_or(env.reference_mass_kg, |props| props.total_mass_kg);
        (env, mass_kg)
    });
    let pec = pkg.pitot_static_calibration().cloned();
    CachedAeroSpecs { envelope, pec }
}

fn cached_c172_specs() -> &'static CachedAeroSpecs {
    C172_SPECS.get_or_init(|| package_aero_specs(builtin_c172_package()))
}

fn cached_p38_specs() -> &'static CachedAeroSpecs {
    P38_SPECS.get_or_init(|| package_aero_specs(builtin_p38g_package()))
}

impl Flight {
    fn active_aero_specs(&self) -> Option<&'static CachedAeroSpecs> {
        match &self.mode {
            Mode::Free { vehicle, .. } => match vehicle {
                FreeFlightVehicle::JsbsimC172
                | FreeFlightVehicle::AviateFixedWingDirect
                | FreeFlightVehicle::Kinematic => Some(cached_c172_specs()),
                FreeFlightVehicle::JsbsimP38g => Some(cached_p38_specs()),
                FreeFlightVehicle::AviateX500Direct => None,
            },
            Mode::Scenario(run) => match run.profile() {
                ScenarioProfile::Airport47NUncontrolledPattern
                | ScenarioProfile::AirportKewrToweredPattern
                | ScenarioProfile::CorridorNominalCruise
                | ScenarioProfile::CorridorOffSeedAcquisition
                | ScenarioProfile::CorridorAggressiveBank
                | ScenarioProfile::CorridorSameCellLoiter => Some(cached_c172_specs()),
                ScenarioProfile::Apollo11LunarLanding | ScenarioProfile::Apollo11CapsuleReentry => {
                    None
                }
            },
        }
    }

    /// Flap-indexed pitot-static position error (`PEC`) calibration when the active
    /// aircraft package ships `performance/pitot_static_calibration.json`.
    #[must_use]
    pub fn pitot_static_calibration(&self) -> Option<&'static PitotStaticCalibration> {
        self.active_aero_specs()?.pec.as_ref()
    }

    /// Active fixed-wing [`ResolvedAirspeedEnvelope`] scaled to live gross mass (`kg`)
    /// when flying a certificated/packaged aeroplane (`None` on rotary-wing or spacecraft).
    #[must_use]
    pub fn resolved_airspeed_envelope(&self) -> Option<ResolvedAirspeedEnvelope> {
        let (env, default_mass_kg) = self.active_aero_specs()?.envelope.as_ref()?;
        let live_mass = self.articulation().gross_mass_kg;
        let mass_kg = if live_mass > 1.0 {
            live_mass
        } else {
            *default_mass_kg
        };
        Some(env.resolve_at_mass(mass_kg))
    }

    /// Body FRD specific force (`m/s²`), when provided by the plant.
    #[must_use]
    pub fn specific_force_frd_m_s2(&self) -> Option<DVec3> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().plant().specific_force_frd_m_s2(),
            Mode::Scenario(_) => None,
        }
    }

    /// Active [`SystemControls`] on JSBSim free-flight aircraft.
    #[must_use]
    pub fn system_controls(&self) -> Option<SystemControls> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().system_controls(),
            Mode::Scenario(_) => None,
        }
    }

    /// Applies `f` to the [`SystemControls`] on JSBSim free-flight aircraft; the change
    /// takes effect at the start of the next physics step.
    pub fn update_system_controls(&mut self, f: impl FnOnce(&mut SystemControls)) {
        if let Mode::Free { session, .. } = &self.mode
            && let Some(mut sys) = session.block().system_controls()
        {
            f(&mut sys);
            session.block().queue_system_controls(sys);
        }
    }

    /// Latest engine status (`running`, `propeller_rpm`) on JSBSim free-flight aircraft.
    #[must_use]
    pub fn engine_status(&self) -> Option<EngineStatus> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().plant().engine_status(),
            Mode::Scenario(run) => {
                let art = run.articulation();
                (art.engine_running || art.propeller_rpm.abs() > 1.0).then_some(EngineStatus {
                    running: art.engine_running,
                    propeller_rpm: art.propeller_rpm,
                })
            }
        }
    }

    /// Full powerplant, propeller, control-surface, and landing-gear articulation.
    #[must_use]
    pub fn articulation(&self) -> AirframeArticulation {
        match &self.mode {
            Mode::Free { session, .. } => {
                let elapsed_s = seconds(session.physics_time());
                session.block().plant().articulation(elapsed_s)
            }
            Mode::Scenario(run) => run.articulation(),
        }
    }

    /// Whether an Aviate direct flight controller is armed (`None` on non-Aviate flights).
    #[must_use]
    pub fn aviate_armed(&self) -> Option<bool> {
        match &self.mode {
            Mode::Free { session, .. } => session.block().plant().aviate_armed(),
            Mode::Scenario(_) => None,
        }
    }
}
