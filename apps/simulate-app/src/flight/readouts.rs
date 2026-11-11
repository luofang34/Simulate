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
use syspack::InstrumentPackDocument;

use super::Flight;
use super::mode::Mode;
use super::rewind::seconds;
use crate::vehicles::FreeFlightVehicle;

/// Metres per inch, the unit of JSBSim structural frames.
const METRES_PER_INCH: f64 = 0.0254;

struct CachedAeroSpecs {
    envelope: Option<(AirspeedEnvelope, f64)>,
    pec: Option<PitotStaticCalibration>,
    instruments: Option<InstrumentPackDocument>,
}

static C172_SPECS: OnceLock<CachedAeroSpecs> = OnceLock::new();
static P38_SPECS: OnceLock<CachedAeroSpecs> = OnceLock::new();

fn package_aero_specs(pkg: Result<LoadedSyspack, sim_pack::PackError>) -> CachedAeroSpecs {
    let Ok(pkg) = pkg else {
        return CachedAeroSpecs {
            envelope: None,
            pec: None,
            instruments: None,
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
    let instruments = pkg.instrument_pack().cloned();
    CachedAeroSpecs {
        envelope,
        pec,
        instruments,
    }
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
                FreeFlightVehicle::AviateX500Direct | FreeFlightVehicle::X500Mission => None,
            },
            Mode::Space(_) => None,
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

    /// The instrument pack the active aircraft package pins.
    #[must_use]
    pub fn instrument_pack(&self) -> Option<&'static InstrumentPackDocument> {
        self.active_aero_specs()?.instruments.as_ref()
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

    /// Unit direction from `observer_fixed_m`, in the fixed axes of the body the flight is
    /// about, to `body`'s centre at the flight's instant, from the ephemeris the dynamics
    /// integrate in; `None` outside the built-in kernels' coverage.
    #[must_use]
    pub fn dynamics_direction_fixed(
        &self,
        body: sim_orbital::NaifId,
        observer_fixed_m: DVec3,
    ) -> Option<DVec3> {
        let about = sim_universe::naif_body(self.world().body())?;
        let at = self.epoch.tdb_at(self.now());
        self.solar
            .direction_fixed(body, about, observer_fixed_m, at)
    }

    /// Dynamics ephemeris and orientation adapter implementing `navigate_celestial::Ephemeris`
    /// so the sky rig evaluates sights from the dynamics kernels first and falls back to
    /// `Navigate`'s analytic ephemeris outside kernel coverage.
    #[must_use]
    pub fn celestial_observer_adapter(
        &self,
    ) -> sim_navigate::CelestialObserverAdapter<
        '_,
        dyn sim_orbital::EphemerisCatalog + '_,
        dyn sim_orbital::BodyOrientationCatalog + '_,
    > {
        sim_navigate::CelestialObserverAdapter {
            ephemeris: self.solar.ephemeris(),
            orientation: self.solar.orientation(),
        }
    }

    /// Body FRD specific force (`m/s²`), when provided by the plant.
    #[must_use]
    pub fn specific_force_frd_m_s2(&self) -> Option<DVec3> {
        match &self.mode {
            Mode::Free { world, .. } => world
                .session()
                .and_then(|session| session.block().plant().specific_force_frd_m_s2()),
            Mode::Scenario(_) | Mode::Space(_) => None,
        }
    }

    /// Active [`SystemControls`] on JSBSim free-flight aircraft.
    #[must_use]
    pub fn system_controls(&self) -> Option<SystemControls> {
        match &self.mode {
            Mode::Free { world, .. } => world
                .session()
                .and_then(|session| session.block().system_controls()),
            Mode::Scenario(_) | Mode::Space(_) => None,
        }
    }

    /// Applies `f` to the [`SystemControls`] on JSBSim free-flight aircraft; the change
    /// takes effect at the start of the next physics step.
    pub fn update_system_controls(&mut self, f: impl FnOnce(&mut SystemControls)) {
        if let Mode::Free { world, .. } = &self.mode
            && let Some(session) = world.session()
            && let Some(mut sys) = session.block().system_controls()
        {
            f(&mut sys);
            session.block().queue_system_controls(sys);
        }
    }

    /// Body FRD metres from the centre of gravity, which the vehicle position tracks, to
    /// the drawn airframe's origin at its fixed structural datum
    /// ([`crate::vehicles::ModelOrigins`]), so loading and fuel burn
    /// move the CG inside the model rather than the model against its wheels. Zero for a
    /// model or plant without a structural frame.
    #[must_use]
    pub fn model_origin_from_cg_frd_m(&self) -> DVec3 {
        if matches!(self.mode, Mode::Space(_)) {
            return super::space_world::SpaceWorld::model_origin_from_cg_frd_m();
        }
        let Mode::Free { vehicle, world, .. } = &self.mode else {
            return DVec3::ZERO;
        };
        let visual = crate::map_rig::aircraft::ownship_visual(None, Some(*vehicle));
        let (Some(origin), Some(cg)) = (
            self.model_origins.origin_in(visual),
            world
                .session()
                .and_then(|session| session.block().plant().cg_structural_in()),
        ) else {
            return DVec3::ZERO;
        };
        let aft_right_up = origin - cg;
        DVec3::new(-aft_right_up.x, aft_right_up.y, -aft_right_up.z) * METRES_PER_INCH
    }

    /// The ellipsoidal ground height the free-flight JSBSim plant was last given.
    #[cfg(test)]
    pub fn fdm_ground_m(&self) -> Option<f64> {
        match &self.mode {
            Mode::Free { world, .. } => world
                .session()
                .and_then(|session| session.block().plant().fdm_ground_m()),
            Mode::Scenario(_) | Mode::Space(_) => None,
        }
    }

    /// Latest engine status (`running`, `propeller_rpm`) on JSBSim free-flight aircraft.
    #[must_use]
    pub fn engine_status(&self) -> Option<EngineStatus> {
        match &self.mode {
            Mode::Free { world, .. } => world
                .session()
                .and_then(|session| session.block().plant().engine_status()),
            Mode::Scenario(run) => {
                let art = run.articulation();
                (art.engine_running || art.propeller_rpm.abs() > 1.0).then_some(EngineStatus {
                    running: art.engine_running,
                    propeller_rpm: art.propeller_rpm,
                })
            }
            Mode::Space(_) => None,
        }
    }

    /// Full powerplant, propeller, control-surface, and landing-gear articulation.
    #[must_use]
    pub fn articulation(&self) -> AirframeArticulation {
        match &self.mode {
            Mode::Free { world, .. } => {
                world
                    .session()
                    .map_or_else(AirframeArticulation::default, |session| {
                        let elapsed_s = seconds(session.physics_time());
                        session.block().plant().articulation(elapsed_s)
                    })
            }
            Mode::Scenario(run) => run.articulation(),
            Mode::Space(world) => world.articulation(),
        }
    }

    /// Whether an Aviate direct flight controller is armed (`None` on non-Aviate flights).
    #[must_use]
    pub fn aviate_armed(&self) -> Option<bool> {
        match &self.mode {
            Mode::Free { world, .. } => world
                .session()
                .and_then(|session| session.block().plant().aviate_armed()),
            Mode::Scenario(_) | Mode::Space(_) => None,
        }
    }
}
