//! A free-flight plant at one instant, and the way back to it.

use glam::DVec3;
use sim_core::{Block, JsbsimFdmSnapshot, KinematicModel, SystemControls};

use super::autopilot_c172::AutopilotC172Snapshot;
use super::{FixedWingSnapshot, FreeFlightPlant, X500Snapshot};
use crate::error::AppSimError;

/// Everything a [`FreeFlightPlant`] needs to fly the same bits again from one instant.
#[derive(Debug, Clone)]
pub enum PlantSnapshot {
    /// JSBSim aircraft state, latched systems, and the derived specific force.
    Jsbsim {
        /// Complete JSBSim executive state.
        fdm: Box<JsbsimFdmSnapshot>,
        /// Body FRD specific force (`m/s²`) at the snapshot.
        specific_force_frd_m_s2: DVec3,
        /// Latched operator system controls at the snapshot.
        system: Box<SystemControls>,
    },
    /// The hand-flown C172's whole chain: aeroplane, devices, flight controller and host.
    C172Autopilot(AutopilotC172Snapshot),
    /// Aviate X500 rigid body and position-hold target.
    AviateX500(Box<X500Snapshot>),
    /// Aviate fixed-wing JSBSim aircraft and attitude target.
    AviateFixedWing(Box<FixedWingSnapshot>),
    /// A flight on the real control chain, which keeps no rewindable state: the marker
    /// only says which plant it came from.
    C172Mission,
    /// Kinematic model state.
    Kinematic(Box<KinematicModel>),
}

impl FreeFlightPlant {
    /// Captures the plant for a later [`Self::restore`].
    #[must_use]
    pub fn snapshot(&self) -> PlantSnapshot {
        match self {
            Self::Jsbsim {
                fdm,
                specific_force_frd_m_s2,
            } => PlantSnapshot::Jsbsim {
                fdm: Box::new(fdm.snapshot()),
                specific_force_frd_m_s2: *specific_force_frd_m_s2,
                system: Box::new(fdm.system_controls()),
            },
            Self::C172Autopilot(c172) => PlantSnapshot::C172Autopilot(c172.snapshot()),
            Self::AviateX500(rig) => PlantSnapshot::AviateX500(Box::new(rig.snapshot())),
            Self::AviateFixedWing(rig) => PlantSnapshot::AviateFixedWing(Box::new(rig.snapshot())),
            Self::C172Mission(_) => PlantSnapshot::C172Mission,
            Self::Kinematic(model) => PlantSnapshot::Kinematic(Box::new(model.snapshot())),
        }
    }

    /// Rewinds to `snap`; Aviate controllers are rebuilt, fast-warmed, and re-armed there.
    /// Fails, leaving the plant as it was, on a snapshot of another kind of plant.
    pub fn restore(&mut self, snap: &PlantSnapshot) -> Result<(), AppSimError> {
        match (self, snap) {
            (
                Self::Jsbsim {
                    fdm,
                    specific_force_frd_m_s2,
                },
                PlantSnapshot::Jsbsim {
                    fdm: saved,
                    specific_force_frd_m_s2: force,
                    system,
                },
            ) => {
                fdm.try_restore(saved)
                    .map_err(sim_scenario::ScenarioError::from)?;
                fdm.set_system_controls(**system);
                *specific_force_frd_m_s2 = *force;
                Ok(())
            }
            (Self::C172Autopilot(c172), PlantSnapshot::C172Autopilot(saved)) => c172.restore(saved),
            (Self::AviateX500(rig), PlantSnapshot::AviateX500(saved)) => rig.restore(saved),
            (Self::AviateFixedWing(rig), PlantSnapshot::AviateFixedWing(saved)) => {
                rig.restore(saved)
            }
            (Self::Kinematic(model), PlantSnapshot::Kinematic(saved)) => {
                model.restore(saved);
                Ok(())
            }
            (Self::C172Mission(_), PlantSnapshot::C172Mission) => {
                Err(sim_scenario::ScenarioError::Unsupported {
                    detail: "a flight on the real control chain cannot be rewound; restart it",
                }
                .into())
            }
            _ => Err(sim_scenario::ScenarioError::SnapshotMismatch {
                detail: "the snapshot was taken from another free-flight plant",
            }
            .into()),
        }
    }
}
