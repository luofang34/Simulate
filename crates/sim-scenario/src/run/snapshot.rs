use sim_core::{Block, JsbsimFdmBlock, JsbsimFdmSnapshot, KinematicModel};

use crate::ScenarioError;

/// Complete state of whichever flight dynamics source a fixed-wing scenario
/// flies on. A snapshot only restores into the same kind of source, because
/// JSBSim state cannot seed a kinematic model or the other way round.
#[derive(Debug, Clone)]
pub(crate) enum FdmSnapshot {
    Kinematic(Box<KinematicModel>),
    Jsbsim(Box<JsbsimFdmSnapshot>),
}

impl FdmSnapshot {
    pub(crate) fn of_kinematic(model: &KinematicModel) -> Self {
        Self::Kinematic(Box::new(model.snapshot()))
    }

    pub(crate) fn of_jsbsim(model: &JsbsimFdmBlock) -> Self {
        Self::Jsbsim(Box::new(model.snapshot()))
    }

    pub(crate) fn restore_kinematic(
        &self,
        model: &mut KinematicModel,
    ) -> Result<(), ScenarioError> {
        match self {
            Self::Kinematic(snapshot) => {
                model.restore(snapshot);
                Ok(())
            }
            Self::Jsbsim(_) => Err(ScenarioError::SnapshotMismatch {
                detail: "a JSBSim snapshot cannot restore a kinematic model",
            }),
        }
    }

    pub(crate) fn restore_jsbsim(&self, model: &mut JsbsimFdmBlock) -> Result<(), ScenarioError> {
        match self {
            Self::Jsbsim(snapshot) => Ok(model.try_restore(snapshot)?),
            Self::Kinematic(_) => Err(ScenarioError::SnapshotMismatch {
                detail: "a kinematic snapshot cannot restore a JSBSim model",
            }),
        }
    }
}
