//! Where a JSBSim aircraft's centre of gravity is in its structural frame. The block's
//! position is the CG, which moves as fuel burns and loads change, so anything fixed to
//! the airframe (a drawn model, its wheels) is placed from the structure, not the CG.

use glam::DVec3;
use jsbsim::{Fdm, NodeId};

use super::{BLOCK, JsbsimFdmBlock, fdm_error};
use crate::BlockError;

/// The CG properties, resolved at load.
#[derive(Debug, Clone, Copy)]
pub(super) struct MassNodes {
    cg_in: [NodeId; 3],
}

impl MassNodes {
    pub(super) fn resolve(fdm: &Fdm) -> Result<Self, BlockError> {
        let node = |path: &'static str| {
            fdm.node(path)
                .map_err(fdm_error)?
                .ok_or(BlockError::InputOutOfRange {
                    block: BLOCK,
                    input: "aircraft",
                    detail: format!("the loaded aircraft has no property {path}"),
                })
        };
        Ok(Self {
            cg_in: [
                node("inertia/cg-x-in")?,
                node("inertia/cg-y-in")?,
                node("inertia/cg-z-in")?,
            ],
        })
    }

    /// The CG now, structural inches (`x` aft, `y` right, `z` up).
    pub(super) fn sample(self, fdm: &mut Fdm) -> Result<DVec3, BlockError> {
        let [x, y, z] = self.cg_in;
        Ok(DVec3::new(
            fdm.read_node(x).map_err(fdm_error)?,
            fdm.read_node(y).map_err(fdm_error)?,
            fdm.read_node(z).map_err(fdm_error)?,
        ))
    }
}

impl JsbsimFdmBlock {
    /// The centre of gravity, the point [`crate::VehicleState::position_ecef`] tracks, in
    /// the aircraft's structural frame: inches, `x` aft, `y` right, `z` up, as of the
    /// latest step.
    #[must_use]
    pub fn cg_structural_in(&self) -> DVec3 {
        self.latest.cg_structural_in
    }
}
