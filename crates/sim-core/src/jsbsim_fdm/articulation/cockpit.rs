//! The flight model's control commands the cockpit's controls are drawn from.

use jsbsim::{Fdm, NodeId};

use crate::BlockError;
use crate::airframe::{CockpitControls, FlapSelector};
use crate::jsbsim_fdm::fdm_error;
use crate::jsbsim_fdm::system::SystemControls;

/// The command properties of the flight controls, both engines' levers, the
/// trims, the flaps and the gear. JSBSim's elevator and rudder commands run
/// opposite to the pilot's column and pedals.
#[derive(Debug, Clone, Copy)]
pub(super) struct CockpitCommandNodes {
    aileron: Option<NodeId>,
    elevator: Option<NodeId>,
    rudder: Option<NodeId>,
    throttle: [Option<NodeId>; 2],
    advance: [Option<NodeId>; 2],
    mixture: [Option<NodeId>; 2],
    trim: [Option<NodeId>; 3],
    flaps: Option<NodeId>,
    gear: Option<NodeId>,
}

impl CockpitCommandNodes {
    pub(super) fn resolve(fdm: &Fdm) -> Result<Self, BlockError> {
        let opt = |path: &str| fdm.node(path).map_err(fdm_error);
        let pair = |path: &str| -> Result<[Option<NodeId>; 2], BlockError> {
            Ok([opt(path)?, opt(&format!("{path}[1]"))?])
        };
        Ok(Self {
            aileron: opt("fcs/aileron-cmd-norm")?,
            elevator: opt("fcs/elevator-cmd-norm")?,
            rudder: opt("fcs/rudder-cmd-norm")?,
            throttle: pair("fcs/throttle-cmd-norm")?,
            advance: pair("fcs/advance-cmd-norm")?,
            mixture: pair("fcs/mixture-cmd-norm")?,
            trim: [
                opt("fcs/roll-trim-cmd-norm")?,
                opt("fcs/pitch-trim-cmd-norm")?,
                opt("fcs/yaw-trim-cmd-norm")?,
            ],
            flaps: opt("fcs/flap-cmd-norm")?,
            gear: opt("gear/gear-cmd-norm")?,
        })
    }

    /// The controls as commanded; a single engine's second levers mirror the
    /// first, and a missing property reads as the lever's rest position.
    pub(super) fn read(
        &self,
        fdm: &mut Fdm,
        system: SystemControls,
    ) -> Result<CockpitControls, BlockError> {
        let mut rd = |n: Option<NodeId>, fallback: f64| -> Result<f64, BlockError> {
            let v = n.map_or(Ok(fallback), |id| fdm.read_node(id).map_err(fdm_error))?;
            Ok(if v.is_finite() { v } else { fallback })
        };
        let mut both = |[a, b]: [Option<NodeId>; 2], rest: f64| -> Result<[f64; 2], BlockError> {
            let first = rd(a, rest)?.clamp(0.0, 1.0);
            Ok([first, rd(b, first)?.clamp(0.0, 1.0)])
        };
        let throttle_norm = both(self.throttle, 0.0)?;
        let prop_norm = both(self.advance, 1.0)?;
        let mixture_norm = both(self.mixture, 1.0)?;
        let axis = |v: f64| v.clamp(-1.0, 1.0);
        let [tr, tp, ty] = self.trim;
        Ok(CockpitControls {
            pitch_norm: axis(-rd(self.elevator, 0.0)?),
            roll_norm: axis(rd(self.aileron, 0.0)?),
            yaw_norm: axis(-rd(self.rudder, 0.0)?),
            throttle_norm,
            prop_norm,
            mixture_norm,
            trim_norm: [axis(rd(tr, 0.0)?), axis(rd(tp, 0.0)?), axis(rd(ty, 0.0)?)],
            flap_selector: FlapSelector::from_command(
                rd(self.flaps, 0.0)?,
                system.flap_valve_closed,
            ),
            gear_lever_down: rd(self.gear, 1.0)? > 0.5,
            canopy: system.canopy,
        })
    }
}
