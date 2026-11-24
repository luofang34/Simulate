//! First-class staging, docking, tethering, and multi-vessel mechanism
//! definitions on [`crate::Universe`].

use glam::DVec3;
use mbd_core::{ComponentId, Placement};
use mbd_multibody::{
    DockingPhase, DockingSpec, DockingState, HingeSwingSpec, HingeSwingState, RailSlideSpec,
    RailSlideState, TetherCommand, TetherCutReason, WinchTetherSpec, WinchTetherState,
};
use sim_core::{InterVesselLink, SimVesselId};
use sim_orbital::VesselId;

use crate::{Universe, UniverseError};

/// Unique identifier for an inter-vessel or mechanism link registered on [`Universe`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LinkId(pub u64);

/// Configuration for registering a compliant soft-capture docking link between two vessels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingLinkSpec {
    /// Chaser/active vessel.
    pub chaser: VesselId,
    /// Docking port component on `chaser`.
    pub port_a: ComponentId,
    /// Port ring origin in `port_a` local axes, m.
    pub offset_a_local_m: DVec3,
    /// Outward unit capture axis in `port_a` local axes.
    pub axis_a_local: DVec3,
    /// Target/passive vessel.
    pub target: VesselId,
    /// Docking port component on `target`.
    pub port_b: ComponentId,
    /// Port ring origin in `port_b` local axes, m.
    pub offset_b_local_m: DVec3,
    /// Relative mated placement of `target` port root in `port_a` axes at hard latch.
    pub mate_placement: Placement,
    /// Capture envelope, stiffness, damping, and undocking impulse specification.
    pub spec: DockingSpec,
    /// Initial mechanism phase and retraction stand-off.
    pub state: DockingState,
}

/// Physical mechanism or field link registered on [`Universe`], evaluated cleanly
/// inside `sim-core` proximity groups without tier desynchronization.
#[derive(Debug, Clone, PartialEq)]
pub enum UniverseLinkSpec {
    /// Prismatic seat/rail ejection actuator driving a joint until release.
    RailSlide {
        /// Parent vessel carrying the rail joint.
        vessel: VesselId,
        /// Rail length, catapult force, and friction specification.
        spec: RailSlideSpec,
        /// Current firing and readiness state.
        state: RailSlideState,
    },
    /// Hinged fairing/petal swing mechanism separating automatically past release angle.
    HingeSwing {
        /// Parent vessel carrying the hinged component.
        vessel: VesselId,
        /// Hinge swing angle and spring/damper specification.
        spec: HingeSwingSpec,
        /// Current latch and readiness state.
        state: HingeSwingState,
    },
    /// Compliant soft-capture docking ring converging to rigid hard-mate.
    Docking(DockingLinkSpec),
    /// Variable-length winch tether, umbilical, or Sky Crane bridle.
    WinchTether {
        /// Carrier/winch vessel.
        carrier: VesselId,
        /// Component carrying winch anchor on `carrier`.
        anchor_carrier: ComponentId,
        /// Anchor point offset in `anchor_carrier` local axes, m.
        offset_carrier_m: DVec3,
        /// Suspended/tethered vessel.
        payload: VesselId,
        /// Component carrying anchor on `payload`.
        anchor_payload: ComponentId,
        /// Anchor point offset in `anchor_payload` local axes, m.
        offset_payload_m: DVec3,
        /// Stiffness, damping, max length, and severing limits.
        spec: WinchTetherSpec,
        /// Current deployed length, intact status, thermal energy, and dwell timer.
        state: WinchTetherState,
        /// Host spooling, heating, and pyrotechnic cut command.
        command: TetherCommand,
    },
    /// Thruster exhaust plume impingement and thermal load transfer onto a target vessel patch.
    PlumeLink {
        /// Vessel firing the thruster.
        source_vessel: VesselId,
        /// Nozzle component on `source_vessel`.
        nozzle_site: ComponentId,
        /// Nozzle exit origin in `nozzle_site` local axes, m.
        nozzle_offset_local_m: DVec3,
        /// Unit exhaust flow direction in `nozzle_site` local axes.
        exhaust_dir_local: DVec3,
        /// Nozzle exit radius, m.
        nozzle_radius_m: f64,
        /// Plume divergence half-angle, rad.
        cone_half_angle_rad: f64,
        /// Exhaust jet thermal power ratio relative to thrust (`thermal_w / thrust_n`).
        exhaust_thermal_w_per_n: f64,
        /// Target vessel struck by the exhaust cone.
        target_vessel: VesselId,
        /// Struck target component on `target_vessel`.
        target_site: ComponentId,
        /// Surface patch centroid in `target_site` local axes, m.
        target_point_local_m: DVec3,
        /// Outward unit surface normal in `target_site` local axes.
        target_normal_local: DVec3,
        /// Struck target surface area, m².
        target_area_m2: f64,
        /// Surface thermal absorptivity `[0..=1]`.
        thermal_absorptivity: f64,
    },
}

/// Active inter-vessel or mechanism link owned by [`Universe`].
#[derive(Debug, Clone, PartialEq)]
pub struct UniverseInterVesselLink {
    /// Stable identifier assigned when the link was added.
    pub id: LinkId,
    /// Link mechanism kind, endpoints, parameters, and mutable state.
    pub spec: UniverseLinkSpec,
}

/// Topology changes and severing events accumulated across coordinated steps or commands.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ProximityStepReport {
    /// New `(parent_id, spawned_child_id)` vessels created by staging, undocking, or release.
    pub spawned_vessels: Vec<(VesselId, VesselId)>,
    /// Merged `(chaser_id, retired_target_id)` pairs joined by docking hard-mate or grapple.
    pub merged_vessels: Vec<(VesselId, VesselId)>,
    /// Severed `(link_id, reason)` tether events.
    pub cut_tethers: Vec<(LinkId, TetherCutReason)>,
}

impl ProximityStepReport {
    /// Whether no proximity spawns, merges, or tether cuts occurred.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.spawned_vessels.is_empty()
            && self.merged_vessels.is_empty()
            && self.cut_tethers.is_empty()
    }

    pub(crate) fn extend(&mut self, other: Self) {
        self.spawned_vessels.extend(other.spawned_vessels);
        self.merged_vessels.extend(other.merged_vessels);
        self.cut_tethers.extend(other.cut_tethers);
    }
}

impl UniverseLinkSpec {
    pub(crate) fn to_core_link(&self, id: u64) -> InterVesselLink {
        match *self {
            Self::RailSlide {
                vessel,
                spec,
                state,
            } => InterVesselLink::RailSlide {
                id,
                vessel: SimVesselId(vessel.0),
                spec,
                state,
            },
            Self::HingeSwing {
                vessel,
                spec,
                state,
            } => InterVesselLink::HingeSwing {
                id,
                vessel: SimVesselId(vessel.0),
                spec,
                state,
            },
            Self::Docking(d) => InterVesselLink::Docking {
                id,
                chaser: SimVesselId(d.chaser.0),
                port_a: d.port_a,
                offset_a_local_m: d.offset_a_local_m,
                axis_a_local: d.axis_a_local,
                target: SimVesselId(d.target.0),
                port_b: d.port_b,
                offset_b_local_m: d.offset_b_local_m,
                mate_placement: d.mate_placement,
                spec: d.spec,
                state: d.state,
            },
            Self::WinchTether {
                carrier,
                anchor_carrier,
                offset_carrier_m,
                payload,
                anchor_payload,
                offset_payload_m,
                spec,
                state,
                command,
            } => InterVesselLink::WinchTether {
                id,
                carrier: SimVesselId(carrier.0),
                anchor_carrier,
                offset_carrier_m,
                payload: SimVesselId(payload.0),
                anchor_payload,
                offset_payload_m,
                spec,
                state,
                command,
            },
            Self::PlumeLink { .. } => self.plume_to_core(id),
        }
    }

    fn plume_to_core(&self, id: u64) -> InterVesselLink {
        let Self::PlumeLink {
            source_vessel,
            nozzle_site,
            nozzle_offset_local_m,
            exhaust_dir_local,
            nozzle_radius_m,
            cone_half_angle_rad,
            exhaust_thermal_w_per_n,
            target_vessel,
            target_site,
            target_point_local_m,
            target_normal_local,
            target_area_m2,
            thermal_absorptivity,
        } = *self
        else {
            return InterVesselLink::PlumeLink {
                id,
                source_vessel: SimVesselId(0),
                nozzle_site: ComponentId(0),
                nozzle_offset_local_m: DVec3::ZERO,
                exhaust_dir_local: DVec3::X,
                nozzle_radius_m: 0.0,
                cone_half_angle_rad: 0.0,
                exhaust_thermal_w_per_n: 0.0,
                target_vessel: SimVesselId(0),
                target_site: ComponentId(0),
                target_point_local_m: DVec3::ZERO,
                target_normal_local: DVec3::X,
                target_area_m2: 0.0,
                thermal_absorptivity: 0.0,
            };
        };
        InterVesselLink::PlumeLink {
            id,
            source_vessel: SimVesselId(source_vessel.0),
            nozzle_site,
            nozzle_offset_local_m,
            exhaust_dir_local,
            nozzle_radius_m,
            cone_half_angle_rad,
            exhaust_thermal_w_per_n,
            target_vessel: SimVesselId(target_vessel.0),
            target_site,
            target_point_local_m,
            target_normal_local,
            target_area_m2,
            thermal_absorptivity,
        }
    }

    pub(crate) fn from_core_link(link: &InterVesselLink) -> Self {
        match *link {
            InterVesselLink::RailSlide {
                vessel,
                spec,
                state,
                ..
            } => Self::RailSlide {
                vessel: VesselId(vessel.0),
                spec,
                state,
            },
            InterVesselLink::HingeSwing {
                vessel,
                spec,
                state,
                ..
            } => Self::HingeSwing {
                vessel: VesselId(vessel.0),
                spec,
                state,
            },
            InterVesselLink::Docking {
                chaser,
                port_a,
                offset_a_local_m,
                axis_a_local,
                target,
                port_b,
                offset_b_local_m,
                mate_placement,
                spec,
                state,
                ..
            } => Self::Docking(DockingLinkSpec {
                chaser: VesselId(chaser.0),
                port_a,
                offset_a_local_m,
                axis_a_local,
                target: VesselId(target.0),
                port_b,
                offset_b_local_m,
                mate_placement,
                spec,
                state,
            }),
            InterVesselLink::WinchTether {
                carrier,
                anchor_carrier,
                offset_carrier_m,
                payload,
                anchor_payload,
                offset_payload_m,
                spec,
                state,
                command,
                ..
            } => Self::WinchTether {
                carrier: VesselId(carrier.0),
                anchor_carrier,
                offset_carrier_m,
                payload: VesselId(payload.0),
                anchor_payload,
                offset_payload_m,
                spec,
                state,
                command,
            },
            InterVesselLink::PlumeLink { .. } => Self::plume_from_core(link),
        }
    }

    fn plume_from_core(link: &InterVesselLink) -> Self {
        let InterVesselLink::PlumeLink {
            source_vessel,
            nozzle_site,
            nozzle_offset_local_m,
            exhaust_dir_local,
            nozzle_radius_m,
            cone_half_angle_rad,
            exhaust_thermal_w_per_n,
            target_vessel,
            target_site,
            target_point_local_m,
            target_normal_local,
            target_area_m2,
            thermal_absorptivity,
            ..
        } = *link
        else {
            return Self::RailSlide {
                vessel: VesselId(0),
                spec: RailSlideSpec {
                    component: ComponentId(0),
                    rail_length_m: 0.0,
                    catapult_force_n: 0.0,
                    guide_friction_n: 0.0,
                },
                state: RailSlideState {
                    fired: false,
                    ready_to_release: false,
                },
            };
        };
        Self::PlumeLink {
            source_vessel: VesselId(source_vessel.0),
            nozzle_site,
            nozzle_offset_local_m,
            exhaust_dir_local,
            nozzle_radius_m,
            cone_half_angle_rad,
            exhaust_thermal_w_per_n,
            target_vessel: VesselId(target_vessel.0),
            target_site,
            target_point_local_m,
            target_normal_local,
            target_area_m2,
            thermal_absorptivity,
        }
    }

    pub(crate) fn endpoints(&self) -> (VesselId, Option<VesselId>) {
        match *self {
            Self::RailSlide { vessel, .. } | Self::HingeSwing { vessel, .. } => (vessel, None),
            Self::Docking(d) => (d.chaser, Some(d.target)),
            Self::WinchTether {
                carrier, payload, ..
            } => (carrier, Some(payload)),
            Self::PlumeLink {
                source_vessel,
                target_vessel,
                ..
            } => (source_vessel, Some(target_vessel)),
        }
    }

    /// Whether the link is still active (not completed, latched, or severed).
    #[must_use]
    pub const fn is_active(&self) -> bool {
        match self {
            Self::RailSlide { state, .. } => !state.ready_to_release,
            Self::HingeSwing { state, .. } => !state.ready_to_release,
            Self::Docking(d) => !matches!(d.state.phase, DockingPhase::HardLatched),
            Self::WinchTether { state, .. } => state.intact,
            Self::PlumeLink { .. } => true,
        }
    }
}

impl Universe {
    /// Active inter-vessel and single-vessel mechanism links registered on this universe.
    #[must_use]
    pub fn inter_vessel_links(&self) -> &[UniverseInterVesselLink] {
        &self.links
    }

    /// Looks up a single registered link by identifier.
    #[must_use]
    pub fn inter_vessel_link(&self, id: LinkId) -> Option<&UniverseInterVesselLink> {
        self.links.iter().find(|link| link.id == id)
    }

    /// Whether `vessel` currently participates in any active two-vessel inter-vessel link.
    #[must_use]
    pub fn is_vessel_coupled(&self, vessel: VesselId) -> bool {
        self.links.iter().any(|link| {
            link.spec.is_active()
                && matches!(
                    link.spec.endpoints(),
                    (a, Some(b)) if a == vessel || b == vessel
                )
        })
    }

    pub(crate) fn is_vessel_in_any_link(&self, vessel: VesselId) -> bool {
        self.links.iter().any(|link| {
            link.spec.is_active()
                && match link.spec.endpoints() {
                    (a, Some(b)) => a == vessel || b == vessel,
                    (a, None) => a == vessel,
                }
        })
    }

    /// Currently focused vessel identifier (if any vessel exists).
    #[must_use]
    pub const fn active_vessel(&self) -> Option<VesselId> {
        self.active_vessel
    }

    /// Switches active focus to existing vessel `id`.
    pub fn set_active_vessel(&mut self, id: VesselId) -> Result<(), UniverseError> {
        if !self.vessels.contains_key(&id) {
            return Err(UniverseError::UnknownVessel(id));
        }
        self.active_vessel = Some(id);
        Ok(())
    }

    /// Cycles active focus to the next live vessel in deterministic `VesselId` order.
    pub fn cycle_active_vessel(&mut self) -> Option<VesselId> {
        let ids: Vec<VesselId> = self.vessels.keys().copied().collect();
        if ids.is_empty() {
            self.active_vessel = None;
            return None;
        }
        let next = self
            .active_vessel
            .and_then(|cur| ids.iter().position(|&id| id == cur))
            .and_then(|pos| ids.get((pos.wrapping_add(1)) % ids.len()))
            .copied()
            .or_else(|| ids.first().copied());
        self.active_vessel = next;
        next
    }

    /// Returns and clears all proximity/staging/docking/tether events accumulated since the last call.
    pub fn take_proximity_report(&mut self) -> ProximityStepReport {
        std::mem::take(&mut self.proximity_report)
    }
}
