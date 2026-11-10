use pilotage_adapter_api::{ActionCapability, IntentCapability, LinkLossPolicy, ScopeDescriptor};
use pilotage_protocol::{ActionKind, IntentFamily, ReferenceFrame, ScopeId, VehicleClass};
use sim_pack::SystemKind;

use super::MOTION_SCOPE;
use super::control::{MAX_ROLL_RAD, MAX_ROLL_RATE_RAD_S, MAX_YAW_RATE_RAD_S};

fn action(action: ActionKind) -> ActionCapability {
    ActionCapability {
        action,
        mode_targets: vec![],
        feel_targets: vec![],
        takeoff_start_state: None,
    }
}

/// The motion scope: typed body rates or NED attitude, each with thrust, and
/// nothing legacy. Roll is the loosest axis of both, so it is the advertised
/// bound; pitch and yaw clamp tighter and say so in the outcome.
pub(super) fn motion_scope_descriptor() -> ScopeDescriptor {
    ScopeDescriptor {
        payload: None,
        authority_group: None,
        scope: ScopeId::new(MOTION_SCOPE),
        axes: vec![],
        intents: vec![
            IntentCapability {
                family: IntentFamily::BodyRate,
                frames: vec![],
                max_linear: 0.0,
                max_vertical: 0.0,
                max_angular: MAX_ROLL_RATE_RAD_S as f32,
                max_yaw_rate: 0.0,
            },
            IntentCapability {
                family: IntentFamily::AttitudeThrust,
                frames: vec![ReferenceFrame::LocalNed],
                max_linear: 0.0,
                max_vertical: 0.0,
                max_angular: MAX_ROLL_RAD as f32,
                max_yaw_rate: MAX_YAW_RATE_RAD_S as f32,
            },
        ],
        actions: vec![action(ActionKind::Arm), action(ActionKind::Disarm)],
        legacy: None,
    }
}

/// The link-loss policies the rate loop can enact.
pub(super) fn supported_link_loss() -> Vec<LinkLossPolicy> {
    vec![
        LinkLossPolicy::Neutralize,
        LinkLossPolicy::HoldBrief { ticks: 0 },
    ]
}

/// Pilotage's class for a system of `kind`, if it has one. A rotorcraft flies
/// through the same hovering control scopes as a multicopter.
pub fn vehicle_class(kind: SystemKind) -> Option<VehicleClass> {
    match kind {
        SystemKind::FixedWing => Some(VehicleClass::FixedWing),
        SystemKind::Rotorcraft | SystemKind::Multirotor => Some(VehicleClass::Multicopter),
        SystemKind::SurfaceVehicle => Some(VehicleClass::GroundRover),
        SystemKind::LunarLander
        | SystemKind::ReentryCapsule
        | SystemKind::Spacecraft
        | SystemKind::GroundStation
        | SystemKind::TestBench
        | SystemKind::GenericSystem
        | SystemKind::InstrumentPack
        | SystemKind::Engine
        | SystemKind::Propeller
        | SystemKind::TypefacePack => None,
    }
}
