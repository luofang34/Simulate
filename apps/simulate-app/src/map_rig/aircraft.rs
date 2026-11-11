//! OpenUSD aircraft the physical scene draws: ownship and traffic, and which ownship
//! parts each camera sees.

use glam::{DMat3, DQuat, DVec3};
use sim_core::{
    AirframeArticulation, AirframeLights, AirframeVisibility, VehicleState, VirtualCameraSpec,
};
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_maplibre::{
    AircraftInstance, AltitudeReference, DynamicEntity3D, DynamicEntityKind, LatLon,
    OwnshipVisualKind, Placement3d, SlotCameraRole, ViewSlotMask, orientation_from_ned_body_quat,
};
use sim_scenario::{OperatorViewportMode, ScenarioProfile};

use crate::vehicles::FreeFlightVehicle;

/// Cruise propeller speed of drawn traffic, rpm.
const TRAFFIC_PROP_RPM: f64 = 2_350.0;

/// The ownship [`input`](super::MapFrameInput) draws, by [`ownship_aircraft`].
pub(super) fn ownship_of(input: &super::MapFrameInput<'_>, undulation_m: f64) -> AircraftInstance {
    ownship_aircraft(
        &input.vehicle,
        input.model_origin_from_cg_frd_m,
        input.articulation,
        undulation_m,
        input.ground_m,
    )
}

/// Ownship at its true state, with its own articulation, its model's origin
/// `origin_from_cg_frd_m` (body FRD metres) from the state's position, the CG. The
/// placement is `AboveMsl`, so the WGS84 ellipsoidal height becomes orthometric by
/// subtracting `undulation_m` (`H = h − N`); without it a real-world airframe sits `N`
/// metres off its runway. The ellipsoidal `ground_m` beneath, when known, takes the same
/// correction and receives the airframe's sun shadow.
pub fn ownship_aircraft(
    vehicle: &VehicleState,
    origin_from_cg_frd_m: DVec3,
    articulation: AirframeArticulation,
    undulation_m: f64,
    ground_m: Option<f64>,
) -> AircraftInstance {
    let geo = ecef_to_geodetic(vehicle.position_ecef + vehicle.attitude * origin_from_cg_frd_m);
    let ecef_to_ned = LocalFrame::at(&geo).ned_to_ecef().transpose();
    let ned_from_body =
        DQuat::from_mat3(&(ecef_to_ned * DMat3::from_quat(vehicle.attitude))).normalize();
    AircraftInstance {
        placement: Placement3d {
            position: LatLon::new(geo.lat_rad.to_degrees(), geo.lon_rad.to_degrees()),
            altitude_meters: geo.height_m - undulation_m,
            altitude_reference: AltitudeReference::AboveMsl,
            orientation: orientation_from_ned_body_quat(ned_from_body),
            view_mask: ViewSlotMask::ALL,
        },
        articulation,
        ownship: true,
        ground_height_m: ground_m.map(|h| h - undulation_m),
    }
}

/// Fixed-wing truth traffic drawn as cruising C172s with spinning propellers.
pub fn traffic_aircraft(entities: &[DynamicEntity3D], elapsed_s: f64) -> Vec<AircraftInstance> {
    let phase = (elapsed_s * TRAFFIC_PROP_RPM / 60.0 * std::f64::consts::TAU)
        .rem_euclid(std::f64::consts::TAU);
    entities
        .iter()
        .filter(|entity| entity.kind == DynamicEntityKind::FixedWing)
        .map(|entity| AircraftInstance {
            placement: entity.placement,
            articulation: AirframeArticulation {
                engine_running: true,
                engine_rpm: TRAFFIC_PROP_RPM,
                propeller_rpm: TRAFFIC_PROP_RPM,
                propeller_phase_rad: phase,
                lights: AirframeLights {
                    beacon_flash_norm: if (elapsed_s % 1.2) < 0.12 { 1.0 } else { 0.0 },
                    ..AirframeLights::default()
                },
                ..AirframeArticulation::default()
            },
            ownship: false,
            ground_height_m: None,
        })
        .collect()
}

/// Which ownship parts the main window sees in `mode` flying `visual`: a bubble-canopy
/// fighter's pilot sees the airframe around the seat, a cabin pilot looks past it.
pub const fn window_camera_role(
    mode: OperatorViewportMode,
    visual: OwnshipVisualKind,
) -> SlotCameraRole {
    match (mode, visual) {
        (OperatorViewportMode::Cockpit, OwnshipVisualKind::P38g) => SlotCameraRole::PilotEye,
        (OperatorViewportMode::Cockpit, _) => SlotCameraRole::HudForward,
        (
            OperatorViewportMode::Chase
            | OperatorViewportMode::VolumetricScene
            | OperatorViewportMode::ChartMap
            | OperatorViewportMode::SensorAperture,
            _,
        ) => SlotCameraRole::External,
    }
}

/// Which ownship parts a virtual camera sees, as its spec declares.
pub fn virtual_camera_role(spec: &VirtualCameraSpec) -> SlotCameraRole {
    match spec.airframe {
        AirframeVisibility::External => SlotCameraRole::External,
        AirframeVisibility::PilotEye => SlotCameraRole::PilotEye,
        AirframeVisibility::CabinWindow => SlotCameraRole::LeftCabinWindow,
        AirframeVisibility::Hidden => SlotCameraRole::BellyNadir,
    }
}

/// The ownship airframe for what is flying: the scenario's vehicle, else the free-flight
/// plant's airframe, else the spaceflight's Raketa.
pub const fn ownship_visual(
    profile: Option<ScenarioProfile>,
    vehicle: Option<FreeFlightVehicle>,
) -> OwnshipVisualKind {
    match (profile, vehicle) {
        (Some(ScenarioProfile::Apollo11CapsuleReentry), _) => OwnshipVisualKind::Hidden,
        (Some(ScenarioProfile::Apollo11LunarLanding), _) => OwnshipVisualKind::LunarLander,
        (
            Some(
                ScenarioProfile::CorridorNominalCruise
                | ScenarioProfile::CorridorOffSeedAcquisition
                | ScenarioProfile::CorridorAggressiveBank
                | ScenarioProfile::CorridorSameCellLoiter,
            ),
            _,
        ) => OwnshipVisualKind::FixedWingUav,
        (
            Some(
                ScenarioProfile::Airport47NUncontrolledPattern
                | ScenarioProfile::AirportKewrToweredPattern,
            ),
            _,
        )
        | (None, Some(FreeFlightVehicle::JsbsimC172 | FreeFlightVehicle::AviateFixedWingDirect)) => {
            OwnshipVisualKind::C172
        }
        (None, None) => OwnshipVisualKind::Raketa,
        (None, Some(FreeFlightVehicle::JsbsimP38g)) => OwnshipVisualKind::P38g,
        (None, Some(FreeFlightVehicle::Kinematic)) => OwnshipVisualKind::FixedWingUav,
        (None, Some(FreeFlightVehicle::AviateX500Direct | FreeFlightVehicle::X500Mission)) => {
            OwnshipVisualKind::QuadrotorX500
        }
    }
}
