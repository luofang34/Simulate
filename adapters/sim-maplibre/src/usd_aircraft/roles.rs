//! Camera mounting roles and the semantic part roles they cull ownship parts by.

/// Camera mounting role governing interior/glass/wing self-occlusion culling on ownship.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SlotCameraRole {
    /// Unobstructed forward HUD / boresight view hiding all ownship airframe and propeller parts.
    HudForward,
    /// Pilot eye inside the cabin looking forward through the windshield and propeller arc.
    CockpitInterior,
    /// Pilot eye in an open-view cockpit (a fighter's bubble canopy), which sees the
    /// whole airframe around it but not the pilot's own body.
    PilotEye,
    /// Left cabin window camera looking abeam under the high wing and strut.
    LeftCabinWindow,
    /// Belly nadir optical navigation camera with unobstructed downward view.
    BellyNadir,
    /// External camera (chase, tower PTZ, wingtip, flyby) viewing the complete aircraft.
    #[default]
    External,
}

/// Semantic articulation role authored via `custom token sim:partRole` on each USD `Mesh`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartRole {
    /// Main fuselage, empennage, and wheel-pant exterior skin.
    CabinExterior,
    /// Cabin interior liner and instrument glare shield.
    CabinInterior,
    /// The seated pilot, hidden from the pilot's own eye.
    PilotFigure,
    /// Forward engine cowling visible ahead of the windshield from the pilot seat.
    NoseCowling,
    /// High-wing structure visible from external and cabin-window cameras.
    WingStructure,
    /// Diagonal wing lift struts and pitot hardware.
    WingStrut,
    /// Fixed or retractable landing-gear struts and open wheel-bay doors.
    GearStruts,
    /// Flashing red anti-collision fin beacon lens.
    BeaconLight,
    /// Retractable under-wing or leading-edge landing light assembly.
    LandingLight,
    /// Translucent cabin windshield and side/rear windows.
    CabinGlass,
    /// Polished nose spinner cone rotating with the propeller hub.
    PropellerSpinner,
    /// Solid twisted propeller blades and safety tips (visible at low RPM / stopped).
    PropellerBlades,
    /// Translucent motion-blurred propeller disc annulus (visible at running RPM).
    PropellerDisc,
    /// Left trailing-edge electric slotted flap.
    FlapLeft,
    /// Right trailing-edge electric slotted flap.
    FlapRight,
    /// Left outboard differential aileron.
    AileronLeft,
    /// Right outboard differential aileron.
    AileronRight,
    /// Horizontal-tail elevator.
    Elevator,
    /// Swept vertical-fin rudder.
    Rudder,
    /// Left V-tail ruddervator hinged about local `+X`, deflecting by
    /// `elevator_rad - rudder_rad` so it mixes pitch and yaw.
    RuddervatorLeft,
    /// Right V-tail ruddervator hinged about local `+X`, deflecting by
    /// `elevator_rad + rudder_rad` so it mixes pitch and yaw.
    RuddervatorRight,
    /// Steerable nosewheel tire and hubcap.
    NoseWheel,
    /// Left main-gear tire and hubcap.
    WheelLeft,
    /// Right main-gear tire and hubcap.
    WheelRight,
    /// An instrument needle turning about its part-local `+Y` (the dial normal,
    /// into the panel) by the angle of its [`UsdMeshPart::needle_channel`].
    ///
    /// [`UsdMeshPart::needle_channel`]: super::UsdMeshPart::needle_channel
    InstrumentNeedle,
}

impl PartRole {
    pub(crate) fn from_token(token: &str) -> Self {
        match token {
            "cabin_interior" => Self::CabinInterior,
            "pilot_figure" => Self::PilotFigure,
            "nose_cowling" => Self::NoseCowling,
            "wing_structure" => Self::WingStructure,
            "wing_strut" => Self::WingStrut,
            "gear_struts" => Self::GearStruts,
            "beacon_light" => Self::BeaconLight,
            "landing_light" => Self::LandingLight,
            "cabin_glass" => Self::CabinGlass,
            "propeller_spinner" => Self::PropellerSpinner,
            "propeller_blades" => Self::PropellerBlades,
            "propeller_disc" => Self::PropellerDisc,
            "flap_left" => Self::FlapLeft,
            "flap_right" => Self::FlapRight,
            "aileron_left" => Self::AileronLeft,
            "aileron_right" => Self::AileronRight,
            "elevator" => Self::Elevator,
            "rudder" => Self::Rudder,
            "ruddervator_left" => Self::RuddervatorLeft,
            "ruddervator_right" => Self::RuddervatorRight,
            "nose_wheel" => Self::NoseWheel,
            "wheel_left" => Self::WheelLeft,
            "wheel_right" => Self::WheelRight,
            "instrument_needle" => Self::InstrumentNeedle,
            _ => Self::CabinExterior,
        }
    }
}

pub(super) fn role_visible_to_camera(role: PartRole, camera: SlotCameraRole) -> bool {
    use PartRole::{
        CabinExterior, CabinGlass, CabinInterior, PilotFigure, PropellerBlades, PropellerDisc,
        PropellerSpinner,
    };
    match camera {
        SlotCameraRole::External => true,
        SlotCameraRole::HudForward | SlotCameraRole::BellyNadir => false,
        SlotCameraRole::CockpitInterior => {
            matches!(role, PropellerSpinner | PropellerBlades | PropellerDisc)
        }
        SlotCameraRole::PilotEye => role != PilotFigure,
        SlotCameraRole::LeftCabinWindow => !matches!(
            role,
            CabinExterior | CabinInterior | CabinGlass | PilotFigure
        ),
    }
}
