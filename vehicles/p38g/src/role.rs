//! The semantic part roles cameras and articulation select airframe parts by.

/// Semantic articulation role authored via `custom token sim:partRole` on each USD `Mesh`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum P38PartRole {
    /// Main fuselage, boom, empennage, and exterior hardware.
    CabinExterior,
    /// Reserved cockpit interior tub, bulkheads, instrument panel, gunsight and seat.
    CabinInterior,
    /// The seated pilot, whom every camera but the pilot's own eye sees.
    PilotFigure,
    /// Forward upper nose deck visible ahead of the windscreen from the cockpit eyepoint.
    NoseCowling,
    /// Cambered main wing structure.
    WingStructure,
    /// Wing leading-edge intercoolers, pylons, and pitot hardware.
    WingStrut,
    /// Retractable tricycle landing-gear struts and wheel-well doors.
    GearStruts,
    /// Flashing red anti-collision fin beacon lens.
    BeaconLight,
    /// Retractable under-wing leading-edge landing light.
    LandingLight,
    /// Translucent multi-pane canopy glass and internal armored windshield.
    CabinGlass,
    /// Elongated Curtiss Electric propeller spinner cone and unit bands.
    PropellerSpinner,
    /// Individual hollow-steel propeller blades and warning tips (`sim:hingeAxis = +X`).
    PropellerBlades,
    /// Translucent 3-blade motion-blur propeller disc.
    PropellerDisc,
    /// Left Fowler flap segment.
    FlapLeft,
    /// Right Fowler flap segment.
    FlapRight,
    /// Left differential aileron.
    AileronLeft,
    /// Right differential aileron.
    AileronRight,
    /// Horizontal-tail elevator.
    Elevator,
    /// Twin vertical-fin rudder.
    Rudder,
    /// Steerable nosewheel tire and hubcap.
    NoseWheel,
    /// Left main-gear tire and hubcap.
    WheelLeft,
    /// Right main-gear tire and hubcap.
    WheelRight,
}

impl P38PartRole {
    /// Returns the OpenUSD `sim:partRole` token string for this role.
    #[must_use]
    pub const fn as_token(self) -> &'static str {
        match self {
            Self::CabinExterior => "cabin_exterior",
            Self::CabinInterior => "cabin_interior",
            Self::PilotFigure => "pilot_figure",
            Self::NoseCowling => "nose_cowling",
            Self::WingStructure => "wing_structure",
            Self::WingStrut => "wing_strut",
            Self::GearStruts => "gear_struts",
            Self::BeaconLight => "beacon_light",
            Self::LandingLight => "landing_light",
            Self::CabinGlass => "cabin_glass",
            Self::PropellerSpinner => "propeller_spinner",
            Self::PropellerBlades => "propeller_blades",
            Self::PropellerDisc => "propeller_disc",
            Self::FlapLeft => "flap_left",
            Self::FlapRight => "flap_right",
            Self::AileronLeft => "aileron_left",
            Self::AileronRight => "aileron_right",
            Self::Elevator => "elevator",
            Self::Rudder => "rudder",
            Self::NoseWheel => "nose_wheel",
            Self::WheelLeft => "wheel_left",
            Self::WheelRight => "wheel_right",
        }
    }
}
