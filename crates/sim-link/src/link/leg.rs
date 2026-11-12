//! One leg of the route: an ARINC 424 path terminator, the fix it ends at
//! or starts from, and the attributes the procedure publishes for it.

mod shape;

pub use shape::{EVERY_LEG, Shape, distance_from_navaid, shape};

use crate::{Bearing, Distance, Height, HoldLength, Limit, Micro, Milli, SecondAltitude, Speed};

/// The ARINC 424 path terminator of a leg: the first letter names the path,
/// the second what ends it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PathTerminator {
    /// `IF`, the initial fix of the route.
    If,
    /// `TF`, a great-circle track from the fix before to this fix.
    Tf,
    /// `CF`, a course to this fix.
    Cf,
    /// `DF`, direct from wherever the vehicle is to this fix.
    Df,
    /// `FA`, a course from this fix until an altitude.
    Fa,
    /// `FC`, a course from this fix for a distance.
    Fc,
    /// `FD`, a course from this fix until a DME distance from a navaid.
    Fd,
    /// `FM`, a course from this fix until the pilot or controller ends it.
    Fm,
    /// `CA`, a course until an altitude.
    Ca,
    /// `CD`, a course until a DME distance from a navaid.
    Cd,
    /// `CI`, a course until it intercepts the next leg.
    Ci,
    /// `CR`, a course until a radial of a navaid.
    Cr,
    /// `RF`, a constant-radius arc about a centre to this fix.
    Rf,
    /// `AF`, a DME arc about a navaid to this fix.
    Af,
    /// `VA`, a heading until an altitude.
    Va,
    /// `VD`, a heading until a DME distance from a navaid.
    Vd,
    /// `VI`, a heading until it intercepts the next leg.
    Vi,
    /// `VM`, a heading until the pilot or controller ends it.
    Vm,
    /// `VR`, a heading until a radial of a navaid.
    Vr,
    /// `PI`, a procedure turn at this fix.
    Pi,
    /// `HA`, a hold at this fix until an altitude.
    Ha,
    /// `HF`, one circuit of a hold at this fix.
    Hf,
    /// `HM`, a hold at this fix until the pilot or controller ends it.
    Hm,
}

impl PathTerminator {
    /// Every path terminator.
    pub const ALL: [Self; 23] = [
        Self::If,
        Self::Tf,
        Self::Cf,
        Self::Df,
        Self::Fa,
        Self::Fc,
        Self::Fd,
        Self::Fm,
        Self::Ca,
        Self::Cd,
        Self::Ci,
        Self::Cr,
        Self::Rf,
        Self::Af,
        Self::Va,
        Self::Vd,
        Self::Vi,
        Self::Vm,
        Self::Vr,
        Self::Pi,
        Self::Ha,
        Self::Hf,
        Self::Hm,
    ];

    /// The two-letter code, as procedure coding and a link write it.
    pub const fn code(self) -> &'static str {
        match self {
            Self::If => "IF",
            Self::Tf => "TF",
            Self::Cf => "CF",
            Self::Df => "DF",
            Self::Fa => "FA",
            Self::Fc => "FC",
            Self::Fd => "FD",
            Self::Fm => "FM",
            Self::Ca => "CA",
            Self::Cd => "CD",
            Self::Ci => "CI",
            Self::Cr => "CR",
            Self::Rf => "RF",
            Self::Af => "AF",
            Self::Va => "VA",
            Self::Vd => "VD",
            Self::Vi => "VI",
            Self::Vm => "VM",
            Self::Vr => "VR",
            Self::Pi => "PI",
            Self::Ha => "HA",
            Self::Hf => "HF",
            Self::Hm => "HM",
        }
    }

    /// Whether the leg ends where it meets the next leg, so a leg must
    /// follow it.
    pub const fn intercepts_next(self) -> bool {
        matches!(self, Self::Ci | Self::Vi)
    }
}

/// An attribute of a leg, by its one-letter key, in the order a link
/// writes them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Key {
    /// `o`, fly over the fix.
    O,
    /// `m`, the first leg of the missed approach.
    M,
    /// `r`, the roles of the fix.
    R,
    /// `t`, the turn direction.
    T,
    /// `a`, the altitude constraint.
    A,
    /// `g`, the second altitude.
    G,
    /// `v`, the vertical angle.
    V,
    /// `x`, the threshold crossing height.
    X,
    /// `s`, the speed constraint.
    S,
    /// `c`, the course.
    C,
    /// `h`, the heading.
    H,
    /// `d`, the distance.
    D,
    /// `b`, the bearing from the navaid (theta).
    B,
    /// `n`, the navaid.
    N,
    /// `z`, the arc centre.
    Z,
    /// `l`, the hold leg length.
    L,
    /// `p`, the required navigation performance.
    P,
}

impl Key {
    /// Every key, in the order a link writes them.
    pub const ALL: [Self; 17] = [
        Self::O,
        Self::M,
        Self::R,
        Self::T,
        Self::A,
        Self::G,
        Self::V,
        Self::X,
        Self::S,
        Self::C,
        Self::H,
        Self::D,
        Self::B,
        Self::N,
        Self::Z,
        Self::L,
        Self::P,
    ];

    /// The key's letter.
    pub const fn letter(self) -> char {
        match self {
            Self::O => 'o',
            Self::M => 'm',
            Self::R => 'r',
            Self::T => 't',
            Self::A => 'a',
            Self::G => 'g',
            Self::V => 'v',
            Self::X => 'x',
            Self::S => 's',
            Self::C => 'c',
            Self::H => 'h',
            Self::D => 'd',
            Self::B => 'b',
            Self::N => 'n',
            Self::Z => 'z',
            Self::L => 'l',
            Self::P => 'p',
        }
    }

    /// What the key gives, for a refusal that asks for it.
    pub const fn meaning(self) -> &'static str {
        match self {
            Self::O => "fly-over",
            Self::M => "first leg of the missed approach",
            Self::R => "roles of the fix",
            Self::T => "turn direction",
            Self::A => "altitude",
            Self::G => "second altitude",
            Self::V => "vertical angle",
            Self::X => "threshold crossing height",
            Self::S => "speed",
            Self::C => "course",
            Self::H => "heading",
            Self::D => "distance",
            Self::B => "bearing from the navaid",
            Self::N => "navaid",
            Self::Z => "arc centre",
            Self::L => "hold leg length",
            Self::P => "required navigation performance",
        }
    }
}

/// The role of a fix in a procedure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Role {
    /// Initial approach fix, `IAF`.
    Iaf,
    /// Intermediate fix, `IF`.
    If,
    /// Final approach course fix, `FACF`.
    Facf,
    /// Final approach fix, `FAF`.
    Faf,
    /// Step-down fix, `SDF`.
    Sdf,
    /// Missed approach point, `MAP`.
    Map,
    /// Missed approach holding fix, `MAHF`.
    Mahf,
    /// Final end point, `FEP`.
    Fep,
    /// Runway threshold, `RWY`.
    Rwy,
}

impl Role {
    /// Every role.
    pub const ALL: [Self; 9] = [
        Self::Iaf,
        Self::If,
        Self::Facf,
        Self::Faf,
        Self::Sdf,
        Self::Map,
        Self::Mahf,
        Self::Fep,
        Self::Rwy,
    ];

    /// The role's word in a link.
    pub const fn word(self) -> &'static str {
        match self {
            Self::Iaf => "IAF",
            Self::If => "IF",
            Self::Facf => "FACF",
            Self::Faf => "FAF",
            Self::Sdf => "SDF",
            Self::Map => "MAP",
            Self::Mahf => "MAHF",
            Self::Fep => "FEP",
            Self::Rwy => "RWY",
        }
    }
}

/// The direction of a turn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TurnDirection {
    /// Left, `L`.
    Left,
    /// Right, `R`.
    Right,
}

impl TurnDirection {
    /// The direction's letter in a link.
    pub const fn letter(self) -> char {
        match self {
            Self::Left => 'L',
            Self::Right => 'R',
        }
    }
}

/// A named or unnamed position on the WGS84 ellipsoid.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Fix {
    /// Name, 1 to 32 of `[A-Za-z0-9_-]`.
    pub name: Option<String>,
    /// Latitude, degrees north, -90 to 90.
    pub lat: Micro,
    /// Longitude, degrees east, -180 to 180.
    pub lon: Micro,
}

impl Fix {
    /// Latitude, degrees north.
    pub fn lat_deg(&self) -> f64 {
        self.lat.as_f64()
    }

    /// Longitude, degrees east.
    pub fn lon_deg(&self) -> f64 {
        self.lon.as_f64()
    }
}

/// One leg as the link states it. Which attributes a leg must and may
/// carry follows from its path terminator; see [`shape`].
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Leg {
    /// The path terminator.
    pub path: PathTerminator,
    /// The fix the leg ends at, or starts from for `FA`, `FC`, `FD`, `FM`
    /// and `PI`; absent for the legs that have none.
    pub fix: Option<Fix>,
    /// `o`: cross the fix before turning.
    pub fly_over: bool,
    /// `m`: the first leg of the missed approach.
    pub missed_approach: bool,
    /// `r`: the roles of the fix, in the order written.
    pub roles: Vec<Role>,
    /// `t`: the turn direction.
    pub turn: Option<TurnDirection>,
    /// `a`: the altitude constraint at the fix, or the altitude that ends
    /// an `FA`, `CA`, `VA` or `HA` leg.
    pub altitude: Option<Limit<Height>>,
    /// `g`: the second altitude of an approach fix.
    pub second_altitude: Option<SecondAltitude>,
    /// `v`: the vertical angle, degrees, negative descending.
    pub vertical_angle: Option<Milli>,
    /// `x`: the threshold crossing height.
    pub crossing_height: Option<Height>,
    /// `s`: the speed constraint.
    pub speed: Option<Limit<Speed>>,
    /// `c`: the course: flown, from the fix, outbound for `PI`, inbound for
    /// a hold.
    pub course: Option<Bearing>,
    /// `h`: the heading of a `V` leg.
    pub heading: Option<Bearing>,
    /// `d`: along the course from the fix for `FC`; the excursion limit for
    /// `PI`; otherwise the DME distance from the navaid (rho), the arc
    /// radius for `AF`.
    pub distance: Option<Distance>,
    /// `b`: the bearing from the navaid (theta): the radial that ends a
    /// `CR` or `VR` leg, the radial an `AF` arc begins on.
    pub theta: Option<Bearing>,
    /// `n`: the recommended navaid `b` and `d` are measured from.
    pub navaid: Option<Fix>,
    /// `z`: the centre of an `RF` arc.
    pub centre: Option<Fix>,
    /// `l`: the inbound leg length of a hold.
    pub hold_length: Option<HoldLength>,
    /// `p`: the required navigation performance.
    pub rnp: Option<Distance>,
}

impl Leg {
    /// A leg with no attributes.
    pub const fn new(path: PathTerminator, fix: Option<Fix>) -> Self {
        Self {
            path,
            fix,
            fly_over: false,
            missed_approach: false,
            roles: Vec::new(),
            turn: None,
            altitude: None,
            second_altitude: None,
            vertical_angle: None,
            crossing_height: None,
            speed: None,
            course: None,
            heading: None,
            distance: None,
            theta: None,
            navaid: None,
            centre: None,
            hold_length: None,
            rnp: None,
        }
    }

    /// The keys of the attributes the leg carries, in link order.
    pub fn keys(&self) -> Vec<Key> {
        let present = [
            self.fly_over,
            self.missed_approach,
            !self.roles.is_empty(),
            self.turn.is_some(),
            self.altitude.is_some(),
            self.second_altitude.is_some(),
            self.vertical_angle.is_some(),
            self.crossing_height.is_some(),
            self.speed.is_some(),
            self.course.is_some(),
            self.heading.is_some(),
            self.distance.is_some(),
            self.theta.is_some(),
            self.navaid.is_some(),
            self.centre.is_some(),
            self.hold_length.is_some(),
            self.rnp.is_some(),
        ];
        Key::ALL
            .into_iter()
            .zip(present)
            .filter_map(|(key, present)| present.then_some(key))
            .collect()
    }
}
