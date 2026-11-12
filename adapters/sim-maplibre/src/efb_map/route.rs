//! The flight plan as the EFB map draws it: its fixes, where the mission host says the
//! flight is along them, and the state of each leg and fix in the usual avionics colours
//! (the active leg magenta, legs to come white, legs flown dim).
//!
//! The map never decides where the flight is on its plan. [`RouteProgress`] is what the
//! mission host reports of its own sequencing, and every state here follows from it.

use serde_json::{Value, json};

use super::hold::HoldPattern;

/// A fix's identity in its plan, stable while the plan is amended, so a pick or an edit
/// names the same fix after fixes before it are inserted or removed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FixId(pub u32);

/// One fix of the plan.
#[derive(Clone, Debug, PartialEq)]
pub struct RouteFix {
    /// Identity in the plan.
    pub id: FixId,
    /// Identifier shown beside the fix.
    pub ident: String,
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid to cross at, metres, when the plan states one.
    pub altitude_m: Option<f64>,
}

/// The fixes in fly order, and where the flight starts from.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlightPlan {
    /// `[lat, lon]` of the flight's start, degrees: the first leg runs from here to the
    /// first fix. Without one the first fix has no leg into it.
    pub origin: Option<[f64; 2]>,
    /// Fixes in fly order.
    pub fixes: Vec<RouteFix>,
}

/// Where the mission host says the flight is along its plan.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RouteProgress {
    /// Index of the fix the plan flies toward; `None` before the host flies the plan.
    pub to_fix: Option<usize>,
    /// The plan's last fix is sequenced: every leg is flown.
    pub complete: bool,
    /// The holding pattern the host flies, once it holds.
    pub hold: Option<HoldPattern>,
}

impl RouteProgress {
    /// The progress after the mission host's latest sequencing: the fix it last
    /// sequenced (index in fly order) and whether that completed the plan. With nothing
    /// sequenced yet the first leg is active, as the host flies toward its first fix.
    #[must_use]
    pub const fn after_sequenced(last: Option<(usize, bool)>) -> Self {
        match last {
            None => Self {
                to_fix: Some(0),
                complete: false,
                hold: None,
            },
            Some((index, complete)) => Self {
                to_fix: Some(index.saturating_add(1)),
                complete,
                hold: None,
            },
        }
    }
}

/// A leg's or a fix's place in the flight.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegState {
    /// Flown: the leg is behind the flight, the fix sequenced.
    Flown,
    /// The leg flown now, or the fix flown toward.
    Active,
    /// Still to come.
    Future,
}

impl LegState {
    /// The name the map's style matches on.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Flown => "flown",
            Self::Active => "active",
            Self::Future => "future",
        }
    }
}

/// The state of the leg into fix `index`, which is also the state of that fix.
#[must_use]
pub const fn leg_state(index: usize, progress: RouteProgress) -> LegState {
    if progress.complete {
        return LegState::Flown;
    }
    match progress.to_fix {
        None => LegState::Future,
        Some(to) if index < to => LegState::Flown,
        Some(to) if index == to => LegState::Active,
        Some(_) => LegState::Future,
    }
}

/// One leg, from its start to fix `to`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RouteLeg {
    /// Index of the fix the leg flies to.
    pub to: usize,
    /// `[lat, lon]` of the leg's start, degrees.
    pub from_deg: [f64; 2],
    /// `[lat, lon]` of the leg's end, degrees.
    pub to_deg: [f64; 2],
    /// Its state.
    pub state: LegState,
}

impl FlightPlan {
    /// Every leg in fly order with its state under `progress`: the leg into each fix,
    /// the first from the origin when there is one.
    #[must_use]
    pub fn legs(&self, progress: RouteProgress) -> Vec<RouteLeg> {
        let starts = std::iter::once(self.origin).chain(
            self.fixes
                .iter()
                .map(|fix| Some([fix.lat_deg, fix.lon_deg])),
        );
        starts
            .zip(self.fixes.iter().enumerate())
            .filter_map(|(from, (to, fix))| {
                Some(RouteLeg {
                    to,
                    from_deg: from?,
                    to_deg: [fix.lat_deg, fix.lon_deg],
                    state: leg_state(to, progress),
                })
            })
            .collect()
    }

    /// The position in fly order of the fix `id`.
    #[must_use]
    pub fn index_of(&self, id: FixId) -> Option<usize> {
        self.fixes.iter().position(|fix| fix.id == id)
    }

    /// The plan's legs, its holding pattern while the host holds, and its fixes as a
    /// GeoJSON document, each feature carrying its `kind` and `state` (see
    /// [`LegState::name`]; a hold flown is active), fix features their `ident` and
    /// `index`.
    #[must_use]
    pub fn geojson(&self, progress: RouteProgress) -> Value {
        let legs = self.legs(progress).into_iter().map(|leg| {
            json!({
                "type": "Feature",
                "properties": { "kind": "leg", "state": leg.state.name(), "index": leg.to },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [lon_lat(leg.from_deg), lon_lat(leg.to_deg)],
                },
            })
        });
        let fixes = self.fixes.iter().enumerate().map(|(index, fix)| {
            json!({
                "type": "Feature",
                "properties": {
                    "kind": "fix",
                    "state": leg_state(index, progress).name(),
                    "ident": fix.ident,
                    "index": index,
                },
                "geometry": { "type": "Point", "coordinates": [fix.lon_deg, fix.lat_deg] },
            })
        });
        let hold = progress.hold.map(|hold| {
            let coordinates: Vec<[f64; 2]> = hold.racetrack().into_iter().map(lon_lat).collect();
            json!({
                "type": "Feature",
                "properties": { "kind": "hold", "state": LegState::Active.name() },
                "geometry": { "type": "LineString", "coordinates": coordinates },
            })
        });
        let features: Vec<Value> = legs.chain(hold).chain(fixes).collect();
        json!({ "type": "FeatureCollection", "features": features })
    }
}

/// `[lon, lat]`, GeoJSON's order, of `[lat, lon]`.
pub(super) const fn lon_lat([lat, lon]: [f64; 2]) -> [f64; 2] {
    [lon, lat]
}
