//! The link between a page's own map canvas and the app's EFB map, which draws on it:
//! the page hands over the operator's commands, the flight plan with the mission host's
//! progress along it, and the tracks it lays over the plan; the app hands back what the
//! map shows, where the plan's fixes fall on the canvas, and the fix picked.
//!
//! Page and app run on one thread in the browser and neither waits on the other: each
//! side reads what the other last wrote when it next draws.

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use sim_maplibre::{EfbTracks, FlightPlan, LegState, RouteProgress};

use crate::map_rig::{EfbCommand, EfbStatus};

#[cfg(any(target_arch = "wasm32", test))]
pub mod gesture;
#[cfg(test)]
mod tests;

/// A fix's label on the canvas.
#[derive(Debug, Clone, PartialEq)]
pub struct FixLabel {
    /// Position in fly order.
    pub index: usize,
    /// Identifier.
    pub ident: String,
    /// Where the fix is, the canvas's drawing-buffer pixels from its top left.
    pub at_px: [f32; 2],
    /// Its place in the flight.
    pub state: LegState,
}

/// The fix the operator picked.
#[derive(Debug, Clone, PartialEq)]
pub struct PickedFix {
    /// Position in fly order.
    pub index: usize,
    /// Identifier.
    pub ident: String,
    /// `[lat, lon]`, degrees.
    pub at_deg: [f64; 2],
    /// Height above the WGS84 ellipsoid to cross at, metres, when the plan states one.
    pub altitude_m: Option<f64>,
    /// Its place in the flight.
    pub state: LegState,
}

/// What the app's map shows on the canvas.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PageMapView {
    /// The map has drawn on the canvas; until it does the page shows its own map.
    pub live: bool,
    /// The map's view, buttons and modes.
    pub status: Option<EfbStatus>,
    /// The plan's fixes on the canvas.
    pub labels: Vec<FixLabel>,
    /// The fix picked, if any.
    pub picked: Option<PickedFix>,
}

#[derive(Debug, Default)]
struct Shared {
    commands: Vec<EfbCommand>,
    route: Option<(FlightPlan, RouteProgress)>,
    route_new: bool,
    tracks: Option<Arc<EfbTracks>>,
    view: PageMapView,
}

/// Both ends of the link; clones share it.
#[derive(Debug, Clone, Default)]
pub struct PageMapLink {
    shared: Arc<Mutex<Shared>>,
}

impl PageMapLink {
    fn lock(&self) -> MutexGuard<'_, Shared> {
        self.shared.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Page: queues an operator command for the map.
    #[cfg(any(target_arch = "wasm32", test))]
    pub fn command(&self, command: EfbCommand) {
        self.lock().commands.push(command);
    }

    /// Page: hands over the plan at `progress` when either changed since the last one.
    #[cfg(any(target_arch = "wasm32", test))]
    pub fn publish_route(&self, plan: &FlightPlan, progress: RouteProgress) {
        let mut shared = self.lock();
        if shared
            .route
            .as_ref()
            .is_none_or(|(p, r)| p != plan || *r != progress)
        {
            shared.route = Some((plan.clone(), progress));
            shared.route_new = true;
        }
    }

    /// Page: hands over the tracks laid over the plan.
    #[cfg(any(target_arch = "wasm32", test))]
    pub fn publish_tracks(&self, tracks: Arc<EfbTracks>) {
        self.lock().tracks = Some(tracks);
    }

    /// Page: what the map shows now.
    #[cfg(any(target_arch = "wasm32", test))]
    #[must_use]
    pub fn view(&self) -> PageMapView {
        self.lock().view.clone()
    }

    /// App: everything handed over since the last call, oldest command first.
    pub(crate) fn take(&self) -> PageMapInput {
        let mut shared = self.lock();
        PageMapInput {
            commands: std::mem::take(&mut shared.commands),
            route: std::mem::take(&mut shared.route_new)
                .then(|| shared.route.clone())
                .flatten(),
            tracks: shared.tracks.take(),
        }
    }

    /// App: reports what the map shows.
    pub(crate) fn show(&self, view: PageMapView) {
        self.lock().view = view;
    }
}

/// What the page handed over since the app last took it.
#[derive(Debug, Default)]
pub(crate) struct PageMapInput {
    /// Operator commands, oldest first.
    pub commands: Vec<EfbCommand>,
    /// A new plan or progress.
    pub route: Option<(FlightPlan, RouteProgress)>,
    /// New tracks.
    pub tracks: Option<Arc<EfbTracks>>,
}
