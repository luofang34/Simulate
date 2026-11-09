//! Lockheed P-38G-13-LO Lightning vehicle crate (`vehicles/p38g`). Individual
//! airframes (serials, unit markings) are liveries in `Liveries/catalog.json`.
//!
//! Designed so each aircraft model is decoupled from rendering/map adapters (`sim-maplibre`)
//! and can live as an independent repository:
//! - [`stations`]: Lockheed engineering 3-view reference envelope and station ordinate tables
//!   (`FS 0..248` central gondola, `BS 0..428` twin booms, `WS 0..312` cambered `NACA 23016 -> 4412`
//!   wing, and `WL -14..+62` vertical fins),
//! - [`cockpit_reservation`]: Reserved 3D cockpit station contract (`FS 86` instrument panel mount,
//!   optical reflector gunsight mount, spectacles control column socket, left engine/feathering
//!   quadrant, right flap/cooling console, `FS 154` pilot armor bulkhead, and `FS 182` radio bay),
//! - [`powerplant`]: Allison V-1710-51/-55 `60°` V-12 external reduction gearbox, 24 individual
//!   exhaust ejector stacks, recessed dorsal GE B-13 turbosuperchargers, wing leading-edge
//!   intercooler louvers, chin oil cooler, and aft-boom glycol radiator ducts with splitter plates,
//! - [`package`]: Embedded `syspack.json` source, OpenUSD `.usda` stage export, and 3-view
//!   orthographic blueprint verification.

mod airframe;
mod articulated;
mod cockpit_reservation;
mod decals;
mod mesh;
mod package;
mod powerplant;
mod role;
mod stations;
#[cfg(test)]
mod tests;

pub use cockpit_reservation::{
    CockpitInteriorContract, PanelMountFrame, StationBulkheadSpec, p38g_cockpit_contract,
};
pub use mesh::{P38Material, P38MeshPrim, P38MeshVertex, P38PartRole};
pub use package::{
    P38BlueprintCalibrationReport, P38UsdError, p38g_mesh_prims, p38g_package_source, p38g_usda,
    verify_p38g_three_view_and_stations,
};
pub use stations::{
    AILERON_END_X, BOOM_STATIONS, BOOM_X, ELEVATOR_HINGE_Y, FIN_PROFILE, GONDOLA_STATIONS, HTAIL_Z,
    OVERALL_HEIGHT_M, OVERALL_LENGTH_M, OVERALL_SPAN_M, PROP_HUB_Y, PROP_HUB_Z, PROP_RADIUS_M,
    ROOT_CHORD_M, RUDDER_HINGE_Y, SEMI_SPAN, TIP_CHORD_M, WING_AREA_M2, WING_DIHEDRAL_DEG,
    boom_point, fin_point, gondola_point, interpolate_boom, interpolate_gondola,
    three_view_boom_outline_x, three_view_gondola_outline_x, three_view_wing_leading_y,
    three_view_wing_trailing_y, wing_station, wing_surface_point,
};
