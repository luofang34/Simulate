//! Lockheed P-38G-13-LO Lightning vehicle crate (`vehicles/p38g`). Individual
//! airframes (serials, unit markings) are liveries in `Liveries/catalog.json`.
//!
//! Designed so each aircraft model is decoupled from rendering/map adapters (`sim-maplibre`)
//! and can live as an independent repository:
//! - [`stations`]: Lockheed engineering 3-view reference envelope and station ordinate tables
//!   (`BS 0..428` twin booms, `WS 0..312` cambered `NACA 23016 -> 4412` wing, and
//!   `WL -14..+62` vertical fins); [`GONDOLA_SECTIONS`]: the central gondola's mould line and
//!   canopy, section by section,
//! - [`cockpit_reservation`]: Reserved 3D cockpit station contract (instrument panel frame,
//!   design eyepoint and seat reference point, gunsight mount, control column, side consoles,
//!   seat-back armour and floor levels),
//! - [`powerplant`]: Allison V-1710-51/-55 `60°` V-12 external reduction gearbox, exhaust
//!   collectors to the dorsal GE B-13 turbosuperchargers, wing leading-edge
//!   intercooler louvers and chin oil cooler; [`radiators`]: the glycol radiator scoops on the
//!   tail booms and their exit flaps,
//! - [`package`]: Embedded `syspack.json` source and OpenUSD `.usda` stage export,
//! - [`drawing`]: the digitised AN 01-75F-2 general arrangement and its station callouts,
//!   against which [`verify_p38g_three_view_and_stations`] measures the built mesh.

mod airframe;
mod articulated;
mod bays;
mod calibration;
mod canopy;
mod canopy_frames;
mod cockpit_controls;
mod cockpit_fittings;
mod cockpit_interior;
mod cockpit_opening;
mod cockpit_placards;
pub mod cockpit_reservation;
pub mod drawing;
mod fillets;
mod gear;
mod gondola;
mod instrument_panel;
mod mesh;
mod package;
mod powerplant;
mod radiators;
mod role;
mod skin;
mod stations;
#[cfg(test)]
mod tests;

pub use calibration::{P38BlueprintCalibrationReport, verify_p38g_three_view_and_stations};
pub use cockpit_controls::p38g_uv_flood_lamp_m;
pub use cockpit_placards::{P38Placard, p38g_placards};
pub use cockpit_reservation::{
    CockpitInteriorContract, PanelMountFrame, StationBulkheadSpec, p38g_cockpit_contract,
};
pub use gondola::{
    GONDOLA_SECTIONS, GondolaSection, canopy_point, canopy_sill_z, gondola_half_width_at,
    gondola_point, gondola_section, interpolate_gondola, opening_edge_theta, section_y,
    three_view_gondola_outline_x,
};
pub use instrument_panel::p38g_instrument_pack;
pub use mesh::{P38ControlJoint, P38Material, P38MeshPrim, P38MeshVertex, P38PartRole};
pub use package::{P38UsdError, p38g_mesh_prims, p38g_package_source, p38g_usda};
pub use skin::{MarkingKind, MarkingSite, P38Skin, P38SkinError, SkinPaint, SkinQuery};
pub use stations::{
    AILERON_END_X, BOOM_STATIONS, BOOM_X, ELEVATOR_HINGE_Y, FIN_PROFILE, HTAIL_Z, OVERALL_HEIGHT_M,
    OVERALL_LENGTH_M, OVERALL_SPAN_M, PROP_BLADE_OFFSET_M, PROP_HUB_Y, PROP_HUB_Z, PROP_RADIUS_M,
    ROOT_CHORD_M, RUDDER_HINGE_Y, SEMI_SPAN, TIP_CHORD_M, TIP_START_X, TURBO_NOZZLE_Y,
    WING_AREA_M2, WING_DIHEDRAL_DEG, boom_point, fin_point, interpolate_boom,
    three_view_boom_outline_x, three_view_wing_leading_y, three_view_wing_trailing_y, wing_station,
    wing_surface_point,
};
