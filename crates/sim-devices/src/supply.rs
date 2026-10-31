use mbd_core::PowerRailState;

/// Whether a rail can run a part needing `minimum_v`.
pub(crate) fn rail_up(rail: &PowerRailState, minimum_v: f64) -> bool {
    rail.enabled && rail.voltage_v >= minimum_v
}
