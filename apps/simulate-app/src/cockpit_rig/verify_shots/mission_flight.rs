//! The X500 on its Pilotage mission, seen from the chase camera on the pad, in the climb
//! and on the route: the operator's view of the chain that nobody flies by hand.

use std::path::PathBuf;

use sim_core::ControlInputs;
use sim_geo::ecef_to_geodetic;
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use sim_scenery::AerodromeId;

use super::VerifyHarness;
use super::vehicle_views::draw_camera;
use crate::error::AppSimError;
use crate::esc_menu::SpawnMode;
use crate::vehicles::FreeFlightVehicle;

/// Flight seconds before each shot, and the shot's name.
const SHOTS: [(f64, &str); 3] = [(2.0, "pad"), (24.0, "climb"), (60.0, "route")];

#[test]
#[ignore = "writes 1280x720 chase shots of the X500 mission flight into $SIMULATE_VERIFY_OUT"]
fn render_x500_mission_flight() -> Result<(), AppSimError> {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return Ok(());
    };
    std::fs::create_dir_all(&dir).map_err(|e| AppSimError::Screenshot {
        detail: e.to_string(),
    })?;
    let Some(mut h) = VerifyHarness::new()? else {
        return Ok(());
    };
    h.rig.set_show_instruments(false);
    h.rig.set_show_conformal_hud(false);
    h.rig
        .supervisor_mut()
        .set_free_aerodrome(AerodromeId::NewarkKewr);
    h.flight.select_vehicle(FreeFlightVehicle::X500Mission)?;
    if let Some(diagram) = h.rig.supervisor().diagram(AerodromeId::NewarkKewr).cloned() {
        h.flight
            .spawn_at_aerodrome(&diagram, 0, SpawnMode::Threshold)?;
    }
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    let pad_height_m = ecef_to_geodetic(h.flight.vehicle_state().position_ecef).height_m;
    let mut heights_m = Vec::new();
    for (seconds, name) in SHOTS {
        for _ in 0..(seconds / 0.05) as usize {
            h.flight
                .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
        }
        let state = h.flight.vehicle_state();
        heights_m.push(ecef_to_geodetic(state.position_ecef).height_m - pad_height_m);
        let chase = h.flight.display_camera(OperatorViewportMode::Chase)?;
        draw_camera(
            &mut h,
            |_| Some(chase),
            &dir.join(format!("x500_mission_{name}.png")),
        )?;
    }
    assert!(
        matches!(heights_m.as_slice(), [pad, climb, route] if pad.abs() < 0.5 && *climb > 20.0 && *route > 50.0),
        "heights above the pad at the shots: {heights_m:?}"
    );
    h.gpu.check_device()?;
    Ok(())
}
