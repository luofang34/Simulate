//! The C172 on the real control chain, seen from the chase camera while Pilotage arms it,
//! after its first turn onto the route and further along: the operator's view of the
//! chain nobody flies by hand.

use std::path::PathBuf;

use sim_core::ControlInputs;
use sim_scenario::{OperatorViewportMode, SimTimeScale};

use super::VerifyHarness;
use super::vehicle_views::draw_camera;
use crate::error::AppSimError;
use crate::vehicles::FreeFlightVehicle;

/// Flight seconds before each shot, and the shot's name.
const SHOTS: [(f64, &str); 3] = [(4.0, "armed"), (40.0, "first_leg"), (90.0, "route")];

#[test]
#[ignore = "writes 1280x720 chase shots of the C172 mission flight into $SIMULATE_VERIFY_OUT"]
fn render_c172_mission_flight() -> Result<(), AppSimError> {
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
    h.flight.select_vehicle(FreeFlightVehicle::C172Mission)?;
    h.rig
        .supervisor_mut()
        .set_viewport_mode(OperatorViewportMode::Chase);
    let start_s = h.flight.now().as_seconds();
    let mut phases = Vec::new();
    for (seconds, name) in SHOTS {
        while h.flight.now().as_seconds() - start_s < seconds {
            h.flight
                .advance_by(0.05, ControlInputs::default(), SimTimeScale::RealTime)?;
        }
        let readout = h.flight.chain_readout();
        tracing::info!(name, ?readout, "C172 mission shot");
        phases.push(readout.and_then(|r| r.phase));
        let chase = h.flight.display_camera(OperatorViewportMode::Chase)?;
        draw_camera(
            &mut h,
            |_| Some(chase),
            &dir.join(format!("c172_mission_{name}.png")),
        )?;
    }
    assert_eq!(h.flight.aviate_armed(), Some(true));
    assert!(
        phases
            .iter()
            .skip(1)
            .all(|p| p.as_deref() == Some("Enroute")),
        "phases at the shots: {phases:?}"
    );
    h.gpu.check_device()?;
    Ok(())
}
