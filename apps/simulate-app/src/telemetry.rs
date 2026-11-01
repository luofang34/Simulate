use sim_apollo::{CockpitInstrumentSnapshot, GuidanceSource, InstrumentChange};
use sim_core::SourceValidity;
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use tracing::info;

use crate::flight::Flight;

/// Reports what changed on the panel since `previous`.
pub fn log_changes(previous: &CockpitInstrumentSnapshot, current: &CockpitInstrumentSnapshot) {
    for change in current.changes_since(previous) {
        let height_m = current.height_m.value;
        match change {
            InstrumentChange::GuidanceSource(source) => {
                info!(?source, height_m, "guidance source changed");
            }
            InstrumentChange::Attitude(validity) => {
                info!(?validity, height_m, "attitude source health changed");
            }
            InstrumentChange::PgnsAttitude(validity) => {
                info!(?validity, height_m, "PGNS attitude health changed");
            }
            InstrumentChange::GimbalLockCaution(lit) => {
                info!(lit, height_m, "gimbal lock caution changed");
            }
            InstrumentChange::GimbalLockCaptured(captured) => {
                info!(captured, height_m, "gimbal lock capture changed");
            }
            InstrumentChange::Radar(validity) => {
                info!(?validity, height_m, "landing radar health changed");
            }
            InstrumentChange::ContactLight(lit) => {
                info!(lit, height_m, "contact light changed");
            }
        }
    }
}

/// Formats a concise cockpit status line for the desktop window or browser tab title.
pub fn title(
    flight: &Flight,
    viewport_mode: OperatorViewportMode,
    time_scale: SimTimeScale,
    cockpit: &CockpitInstrumentSnapshot,
) -> String {
    let name = match flight.profile() {
        Some(profile) => profile.title(),
        None => flight.free_vehicle().map_or("Free Flight", |v| v.title()),
    };
    let view_label = match viewport_mode {
        OperatorViewportMode::Cockpit => "OTW",
        OperatorViewportMode::Chase => "Chase",
        OperatorViewportMode::SensorAperture => "Sensor",
        OperatorViewportMode::ChartMap => "Chart",
        OperatorViewportMode::VolumetricScene => "Volume",
    };
    let mut line = format!(
        "Simulate | {name} | {view_label} | {:.0} m | {:.0} m/s",
        cockpit.height_m.value, cockpit.speed_m_s.value
    );
    let mut add = |text: String| {
        line.push_str(" | ");
        line.push_str(&text);
    };
    match time_scale {
        SimTimeScale::Paused => add("PAUSED".to_owned()),
        SimTimeScale::RealTime => {}
        SimTimeScale::Warp(w) => add(format!("{w:.0}x")),
    }
    if let Some(engine) = flight.engine_status() {
        if engine.running {
            add(format!("{:.0} RPM", engine.propeller_rpm));
        } else {
            add("ENG OFF".to_owned());
        }
    }
    if let Some(sys) = flight.system_controls()
        && sys.flaps_norm > 0.01
    {
        add(format!("Flaps {:.0}%", sys.flaps_norm * 100.0));
    }
    if let Some(armed) = flight.aviate_armed() {
        add(if armed { "FC ARMED" } else { "FC DISARMED" }.to_owned());
    }
    append_apollo_status(cockpit, &mut add);
    line
}

fn append_apollo_status(cockpit: &CockpitInstrumentSnapshot, add: &mut impl FnMut(String)) {
    match cockpit.guidance_source {
        Some(GuidanceSource::Pgns) => add("PGNS".to_owned()),
        Some(GuidanceSource::Ags) => add("AGS".to_owned()),
        None => {}
    }
    if cockpit.gimbal_lock_captured {
        add("GIMBAL LOCK".to_owned());
    } else if cockpit.gimbal_lock_caution {
        add("GIMBAL CAUTION".to_owned());
    }
    if cockpit.radar_slant_range_m.validity != SourceValidity::Missing {
        add(format!("radar {:.0} m", cockpit.radar_slant_range_m.value));
    }
    if let Some(elevation_deg) = cockpit.lpd_elevation_deg {
        add(format!("LPD {elevation_deg:.0}°"));
    }
    if let Some(g_load) = cockpit.ems_g_load {
        add(format!("{g_load:.1} g"));
    }
    if cockpit.contact_light {
        add("CONTACT LIGHT".to_owned());
    }
}
