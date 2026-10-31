use sim_apollo::{CockpitInstrumentSnapshot, GuidanceSource, InstrumentChange};
use sim_core::SourceValidity;
use sim_scenario::ScenarioProfile;
use tracing::info;

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

/// A short reading of the panel for the window title.
pub fn title(profile: Option<ScenarioProfile>, cockpit: &CockpitInstrumentSnapshot) -> String {
    let name = profile.map_or("Free Flight", ScenarioProfile::title);
    let mut line = format!(
        "Simulate | {name} | {:.0} m | {:.0} m/s",
        cockpit.height_m.value, cockpit.speed_m_s.value
    );
    let mut add = |text: String| {
        line.push_str(" | ");
        line.push_str(&text);
    };
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
    line
}
