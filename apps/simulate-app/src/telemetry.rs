use sim_apollo::{CockpitInstrumentSnapshot, GuidanceSource, InstrumentChange};
use sim_core::SourceValidity;
use sim_scenario::{OperatorViewportMode, SimTimeScale};
use tracing::{debug, info};

use crate::flight::{Flight, SpaceLinkStatus, SpaceReferenceFrame};

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
    virtual_camera_title: Option<&str>,
    aerodrome_code: Option<&str>,
    time_scale: SimTimeScale,
    cockpit: &CockpitInstrumentSnapshot,
) -> String {
    let name = match flight.profile() {
        Some(profile) => profile.title(),
        None => match (flight.ascent_guidance(), flight.coasting_in_orbit()) {
            (Some(true), _) => "Raketa (Ascent Guidance)",
            (Some(false), Some(true)) => "Raketa (Coasting)",
            (Some(false), _) => "Raketa (Manual)",
            (None, _) => flight.free_vehicle().map_or("Free Flight", |v| v.title()),
        },
    };
    let view_label = match viewport_mode {
        OperatorViewportMode::Cockpit => "OTW",
        OperatorViewportMode::Chase => "Chase",
        OperatorViewportMode::SensorAperture => virtual_camera_title.unwrap_or("Sensor"),
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
    if let Some(code) = aerodrome_code {
        add(code.to_owned());
    }
    if let Some(leg) = flight.pattern_leg() {
        add(leg.title().to_owned());
    }
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
    if flight.world().body() == sim_geo::BodyEllipsoid::WGS84 {
        let nav_mode = match flight.sensor_realism() {
            crate::sensor_bus::SensorRealism::Realistic => "WAAS [N]",
            crate::sensor_bus::SensorRealism::DegradedNavigation => "DEGRADED [N]",
            crate::sensor_bus::SensorRealism::Ideal => "IDEAL [N]",
        };
        add(nav_mode.to_owned());
    }
    append_space_status(flight, viewport_mode, time_scale, &mut add);
    append_apollo_status(cockpit, &mut add);
    add(pacing_label(flight));
    line
}

/// Rolling real-time pacing, e.g. `RT 1.00x · 60 FPS`.
pub fn pacing_label(flight: &Flight) -> String {
    let stats = flight.pacing_stats();
    debug!(
        fps = stats.fps,
        mean_frame_ms = stats.mean_frame_ms,
        max_frame_ms = stats.max_frame_ms,
        real_time_ratio = stats.real_time_ratio,
        "cockpit pacing"
    );
    format!("RT {:.2}x · {:.0} FPS", stats.real_time_ratio, stats.fps)
}

/// Logs the session's pacing when the app exits.
pub fn log_pacing_summary(flight: &Flight) {
    let session = flight.pacing_session();
    let rolling = flight.pacing_stats();
    info!(
        frames = session.frames,
        wall_s = session.wall_s,
        sim_s = session.sim_s,
        session_real_time_ratio = session.real_time_ratio(),
        last_real_time_ratio = rolling.real_time_ratio,
        last_fps = rolling.fps,
        last_max_frame_ms = rolling.max_frame_ms,
        "real-time pacing summary"
    );
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

fn append_space_status(
    flight: &Flight,
    viewport_mode: OperatorViewportMode,
    time_scale: SimTimeScale,
    add: &mut impl FnMut(String),
) {
    if let (Some(frame), Some((pos, vel))) = (
        flight.space_reference_frame(),
        flight.space_state_in_reference_frame(),
    ) {
        let frame_label = match frame {
            SpaceReferenceFrame::BodyInertial
            | SpaceReferenceFrame::BodyFixed
            | SpaceReferenceFrame::SunEarthSynodic => frame.label(),
        };
        add(format!(
            "{frame_label} {:.0} km {:.0} m/s",
            pos.length() * 1.0e-3,
            vel.length()
        ));
    }
    if let Some(fdai) = flight.space_fdai_navball() {
        let prograde_deg = fdai
            .prograde
            .dir_body
            .y
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        add(format!("FDAI {prograde_deg:.0}°"));
    }
    if let Some(plume) = flight.space_aerospike_plume()
        && plume.visible
    {
        add(format!("Plume {:.1}x", plume.expansion_ratio));
    }
    if let SimTimeScale::Warp(w) = time_scale
        && let Some(warp) = flight.space_warp_status(w, sim_core::ControlInputs::default())
        && warp.achieved_warp < warp.requested_warp
    {
        add(format!(
            "Warp {:.0}x/{:.0}x ({:?})",
            warp.achieved_warp, warp.requested_warp, warp.bottleneck_reason
        ));
    }
    if let Some(dsn) = flight.space_dsn_link_status() {
        append_dsn_link(&dsn, add);
    }
    if matches!(
        viewport_mode,
        OperatorViewportMode::ChartMap | OperatorViewportMode::VolumetricScene
    ) && let Some(nodes) = flight.space_orrery_projection(
        glam::DVec3::new(0.0, 0.0, 2.0e9),
        glam::DQuat::IDENTITY,
        1.0,
        1.5,
    ) {
        let visible = nodes.iter().filter(|n| n.depth_m > 0.0).count();
        add(format!("Orrery {visible}/{}", nodes.len()));
    }
}

fn append_dsn_link(dsn: &SpaceLinkStatus, add: &mut impl FnMut(String)) {
    let el_deg = dsn.elevation_at_station_rad.to_degrees();
    let range_mm = dsn.slant_range_m * 1.0e-6;
    if dsn.accessible {
        add(format!(
            "DSN {:.2}s {range_mm:.0}Mm {el_deg:.0}°",
            dsn.light_time_s
        ));
    } else if let Some(body) = dsn.occulted_by {
        add(format!("DSN OCC ({})", body.0));
    } else {
        add(format!("DSN MASK ({el_deg:.0}°)"));
    }
}
