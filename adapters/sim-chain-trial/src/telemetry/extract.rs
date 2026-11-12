//! Extraction of truth, sensor/spoofing inputs, camera diagnostics, and
//! flight-controller state from [`ChainTrial`].

use glam::{DMat3, DVec3};
use sim_core::VehicleState;
use sim_flight_chain::Airframe;
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, egm96_undulation_m};
use sim_navigate::trial::{Outcome, SensorKind};

use crate::trial::FLIGHT_CONTROLLER_SET;
use crate::{ChainTrial, ChainTrialError};

use super::types::{CameraFrameTelemetry, InputTelemetry, NavigatorTelemetry, TruthTelemetry};

pub(super) fn build_truth(
    trial: &ChainTrial,
    terrain_msl_fn: &impl Fn(f64, f64) -> Option<f64>,
) -> Result<TruthTelemetry, ChainTrialError> {
    let snap = trial.truth();
    let state = snap.state;
    let geo = ecef_to_geodetic(state.position_ecef);
    let (lat_deg, lon_deg) = (geo.lat_rad.to_degrees(), geo.lon_rad.to_degrees());
    let alt_msl_m = geo.height_m - egm96_undulation_m(geo.lat_rad, geo.lon_rad);
    let terrain_elev_m = terrain_msl_fn(lat_deg, lon_deg).unwrap_or(0.0);
    let frame = LocalFrame::at(&geo);
    let vel_ned = frame.ecef_to_ned_vector(state.velocity_ecef);
    let (roll_deg, pitch_deg, yaw_deg) = euler_deg_ned(&frame, state.attitude);
    let sensed = trial.chain().board().airframe().sensed()?;
    let wind_ned = sensed.air.wind_ned_m_s;
    let tas_mps = (vel_ned - wind_ned).length();
    Ok(TruthTelemetry {
        t_s: trial.now_s(),
        tick: snap.at.ticks_u128(),
        lat_deg,
        lon_deg,
        alt_msl_m,
        alt_agl_m: alt_msl_m - terrain_elev_m,
        terrain_elev_m,
        vn_mps: vel_ned.x,
        ve_mps: vel_ned.y,
        vd_mps: vel_ned.z,
        ground_speed_mps: vel_ned.x.hypot(vel_ned.y),
        true_track_deg: vel_ned.y.atan2(vel_ned.x).to_degrees().rem_euclid(360.0),
        roll_deg,
        pitch_deg,
        yaw_deg,
        p_rps: state.angular_rate_body.x,
        q_rps: state.angular_rate_body.y,
        r_rps: state.angular_rate_body.z,
        tas_mps,
        wind_ned_mps: wind_ned.to_array(),
    })
}

fn euler_deg_ned(frame: &LocalFrame, body_to_ecef: glam::DQuat) -> (f64, f64, f64) {
    let r_ned_body = frame.ned_to_ecef().transpose() * DMat3::from_quat(body_to_ecef);
    let fwd = r_ned_body.x_axis;
    let right = r_ned_body.y_axis;
    let down = r_ned_body.z_axis;
    let pitch_rad = (-fwd.z).clamp(-1.0, 1.0).asin();
    let roll_rad = right.z.atan2(down.z);
    let yaw_rad = fwd.y.atan2(fwd.x);
    (
        roll_rad.to_degrees(),
        pitch_rad.to_degrees(),
        yaw_rad.to_degrees().rem_euclid(360.0),
    )
}

pub(super) fn build_inputs(trial: &ChainTrial, truth: &TruthTelemetry) -> InputTelemetry {
    let t_s = truth.t_s;
    let latch = trial.sensor_latch();
    let receiver = trial.chain().board().suite().receiver();
    let gnss_out = trial.is_sensor_out(SensorKind::Gnss, t_s);
    let baro_out = trial.is_sensor_out(SensorKind::Baro, t_s);
    let cam_out = trial.is_sensor_out(SensorKind::TerrainCamera, t_s);
    let pitot_diff_pressure_pa = latch.air_data.and_then(|a| a.differential_pressure_pa);
    let mag_heading_deg = latch
        .mag_body_ut
        .map(|mag| tilt_compensated_mag_heading_deg(mag, truth.roll_deg, truth.pitch_deg));
    let spoof = trial.spoofer_state();
    let lla_msl = |g: Geodetic| {
        [
            g.lat_rad.to_degrees(),
            g.lon_rad.to_degrees(),
            g.height_m - egm96_undulation_m(g.lat_rad, g.lon_rad),
        ]
    };
    InputTelemetry {
        gnss_available: !gnss_out && receiver.is_tracking(),
        imu_ok: trial.chain().board().last_samples().imu.is_some(),
        baro_ok: !baro_out && latch.air_data.is_some(),
        mag_ok: latch.mag_body_ut.is_some(),
        pitot_ok: !baro_out && pitot_diff_pressure_pa.is_some(),
        camera_ok: trial.has_camera() && !cam_out,
        pitot_diff_pressure_pa,
        ias_mps: pitot_diff_pressure_pa.map(sim_devices::indicated_airspeed_m_s),
        baro_alt_msl_m: latch
            .air_data
            .map(|a| isa_pressure_altitude_m(a.static_pressure_pa)),
        mag_body_ut: latch.mag_body_ut.map(|v| v.to_array()),
        mag_heading_deg,
        spoof_active: spoof.active,
        spoof_kind: spoof.kind,
        spoof_offset_enu_m: spoof.offset_enu_m,
        spoof_rate_enu_mps: spoof.rate_enu_mps,
        spoof_horizontal_mag_m: spoof.horizontal_mag_m,
        gnss_measured_lla: latch.gnss_fix.map(|f| lla_msl(f.measured.position)),
        gnss_true_lla: latch.gnss_fix.map(|f| lla_msl(f.true_position)),
        camera_frame: camera_telemetry(trial),
    }
}

fn tilt_compensated_mag_heading_deg(mag_body: DVec3, roll_deg: f64, pitch_deg: f64) -> f64 {
    let (sr, cr) = roll_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let bx = mag_body.x * cp + mag_body.y * sr * sp + mag_body.z * cr * sp;
    let by = mag_body.y * cr - mag_body.z * sr;
    (-by).atan2(bx).to_degrees().rem_euclid(360.0)
}

fn isa_pressure_altitude_m(static_pressure_pa: f64) -> f64 {
    const P0_PA: f64 = 101_325.0;
    const P_TROP_PA: f64 = 22_632.06;
    let p = static_pressure_pa.max(1.0);
    if p >= P_TROP_PA {
        44_330.769_230_769_23 * (1.0 - (p / P0_PA).powf(0.190_263_236_5))
    } else {
        11_000.0 - 6_341.620_08 * (p / P_TROP_PA).ln()
    }
}

fn camera_telemetry(trial: &ChainTrial) -> Option<CameraFrameTelemetry> {
    let frame = trial.sensor_latch().camera_frame?;
    let matched = trial.terrain_matches().next().map(|(_, found)| found);
    let (match_outcome, inliers) = match matched {
        Some(found) => {
            let outcome = match found.outcome() {
                Outcome::Accepted => "accepted",
                Outcome::Rejected(_) => "rejected",
                Outcome::Unused(reason)
                    if reason.contains("contrast")
                        || reason.contains("faint")
                        || reason.contains("gradient")
                        || reason.contains("dark") =>
                {
                    "faint_refused"
                }
                Outcome::Unused(_) => "no_match",
            };
            (outcome.to_owned(), found.solved().map(|s| s.inliers))
        }
        None => ("uninspected".to_owned(), None),
    };
    Some(CameraFrameTelemetry {
        frame_index: frame.index,
        nadir_obliquity_deg: frame.nadir_obliquity_deg,
        local_contrast: frame.local_contrast,
        match_outcome,
        inliers,
    })
}

pub(super) fn flight_controller_entry(
    estimate: &VehicleState,
    truth: &VehicleState,
) -> NavigatorTelemetry {
    let truth_geo = ecef_to_geodetic(truth.position_ecef);
    let est_geo = ecef_to_geodetic(estimate.position_ecef);
    let frame = LocalFrame::at(&truth_geo);
    let err_ned = frame.ecef_to_ned_vector(estimate.position_ecef - truth.position_ecef);
    let est_vel_ned = frame.ecef_to_ned_vector(estimate.velocity_ecef);
    let truth_vel_ned = frame.ecef_to_ned_vector(truth.velocity_ecef);
    let err_up_m = est_geo.height_m - truth_geo.height_m;
    NavigatorTelemetry {
        set: FLIGHT_CONTROLLER_SET.to_owned(),
        est_lat_deg: est_geo.lat_rad.to_degrees(),
        est_lon_deg: est_geo.lon_rad.to_degrees(),
        est_alt_msl_m: est_geo.height_m - egm96_undulation_m(est_geo.lat_rad, est_geo.lon_rad),
        est_vel_ned_mps: est_vel_ned.to_array(),
        error_enu_m: [err_ned.y, err_ned.x, err_up_m],
        horizontal_error_m: err_ned.x.hypot(err_ned.y),
        vertical_error_m: err_up_m.abs(),
        velocity_error_mps: (est_vel_ned - truth_vel_ned).length(),
        sigma_east_m: None,
        sigma_north_m: None,
        sigma_up_m: None,
        sigma_horiz_m: None,
        epu_95_m: None,
        vpu_95_m: None,
        nees_horiz_2d: None,
        nees_pos_3d: None,
        calibration_ratio: None,
        position_source: "flight_controller".to_owned(),
        gnss_integrity: "not_monitored".to_owned(),
        contributing_sources: vec!["fc_state".to_owned()],
        source_hold_remaining_s: None,
        map_cell_id: None,
        independent_fix: None,
    }
}
