use indicate_instrument_state::{
    AirframeDisplayProfile, FreshnessPolicy, PanelData, SignalStatus, UnusualAttitudeState,
    resolve_stateful,
};
use sim_core::{SimInstant, VehicleState};

use super::*;
use crate::{AltitudeSource, DisplayAxes, FeedConfig, FlightSample, StateFeed};

const AT: SimInstant = SimInstant::START;

fn threshold() -> Geodetic {
    Geodetic::from_degrees(39.12, -75.47, 10.0)
}

fn feed() -> StateFeed {
    StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin: threshold(),
        origin_id: 3,
        altitude: AltitudeSource::OriginRelative,
        heading_mode: crate::HeadingMode::default(),
    })
}

fn guidance() -> RunwayNavGuidance {
    RunwayNavGuidance {
        threshold: threshold(),
        course_rad: 0.0,
        length_m: 2_000.0,
        glide_path_rad: 3.0_f64.to_radians(),
        ident: IdentStr::new("RW01").unwrap_or_default(),
    }
}

/// A vehicle `back_m` short of the threshold, `right_m` right of the centreline, `up_m` high.
fn on_approach(back_m: f64, right_m: f64, up_m: f64) -> VehicleState {
    let local = LocalFrame::at(&threshold());
    let fixed = BodyEllipsoid::WGS84.geodetic_to_fixed(&threshold())
        + local.north() * -back_m
        + local.east() * right_m
        + local.up() * up_m;
    let geodetic = BodyEllipsoid::WGS84.fixed_to_geodetic(fixed);
    VehicleState::level_flight(&geodetic, 0.0, 40.0)
}

fn panel(vehicle: VehicleState, with_selections: bool) -> PanelData {
    let sample = FlightSample {
        guidance: with_selections.then_some(HostFlightGuidance {
            selections: PilotSelections {
                heading_rad: 0.4,
                altitude_m: Some(600.0),
                baro_setting_hpa: Some(1_013.25),
            },
            nav: HostNavGuidance::RunwayReceiver(guidance()),
            autopilot: AutopilotModeSummary::default(),
            source_epoch: 0,
        }),
        ..FlightSample::truth(AT, vehicle)
    };
    let state = feed().state(&sample).unwrap();
    resolve_stateful(
        &state,
        &FreshnessPolicy::default(),
        &AirframeDisplayProfile::simulator(),
        &mut UnusualAttitudeState::default(),
    )
}

#[test]
fn selections_give_the_heading_bug_a_reference_the_rose_can_show() {
    let without = panel(on_approach(3_000.0, 0.0, 173.0), false);
    assert!(!without.heading_bug_rose_rad.status.shows_value());
    assert!(!without.nav.status.shows_value());
    let with = panel(on_approach(3_000.0, 0.0, 173.0), true);
    assert_eq!(with.heading_bug_rose_rad.status, SignalStatus::Valid);
    assert!((with.heading_bug_rose_rad.value - 0.4).abs() < 1e-4);
}

#[test]
fn the_deviation_bar_points_back_to_the_centreline() {
    let left = panel(on_approach(3_000.0, -60.0, 173.0), true);
    let right = panel(on_approach(3_000.0, 60.0, 173.0), true);
    assert!(left.nav.status.shows_value());
    assert!(left.nav.data.cdi_dots > 0.3, "{}", left.nav.data.cdi_dots);
    assert!(
        right.nav.data.cdi_dots < -0.3,
        "{}",
        right.nav.data.cdi_dots
    );
    assert_eq!(left.nav.data.fromto, NavFromTo::To);
}

#[test]
fn the_glideslope_diamond_sits_below_centre_when_high() {
    // 3 deg from a 300 m intercept at 3.3 km from the intercept is 173 m.
    let on_path = panel(on_approach(3_000.0, 0.0, 173.0), true);
    let high = panel(on_approach(3_000.0, 0.0, 260.0), true);
    let on_dots = on_path.nav.data.vdev_dots.unwrap();
    assert!(on_dots.abs() < 0.2, "{on_dots}");
    assert!(high.nav.data.vdev_dots.unwrap() > 1.0);
    let dist = on_path.nav.data.dist_nm.unwrap();
    assert!((dist - 3_004.0 / 1_852.0).abs() < 0.01, "{dist}");
}

fn nzsp_guidance() -> RunwayNavGuidance {
    RunwayNavGuidance {
        threshold: Geodetic::from_degrees(-89.9975, 139.2729, 2_835.0),
        course_rad: 20.0_f64.to_radians(),
        length_m: 3_658.0,
        glide_path_rad: 3.0_f64.to_radians(),
        ident: IdentStr::new("02T").unwrap_or_default(),
    }
}

/// Ownship at `(along_m, right_m)` relative to `NZSP RWY 02T` in 3D ECEF, flying
/// straight parallel to the 3D runway centerline.
fn nzsp_approach_vehicle(along_m: f64, right_m: f64) -> VehicleState {
    let g = nzsp_guidance();
    let thr_local = LocalFrame::at(&g.threshold);
    let (sin_c, cos_c) = g.course_rad.sin_cos();
    let rwy_dir = thr_local.north() * cos_c + thr_local.east() * sin_c;
    let rwy_right = thr_local.east() * cos_c - thr_local.north() * sin_c;
    let up_m = (GLIDE_PATH_INTERCEPT_M - along_m).max(0.0) * g.glide_path_rad.tan();
    let pos_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&g.threshold)
        + rwy_dir * along_m
        + rwy_right * right_m
        + thr_local.up() * up_m;
    let own_geo = BodyEllipsoid::WGS84.fixed_to_geodetic(pos_ecef);
    let own_ned = LocalFrame::at(&own_geo).ecef_to_ned_vector(rwy_dir);
    let own_hdg = own_ned.y.atan2(own_ned.x).rem_euclid(std::f64::consts::TAU);
    VehicleState::level_flight(&own_geo, own_hdg, 65.0)
}

fn nzsp_panel(vehicle: VehicleState, heading_mode: HeadingMode) -> PanelData {
    let g = nzsp_guidance();
    let mut state_feed = StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin: g.threshold,
        origin_id: 9,
        altitude: AltitudeSource::OriginRelative,
        heading_mode,
    });
    let sample = FlightSample {
        guidance: Some(HostFlightGuidance {
            selections: PilotSelections {
                heading_rad: 0.0,
                altitude_m: Some(3_000.0),
                baro_setting_hpa: Some(1_013.25),
            },
            nav: HostNavGuidance::RunwayReceiver(g),
            autopilot: AutopilotModeSummary::default(),
            source_epoch: 0,
        }),
        ..FlightSample::truth(AT, vehicle)
    };
    let state = state_feed.state(&sample).unwrap();
    resolve_stateful(
        &state,
        &FreshnessPolicy::default(),
        &AirframeDisplayProfile::simulator(),
        &mut UnusualAttitudeState::default(),
    )
}

#[test]
fn nzsp_rwy_02t_approach_across_south_pole_keeps_course_aligned_with_heading() {
    let angle_diff_deg = |a: f32, b: f32| -> f32 {
        let d = (a - b).rem_euclid(std::f32::consts::TAU);
        d.min(std::f32::consts::TAU - d).to_degrees()
    };
    // 3 NM (-5,556 m) and 0.5 NM (-926 m) are on the opposite side of 90° S from
    // the threshold (lon ≈ -20° vs +139.27°); -100 m, 0 m, and +1,500 m are past 90° S.
    let three_nm = nzsp_approach_vehicle(-3.0 * METRES_PER_NM, 0.0);
    let three_nm_geo = BodyEllipsoid::WGS84.fixed_to_geodetic(three_nm.position_ecef);
    assert!(
        three_nm_geo.lon_deg() < 0.0,
        "3 NM out is across 90° S: {}",
        three_nm_geo.lon_deg()
    );

    for mode in [HeadingMode::SimLocalTrue, HeadingMode::EarthMagnetic] {
        for along_m in [
            -3.0 * METRES_PER_NM,
            -0.5 * METRES_PER_NM,
            -100.0,
            0.0,
            1_500.0,
        ] {
            let resolved = nzsp_panel(nzsp_approach_vehicle(along_m, 0.0), mode);
            assert!(resolved.nav.status.shows_value());
            assert!(
                resolved.nav.data.cdi_dots.abs() < 1e-4,
                "{mode:?} along={along_m}: cdi={}",
                resolved.nav.data.cdi_dots
            );
            assert_eq!(resolved.nav.data.fromto, NavFromTo::To);
            let hdg = resolved.heading.value_rad.value;
            let crs = resolved.nav.course_rose_rad.value;
            let diff = angle_diff_deg(crs, hdg);
            assert!(
                diff < 0.5,
                "{mode:?} along={along_m}: crs={crs} vs hdg={hdg} ({diff}°)"
            );
        }
    }

    let left = nzsp_panel(
        nzsp_approach_vehicle(-3.0 * METRES_PER_NM, -60.0),
        HeadingMode::SimLocalTrue,
    );
    let right = nzsp_panel(
        nzsp_approach_vehicle(-3.0 * METRES_PER_NM, 60.0),
        HeadingMode::SimLocalTrue,
    );
    assert!(left.nav.data.cdi_dots > 0.2, "{}", left.nav.data.cdi_dots);
    assert!(
        right.nav.data.cdi_dots < -0.2,
        "{}",
        right.nav.data.cdi_dots
    );
}
