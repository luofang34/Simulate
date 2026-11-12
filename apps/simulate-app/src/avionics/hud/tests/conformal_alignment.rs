//! End-to-end registration tests across `StateFeed` (`vehicle_cg`), `pilot_eye_mount`
//! (`[-0.26 m]` lateral seat lever arm), `hud_context`, `VectorHudPanel::design_frame`, and
//! `maplibre`'s spherical `project_geodetic_to_screen` at both South Pole (`NZSP`) and
//! continental US (`KEWR`, `KDOV`, `47N`) airports.

use glam::{DMat3, DQuat, DVec3};
use sim_core::{CameraPoseEcef, SimInstant, VehicleState};
use sim_geo::Geodetic;
use sim_geo::{BodyEllipsoid, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_indicate::{
    AltitudeSource, DisplayAxes, FeedConfig, FlightSample, HeadingMode, SourceMonitor, StateFeed,
    VectorHudPanel,
};
use sim_maplibre::{
    DEFAULT_FAR_CLIP_M, DEFAULT_NEAR_CLIP_M, conformal_stamp_from_pose_on,
    project_geodetic_to_screen, symmetric_frustum,
};
use sim_scenery::{
    AirportDatabase, AirportRecord, CatalogVerticalDatum, LandingRunwayGeometry, RunwayDatum,
};

use super::super::*;
use crate::camera_view::{pilot_eye_mount, scenario_camera};
use crate::sensor_bus::{OnboardSensorBus, SensorRealism};

fn indicate_ned_to_body(roll: f32, pitch: f32, yaw: f32) -> DMat3 {
    let (sr, cr) = f64::from(roll).sin_cos();
    let (sp, cp) = f64::from(pitch).sin_cos();
    let (sy, cy) = f64::from(yaw).sin_cos();
    DMat3::from_cols(
        DVec3::new(cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        DVec3::new(cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        DVec3::new(-sp, sr * cp, cr * cp),
    )
}

fn build_vehicle_at_station(
    rwy: &LandingRunwayGeometry,
    along_m: f64,
    agl_m: f64,
    pitch_deg: f64,
    roll_deg: f64,
) -> VehicleState {
    let thr_local = LocalFrame::at(&rwy.threshold);
    let (sb, cb) = rwy.true_bearing_rad.sin_cos();
    let fwd_thr = thr_local.north() * cb + thr_local.east() * sb;
    let mut cg_geo = ecef_to_geodetic(geodetic_to_ecef(&rwy.threshold) + fwd_thr * along_m);
    cg_geo.height_m = rwy.threshold.height_m + agl_m;
    let cg_local = LocalFrame::at(&cg_geo);
    let hdg_cg = fwd_thr
        .dot(cg_local.east())
        .atan2(fwd_thr.dot(cg_local.north()));
    let ned_from_body = DQuat::from_euler(
        glam::EulerRot::ZYX,
        hdg_cg,
        pitch_deg.to_radians(),
        roll_deg.to_radians(),
    );
    let attitude = (DQuat::from_mat3(&cg_local.ned_to_ecef()) * ned_from_body).normalize();
    VehicleState {
        position_ecef: geodetic_to_ecef(&cg_geo),
        velocity_ecef: attitude * DVec3::new(58.0, 0.0, 0.0),
        attitude,
        angular_rate_body: DVec3::ZERO,
    }
}

fn run_feed_and_tessellate(
    vehicle: VehicleState,
    origin: sim_geo::Geodetic,
    ctx: &HudContext,
    target_px: [u32; 2],
) -> DMat3 {
    let mut feed = StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin,
        origin_id: 1,
        altitude: AltitudeSource::OriginRelative,
        heading_mode: HeadingMode::EarthMagnetic,
        deviation_scale: sim_indicate::DeviationScaleProfile::GNSS_NAVIGATOR,
    });
    let monitored = SourceMonitor::new()
        .step(&mut feed, &FlightSample::truth(SimInstant::START, vehicle))
        .expect("monitored frame");
    let mut panel = VectorHudPanel::new();
    let verts = panel
        .tessellate(
            &monitored.panel,
            ctx,
            Some(&monitored.alerts),
            target_px[0],
            target_px[1],
        )
        .expect("tessellate HUD");
    assert!(!verts.is_empty());
    let [design_w, design_h] =
        VectorHudPanel::design_frame(target_px[0], target_px[1]).expect("df");
    assert!(
        (design_w / design_h - target_px[0] as f32 / target_px[1] as f32).abs() < 1e-5,
        "design_frame aspect must not be distorted by rounding"
    );
    indicate_view_from_ned(
        monitored.panel.roll_rad.value,
        monitored.panel.pitch_rad.value,
        monitored.panel.heading.true_heading_rad().value,
        ctx,
    )
}

/// One HUD frame through the full pipeline, with the conformal stamp the window camera
/// draws the scene through.
struct Pipeline {
    rwy: LandingRunwayGeometry,
    hud: sim_indicate::HudRunway,
    stamp: sim_maplibre::ConformalFrameStamp,
    surface: ConformalSurface,
    datum: CatalogVerticalDatum,
    target_px: [u32; 2],
    view_from_ned: DMat3,
    fy_px: f64,
}

impl Pipeline {
    fn new(
        code: &str,
        desig: &str,
        (along_m, agl_m): (f64, f64),
        (pitch_deg, roll_deg): (f64, f64),
        target_px: [u32; 2],
        runway_datum: Option<&RunwayDatum>,
    ) -> Self {
        let db = AirportDatabase::builtin();
        let (_, rec) = db.by_code(code).expect("airport");
        let datum = CatalogVerticalDatum::OrthometricMslEgm96;
        let rwy = rec.landing_runway(desig, datum).expect("landing runway");
        let rwy = runway_datum.map_or(rwy, |runway_datum| {
            runway_datum.touchdown_plane(&rwy, datum)
        });
        let vehicle = build_vehicle_at_station(&rwy, along_m, agl_m, pitch_deg, roll_deg);

        // Non-zero C172 left-seat pilot eye lever arm `[+0.10, -0.26, -0.12]`.
        let cam = scenario_camera(pilot_eye_mount().pose(&vehicle, DVec3::ZERO));
        let surface = ConformalSurface::earth_globe();
        let view = HudView {
            surface_size: target_px,
            camera: cam,
            traffic: &[],
            airframe_attitude: vehicle.attitude,
            eye_body_lever_m: vehicle.attitude.inverse()
                * (cam.pose.position_ecef - vehicle.position_ecef),
            conformal_surface: surface,
            vertical_datum: Some(datum),
        };
        let ctx = hud_context(&vehicle, &view, Some(&rwy), None);
        let hud = ctx.approach.expect("HUD approach runway");
        let view_from_ned = run_feed_and_tessellate(vehicle, rwy.threshold, &ctx, target_px);
        let fy_px = (0.5 * f64::from(target_px[1]))
            / (0.5 * f64::from(ctx.projection.value.vertical_fov_rad)).tan();
        let eye_geo = BodyEllipsoid::WGS84.fixed_to_geodetic(cam.pose.position_ecef);
        let frustum = symmetric_frustum(
            cam.fov_y_rad,
            target_px[0],
            target_px[1],
            DEFAULT_NEAR_CLIP_M,
            DEFAULT_FAR_CLIP_M,
        )
        .expect("frustum");
        let stamp = conformal_stamp_from_pose_on(
            SimInstant::START,
            BodyEllipsoid::WGS84,
            sim_maplibre::RenderBody::EARTH,
            &CameraPoseEcef {
                position_ecef: cam.pose.position_ecef,
                orientation: cam.pose.orientation,
            },
            frustum,
            undulation_for_surface(surface, Some(datum), &eye_geo),
        )
        .expect("stamp");
        Self {
            rwy,
            hud,
            stamp,
            surface,
            datum,
            target_px,
            view_from_ned,
            fy_px,
        }
    }

    /// Where the window camera draws `geo`, a point at its ellipsoidal height.
    fn otw_px(&self, geo: &sim_geo::Geodetic) -> Option<[f32; 2]> {
        project_geodetic_to_screen(
            &self.stamp,
            geo,
            undulation_for_surface(self.surface, Some(self.datum), geo),
            self.target_px,
        )
        .map(|(px, _)| px)
    }

    /// Where the HUD draws its runway centreline `along_m` past the threshold.
    fn hud_px(&self, along_m: f32) -> Option<[f32; 2]> {
        let hud = &self.hud;
        let (sc, cc) = hud.course_rad.sin_cos();
        let slope = hud.far_down_offset_m / hud.length_m;
        let hud_ned = DVec3::new(
            f64::from(hud.threshold_m[0] + cc * along_m),
            f64::from(hud.threshold_m[1] + sc * along_m),
            f64::from(hud.threshold_m[2] + slope * along_m),
        );
        let v = self.view_from_ned * hud_ned;
        if v.x <= DEFAULT_NEAR_CLIP_M {
            return None;
        }
        let (cx, cy) = (
            0.5 * f64::from(self.target_px[0]),
            0.5 * f64::from(self.target_px[1]),
        );
        Some([
            (cx + self.fy_px * (v.y / v.x)) as f32,
            (cy + self.fy_px * (v.z / v.x)) as f32,
        ])
    }
}

fn assert_full_pipeline_hud_alignment(
    code: &str,
    desig: &str,
    along_m: f64,
    agl_m: f64,
    pitch_deg: f64,
    roll_deg: f64,
    target_px: [u32; 2],
) {
    let pipe = Pipeline::new(
        code,
        desig,
        (along_m, agl_m),
        (pitch_deg, roll_deg),
        target_px,
        None,
    );
    for (name, geo, a_m) in [
        ("threshold", &pipe.rwy.threshold, 0.0_f32),
        ("far_end", &pipe.rwy.far_end, pipe.hud.length_m),
    ] {
        let (Some(otw_px), Some(hud_px)) = (pipe.otw_px(geo), pipe.hud_px(a_m)) else {
            continue;
        };
        let err = (hud_px[0] - otw_px[0]).hypot(hud_px[1] - otw_px[1]);
        assert!(
            err < 0.20,
            "{code} {desig} {name} (along={along_m} m): HUD {hud_px:?} vs OTW {otw_px:?} (err={err:.4} px)"
        );
    }
}

fn indicate_view_from_ned(roll: f32, pitch: f32, true_heading: f32, ctx: &HudContext) -> DMat3 {
    let ned_to_body = indicate_ned_to_body(roll, pitch, true_heading);
    let q = ctx.projection.value.body_from_view;
    let view_from_body = DMat3::from_quat(
        DQuat::from_xyzw(
            f64::from(q.x),
            f64::from(q.y),
            f64::from(q.z),
            f64::from(q.w),
        )
        .inverse(),
    );
    view_from_body * ned_to_body
}

#[test]
fn pilot_seat_lever_arm_registers_at_nzsp_south_pole_and_conus_airports() {
    for (code, desig, along_m, agl_m, pitch_deg, roll_deg, vp) in [
        ("NZSP", "02T", -95.0, 25.0, -3.0, 0.0, [916, 876]),
        ("NZSP", "02T", -5_556.0, 305.0, -3.0, 6.0, [916, 876]),
        ("NZSP", "02T", 120.0, 260.0, -12.0, 0.0, [916, 876]),
        ("KEWR", "22L", -950.0, 65.0, -3.0, 8.0, [916, 876]),
        ("KEWR", "04L", -1_200.0, 80.0, -3.0, -5.0, [2560, 1440]),
        ("KDOV", "14", -950.0, 65.0, -3.0, 4.0, [916, 876]),
        ("47N", "07", -600.0, 45.0, -3.0, -6.0, [1920, 1055]),
    ] {
        assert_full_pipeline_hud_alignment(code, desig, along_m, agl_m, pitch_deg, roll_deg, vp);
    }
}

/// Terrain over `record`'s runways: each runway's charted line, 2 m higher, crowned
/// `crown_m` mid-runway.
fn crowned_terrain(record: &AirportRecord, crown_m: f64) -> impl Fn(&Geodetic) -> Option<f64> + '_ {
    move |point: &Geodetic| {
        let at = geodetic_to_ecef(point);
        record
            .runways
            .iter()
            .map(|runway| {
                let [a, b] = &runway.ends;
                let a_ecef = geodetic_to_ecef(&Geodetic::from_degrees(a.lat_deg, a.lon_deg, 0.0));
                let b_ecef = geodetic_to_ecef(&Geodetic::from_degrees(b.lat_deg, b.lon_deg, 0.0));
                let t = (at - a_ecef).dot(b_ecef - a_ecef) / (b_ecef - a_ecef).length_squared();
                let off = (at - (a_ecef + (b_ecef - a_ecef) * t)).length();
                let line = f64::from(a.elevation_msl_m)
                    + f64::from(b.elevation_msl_m - a.elevation_msl_m) * t;
                (
                    off,
                    line + 2.0 + crown_m * (1.0 - (2.0 * t - 1.0).powi(2)).max(0.0),
                )
            })
            .min_by(|x, y| x.0.total_cmp(&y.0))
            .map(|(_, height)| height)
    }
}

/// The HUD centreline lands on the runway the window draws: the centreline at the
/// threshold and 1,000 m on (or 100 m short of the far end), at the ground the world surface has there (the runway datum
/// the maps also lay into the terrain they draw, here with crowned profiles), projects
/// within a quarter pixel of the HUD's centreline, from sea level to a mile high and on
/// N51's crowned runway.
#[test]
fn hud_centreline_lands_on_the_drawn_runway_at_threshold_and_1000_m() {
    let ground = crate::world_surface::WorldSurfaceHandle::new();
    ground
        .configure(&crate::map_rig::ScenerySource::RealWorld { cache_dir: None })
        .expect("real-world surface");
    let runway_datum = ground.runway_datum();
    for (code, desig) in [
        ("KEWR", "22L"),
        ("KDEN", "16R"),
        ("47N", "07"),
        ("N51", "04"),
    ] {
        let (idx, record) = AirportDatabase::builtin().by_code(code).expect("airport");
        // Airports inside the offline catalog already took their profile from it.
        runway_datum.resolve_airport(idx, &crowned_terrain(record, 3.0));
        assert!(runway_datum.is_resolved(idx));
        let pipe = Pipeline::new(
            code,
            desig,
            (-950.0, 65.0),
            (-3.0, 0.0),
            [1920, 1080],
            Some(runway_datum),
        );
        let rwy = &pipe.rwy;
        let thr_local = LocalFrame::at(&rwy.threshold);
        let (sb, cb) = rwy.true_bearing_rad.sin_cos();
        let ahead = thr_local.north() * cb + thr_local.east() * sb;
        let hud_scale = f64::from(pipe.hud.length_m) / rwy.landing_length_m;
        // 47N and N51 land on less than 1,100 m past their displaced thresholds.
        for along_m in [0.0, 1_000.0_f64.min(rwy.landing_length_m - 100.0)] {
            let mut centre = ecef_to_geodetic(geodetic_to_ecef(&rwy.threshold) + ahead * along_m);
            centre.height_m = ground.ellipsoidal_ground_m(&centre).expect("runway ground");
            let otw = pipe.otw_px(&centre).expect("centreline in view");
            let hud = pipe
                .hud_px((along_m * hud_scale) as f32)
                .expect("HUD centreline in view");
            let err = (hud[0] - otw[0]).hypot(hud[1] - otw[1]);
            assert!(
                err < 0.25,
                "{code} {desig} +{along_m} m: HUD {hud:?} vs drawn {otw:?} ({err:.3} px)"
            );
        }
    }
}

#[test]
fn calibrated_waas_realistic_mode_keeps_far_end_hud_drift_sub_two_metres() {
    let mut bus = OnboardSensorBus::new(SensorRealism::Realistic);
    for _ in 0..660_usize {
        bus.step(1.0 / 120.0);
    }
    let e = bus.current();
    let lateral_at_2400m =
        e.position_ned_m.truncate().length() + 2_400.0 * e.attitude_body_rad.z.abs();
    assert!(
        lateral_at_2400m > 0.0 && lateral_at_2400m < 1.8,
        "expected calibrated WAAS+IRS drift <1.8 m at 2.4 km, got {lateral_at_2400m:.3} m"
    );
}
