//! Four-pass multi-scale spaceflight rendering contract, cross-pass planetary depth
//! occlusion, gimbal-free quaternion `FDAI/Navball`, and 3D `Orrery` projection.
//!
//! Covers:
//! 1. **Pass 1 (`SkyAndCelestialPass`)**: Finite-distance Sun, Earth, Moon, and Mars sorted
//!    strictly back-to-front by geometric range $d_k = \|\mathbf{s}_k - \mathbf{x}_{\text{eye}}\|_2$
//!    with true angular radius $\theta_k = \arcsin(\min(1, R_k / d_k))$ and analytic eclipse
//!    umbra/penumbra shadow cones.
//! 2. **Pass 2 (`PlanetaryTerrainPass`)**: Dynamic planetary frustum
//!    $z_{\text{near,p}} = \operatorname{clamp}(0.1 h_{\text{agl}}, 0.5\text{ m}, 5\text{ km})$,
//!    $z_{\text{far,p}} = d_c + 2.2 R_{\max}$, apparent-pixel-radius (`48..96 px`) LOD cross-fade
//!    decoupled from physics `primary_body` transitions, and polar cap blend (`|lat| > 85.0511°`).
//! 3. **Pass 3 (`NearFieldUsdPass`)**: Dedicated `[0.10 m, 100 km]` local depth buffer
//!    reconstructing linear metric eye-Z $z_{\text{planet,m}}(d_p)$ from Pass 2's reversed-Z
//!    `t_planet_depth` (`0.0 => +∞`) and discarding when $z_{\text{local,m}} > z_{\text{planet,m}}$,
//!    plus Mach/pressure-adaptive Aerospike plume parameters.
//! 4. **Pass 4 (`CompositorPass` / Space Avionics)**: Gimbal-free quaternion `FDAI/Navball`
//!    (`Prograde/Retrograde`, `Radial In/Out`, `Normal/Anti-Normal`, `Target`, `Maneuver Burn`)
//!    and 3D solar-system `Orrery` view projection.

use glam::{DQuat, DVec3};

/// Web Mercator latitude cutoff (`deg`) above which polar cap geometry blends in.
pub const WEB_MERCATOR_MAX_LAT_DEG: f64 = 85.051_128_78;

/// Default near-field USD pass near clip plane (`0.10 m`, supporting `0.25 m` docking cameras).
pub const NEAR_FIELD_USD_Z_NEAR_M: f64 = 0.10;

/// Default near-field USD pass far clip plane (`100 km`).
pub const NEAR_FIELD_USD_Z_FAR_M: f64 = 100_000.0;

/// Finite-distance celestial body sample for Pass 1 (`SkyAndCelestialPass`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CelestialBodySample {
    /// NAIF body identifier (`10` Sun, `399` Earth, `301` Moon, `499` Mars).
    pub body_id: i32,
    /// Body center position relative to camera eye in `ICRF` or `ECEF` (`m`).
    pub pos_rel_eye_m: DVec3,
    /// Mean physical radius $R_k$ (`m`).
    pub radius_m: f64,
}

/// Back-to-front sorted celestial disc layer for Pass 1 (`DepthWrite = false`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SortedCelestialLayer {
    /// NAIF body identifier.
    pub body_id: i32,
    /// True geometric center distance $d_k = \|\mathbf{s}_k - \mathbf{x}_{\text{eye}}\|_2$ (`m`).
    pub distance_m: f64,
    /// Unit direction from eye to body center.
    pub unit_dir: DVec3,
    /// True finite-distance angular radius $\theta_k = \arcsin(\min(1, R_k / d_k))$ (`rad`).
    pub angular_radius_rad: f64,
    /// Apparent pixel radius on screen for `viewport_height_px` and `fov_y_rad`.
    pub apparent_radius_px: f64,
    /// Terrain globe cross-fade weight $w_{\text{terrain}} \in [0, 1]$ (`[48 px, 96 px]` smoothstep).
    pub terrain_globe_blend: f64,
}

/// Sorts finite-distance celestial bodies **strictly back-to-front** (`distance_m` descending)
/// and computes their true angular radius, apparent pixel radius, and terrain LOD blend weight.
#[must_use]
pub fn sort_celestial_bodies_back_to_front(
    bodies: &[CelestialBodySample],
    viewport_height_px: u32,
    fov_y_rad: f64,
) -> Vec<SortedCelestialLayer> {
    let focal_px = 0.5 * f64::from(viewport_height_px.max(1)) / (0.5 * fov_y_rad.max(1.0e-3)).tan();
    let mut layers: Vec<SortedCelestialLayer> = bodies
        .iter()
        .map(|b| {
            let d = b.pos_rel_eye_m.length().max(1.0e-3);
            let dir = b.pos_rel_eye_m / d;
            let sin_theta = (b.radius_m.max(0.0) / d).clamp(0.0, 1.0);
            let theta = sin_theta.asin();
            let r_px = focal_px * theta.tan().max(0.0);
            SortedCelestialLayer {
                body_id: b.body_id,
                distance_m: d,
                unit_dir: dir,
                angular_radius_rad: theta,
                apparent_radius_px: r_px,
                terrain_globe_blend: evaluate_planet_impostor_to_terrain_lod(r_px),
            }
        })
        .collect();
    layers.sort_by(|a, b| {
        b.distance_m
            .total_cmp(&a.distance_m)
            .then_with(|| a.body_id.cmp(&b.body_id))
    });
    layers
}

/// Analytic solar eclipse umbra/penumbra shadow factor $\in [0, 1]$ (`1.0` = unoccluded sunlight,
/// `0.0` = total umbra) evaluated at `sample_pos_icrf_m` from `sun_pos_icrf_m` (`R_sun`)
/// against a blocking sphere at `occluder_pos_icrf_m` (`R_occ`).
#[must_use]
pub fn evaluate_eclipse_shadow_factor(
    sample_pos_icrf_m: DVec3,
    sun_pos_icrf_m: DVec3,
    sun_radius_m: f64,
    occluder_pos_icrf_m: DVec3,
    occluder_radius_m: f64,
) -> f64 {
    let to_sun = sun_pos_icrf_m - sample_pos_icrf_m;
    let to_occ = occluder_pos_icrf_m - sample_pos_icrf_m;
    let d_sun = to_sun.length();
    let d_occ = to_occ.length();
    if d_sun <= 1.0 || d_occ <= 1.0 || d_occ >= d_sun {
        return 1.0;
    }
    let u_sun = to_sun / d_sun;
    let u_occ = to_occ / d_occ;
    let sep = u_sun.dot(u_occ).clamp(-1.0, 1.0).acos();
    let alpha_sun = (sun_radius_m / d_sun).clamp(0.0, 1.0).asin();
    let alpha_occ = (occluder_radius_m / d_occ).clamp(0.0, 1.0).asin();

    if sep >= alpha_sun + alpha_occ {
        1.0
    } else if alpha_occ >= alpha_sun && sep <= alpha_occ - alpha_sun {
        0.0
    } else if alpha_sun > alpha_occ && sep <= alpha_sun - alpha_occ {
        let ratio = alpha_occ / alpha_sun.max(1.0e-12);
        (1.0 - ratio * ratio).clamp(0.0, 1.0)
    } else {
        let t = ((sep - (alpha_occ - alpha_sun).abs())
            / (2.0 * alpha_sun.min(alpha_occ)).max(1.0e-12))
        .clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    }
}

/// Dynamic clip planes for Pass 2 (`PlanetaryTerrainPass`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicPlanetaryFrustum {
    /// Dynamic near clip distance $z_{\text{near,p}} = \operatorname{clamp}(0.1 h_{\text{agl}}, 0.5\text{ m}, 5\,000\text{ m})$.
    pub z_near_m: f64,
    /// Dynamic far clip distance $z_{\text{far,p}} = d_c + 2.2 R_{\max}$.
    pub z_far_m: f64,
}

/// Computes the Pass 2 dynamic planetary frustum from `altitude_agl_m`, center distance
/// `center_dist_m`, and maximum body radius `max_radius_m`.
#[must_use]
pub fn compute_dynamic_planetary_frustum(
    altitude_agl_m: f64,
    center_dist_m: f64,
    max_radius_m: f64,
) -> DynamicPlanetaryFrustum {
    let z_near_m = (0.1 * altitude_agl_m.max(0.0)).clamp(0.5, 5_000.0);
    let z_far_m =
        (center_dist_m.max(max_radius_m) + 2.2 * max_radius_m.max(1_000.0)).max(z_near_m + 1_000.0);
    DynamicPlanetaryFrustum { z_near_m, z_far_m }
}

/// Evaluates the Pass 1 -> Pass 2 apparent-pixel-radius smoothstep transition weight
/// $w_{\text{terrain}} \in [0, 1]$ across `[48.0 px, 96.0 px]`, strictly decoupled from physics
/// `primary_body` transitions.
#[must_use]
pub fn evaluate_planet_impostor_to_terrain_lod(apparent_radius_px: f64) -> f64 {
    let u = ((apparent_radius_px - 48.0) / (96.0 - 48.0)).clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

/// Evaluates the polar cap blend factor $\in [0, 1]$ across $|lat| \in [84.5^\circ, 85.05112878^\circ]$.
#[must_use]
pub fn polar_cap_blend_weight(lat_deg: f64) -> f64 {
    let abs_lat = lat_deg.abs();
    let u = ((abs_lat - 84.5) / (WEB_MERCATOR_MAX_LAT_DEG - 84.5)).clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

/// Encodes a linear metric eye-Z distance `z_eye_m` into a finite reversed-Z depth buffer
/// value $d \in [0, 1]$ (`1.0` at `z_near_m`, `0.0` at `z_far_m`).
#[must_use]
pub fn encode_finite_reversed_z(z_eye_m: f64, z_near_m: f64, z_far_m: f64) -> f32 {
    if z_eye_m <= z_near_m {
        return 1.0;
    }
    if z_eye_m >= z_far_m || z_far_m <= z_near_m {
        return 0.0;
    }
    let num = z_near_m * (z_far_m - z_eye_m);
    let den = z_eye_m * (z_far_m - z_near_m);
    (num / den.max(1.0e-15)).clamp(0.0, 1.0) as f32
}

/// Reconstructs linear metric eye-Z $z_{\text{planet,m}}$ (`m`) from Pass 2's finite reversed-Z
/// `t_planet_depth` sample $d_p \in [0, 1]$:
/// - $d_p \le 0.0 \implies +\infty$ (sky background pixel; never occludes near-field objects).
/// - $d_p > 0.0 \implies \frac{z_{\text{near,p}} z_{\text{far,p}}}{z_{\text{near,p}} + d_p(z_{\text{far,p}} - z_{\text{near,p}})}$.
#[must_use]
pub fn reconstruct_planetary_eye_depth_m(
    planet_depth_rev_z: f32,
    frustum: DynamicPlanetaryFrustum,
) -> f64 {
    if planet_depth_rev_z <= 0.0 {
        return f64::INFINITY;
    }
    let dp = f64::from(planet_depth_rev_z.clamp(0.0, 1.0));
    let num = frustum.z_near_m * frustum.z_far_m;
    let den = frustum.z_near_m + dp * (frustum.z_far_m - frustum.z_near_m);
    num / den.max(1.0e-15)
}

/// Returns `true` iff a Pass 3 (`NearFieldUsdPass`) fragment at linear metric eye distance
/// `local_usd_eye_z_m` (`[0.10 m, 100 km]`) is occluded by Pass 2's planetary terrain depth
/// `planet_depth_rev_z`.
#[must_use]
pub fn near_field_occluded_by_planet(
    local_usd_eye_z_m: f64,
    planet_depth_rev_z: f32,
    planet_frustum: DynamicPlanetaryFrustum,
) -> bool {
    let z_planet_m = reconstruct_planetary_eye_depth_m(planet_depth_rev_z, planet_frustum);
    local_usd_eye_z_m > z_planet_m
}

/// Mach- and ambient-pressure-adaptive truncated plug aerospike exhaust plume visual state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AerospikePlumeVisual {
    /// Whether the plume is visible (`throttle > 1e-3`).
    pub visible: bool,
    /// Plume length along `-Y` below `/Rocket/Propulsion` (`m`).
    pub plume_length_m: f32,
    /// Outer expansion radius ratio relative to nozzle base (`1.0` at sea level -> `3.4` in vacuum).
    pub expansion_ratio: f32,
    /// Axial spacing between Mach shock diamonds (`m`, `0.0` in vacuum expansion).
    pub shock_diamond_spacing_m: f32,
    /// Linear sRGB emissive core color.
    pub core_emissive_rgb: [f32; 3],
}

/// Evaluates `Raketa`'s aerospike plume geometry from `throttle` (`0..=1`) and `ambient_pressure_pa`.
#[must_use]
pub fn evaluate_aerospike_plume_visual(
    throttle: f64,
    ambient_pressure_pa: f64,
) -> AerospikePlumeVisual {
    let thr = throttle.clamp(0.0, 1.0);
    if thr <= 1.0e-3 {
        return AerospikePlumeVisual {
            visible: false,
            plume_length_m: 0.0,
            expansion_ratio: 1.0,
            shock_diamond_spacing_m: 0.0,
            core_emissive_rgb: [0.0; 3],
        };
    }
    let p_ratio = (ambient_pressure_pa.max(0.0) / 101_325.0).clamp(0.0, 1.5);
    let vacuum_factor = 1.0 - (p_ratio / 1.0).clamp(0.0, 1.0);
    let expansion = (0.95 + 2.45 * vacuum_factor * vacuum_factor) as f32;
    let length = ((4.5 + 9.5 * vacuum_factor) * (0.35 + 0.65 * thr)) as f32;
    let diamond = if p_ratio > 0.02 {
        (1.35 * thr.sqrt() / p_ratio.powf(0.25)) as f32
    } else {
        0.0
    };
    AerospikePlumeVisual {
        visible: true,
        plume_length_m: length,
        expansion_ratio: expansion,
        shock_diamond_spacing_m: diamond,
        core_emissive_rgb: [
            (2.8 * thr) as f32,
            (4.2 * thr) as f32,
            (6.5 * thr * (0.7 + 0.3 * vacuum_factor)) as f32,
        ],
    }
}

/// Direction marker projected onto the 3D gimbal-free `FDAI / Navball`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FdaiMarkerDir {
    /// Unit vector in spacecraft body coordinates (`+X` right, `+Y` thrust/forward, `+Z` dorsal).
    pub dir_body: DVec3,
    /// Projected 2D front-hemisphere disk coordinates `[-1, 1]^2` (`z_front >= 0` when on front hemisphere).
    pub disk_xy: [f32; 2],
    /// `true` when the marker lies on the visible front hemisphere of the navball.
    pub front_hemisphere: bool,
}

/// Complete gimbal-free quaternion `FDAI / Navball` state for Pass 4 (`CompositorPass`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FdaiNavballState {
    /// Quaternion rotating the reference frame (`ORBIT` or `SURFACE`) into spacecraft body axes.
    pub q_ref_to_body: DQuat,
    /// Prograde velocity marker ($\hat{\mathbf{v}}$).
    pub prograde: FdaiMarkerDir,
    /// Retrograde velocity marker ($-\hat{\mathbf{v}}$).
    pub retrograde: FdaiMarkerDir,
    /// Radial-out marker ($\hat{\mathbf{r}}$).
    pub radial_out: FdaiMarkerDir,
    /// Radial-in marker ($-\hat{\mathbf{r}}$).
    pub radial_in: FdaiMarkerDir,
    /// Orbit normal marker ($\hat{\mathbf{r}} \times \hat{\mathbf{v}}$).
    pub normal: FdaiMarkerDir,
    /// Orbit anti-normal marker ($-\hat{\mathbf{r}} \times \hat{\mathbf{v}}$).
    pub anti_normal: FdaiMarkerDir,
    /// Target direction marker.
    pub target: FdaiMarkerDir,
    /// Commanded maneuver burn direction marker.
    pub maneuver_burn: FdaiMarkerDir,
}

/// Evaluates the gimbal-free quaternion `FDAI / Navball` at arbitrary pitch/roll/yaw (including
/// exact $\pm 90^\circ$ vertical launch and tail-sitting touchdown without Euler lock).
#[must_use]
pub fn evaluate_fdai_navball(
    q_body_to_icrf: DQuat,
    pos_rel_icrf_m: DVec3,
    vel_rel_icrf_m_s: DVec3,
    target_dir_icrf: DVec3,
    burn_dir_icrf: DVec3,
) -> FdaiNavballState {
    let q_i2b = q_body_to_icrf.normalize().conjugate();
    let u_r = pos_rel_icrf_m.try_normalize().unwrap_or(DVec3::X);
    let u_v = vel_rel_icrf_m_s.try_normalize().unwrap_or(DVec3::Y);
    let u_n = u_r.cross(u_v).try_normalize().unwrap_or(DVec3::Z);
    let u_t = target_dir_icrf.try_normalize().unwrap_or(u_v);
    let u_b = burn_dir_icrf.try_normalize().unwrap_or(u_v);

    FdaiNavballState {
        q_ref_to_body: q_i2b,
        prograde: project_fdai_marker(q_i2b, u_v),
        retrograde: project_fdai_marker(q_i2b, -u_v),
        radial_out: project_fdai_marker(q_i2b, u_r),
        radial_in: project_fdai_marker(q_i2b, -u_r),
        normal: project_fdai_marker(q_i2b, u_n),
        anti_normal: project_fdai_marker(q_i2b, -u_n),
        target: project_fdai_marker(q_i2b, u_t),
        maneuver_burn: project_fdai_marker(q_i2b, u_b),
    }
}

fn project_fdai_marker(q_i2b: DQuat, dir_icrf: DVec3) -> FdaiMarkerDir {
    let b = (q_i2b * dir_icrf).try_normalize().unwrap_or(DVec3::Y);
    FdaiMarkerDir {
        dir_body: b,
        disk_xy: [b.x as f32, b.z as f32],
        front_hemisphere: b.y >= 0.0,
    }
}

/// One celestial body or spacecraft node in the 3D `Orrery` view.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrreryBodyNode {
    /// Identifier (`10` Sun, `399` Earth, `301` Moon, `499` Mars, or vessel ID).
    pub id: i64,
    /// Position relative to the Orrery focus body in `ICRF` (`m`).
    pub pos_rel_focus_icrf_m: DVec3,
    /// Sphere of influence radius (`m`, `0.0` for vessels).
    pub soi_radius_m: f64,
}

/// Projected 2D screen node for the interactive 3D `Orrery` tactical display.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProjectedOrreryNode {
    /// Node identifier.
    pub id: i64,
    /// Normalized screen coordinates `[-1, 1]^2`.
    pub ndc_xy: [f32; 2],
    /// Camera-space depth (`m`, `> 0` when in front of the Orrery camera).
    pub depth_m: f64,
    /// Projected SOI circle radius in NDC units.
    pub soi_radius_ndc: f32,
}

/// Projects `nodes` into the interactive 3D `Orrery` camera (`camera_pos_rel_focus_m`, `q_cam_to_icrf`).
#[must_use]
pub fn project_orrery_scene(
    nodes: &[OrreryBodyNode],
    camera_pos_rel_focus_m: DVec3,
    q_cam_to_icrf: DQuat,
    fov_y_rad: f64,
    aspect: f64,
) -> Vec<ProjectedOrreryNode> {
    let q_i2c = q_cam_to_icrf.normalize().conjugate();
    let fy = 1.0 / (0.5 * fov_y_rad.max(1.0e-3)).tan();
    let fx = fy / aspect.max(1.0e-3);
    nodes
        .iter()
        .map(|n| {
            let rel = q_i2c * (n.pos_rel_focus_icrf_m - camera_pos_rel_focus_m);
            let z = (-rel.z).max(1.0);
            let x_ndc = (rel.x * fx / z) as f32;
            let y_ndc = (rel.y * fy / z) as f32;
            let r_ndc = (n.soi_radius_m.max(0.0) * fy / z) as f32;
            ProjectedOrreryNode {
                id: n.id,
                ndc_xy: [x_ndc, y_ndc],
                depth_m: z,
                soi_radius_ndc: r_ndc,
            }
        })
        .collect()
}
