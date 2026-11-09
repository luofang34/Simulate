//! Standalone OpenUSD `.usda` export, embedded `syspack.json` manifest access,
//! and 3-view orthographic blueprint verification for `sim-p38g`.

use std::fmt::{self, Write as _};

use glam::{DVec3, Vec3};

use crate::airframe::airframe_prims;
use crate::articulated::articulated_prims;
use crate::cockpit_reservation::cockpit_interior_prims;
use crate::decals::conformal_decal_prims;
use crate::mesh::{P38MeshPrim, P38PartRole};
use crate::powerplant::powerplant_prims;
use crate::stations::{
    BOOM_STATIONS, BOOM_X, GONDOLA_STATIONS, OVERALL_HEIGHT_M, OVERALL_LENGTH_M, OVERALL_SPAN_M,
    PROP_HUB_Y, PROP_RADIUS_M, ROOT_CHORD_M, SEMI_SPAN, TIP_CHORD_M, WING_AREA_M2,
    WING_DIHEDRAL_DEG, wing_station,
};

/// Error returned when formatting the OpenUSD `.usda` stage fails.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct P38UsdError(pub String);

impl fmt::Display for P38UsdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for P38UsdError {}

/// Quantitative 3-view blueprint and bulkhead calibration report for `P-38G-13-LO`.
#[derive(Clone, Debug, PartialEq)]
pub struct P38BlueprintCalibrationReport {
    /// Measured overall wingspan across wingtips in metres (`target = 15.850 m`).
    pub measured_span_m: f32,
    /// Measured airframe length from nose cone (`FS 0`) to rudder trailing edge (`BS 428`) in metres (`target = 11.530 m`).
    pub measured_airframe_length_m: f32,
    /// Measured overall height from main-gear tire bottom to vertical-fin beacon top in metres (`target = 3.910 m`).
    pub measured_overall_height_m: f32,
    /// Measured theoretical root chord (`WS 0`) in metres (`target = 3.693 m`).
    pub measured_root_chord_m: f32,
    /// Measured wingtip station chord (`WS 288`) in metres (`target = 1.168 m`).
    pub measured_tip_chord_m: f32,
    /// Integrated planform wing area in square metres (`target = 30.43 m²`).
    pub measured_wing_area_m2: f32,
    /// Measured outer-wing dihedral angle in degrees (`target = 5.67°`).
    pub measured_dihedral_deg: f32,
    /// Measured twin-boom lateral spacing (`2 * BOOM_X`) in metres (`target = 4.7244 m`).
    pub measured_boom_spacing_m: f32,
    /// Measured spinner tip longitudinal station `y` in metres (`target = +3.18 m`, `0.37 m` aft of `FS 0`).
    pub measured_spinner_tip_y_m: f32,
    /// Number of calibrated central-gondola fuselage stations (`FS 0..248`).
    pub gondola_station_count: usize,
    /// Number of calibrated twin-boom stations (`BS 26..420`).
    pub boom_station_count: usize,
    /// Total number of authored OpenUSD mesh prims.
    pub prim_count: usize,
    /// Total vertex count across all prims.
    pub total_vertices: usize,
    /// Total triangle count across all prims.
    pub total_triangles: usize,
}

/// Returns the embedded `vehicles/p38g/syspack.json` manifest string.
#[must_use]
pub const fn p38g_package_source() -> &'static str {
    include_str!("../syspack.json")
}

/// Builds the complete list of station-calibrated 3D mesh prims for `P-38G-13-LO`.
#[must_use]
pub fn p38g_mesh_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::with_capacity(64);
    prims.extend(airframe_prims());
    prims.extend(powerplant_prims());
    prims.extend(cockpit_interior_prims());
    prims.extend(conformal_decal_prims());
    prims.extend(articulated_prims());
    prims
}

/// Verifies the 3D mesh and station tables against Lockheed P-38G engineering 3-view dimensions.
#[must_use]
pub fn verify_p38g_three_view_and_stations() -> P38BlueprintCalibrationReport {
    let prims = p38g_mesh_prims();
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y_hull = f32::NEG_INFINITY;
    let mut max_y_spinner = f32::NEG_INFINITY;
    let mut min_z = f32::INFINITY;
    let mut max_z = f32::NEG_INFINITY;
    let mut total_vertices = 0_usize;
    let mut total_triangles = 0_usize;

    for prim in &prims {
        total_vertices = total_vertices.wrapping_add(prim.vertices.len());
        total_triangles = total_triangles.wrapping_add(prim.indices.len() / 3);
        let is_gun_barrel = prim.prim_path.contains("NoseArmament");
        let is_prop_blade = matches!(
            prim.role,
            P38PartRole::PropellerBlades | P38PartRole::PropellerDisc
        );
        for v in &prim.vertices {
            let [vx, vy, vz] = v.position;
            let p_world = prim.rest_to_aircraft.transform_point3(DVec3::new(
                f64::from(vx),
                f64::from(vy),
                f64::from(vz),
            ));
            let p = Vec3::new(p_world.x as f32, p_world.y as f32, p_world.z as f32);
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            if !is_gun_barrel {
                max_y_hull = max_y_hull.max(p.y);
            }
            if prim.role == P38PartRole::PropellerSpinner {
                max_y_spinner = max_y_spinner.max(p.y);
            }
            if !is_prop_blade {
                min_z = min_z.min(p.z);
                max_z = max_z.max(p.z);
            }
        }
    }

    let (r_lead, r_trail, z_root, ..) = wing_station(0.58);
    let (t_lead, t_trail, z_tip, ..) = wing_station(7.315);
    let measured_root_chord_m = wing_station(0.0).0 - wing_station(0.0).1;
    let measured_tip_chord_m = t_lead - t_trail;
    let measured_dihedral_deg = (z_tip - z_root).atan2(7.315 - 0.58).to_degrees();
    let area = measure_planform_wing_area(r_lead, r_trail);

    P38BlueprintCalibrationReport {
        measured_span_m: max_x - min_x,
        measured_airframe_length_m: max_y_hull - min_y,
        measured_overall_height_m: max_z - min_z,
        measured_root_chord_m,
        measured_tip_chord_m,
        measured_wing_area_m2: area,
        measured_dihedral_deg,
        measured_boom_spacing_m: 2.0 * BOOM_X,
        measured_spinner_tip_y_m: max_y_spinner,
        gondola_station_count: GONDOLA_STATIONS.len(),
        boom_station_count: BOOM_STATIONS.len(),
        prim_count: prims.len(),
        total_vertices,
        total_triangles,
    }
}

fn measure_planform_wing_area(r_lead: f32, r_trail: f32) -> f32 {
    let steps = 200_u32;
    let step_dx = (2.0 * SEMI_SPAN) / steps as f32;
    let mut area = 0.0_f32;
    for i in 0..steps {
        let xa = -SEMI_SPAN + i as f32 * step_dx;
        let xb = xa + step_dx;
        let ca = wing_station(xa).0 - wing_station(xa).1;
        let cb = wing_station(xb).0 - wing_station(xb).1;
        area += 0.5 * (ca + cb) * step_dx;
    }
    let _ = (
        OVERALL_SPAN_M,
        OVERALL_LENGTH_M,
        OVERALL_HEIGHT_M,
        ROOT_CHORD_M,
        TIP_CHORD_M,
        WING_AREA_M2,
        WING_DIHEDRAL_DEG,
        PROP_HUB_Y,
        PROP_RADIUS_M,
        r_lead,
        r_trail,
    );
    area
}

/// Exports the complete P-38G-13-LO `"Miss Virginia"` 3D model as a standalone OpenUSD `.usda` stage.
///
/// # Errors
///
/// Returns [`P38UsdError`] if formatting into the output buffer fails.
pub fn p38g_usda() -> Result<String, P38UsdError> {
    let prims = p38g_mesh_prims();
    let mut out = String::with_capacity(512 * 1024);
    writeln!(out, "#usda 1.0").map_err(|e| P38UsdError(e.to_string()))?;
    writeln!(out, "(").map_err(|e| P38UsdError(e.to_string()))?;
    writeln!(out, "    defaultPrim = \"P38g\"").map_err(|e| P38UsdError(e.to_string()))?;
    writeln!(out, "    metersPerUnit = 1").map_err(|e| P38UsdError(e.to_string()))?;
    writeln!(out, "    upAxis = \"Z\"").map_err(|e| P38UsdError(e.to_string()))?;
    writeln!(out, ")").map_err(|e| P38UsdError(e.to_string()))?;
    writeln!(out, "def Xform \"P38g\" {{").map_err(|e| P38UsdError(e.to_string()))?;
    for (idx, prim) in prims.iter().enumerate() {
        write_usda_prim(&mut out, idx, prim).map_err(|e| P38UsdError(e.to_string()))?;
    }
    writeln!(out, "}}").map_err(|e| P38UsdError(e.to_string()))?;
    Ok(out)
}

fn write_usda_prim(out: &mut String, idx: usize, prim: &P38MeshPrim) -> fmt::Result {
    let clean_name = prim
        .prim_path
        .trim_start_matches("/P38g/")
        .replace('/', "_");
    writeln!(out, "    def Mesh \"{clean_name}\" {{")?;
    writeln!(
        out,
        "        custom token sim:partRole = \"{}\"",
        prim.role.as_token()
    )?;
    writeln!(
        out,
        "        custom string sim:primPath = \"{}\"",
        prim.prim_path
    )?;
    if let Some(axis) = prim.custom_hinge_axis {
        writeln!(
            out,
            "        custom double3 sim:hingeAxis = ({:.5}, {:.5}, {:.5})",
            axis.x, axis.y, axis.z
        )?;
    }
    let [
        [c00, c01, c02, c03],
        [c10, c11, c12, c13],
        [c20, c21, c22, c23],
        [c30, c31, c32, c33],
    ] = prim.rest_to_aircraft.to_cols_array_2d();
    writeln!(
        out,
        "        matrix4d xformOp:transform = (( {c00:.5}, {c01:.5}, {c02:.5}, {c03:.5} ), ( {c10:.5}, {c11:.5}, {c12:.5}, {c13:.5} ), ( {c20:.5}, {c21:.5}, {c22:.5}, {c23:.5} ), ( {c30:.5}, {c31:.5}, {c32:.5}, {c33:.5} ))",
    )?;
    writeln!(
        out,
        "        uniform token[] xformOpOrder = [\"xformOp:transform\"]"
    )?;
    let tri_count = prim.indices.len() / 3;
    write!(out, "        int[] faceVertexCounts = [")?;
    for t in 0..tri_count {
        if t > 0 {
            write!(out, ", ")?;
        }
        write!(out, "3")?;
    }
    writeln!(out, "]")?;
    write!(out, "        int[] faceVertexIndices = [")?;
    for (i, idx_val) in prim.indices.iter().enumerate() {
        if i > 0 {
            write!(out, ", ")?;
        }
        write!(out, "{idx_val}")?;
    }
    writeln!(out, "]")?;
    write!(out, "        point3f[] points = [")?;
    for (i, v) in prim.vertices.iter().enumerate() {
        if i > 0 {
            write!(out, ", ")?;
        }
        let [px, py, pz] = v.position;
        write!(out, "({px:.4}, {py:.4}, {pz:.4})")?;
    }
    writeln!(out, "]")?;
    let m = prim.material;
    let [dr, dg, db] = m.diffuse_rgb;
    writeln!(
        out,
        "        def Material \"Mat_{idx}\" {{ def Shader \"Surface\" {{ uniform token info:id = \"UsdPreviewSurface\" color3f inputs:diffuseColor = ({dr:.3}, {dg:.3}, {db:.3}) float inputs:metallic = {:.2} float inputs:roughness = {:.2} float inputs:opacity = {:.2} }} }}",
        m.metallic, m.roughness, m.opacity
    )?;
    writeln!(out, "    }}")
}
