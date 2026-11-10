//! Standalone OpenUSD `.usda` export and embedded `syspack.json` manifest access for
//! `sim-p38g`.

use std::fmt::{self, Write as _};

use crate::airframe::airframe_prims;
use crate::articulated::articulated_prims;
use crate::cockpit_interior::cockpit_interior_prims;
use crate::mesh::{P38ControlJoint, P38MeshPrim};
use crate::powerplant::powerplant_prims;

/// Error returned when formatting the OpenUSD `.usda` stage fails.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct P38UsdError(pub String);

impl fmt::Display for P38UsdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for P38UsdError {}

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
    prims.extend(articulated_prims());
    prims
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
    write_control_joints(out, &prim.control_joints)?;
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

fn write_control_joints(out: &mut String, joints: &[P38ControlJoint]) -> fmt::Result {
    for (i, j) in joints.iter().enumerate() {
        let [ox, oy, oz] = j.origin_m;
        let [ax, ay, az] = j.axis;
        writeln!(
            out,
            "        custom token sim:control{i}:channel = \"{}\"",
            j.channel
        )?;
        writeln!(
            out,
            "        custom double3 sim:control{i}:origin = ({ox:.5}, {oy:.5}, {oz:.5})"
        )?;
        writeln!(
            out,
            "        custom double3 sim:control{i}:axis = ({ax:.5}, {ay:.5}, {az:.5})"
        )?;
        writeln!(
            out,
            "        custom double sim:control{i}:perUnit = {:.6}",
            j.per_unit
        )?;
        writeln!(
            out,
            "        custom double sim:control{i}:rest = {:.4}",
            j.rest
        )?;
        writeln!(
            out,
            "        custom bool sim:control{i}:slides = {}",
            j.slides
        )?;
    }
    Ok(())
}
