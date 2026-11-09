//! From pixels back to 3D: the ray a solved camera sees a pixel along, where it meets a
//! known plane (an instrument panel) or the airframe mesh, and the point several
//! photos' rays agree on.

use glam::DVec3;
use sim_p38g::P38MeshPrim;

use super::resection::PinholePose;

/// A half-line in mesh axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Ray {
    /// Camera centre.
    pub(crate) origin: DVec3,
    /// Unit direction.
    pub(crate) dir: DVec3,
}

impl Ray {
    /// The point `t` metres along the ray.
    pub(crate) fn at(&self, t: f64) -> DVec3 {
        self.origin + self.dir * t
    }
}

/// The ray `pose` sees pixel `[u, v]` along.
pub(crate) fn ray_through(pose: &PinholePose, [u, v]: [f64; 2]) -> Ray {
    let [cx, cy] = pose.principal_px;
    let optical = DVec3::new((u - cx) / pose.focal_px, (v - cy) / pose.focal_px, 1.0);
    Ray {
        origin: pose.eye,
        dir: (pose.mesh_to_optical.inverse() * optical).normalize(),
    }
}

/// Where `ray` meets the plane through `point` with normal `normal`, ahead of the camera.
pub(crate) fn hit_plane(ray: &Ray, point: DVec3, normal: DVec3) -> Option<DVec3> {
    let along = ray.dir.dot(normal);
    if along.abs() < 1e-9 {
        return None;
    }
    let t = (point - ray.origin).dot(normal) / along;
    (t > 0.0).then(|| ray.at(t))
}

/// The nearest point where `ray` meets any triangle of `prims`, with that prim's path.
pub(crate) fn hit_mesh<'a>(ray: &Ray, prims: &'a [P38MeshPrim]) -> Option<(DVec3, &'a str)> {
    let mut best: Option<(f64, &str)> = None;
    for prim in prims {
        let world: Vec<DVec3> = prim
            .vertices
            .iter()
            .map(|v| {
                prim.rest_to_aircraft
                    .transform_point3(DVec3::from(v.position.map(f64::from)))
            })
            .collect();
        for [i, j, k] in prim.indices.as_chunks::<3>().0 {
            let corner = |i: &u32| world.get(usize::try_from(*i).ok()?).copied();
            let (Some(a), Some(b), Some(c)) = (corner(i), corner(j), corner(k)) else {
                continue;
            };
            if let Some(t) = hit_triangle(ray, a, b, c)
                && best.is_none_or(|(bt, _)| t < bt)
            {
                best = Some((t, prim.prim_path.as_str()));
            }
        }
    }
    best.map(|(t, path)| (ray.at(t), path))
}

/// Möller-Trumbore: distance along `ray` to triangle `abc`, from either side.
fn hit_triangle(ray: &Ray, a: DVec3, b: DVec3, c: DVec3) -> Option<f64> {
    let (e1, e2) = (b - a, c - a);
    let p = ray.dir.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-12 {
        return None;
    }
    let s = ray.origin - a;
    let u = s.dot(p) / det;
    let q = s.cross(e1);
    let v = ray.dir.dot(q) / det;
    let t = e2.dot(q) / det;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t > 1e-6).then_some(t)
}

/// The point closest, in least squares, to every ray: a feature seen in two or more
/// photos. `None` when the rays are (nearly) parallel.
pub(crate) fn triangulate(rays: &[Ray]) -> Option<DVec3> {
    let mut a = glam::DMat3::ZERO;
    let mut b = DVec3::ZERO;
    for ray in rays {
        let d = ray.dir;
        // I - d dᵀ projects onto the plane normal to the ray.
        let m = glam::DMat3::IDENTITY - glam::DMat3::from_cols(d * d.x, d * d.y, d * d.z);
        a += m;
        b += m * ray.origin;
    }
    (rays.len() >= 2 && a.determinant().abs() > 1e-9).then(|| a.inverse() * b)
}
