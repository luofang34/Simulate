//! Self-contained mesh builder and OpenUSD `UsdGeomMesh` + `UsdPreviewSurface` descriptors
//! for the standalone `sim-p38g` vehicle crate.

use std::f32::consts::TAU;

use glam::{DMat4, DVec3, Vec2, Vec3};

pub use crate::role::P38PartRole;

/// Needle hands one instrument layout slot may carry; channels run
/// `slot * NEEDLE_HANDS_PER_SLOT + hand`.
pub const NEEDLE_HANDS_PER_SLOT: usize = 3;

/// PBR `UsdPreviewSurface` material parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct P38Material {
    /// Linear base reflectance RGB in `[0, 1]`.
    pub diffuse_rgb: [f32; 3],
    /// Linear emissive radiance RGB.
    pub emissive_rgb: [f32; 3],
    /// Metallic factor in `[0, 1]`.
    pub metallic: f32,
    /// Perceptual roughness in `[0, 1]`.
    pub roughness: f32,
    /// Rest opacity in `[0, 1]`.
    pub opacity: f32,
}

pub(crate) const fn mat(diffuse: [f32; 3], metallic: f32, roughness: f32) -> P38Material {
    P38Material {
        diffuse_rgb: diffuse,
        emissive_rgb: [0.0, 0.0, 0.0],
        metallic,
        roughness,
        opacity: 1.0,
    }
}

pub(crate) const fn glowing(diffuse: [f32; 3], emissive: [f32; 3]) -> P38Material {
    P38Material {
        diffuse_rgb: diffuse,
        emissive_rgb: emissive,
        metallic: 0.0,
        roughness: 0.2,
        opacity: 1.0,
    }
}

/// Interleaved vertex (`position`, `normal`, `uv`) in prim-local metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct P38MeshVertex {
    /// Prim-local position `(x_right, y_fwd, z_up)` in metres.
    pub position: [f32; 3],
    /// Prim-local unit normal `(nx, ny, nz)`.
    pub normal: [f32; 3],
    /// Texture / polar coordinate `(u, v)`.
    pub uv: [f32; 2],
}

/// One articulated or fixed mesh prim in the P-38G OpenUSD stage.
#[derive(Clone, Debug)]
pub struct P38MeshPrim {
    /// Full USD prim path (e.g. `/P38g/Booms/UpperOlive`).
    pub prim_path: String,
    /// Semantic articulation role.
    pub role: P38PartRole,
    /// PBR surface material.
    pub material: P38Material,
    /// Whether backfaces should be rendered.
    pub double_sided: bool,
    /// Rest transform from mesh-local space to aircraft root (`+X` right, `+Y` fwd, `+Z` up).
    pub rest_to_aircraft: DMat4,
    /// Optional custom hinge axis (`sim:hingeAxis`).
    pub custom_hinge_axis: Option<DVec3>,
    /// The panel needle channel an [`P38PartRole::InstrumentNeedle`] follows
    /// (`sim:needleChannel`): layout slot times [`NEEDLE_HANDS_PER_SLOT`] plus hand.
    pub needle_channel: Option<usize>,
    /// Triangulated vertex buffer.
    pub vertices: Vec<P38MeshVertex>,
    /// Triangle index buffer (`0..vertices.len()`).
    pub indices: Vec<u32>,
}

pub(crate) fn part(
    path: &str,
    role: P38PartRole,
    material: P38Material,
    rest: DMat4,
    mesh: MeshBuilder,
) -> P38MeshPrim {
    let (vertices, indices) = mesh.into_buffers();
    P38MeshPrim {
        prim_path: path.to_owned(),
        role,
        material,
        double_sided: false,
        rest_to_aircraft: rest,
        custom_hinge_axis: None,
        needle_channel: None,
        vertices,
        indices,
    }
}

pub(crate) fn fixed(
    path: &str,
    role: P38PartRole,
    material: P38Material,
    mesh: MeshBuilder,
) -> P38MeshPrim {
    part(path, role, material, DMat4::IDENTITY, mesh)
}

#[derive(Clone, Copy)]
pub(crate) enum Inside {
    Axis,
    Point(Vec3),
}

#[derive(Clone, Default)]
pub(crate) struct MeshBuilder {
    vertices: Vec<P38MeshVertex>,
    indices: Vec<u32>,
}

impl MeshBuilder {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn into_buffers(self) -> (Vec<P38MeshVertex>, Vec<u32>) {
        (self.vertices, self.indices)
    }

    fn push_vertex(&mut self, position: Vec3, normal: Vec3, uv: Vec2) -> u32 {
        let index = u32::try_from(self.vertices.len()).unwrap_or(0);
        self.vertices.push(P38MeshVertex {
            position: position.to_array(),
            normal: normal.normalize_or(Vec3::Z).to_array(),
            uv: uv.to_array(),
        });
        index
    }

    fn position(&self, index: u32) -> Vec3 {
        self.vertices
            .get(index as usize)
            .map_or(Vec3::ZERO, |v| Vec3::from_array(v.position))
    }

    fn push_oriented(&mut self, [a, b, c]: [u32; 3], outward: Vec3) -> Option<Vec3> {
        let (pa, pb, pc) = (self.position(a), self.position(b), self.position(c));
        let normal = (pb - pa).cross(pc - pa);
        if normal.length_squared() < 1e-14 {
            return None;
        }
        if normal.dot(outward) < 0.0 {
            self.indices.extend_from_slice(&[a, c, b]);
            Some(-normal)
        } else {
            self.indices.extend_from_slice(&[a, b, c]);
            Some(normal)
        }
    }

    pub(crate) fn polygon(&mut self, points: &[Vec3], outward: Vec3) {
        let mut corners = points.iter().copied();
        let (Some(p0), Some(p1), Some(p2)) = (corners.next(), corners.next(), corners.next())
        else {
            return;
        };
        let mut normal = (p1 - p0).cross(p2 - p0);
        if normal.dot(outward) < 0.0 {
            normal = -normal;
        }
        let first = self.push_vertex(p0, normal, Vec2::ZERO);
        let mut previous = self.push_vertex(p1, normal, Vec2::ZERO);
        for point in points.iter().skip(2) {
            let current = self.push_vertex(*point, normal, Vec2::ZERO);
            self.push_oriented([first, previous, current], outward);
            previous = current;
        }
    }

    /// Lofts an open sheet through `rows` of equal length. Texture
    /// coordinates are metres on the skin: `u` along each column (across the
    /// rows: station or span), `v` along each row (girth or chord).
    pub(crate) fn loft_open(&mut self, rows: &[Vec<Vec3>], inside: Inside) {
        let Some(cols) = rows.first().map(Vec::len) else {
            return;
        };
        if rows.len() < 2 || cols < 2 || rows.iter().any(|r| r.len() != cols) {
            return;
        }
        let uvs = sheet_uvs(rows);
        self.loft_grid(rows, &uvs, false, inside);
    }

    /// Lofts a closed tube through `rings` of equal length, optionally
    /// capping both ends. Texture coordinates are metres as for
    /// [`Self::loft_open`], `v` running once round each ring; the first
    /// column is repeated at the end so the seam does not wrap.
    pub(crate) fn loft(&mut self, rings: &[Vec<Vec3>], cap_ends: bool, inside: Inside) {
        let Some(ring_len) = rings.first().map(Vec::len) else {
            return;
        };
        if rings.len() < 2 || ring_len < 3 || rings.iter().any(|r| r.len() != ring_len) {
            return;
        }
        let closed: Vec<Vec<Vec3>> = rings
            .iter()
            .map(|ring| ring.iter().chain(ring.first()).copied().collect())
            .collect();
        let uvs = sheet_uvs(&closed);
        self.loft_grid(&closed, &uvs, true, inside);
        if cap_ends
            && let (Some(first), Some(last)) = (rings.first(), rings.last())
            && let (Some(c0), Some(c1)) = (row_centroid(Some(first)), row_centroid(Some(last)))
        {
            let axis = (c1 - c0).normalize_or(Vec3::Y);
            self.polygon(first, -axis);
            self.polygon(last, axis);
        }
    }

    /// Triangulates the grid `rows` x columns with smooth normals; when
    /// `seam` is set the last column repeats the first and shares its normal.
    fn loft_grid(&mut self, rows: &[Vec<Vec3>], uvs: &[Vec<Vec2>], seam: bool, inside: Inside) {
        let cols = rows.first().map_or(0, Vec::len);
        let base = u32::try_from(self.vertices.len()).unwrap_or(0);
        for (row, row_uv) in rows.iter().zip(uvs) {
            for (&point, &uv) in row.iter().zip(row_uv) {
                self.push_vertex(point, Vec3::Z, uv);
            }
        }
        let mut normals = vec![Vec3::ZERO; rows.len() * cols];
        let cols_u32 = u32::try_from(cols).unwrap_or(0);
        // The seam's two copies are one point of the surface.
        let canonical = |c: u32| if seam && c + 1 == cols_u32 { 0 } else { c };
        for r in 0..rows.len().saturating_sub(1) {
            let r_u32 = u32::try_from(r).unwrap_or(0);
            // A ring's centroid leaves out the seam's repeated point.
            let distinct = if seam { cols.saturating_sub(1) } else { cols };
            let centroid_of = |row: Option<&Vec<Vec3>>| {
                row.map(|row| row.iter().take(distinct).copied().collect::<Vec<_>>())
            };
            let centroid = match inside {
                Inside::Axis => row_centroid(centroid_of(rows.get(r)).as_ref())
                    .zip(row_centroid(
                        centroid_of(rows.get(r.wrapping_add(1))).as_ref(),
                    ))
                    .map_or(Vec3::ZERO, |(a, b)| (a + b) * 0.5),
                Inside::Point(p) => p,
            };
            for i in 0..cols_u32.saturating_sub(1) {
                let a = base + r_u32 * cols_u32 + i;
                let b = a + 1;
                let c = base + (r_u32 + 1) * cols_u32 + i;
                let d = c + 1;
                let quad_mid =
                    (self.position(a) + self.position(b) + self.position(c) + self.position(d))
                        * 0.25;
                let out = quad_mid - centroid;
                for tri in [[a, b, d], [a, d, c]] {
                    if let Some(fnrm) = self.push_oriented(tri, out) {
                        for v in tri {
                            let offset = v - base;
                            let key = offset - offset % cols_u32 + canonical(offset % cols_u32);
                            if let Some(slot) = normals.get_mut(key as usize) {
                                *slot += fnrm;
                            }
                        }
                    }
                }
            }
        }
        for offset in 0..normals.len() {
            let offset_u32 = u32::try_from(offset).unwrap_or(0);
            let key =
                offset_u32 - offset_u32 % cols_u32.max(1) + canonical(offset_u32 % cols_u32.max(1));
            let n = normals.get(key as usize).copied().unwrap_or(Vec3::Z);
            if let Some(v) = self.vertices.get_mut(base as usize + offset) {
                v.normal = n.normalize_or(Vec3::Z).to_array();
            }
        }
    }

    pub(crate) fn tube(&mut self, start: Vec3, end: Vec3, r_start: f32, r_end: f32, sides: u32) {
        let axis = end - start;
        let Some(dir) = axis.try_normalize() else {
            return;
        };
        let (u, v) = orthonormal_basis(dir);
        let ring = |centre: Vec3, radius: f32| -> Vec<Vec3> {
            (0..sides.max(3))
                .map(|i| {
                    let a = TAU * i as f32 / sides.max(3) as f32;
                    centre + (u * a.cos() + v * a.sin()) * radius
                })
                .collect()
        };
        self.loft(
            &[ring(start, r_start), ring(end, r_end)],
            true,
            Inside::Axis,
        );
    }

    pub(crate) fn ellipsoid(&mut self, centre: Vec3, radii: Vec3, slices: u32, stacks: u32) {
        let slices = slices.max(4);
        let stacks = stacks.max(3);
        let eps = 0.02_f32;
        let rings: Vec<Vec<Vec3>> = (0..=stacks)
            .map(|st| {
                let t = eps + (1.0 - 2.0 * eps) * (st as f32 / stacks as f32);
                let phi = std::f32::consts::PI * (t - 0.5);
                let (sin_p, cos_p) = phi.sin_cos();
                (0..slices)
                    .map(|sl| {
                        let theta = TAU * sl as f32 / slices as f32;
                        let (sin_t, cos_t) = theta.sin_cos();
                        centre + Vec3::new(cos_p * cos_t, sin_p, cos_p * sin_t) * radii
                    })
                    .collect()
            })
            .collect();
        self.loft(&rings, true, Inside::Point(centre));
    }

    pub(crate) fn cuboid(&mut self, min: Vec3, max: Vec3) {
        let [x0, y0, z0] = min.to_array();
        let [x1, y1, z1] = max.to_array();
        let p = [
            Vec3::new(x0, y0, z0),
            Vec3::new(x1, y0, z0),
            Vec3::new(x1, y1, z0),
            Vec3::new(x0, y1, z0),
            Vec3::new(x0, y0, z1),
            Vec3::new(x1, y0, z1),
            Vec3::new(x1, y1, z1),
            Vec3::new(x0, y1, z1),
        ];
        for ([a, b, c, d], n) in [
            ([1, 2, 6, 5], Vec3::X),
            ([3, 0, 4, 7], Vec3::NEG_X),
            ([2, 3, 7, 6], Vec3::Y),
            ([0, 1, 5, 4], Vec3::NEG_Y),
            ([4, 5, 6, 7], Vec3::Z),
            ([3, 2, 1, 0], Vec3::NEG_Z),
        ] {
            if let (Some(&va), Some(&vb), Some(&vc), Some(&vd)) =
                (p.get(a), p.get(b), p.get(c), p.get(d))
            {
                self.polygon(&[va, vb, vc, vd], n);
            }
        }
    }

    pub(crate) fn disc_xz(&mut self, radius: f32, segments: u32) {
        let segs = segments.max(6);
        let centre = self.push_vertex(Vec3::ZERO, Vec3::NEG_Y, Vec2::ZERO);
        let rim_base = u32::try_from(self.vertices.len()).unwrap_or(0);
        for i in 0..segs {
            let a = TAU * i as f32 / segs as f32;
            let (s, c) = a.sin_cos();
            self.push_vertex(
                Vec3::new(c * radius, 0.0, s * radius),
                Vec3::NEG_Y,
                // The unit disc about the hub, which the propeller blur is drawn in.
                Vec2::new(c, s),
            );
        }
        for i in 0..segs {
            let cur = rim_base + i;
            let next = rim_base + ((i + 1) % segs);
            self.push_oriented([centre, cur, next], Vec3::NEG_Y);
        }
    }
}

/// Segments for a full circle of radius `radius_m` whose chords sag at most
/// [`SAGITTA_M`] from the arc, so large curves are not faceted and small ones not wasted.
pub(crate) fn circle_segments(radius_m: f32) -> u32 {
    let half_angle = (1.0 - SAGITTA_M / radius_m.max(SAGITTA_M))
        .clamp(-1.0, 1.0)
        .acos();
    ((std::f32::consts::PI / half_angle.max(1e-3)).ceil() as u32).clamp(12, 96)
}

/// Largest gap between a curved surface and the flat facets standing in for it.
pub(crate) const SAGITTA_M: f32 = 0.001;

pub(crate) fn aerofoil(leading: Vec3, chord_vec: Vec3, half_thickness: Vec3) -> Vec<Vec3> {
    const SAMPLES: usize = 32;
    let thickness = |u: f32| -> f32 {
        let u = u.clamp(0.0, 1.0);
        5.0 * (0.2969 * u.sqrt() - 0.1260 * u - 0.3516 * u * u + 0.2843 * u.powi(3)
            - 0.1015 * u.powi(4))
    };
    (0..SAMPLES)
        .map(|i| {
            let a = TAU * i as f32 / SAMPLES as f32;
            let u = 0.5 * (1.0 - a.cos());
            let side = a.sin().signum();
            leading + chord_vec * u + half_thickness * (side * thickness(u))
        })
        .collect()
}

/// Metric texture coordinates of a grid of points: `u` the distance along
/// each column from the first row, `v` the distance along each row from its
/// first point. A closed grid's last column repeats its first, so `v` ends
/// at the ring's girth.
fn sheet_uvs(rows: &[Vec<Vec3>]) -> Vec<Vec<Vec2>> {
    let cols = rows.first().map_or(0, Vec::len);
    let mut along = vec![0.0_f32; cols];
    let mut previous: Option<&Vec<Vec3>> = None;
    rows.iter()
        .map(|row| {
            if let Some(prev) = previous {
                for ((u, a), b) in along.iter_mut().zip(prev).zip(row) {
                    *u += a.distance(*b);
                }
            }
            previous = Some(row);
            let mut girth = 0.0_f32;
            let mut last = row.first().copied().unwrap_or(Vec3::ZERO);
            row.iter()
                .zip(&along)
                .map(|(&p, &u)| {
                    girth += p.distance(last);
                    last = p;
                    Vec2::new(u, girth)
                })
                .collect()
        })
        .collect()
}

fn row_centroid(ring: Option<&Vec<Vec3>>) -> Option<Vec3> {
    let ring = ring?;
    if ring.is_empty() {
        return None;
    }
    Some(ring.iter().copied().sum::<Vec3>() / ring.len() as f32)
}

fn orthonormal_basis(axis: Vec3) -> (Vec3, Vec3) {
    let seed = if axis.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
    let u = axis.cross(seed).normalize_or(Vec3::X);
    let v = axis.cross(u).normalize_or(Vec3::Y);
    (u, v)
}
