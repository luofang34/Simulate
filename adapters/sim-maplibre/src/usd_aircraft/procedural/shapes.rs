//! Triangle-mesh primitives for the procedural built-in vehicles.
//!
//! Every triangle is wound against an outward reference (a solid's interior point or the
//! loft axis), so single-sided back-face culling in the aircraft pipelines never hides an
//! outer surface regardless of the order in which a primitive generates its corners.

use glam::{Mat4, Vec2, Vec3};

use crate::UsdMeshVertex;

/// Accumulates vertices and triangle indices for one part's mesh.
#[derive(Clone, Debug, Default)]
pub(in crate::usd_aircraft) struct MeshBuilder {
    vertices: Vec<UsdMeshVertex>,
    indices: Vec<u32>,
}

/// Interior reference used to decide which side of a lofted surface faces outward.
#[derive(Clone, Copy, Debug)]
pub(in crate::usd_aircraft) enum Inside {
    /// The segment between consecutive ring centroids (open tubes, fuselages, blades).
    Axis,
    /// One interior point of a convex solid (ellipsoids, domes).
    Point(Vec3),
}

impl MeshBuilder {
    pub(in crate::usd_aircraft) fn new() -> Self {
        Self::default()
    }

    pub(in crate::usd_aircraft) fn into_buffers(self) -> (Vec<UsdMeshVertex>, Vec<u32>) {
        (self.vertices, self.indices)
    }

    fn push_vertex(&mut self, position: Vec3, normal: Vec3, uv: Vec2) -> u32 {
        let index = u32::try_from(self.vertices.len()).unwrap_or(u32::MAX);
        self.vertices.push(UsdMeshVertex {
            position: position.to_array(),
            normal: normal.normalize_or(Vec3::Z).to_array(),
            uv: uv.to_array(),
            occlusion: 1.0,
            tangent: crate::UsdMeshVertex::NO_TANGENT,
        });
        index
    }

    fn position(&self, index: u32) -> Vec3 {
        self.vertices
            .get(index as usize)
            .map_or(Vec3::ZERO, |v| Vec3::from_array(v.position))
    }

    /// Appends a triangle wound so its geometric normal agrees with `outward`; returns the
    /// oriented (unnormalized) face normal, or `None` for a degenerate triangle.
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

    /// Flat-shaded convex polygon (fan triangulated) facing `outward`.
    pub(in crate::usd_aircraft) fn polygon(&mut self, points: &[Vec3], outward: Vec3) {
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

    /// Flat-shaded axis-aligned box.
    pub(in crate::usd_aircraft) fn cuboid(&mut self, min: Vec3, max: Vec3) {
        let centre = (min + max) * 0.5;
        let corner = |x: bool, y: bool, z: bool| {
            Vec3::new(
                if x { max.x } else { min.x },
                if y { max.y } else { min.y },
                if z { max.z } else { min.z },
            )
        };
        for axis in 0..3_u8 {
            for high in [false, true] {
                let face: [Vec3; 4] = match axis {
                    0 => [(false, false), (true, false), (true, true), (false, true)]
                        .map(|(a, b)| corner(high, a, b)),
                    1 => [(false, false), (true, false), (true, true), (false, true)]
                        .map(|(a, b)| corner(a, high, b)),
                    _ => [(false, false), (true, false), (true, true), (false, true)]
                        .map(|(a, b)| corner(a, b, high)),
                };
                let face_centre = face.iter().copied().sum::<Vec3>() * 0.25;
                self.polygon(&face, face_centre - centre);
            }
        }
    }

    /// Smooth-shaded surface through closed rings of equal length, optionally capped flat.
    pub(super) fn loft(&mut self, rings: &[Vec<Vec3>], caps: bool, inside: Inside) {
        let Some(count) = rings.first().map(Vec::len).filter(|n| *n >= 3) else {
            return;
        };
        if rings.iter().any(|ring| ring.len() != count) {
            return;
        }
        let base = self.vertices.len();
        for ring in rings {
            // Normals start at zero and accumulate the adjacent face normals below.
            self.vertices.extend(ring.iter().map(|point| UsdMeshVertex {
                position: point.to_array(),
                normal: [0.0; 3],
                uv: [0.0; 2],
                occlusion: 1.0,
                tangent: crate::UsdMeshVertex::NO_TANGENT,
            }));
        }
        let index = |ring: usize, j: usize| {
            u32::try_from(
                base.saturating_add(ring.saturating_mul(count))
                    .saturating_add(j % count),
            )
            .unwrap_or(u32::MAX)
        };
        let centroids: Vec<Vec3> = rings.iter().map(|r| centroid(r)).collect();
        let mut triangles = Vec::with_capacity(rings.len().saturating_mul(count.saturating_mul(2)));
        let mut vote = 0.0_f32;
        for (i, pair) in centroids.windows(2).enumerate() {
            let reference = match inside {
                Inside::Axis => pair.iter().copied().sum::<Vec3>() * 0.5,
                Inside::Point(point) => point,
            };
            for j in 0..count {
                let [q0, q1, q2, q3] = [
                    index(i, j),
                    index(i, j.saturating_add(1)),
                    index(i.saturating_add(1), j.saturating_add(1)),
                    index(i.saturating_add(1), j),
                ];
                for tri in [[q0, q1, q2], [q0, q2, q3]] {
                    let [a, b, c] = tri.map(|q| self.position(q));
                    vote += (b - a).cross(c - a).dot((a + b + c) / 3.0 - reference);
                    triangles.push(tri);
                }
            }
        }
        // Ring traversal is topologically consistent, so one orientation decided by the
        // whole surface is robust where thin edges make per-triangle tests ambiguous.
        for [a, b, c] in triangles {
            let tri = if vote < 0.0 { [a, c, b] } else { [a, b, c] };
            let [pa, pb, pc] = tri.map(|q| self.position(q));
            let normal = (pb - pa).cross(pc - pa);
            if normal.length_squared() >= 1e-14 {
                self.indices.extend_from_slice(&tri);
                self.accumulate_normal(tri, normal);
            }
        }
        for vertex in self.vertices.iter_mut().skip(base) {
            let n = Vec3::from_array(vertex.normal).normalize_or(Vec3::Z);
            vertex.normal = n.to_array();
        }
        if caps {
            self.loft_caps(rings, &centroids);
        }
    }

    /// Flat-shaded (one normal per quad) surface through closed rings, capped at both ends.
    pub(super) fn loft_faceted(&mut self, rings: &[Vec<Vec3>]) {
        let centroids: Vec<Vec3> = rings.iter().map(|r| centroid(r)).collect();
        for (pair, centres) in rings.windows(2).zip(centroids.windows(2)) {
            let (Some(lower), Some(upper)) = (pair.first(), pair.get(1)) else {
                continue;
            };
            let axis_point = centres.iter().copied().sum::<Vec3>() * 0.5;
            let count = lower.len().min(upper.len());
            for j in 0..count {
                let next = j.saturating_add(1) % count;
                let quad = [lower.get(j), lower.get(next), upper.get(next), upper.get(j)];
                let [Some(a), Some(b), Some(c), Some(d)] = quad else {
                    continue;
                };
                let corners = [*a, *b, *c, *d];
                let quad_centre = corners.iter().copied().sum::<Vec3>() * 0.25;
                self.polygon(&corners, quad_centre - axis_point);
            }
        }
        self.loft_caps(rings, &centroids);
    }

    fn accumulate_normal(&mut self, tri: [u32; 3], normal: Vec3) {
        for index in tri {
            if let Some(vertex) = self.vertices.get_mut(index as usize) {
                vertex.normal = (Vec3::from_array(vertex.normal) + normal).to_array();
            }
        }
    }

    fn loft_caps(&mut self, rings: &[Vec<Vec3>], centroids: &[Vec3]) {
        let ends = [
            (rings.first(), centroids.first(), centroids.get(1)),
            (
                rings.last(),
                centroids.last(),
                centroids.iter().rev().nth(1),
            ),
        ];
        for (ring, end, neighbour) in ends {
            if let (Some(ring), Some(end), Some(neighbour)) = (ring, end, neighbour) {
                self.polygon(ring, *end - *neighbour);
            }
        }
    }

    /// Smooth ellipsoid centred on `centre` with semi-axes `radii`.
    pub(in crate::usd_aircraft) fn ellipsoid(
        &mut self,
        centre: Vec3,
        radii: Vec3,
        segments: u32,
        stacks: u32,
    ) {
        let rings: Vec<Vec<Vec3>> = (0..=stacks.max(2))
            .map(|k| {
                let phi = -std::f32::consts::FRAC_PI_2
                    + std::f32::consts::PI * k as f32 / stacks.max(2) as f32;
                let (sin, cos) = phi.sin_cos();
                ring(
                    centre + Vec3::Z * radii.z * sin,
                    Vec3::X * radii.x * cos,
                    Vec3::Y * radii.y * cos,
                    segments,
                )
            })
            .collect();
        self.loft(&rings, false, Inside::Point(centre));
    }

    /// Capped circular frustum from `a` (radius `ra`) to `b` (radius `rb`).
    pub(in crate::usd_aircraft) fn tube(
        &mut self,
        a: Vec3,
        b: Vec3,
        ra: f32,
        rb: f32,
        segments: u32,
    ) {
        let (u, v) = (b - a).normalize_or(Vec3::Z).any_orthonormal_pair();
        let rings = [
            ring(a, u * ra, v * ra, segments),
            ring(b, u * rb, v * rb, segments),
        ];
        self.loft(&rings, true, Inside::Axis);
    }

    /// Double-sided disc in the local `XZ` plane (normal `+Y`) whose texture coordinates
    /// span the unit disc about the hub, as the propeller-disc shader expects.
    pub(super) fn disc_xz(&mut self, radius: f32, segments: u32) {
        let hub = self.push_vertex(Vec3::ZERO, Vec3::Y, Vec2::ZERO);
        let rim: Vec<u32> = (0..segments.max(3))
            .map(|k| {
                let angle = std::f32::consts::TAU * k as f32 / segments.max(3) as f32;
                let (s, c) = angle.sin_cos();
                self.push_vertex(Vec3::new(c, 0.0, s) * radius, Vec3::Y, Vec2::new(c, s))
            })
            .collect();
        for (k, &current) in rim.iter().enumerate() {
            let next = rim
                .get(k.saturating_add(1))
                .or(rim.first())
                .copied()
                .unwrap_or(current);
            self.push_oriented([hub, current, next], Vec3::Y);
        }
    }

    /// Appends `other` transformed by the rigid `transform`.
    pub(in crate::usd_aircraft) fn append(&mut self, other: MeshBuilder, transform: Mat4) {
        let base = u32::try_from(self.vertices.len()).unwrap_or(u32::MAX);
        self.vertices.extend(other.vertices.into_iter().map(|v| {
            UsdMeshVertex {
                position: transform
                    .transform_point3(Vec3::from_array(v.position))
                    .to_array(),
                normal: transform
                    .transform_vector3(Vec3::from_array(v.normal))
                    .normalize_or(Vec3::Z)
                    .to_array(),
                uv: v.uv,
                occlusion: v.occlusion,
                tangent: v.tangent,
            }
        }));
        self.indices
            .extend(other.indices.into_iter().map(|i| i.saturating_add(base)));
    }
}

/// Closed ring of `segments` points `centre + u cos θ + v sin θ`.
pub(super) fn ring(centre: Vec3, u: Vec3, v: Vec3, segments: u32) -> Vec<Vec3> {
    (0..segments.max(3))
        .map(|k| {
            let angle = std::f32::consts::TAU * k as f32 / segments.max(3) as f32;
            let (s, c) = angle.sin_cos();
            centre + u * c + v * s
        })
        .collect()
}

/// Thin lens-shaped aerofoil section in the plane spanned by `chord` and `thickness`
/// directions, leading edge at `leading`.
pub(super) fn aerofoil(leading: Vec3, chord: Vec3, thickness: Vec3) -> Vec<Vec3> {
    const STATIONS: [(f32, f32); 6] = [
        (0.0, 0.0),
        (0.08, 0.62),
        (0.30, 1.0),
        (0.62, 0.72),
        (0.88, 0.30),
        (1.0, 0.0),
    ];
    let upper = STATIONS
        .iter()
        .map(|&(s, t)| leading + chord * s + thickness * (0.5 * t));
    let lower = STATIONS
        .iter()
        .rev()
        .skip(1)
        .take(STATIONS.len().saturating_sub(2))
        .map(|&(s, t)| leading + chord * s - thickness * (0.5 * t));
    upper.chain(lower).collect()
}

fn centroid(points: &[Vec3]) -> Vec3 {
    points.iter().copied().sum::<Vec3>() / points.len().max(1) as f32
}
