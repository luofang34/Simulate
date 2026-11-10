//! Rasterising skinned triangles into the atlas: every texel a triangle
//! covers is asked of the skin's source at the point of the airframe it
//! shows, then written into the three maps. Texels within a fraction of a
//! texel outside a triangle are filled too, from its extrapolated surface,
//! so filtering at an island's edge never reaches an unpainted gutter.

use glam::{Vec2, Vec3, Vec4};

use super::map::{SkinFormat, SkinMap, linear_to_srgb_byte, unorm_byte};
use super::{SkinLayout, SkinSample, SkinSource, SkinTexel, SkinTextures};

/// How far outside a triangle, in texels, its surface is still painted.
const EDGE_REACH: f32 = 0.75;
const EMPTY: u8 = 0;
const EDGE: u8 = 1;
const INSIDE: u8 = 2;

/// One corner of a triangle: where it lands in the atlas, and the surface
/// there in the aircraft frame.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Corner {
    pub(crate) texel: Vec2,
    pub(crate) position: Vec3,
    pub(crate) normal: Vec3,
    /// Tangent and handedness, as the vertex carries them.
    pub(crate) tangent: Vec4,
    /// The vertex's baked ambient occlusion.
    pub(crate) occlusion: f32,
}

/// One triangle to paint: its corners, its part and the skin a texel covers.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Job {
    pub(crate) corners: [Corner; 3],
    pub(crate) part: usize,
    pub(crate) footprint_m: f32,
}

/// The three maps being painted, finest level only.
pub(crate) struct Canvas {
    side: u32,
    pub(crate) albedo: Vec<u8>,
    pub(crate) normal: Vec<u8>,
    pub(crate) orm: Vec<u8>,
    coverage: Vec<u8>,
}

/// A run of whole rows of the canvas, painted by one thread.
struct Band<'a> {
    side: u32,
    first_row: u32,
    albedo: &'a mut [u8],
    normal: &'a mut [u8],
    orm: &'a mut [u8],
    coverage: &'a mut [u8],
}

impl Canvas {
    pub(crate) fn new(side: u32) -> Self {
        let texels = side as usize * side as usize;
        Self {
            side,
            albedo: vec![0; texels * 4],
            normal: vec![128; texels * 2],
            orm: vec![0; texels * 4],
            coverage: vec![EMPTY; texels],
        }
    }

    /// Paints every job, splitting the canvas into bands of rows painted in
    /// parallel where threads exist; each band paints the jobs that reach
    /// its rows, so the result does not depend on the split.
    pub(crate) fn fill_all(&mut self, jobs: &[Job], source: &dyn SkinSource) {
        let side = self.side as usize;
        let threads = std::thread::available_parallelism()
            .map_or(1, |n| n.get())
            .clamp(1, 16);
        let rows = side.div_ceil(threads).max(1);
        let bands = self
            .albedo
            .chunks_mut(rows * side * 4)
            .zip(self.normal.chunks_mut(rows * side * 2))
            .zip(self.orm.chunks_mut(rows * side * 4))
            .zip(self.coverage.chunks_mut(rows * side))
            .enumerate()
            .map(|(i, (((albedo, normal), orm), coverage))| Band {
                side: self.side,
                first_row: u32::try_from(i * rows).unwrap_or(u32::MAX),
                albedo,
                normal,
                orm,
                coverage,
            });
        let paint = |mut band: Band<'_>| {
            for job in jobs {
                band.fill(job, source);
            }
        };
        if threads == 1 || cfg!(target_arch = "wasm32") {
            bands.for_each(paint);
            return;
        }
        std::thread::scope(|scope| {
            for band in bands {
                scope.spawn(move || paint(band));
            }
        });
    }

    /// The painted maps with their mips.
    pub(crate) fn into_textures(self, layout: SkinLayout) -> SkinTextures {
        let side = self.side;
        let orm = SkinMap::with_mips(SkinFormat::Rgba8, side, side, self.orm);
        SkinTextures {
            albedo: SkinMap::with_mips(SkinFormat::Srgba8, side, side, self.albedo),
            normal: SkinMap::with_mips(SkinFormat::Normal8, side, side, self.normal),
            orm: if layout.half_orm {
                orm.without_finest()
            } else {
                orm
            },
        }
    }

    /// Spreads every island's edge texels `passes` texels into its gutter,
    /// each empty texel taking the mean of its painted neighbours.
    pub(crate) fn dilate(&mut self, passes: u32) {
        let side = self.side as usize;
        for _ in 0..passes {
            let snapshot = self.coverage.clone();
            for (index, &cover) in snapshot.iter().enumerate() {
                if cover != EMPTY {
                    continue;
                }
                let (x, y) = (index % side, index / side);
                let mut neighbours = [0_usize; 4];
                let mut count = 0;
                let candidates = [
                    (x > 0).then(|| index - 1),
                    (x + 1 < side).then(|| index + 1),
                    (y > 0).then(|| index - side),
                    (y + 1 < side).then(|| index + side),
                ];
                for n in candidates.into_iter().flatten() {
                    if snapshot.get(n).is_some_and(|c| *c != EMPTY)
                        && let Some(slot) = neighbours.get_mut(count)
                    {
                        *slot = n;
                        count = count.wrapping_add(1);
                    }
                }
                let Some(painted) = neighbours.get(..count).filter(|n| !n.is_empty()) else {
                    continue;
                };
                for (buffer, bytes) in [
                    (&mut self.albedo, 4),
                    (&mut self.normal, 2),
                    (&mut self.orm, 4),
                ] {
                    for channel in 0..bytes {
                        let sum: u32 = painted
                            .iter()
                            .map(|n| {
                                u32::from(buffer.get(n * bytes + channel).copied().unwrap_or(0))
                            })
                            .sum();
                        if let Some(slot) = buffer.get_mut(index * bytes + channel) {
                            *slot = u8::try_from(sum / count as u32).unwrap_or(u8::MAX);
                        }
                    }
                }
                if let Some(slot) = self.coverage.get_mut(index) {
                    *slot = EDGE;
                }
            }
        }
    }
}

impl Band<'_> {
    /// Paints the texels of `job` in this band.
    fn fill(&mut self, job: &Job, source: &dyn SkinSource) {
        let corners = job.corners;
        let [a, b, c] = corners.map(|k| k.texel);
        let area = (b - a).perp_dot(c - a);
        if area.abs() < 1e-9 {
            return;
        }
        let edges = [(c - b).length(), (a - c).length(), (b - a).length()];
        let lo = a.min(b).min(c).floor() - Vec2::ONE;
        let hi = a.max(b).max(c).ceil() + Vec2::ONE;
        let max = self.side.saturating_sub(1) as f32;
        let rows = u32::try_from(self.coverage.len() / self.side.max(1) as usize).unwrap_or(0);
        let (top, bottom) = (self.first_row as f32, (self.first_row + rows) as f32 - 1.0);
        if rows == 0 || hi.y < top || lo.y > bottom {
            return;
        }
        let (x0, y0) = (lo.x.clamp(0.0, max) as u32, lo.y.clamp(top, bottom) as u32);
        let (x1, y1) = (hi.x.clamp(0.0, max) as u32, hi.y.clamp(top, bottom) as u32);
        for y in y0..=y1 {
            for x in x0..=x1 {
                let p = Vec2::new(x as f32 + 0.5, y as f32 + 0.5);
                let w = [
                    (c - b).perp_dot(p - b) / area,
                    (a - c).perp_dot(p - c) / area,
                    (b - a).perp_dot(p - a) / area,
                ];
                let reach = w
                    .iter()
                    .zip(edges)
                    .map(|(wi, e)| wi * area.abs() / e.max(1e-6))
                    .fold(f32::INFINITY, f32::min);
                let class = if w.iter().all(|wi| *wi >= 0.0) {
                    INSIDE
                } else if reach >= -EDGE_REACH {
                    EDGE
                } else {
                    continue;
                };
                let index = (y - self.first_row) as usize * self.side as usize + x as usize;
                if self.coverage.get(index).copied().unwrap_or(INSIDE) >= class {
                    continue;
                }
                let texel = interpolate(&corners, w, job.part, job.footprint_m);
                let sample = source.sample(&texel);
                self.write(index, &texel, &sample, class);
            }
        }
    }

    fn write(&mut self, index: usize, texel: &SkinTexel, sample: &SkinSample, class: u8) {
        let n = sample.normal.try_normalize().unwrap_or(texel.normal);
        let local = Vec3::new(
            n.dot(texel.tangent),
            n.dot(texel.bitangent),
            n.dot(texel.normal),
        );
        let local = Vec3::new(local.x, local.y, local.z.max(0.05)).normalize();
        let albedo = sample.albedo.to_array().map(linear_to_srgb_byte);
        if let Some(px) = self.albedo.get_mut(index * 4..index * 4 + 4) {
            px.copy_from_slice(&[albedo[0], albedo[1], albedo[2], 255]);
        }
        if let Some(px) = self.normal.get_mut(index * 2..index * 2 + 2) {
            px.copy_from_slice(&[
                unorm_byte(local.x * 0.5 + 0.5),
                unorm_byte(local.y * 0.5 + 0.5),
            ]);
        }
        if let Some(px) = self.orm.get_mut(index * 4..index * 4 + 4) {
            px.copy_from_slice(&[
                unorm_byte(sample.occlusion),
                unorm_byte(sample.roughness),
                unorm_byte(sample.metallic),
                255,
            ]);
        }
        if let Some(slot) = self.coverage.get_mut(index) {
            *slot = class;
        }
    }
}

/// The surface at barycentric weights `w` of `corners`.
fn interpolate(corners: &[Corner; 3], w: [f32; 3], part: usize, footprint_m: f32) -> SkinTexel {
    let mix3 =
        |f: &dyn Fn(&Corner) -> Vec3| corners.iter().zip(w).map(|(k, wi)| f(k) * wi).sum::<Vec3>();
    let normal = mix3(&|k| k.normal).try_normalize().unwrap_or(Vec3::Z);
    let tangent = mix3(&|k| k.tangent.truncate());
    let tangent = (tangent - normal * normal.dot(tangent))
        .try_normalize()
        .unwrap_or_else(|| normal.any_orthonormal_vector());
    let handed = if corners.iter().any(|k| k.tangent.w < 0.0) {
        -1.0
    } else {
        1.0
    };
    let occlusion = corners
        .iter()
        .zip(w)
        .map(|(k, wi)| k.occlusion * wi)
        .sum::<f32>();
    SkinTexel {
        part,
        occlusion: occlusion.clamp(0.0, 1.0),
        position: mix3(&|k| k.position),
        normal,
        tangent,
        bitangent: normal.cross(tangent) * handed,
        footprint_m,
    }
}
