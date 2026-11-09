//! Camera resection: the pinhole pose and focal length that best project known 3D
//! landmarks onto their measured pixels, by Levenberg-Marquardt from a spread of
//! starting viewpoints around the airframe.

use glam::{DMat3, DQuat, DVec3};

/// A pinhole camera in mesh axes. Optical axes: `x` right, `y` down, `z` forward.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PinholePose {
    /// Rotates mesh-axis vectors into optical axes.
    pub(crate) mesh_to_optical: DQuat,
    /// Camera centre, mesh axes, metres.
    pub(crate) eye: DVec3,
    /// Focal length, pixels.
    pub(crate) focal_px: f64,
    /// Principal point, pixels from the top-left corner.
    pub(crate) principal_px: [f64; 2],
}

impl PinholePose {
    /// Pixel of `point`, `None` behind the camera.
    pub(crate) fn project(&self, point: DVec3) -> Option<[f64; 2]> {
        let c = self.mesh_to_optical * (point - self.eye);
        (c.z > 1e-3).then(|| {
            [
                self.principal_px[0] + self.focal_px * c.x / c.z,
                self.principal_px[1] + self.focal_px * c.y / c.z,
            ]
        })
    }

    /// Vertical field of view of an image `height_px` tall, radians.
    pub(crate) fn fov_y_rad(&self, height_px: f64) -> f64 {
        2.0 * (0.5 * height_px / self.focal_px).atan()
    }
}

/// The solved camera and how well it fits.
#[derive(Clone, Debug)]
pub(crate) struct Resection {
    pub(crate) pose: PinholePose,
    /// Root-mean-square reprojection error, pixels.
    pub(crate) rms_px: f64,
    /// Per-landmark reprojection error, pixels.
    pub(crate) residual_px: Vec<f64>,
}

/// Pixel penalty for a landmark behind the camera.
const BEHIND_PENALTY_PX: f64 = 1.0e4;
const ITERATIONS: usize = 80;

/// Solves for the camera seeing `correspondences` (`mesh point`, `pixel`) in an image
/// `size_px` from outside the airframe, with the principal point free when
/// `free_principal` and at the centre otherwise. Needs four landmarks more than the
/// unknowns.
pub(crate) fn resect(
    correspondences: &[(DVec3, [f64; 2])],
    size_px: [f64; 2],
    free_principal: bool,
) -> Option<Resection> {
    let count = correspondences.len().max(1) as f64;
    let centre = correspondences.iter().map(|(p, _)| *p).sum::<DVec3>() / count;
    resect_from(correspondences, free_principal, &starts(centre, size_px))
}

/// [`resect`] refined from `starts`, such as [`cockpit_starts`] for a photograph taken
/// from the seat.
pub(crate) fn resect_from(
    correspondences: &[(DVec3, [f64; 2])],
    free_principal: bool,
    starts: &[PinholePose],
) -> Option<Resection> {
    let unknowns = if free_principal { 9 } else { 7 };
    if correspondences.len() < unknowns / 2 + 2 {
        return None;
    }
    let mut best: Option<Resection> = None;
    for start in starts {
        let solved = refine(*start, correspondences, free_principal);
        if best.as_ref().is_none_or(|b| solved.rms_px < b.rms_px) {
            best = Some(solved);
        }
    }
    best
}

/// Half-extent of the box a seated photographer's head moves in, mesh axes, metres:
/// leaning sideways, forward and back, and rising or slumping.
const HEAD_BOX_M: DVec3 = DVec3::new(0.20, 0.25, 0.12);

/// Viewpoints for a photograph taken from the seat: eyes spread through the head box
/// about `design_eye`, each looking ahead and to either side and down toward the panel,
/// at wide to normal focal lengths.
pub(crate) fn cockpit_starts(design_eye: DVec3, [w, h]: [f64; 2]) -> Vec<PinholePose> {
    let mut poses = Vec::new();
    let steps = [-1.0, 0.0, 1.0];
    let offsets: Vec<DVec3> = steps
        .iter()
        .flat_map(|&x| steps.iter().map(move |&y| (x, y)))
        .flat_map(|(x, y)| steps.iter().map(move |&z| DVec3::new(x, y, z)))
        .collect();
    for offset in offsets {
        let eye = design_eye + HEAD_BOX_M * offset;
        for yaw_deg in [-40.0_f64, 0.0, 40.0] {
            for pitch_deg in [-35.0_f64, -12.0, 5.0] {
                let (yaw, pitch) = (yaw_deg.to_radians(), pitch_deg.to_radians());
                let dir = DVec3::new(
                    yaw.sin() * pitch.cos(),
                    yaw.cos() * pitch.cos(),
                    pitch.sin(),
                );
                for focal_scale in [0.45, 0.8, 1.4] {
                    let focal_px = focal_scale * w.max(h);
                    poses.push(look_at(eye, eye + dir, focal_px, [w, h]));
                }
            }
        }
    }
    poses
}

/// Viewpoints all round the airframe at three ranges, looking at its centre.
fn starts(centre: DVec3, [w, h]: [f64; 2]) -> Vec<PinholePose> {
    let mut poses = Vec::new();
    for focal_scale in [0.9, 1.8, 3.6] {
        let focal_px = focal_scale * w.max(h);
        // A 16 m airframe across about two thirds of the frame width.
        let range = focal_px * 16.0 / (0.66 * w);
        for az in 0..12 {
            for el in [-50.0_f64, -20.0, 5.0, 25.0, 55.0, 80.0] {
                let az = f64::from(az) * 30.0_f64.to_radians();
                let el = el.to_radians();
                let dir = DVec3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin());
                poses.push(look_at(centre + dir * range, centre, focal_px, [w, h]));
            }
        }
    }
    poses
}

fn look_at(eye: DVec3, target: DVec3, focal_px: f64, [w, h]: [f64; 2]) -> PinholePose {
    let forward = (target - eye).normalize();
    let up = if forward.z.abs() > 0.99 {
        DVec3::Y
    } else {
        DVec3::Z
    };
    let right = forward.cross(up).normalize();
    let down = forward.cross(right);
    let optical_to_mesh = DMat3::from_cols(right, down, forward);
    PinholePose {
        mesh_to_optical: DQuat::from_mat3(&optical_to_mesh.transpose()).normalize(),
        eye,
        focal_px,
        principal_px: [0.5 * w, 0.5 * h],
    }
}

fn residuals(pose: &PinholePose, data: &[(DVec3, [f64; 2])]) -> Vec<f64> {
    data.iter()
        .flat_map(|(point, [u, v])| match pose.project(*point) {
            Some([pu, pv]) => [pu - u, pv - v],
            None => [BEHIND_PENALTY_PX, BEHIND_PENALTY_PX],
        })
        .collect()
}

/// `pose` moved by `delta`: rotation vector (rad), eye (m), log focal, principal (px).
fn nudged(pose: &PinholePose, delta: &[f64]) -> PinholePose {
    let at = |i: usize| delta.get(i).copied().unwrap_or(0.0);
    let turn = DVec3::new(at(0), at(1), at(2));
    PinholePose {
        mesh_to_optical: (DQuat::from_scaled_axis(turn) * pose.mesh_to_optical).normalize(),
        eye: pose.eye + DVec3::new(at(3), at(4), at(5)),
        focal_px: pose.focal_px * at(6).exp(),
        principal_px: [pose.principal_px[0] + at(7), pose.principal_px[1] + at(8)],
    }
}

fn refine(start: PinholePose, data: &[(DVec3, [f64; 2])], free_principal: bool) -> Resection {
    let n = if free_principal { 9 } else { 7 };
    let steps = [1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4, 1e-6, 1e-3, 1e-3];
    let mut pose = start;
    let mut r = residuals(&pose, data);
    let mut cost: f64 = r.iter().map(|x| x * x).sum();
    let mut lambda = 1e-3;
    for _ in 0..ITERATIONS {
        let jac: Vec<Vec<f64>> = (0..n)
            .map(|k| {
                let mut d = vec![0.0; n];
                d[k] = steps[k];
                let rk = residuals(&nudged(&pose, &d), data);
                rk.iter().zip(&r).map(|(a, b)| (a - b) / steps[k]).collect()
            })
            .collect();
        let jtj: Vec<Vec<f64>> = (0..n)
            .map(|a| (0..n).map(|b| dot(&jac[a], &jac[b])).collect())
            .collect();
        let jtr: Vec<f64> = (0..n).map(|a| -dot(&jac[a], &r)).collect();
        let mut improved = false;
        for _ in 0..8 {
            let mut m = jtj.clone();
            for (i, row) in m.iter_mut().enumerate() {
                row[i] += lambda * (row[i] + 1e-9);
            }
            let Some(delta) = solve(m, jtr.clone()) else {
                lambda *= 10.0;
                continue;
            };
            let trial = nudged(&pose, &delta);
            let tr = residuals(&trial, data);
            let tc: f64 = tr.iter().map(|x| x * x).sum();
            if tc < cost {
                (pose, r, cost) = (trial, tr, tc);
                lambda = (lambda * 0.3).max(1e-12);
                improved = true;
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    let residual_px = r.chunks(2).map(|c| c[0].hypot(c[1])).collect::<Vec<_>>();
    Resection {
        pose,
        rms_px: (cost / data.len().max(1) as f64).sqrt(),
        residual_px,
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Gaussian elimination with partial pivoting; `None` when singular.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-15 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        let pivot_row = a[col].clone();
        for row in col + 1..n {
            let f = a[row][col] / pivot_row[col];
            for (dst, src) in a[row].iter_mut().zip(&pivot_row).skip(col) {
                *dst -= f * src;
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let s: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row][row];
    }
    Some(x)
}
