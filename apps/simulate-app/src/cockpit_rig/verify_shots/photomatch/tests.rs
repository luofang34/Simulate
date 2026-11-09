use std::io::Write;

use glam::{DQuat, DVec3};

use super::landmarks::p38g_landmarks;
use super::resection::{PinholePose, resect};

#[test]
fn the_mesh_yields_every_landmark_on_its_own_side() {
    let marks = p38g_landmarks();
    assert_eq!(marks.len(), 17, "{marks:?}");
    for (id, at) in &marks {
        assert!(at.is_finite(), "{id}");
        if id.ends_with("_l") {
            assert!(at.x < -1.0, "{id} {at}");
        }
        if id.ends_with("_r") {
            assert!(at.x > 1.0, "{id} {at}");
        }
    }
    let at = |id: &str| {
        marks
            .iter()
            .find(|(m, _)| *m == id)
            .map(|(_, p)| *p)
            .unwrap()
    };
    assert!(at("spinner_l").y > 2.8 && at("fin_top_r").z > 1.7 && at("wingtip_r").x > 7.8);
}

#[test]
fn resection_recovers_a_known_camera_from_noisy_landmarks() {
    let marks = p38g_landmarks();
    for (eye, focal_px, free) in [
        (DVec3::new(-14.0, 18.0, 2.0), 2600.0, false),
        (DVec3::new(20.0, -9.0, 9.0), 1500.0, true),
        (DVec3::new(-30.0, -2.0, 1.5), 4200.0, false),
    ] {
        let target = DVec3::new(0.3, -1.0, 0.4);
        let forward = (target - eye).normalize();
        let right = forward.cross(DVec3::Z).normalize();
        let down = forward.cross(right);
        let truth = PinholePose {
            mesh_to_optical: DQuat::from_mat3(
                &glam::DMat3::from_cols(right, down, forward).transpose(),
            ),
            eye,
            focal_px,
            principal_px: [1580.0, 1020.0],
        };
        // Deterministic half-pixel jitter.
        let observed: Vec<(DVec3, [f64; 2])> = marks
            .iter()
            .enumerate()
            .filter_map(|(i, (_, p))| {
                let [u, v] = truth.project(*p)?;
                let j = 0.5 * ((i as f64) * 2.39).sin();
                Some((*p, [u + j, v - j]))
            })
            .collect();
        let solved = resect(&observed, [3200.0, 2000.0], free).unwrap();
        assert!(solved.rms_px < 1.5, "{}", solved.rms_px);
        assert!(
            solved.pose.eye.distance(eye) < 0.03 * eye.length(),
            "{:?}",
            solved.pose
        );
        assert!(
            (solved.pose.focal_px / focal_px - 1.0).abs() < 0.03,
            "{:?}",
            solved.pose
        );
    }
}

#[test]
fn every_photo_resects_onto_the_mesh_within_two_percent_of_its_diagonal() {
    let photos = super::photos();
    assert!(photos.len() >= 6, "{}", photos.len());
    for photo in &photos {
        let matched = super::match_photo(photo).unwrap();
        let [w, h] = photo.size_px;
        let diagonal = w.hypot(h);
        let pose = matched.solved.pose;
        writeln!(
            std::io::stderr(),
            "{}: rms {:.1} px ({:.2}% of diagonal), eye {:.2?} m, f {:.0} px, principal {:.0?}, fov_y {:.1} deg, residuals {:.0?}",
            photo.id,
            matched.solved.rms_px,
            100.0 * matched.solved.rms_px / diagonal,
            pose.eye,
            pose.focal_px,
            pose.principal_px,
            pose.fov_y_rad(h).to_degrees(),
            matched.solved.residual_px,
        )
        .ok();
        assert_eq!(matched.used.len(), photo.landmarks_px.len(), "{}", photo.id);
        assert!(matched.solved.rms_px < 0.02 * diagonal, "{}", photo.id);
    }
}

/// With every photo's camera held, the 3D shift of each landmark (mesh axes, metres)
/// that best explains the residuals across all photos: where the photos say the model's
/// landmark should be. Printed for review; asserts only that it is finite.
#[test]
fn landmark_shifts_the_photos_imply() {
    use std::collections::BTreeMap;
    /// Normal matrix, right-hand side and photo count of one landmark's shift.
    type Normal = ([[f64; 3]; 3], [f64; 3], usize);
    let mut normal: BTreeMap<String, Normal> = BTreeMap::new();
    for photo in super::photos() {
        let matched = super::match_photo(&photo).unwrap();
        let pose = matched.solved.pose;
        for (id, point, [u, v]) in &matched.used {
            let Some([pu, pv]) = pose.project(*point) else {
                continue;
            };
            let axes = [DVec3::X, DVec3::Y, DVec3::Z];
            let jac = axes.map(|a| {
                let [qu, qv] = pose.project(*point + a * 1e-3).unwrap();
                [(qu - pu) / 1e-3, (qv - pv) / 1e-3]
            });
            let r = [u - pu, v - pv];
            let entry = normal
                .entry(id.clone())
                .or_insert(([[0.0; 3]; 3], [0.0; 3], 0));
            for i in 0..3 {
                for j in 0..3 {
                    entry.0[i][j] += jac[i][0] * jac[j][0] + jac[i][1] * jac[j][1];
                }
                entry.1[i] += jac[i][0] * r[0] + jac[i][1] * r[1];
            }
            entry.2 += 1;
        }
    }
    for (id, (mut a, b, n)) in normal {
        let ridge = 1e-3 * (a[0][0] + a[1][1] + a[2][2]);
        for (i, row) in a.iter_mut().enumerate() {
            row[i] += ridge;
        }
        let m = glam::DMat3::from_cols_array_2d(&a);
        let shift = m.inverse() * DVec3::from(b);
        assert!(shift.is_finite(), "{id}");
        writeln!(
            std::io::stderr(),
            "{id:>14} in {n} photos: right {:+.2} m, fwd {:+.2} m, up {:+.2} m",
            shift.x,
            shift.y,
            shift.z
        )
        .ok();
    }
}
