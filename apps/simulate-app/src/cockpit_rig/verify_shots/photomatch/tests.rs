use std::io::Write;

use glam::{DQuat, DVec3};

use super::landmarks::p38g_landmarks;
use super::{
    PinholePose, cockpit_starts, hit_mesh, hit_plane, ray_through, resect, resect_from, triangulate,
};

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
    // Relations any corrected P-38G keeps, not today's coordinates.
    for side in ["l", "r"] {
        let p = |name: &str| at(&format!("{name}_{side}"));
        assert!(
            p("spinner").y > p("wingtip").y + 1.0,
            "spinner ahead of the wing"
        );
        assert!(p("fin_top").z > at("canopy_top").z, "fin above the canopy");
        assert!(
            p("fin_top").z > p("fin_bottom").z + 1.5,
            "fin taller than 1.5 m"
        );
        assert!(
            p("fin_top").y < at("gondola_tail").y,
            "fins aft of the gondola"
        );
    }
    assert!(
        at("nose_tip").y > at("spinner_l").y,
        "gondola nose ahead of the spinners"
    );
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
fn every_photo_resects_onto_the_mesh_within_four_percent_of_its_diagonal() {
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
        // A loose bound: per-photo error is reported, and a corrected model may move it.
        assert!(matched.solved.rms_px < 0.04 * diagonal, "{}", photo.id);
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

fn looking_at(eye: DVec3, target: DVec3, focal_px: f64) -> PinholePose {
    let forward = (target - eye).normalize();
    let right = forward.cross(DVec3::Z).normalize();
    let down = forward.cross(right);
    PinholePose {
        mesh_to_optical: DQuat::from_mat3(
            &glam::DMat3::from_cols(right, down, forward).transpose(),
        ),
        eye,
        focal_px,
        principal_px: [1000.0, 750.0],
    }
}

#[test]
fn a_projected_point_back_projects_onto_its_plane_and_triangulates_from_two_photos() {
    let point = DVec3::new(0.12, 1.22, 0.40);
    let normal = DVec3::new(0.0, 1.0, -0.14).normalize();
    let a = looking_at(DVec3::new(0.0, 0.42, 0.68), point, 1400.0);
    let b = looking_at(DVec3::new(0.30, 0.20, 0.75), point + DVec3::X * 0.1, 1100.0);
    let ray_a = ray_through(&a, a.project(point).unwrap());
    let ray_b = ray_through(&b, b.project(point).unwrap());
    let on_plane = hit_plane(
        &ray_a,
        point + normal.any_orthonormal_vector() * 0.2,
        normal,
    );
    assert!(on_plane.unwrap().distance(point) < 1e-9);
    assert!(hit_plane(&ray_a, point, DVec3::X.cross(ray_a.dir).normalize()).is_none());
    assert!(triangulate(&[ray_a, ray_b]).unwrap().distance(point) < 1e-9);
    assert!(triangulate(&[ray_a]).is_none());
}

#[test]
fn a_spinner_pixel_back_projects_onto_the_spinner_mesh() {
    let prims = sim_p38g::p38g_mesh_prims();
    let marks = p38g_landmarks();
    let spinner = marks.iter().find(|(id, _)| *id == "spinner_r").unwrap().1;
    let pose = looking_at(DVec3::new(9.0, 14.0, 1.5), spinner, 2400.0);
    let (hit, path) =
        hit_mesh(&ray_through(&pose, pose.project(spinner).unwrap()), &prims).unwrap();
    assert!(path.ends_with("/PropRight/SpinnerNose"), "{path}");
    assert!(hit.distance(spinner) < 0.05, "{hit} vs {spinner}");
}

#[test]
fn a_seat_photograph_resects_from_cockpit_starts_on_fixed_structure() {
    // Fixed structure ahead of and around a seated eye: panel corners, armour-glass frame,
    // canopy-frame junctions and sill edges, roughly where the P-38G has them.
    let anchors = [
        DVec3::new(-0.36, 1.22, 0.52),
        DVec3::new(0.36, 1.22, 0.52),
        DVec3::new(-0.36, 1.24, 0.24),
        DVec3::new(0.36, 1.24, 0.24),
        DVec3::new(-0.18, 1.17, 0.88),
        DVec3::new(0.18, 1.17, 0.88),
        DVec3::new(-0.45, 1.10, 0.62),
        DVec3::new(0.45, 1.10, 0.62),
        DVec3::new(-0.55, 0.60, 0.30),
        DVec3::new(0.55, 0.60, 0.30),
    ];
    let design_eye = DVec3::new(0.0, 0.42, 0.68);
    let truth = looking_at(
        design_eye + DVec3::new(0.08, -0.12, 0.05),
        DVec3::new(0.05, 1.2, 0.42),
        900.0,
    );
    let observed: Vec<(DVec3, [f64; 2])> = anchors
        .iter()
        .filter_map(|p| Some((*p, truth.project(*p)?)))
        .collect();
    assert!(observed.len() >= 8, "{}", observed.len());
    let size = [2000.0, 1500.0];
    let solved = resect_from(&observed, false, &cockpit_starts(design_eye, size)).unwrap();
    assert!(solved.rms_px < 0.5, "{}", solved.rms_px);
    assert!(
        solved.pose.eye.distance(truth.eye) < 0.01,
        "{:?}",
        solved.pose
    );
    assert!(
        (solved.pose.focal_px / 900.0 - 1.0).abs() < 0.01,
        "{:?}",
        solved.pose
    );
}

#[test]
fn features_the_photos_place_against_the_model() {
    let matches: Vec<_> = super::photos()
        .iter()
        .filter_map(super::match_photo)
        .collect();
    let placed = super::features::place_features(&matches);
    assert!(
        placed.iter().any(|f| f.triangulated.is_some()),
        "{placed:?}"
    );
    for f in &placed {
        let surface = f.on_surface();
        writeln!(
            std::io::stderr(),
            "{:>18}: surface hits {:.2?} mean {:.2?}, triangulated {:.2?}, centreline {:.2?}, \
             model {:.2?}, surface - model {:.2?}",
            f.id,
            f.surface_hits,
            surface,
            f.triangulated,
            f.on_centreline,
            f.model,
            surface.zip(f.model).map(|(p, m)| p - m)
        )
        .ok();
        assert!(
            f.surface_hits
                .iter()
                .all(|p| p.is_finite() && p.length() < 12.0),
            "{f:?}"
        );
    }
}

/// Side edges of the tyres' outlines measured on photograph 02 (the 339th FS's 43-2239,
/// port front quarter), in photo pixels: `(prim path suffix, edge, pixel)`. The tops are
/// left out: the tyres stand flattened on the ground while the mesh's are round, and the
/// cameras are resected on the round tyres' lowest points.
const TYRE_EDGES_02: [(&str, char, f64); 5] = [
    ("/Gear/NoseWheel", 'L', 1270.0),
    ("/Gear/NoseWheel", 'R', 1420.0),
    ("/Gear/WheelLeft", 'L', 2237.0),
    ("/Gear/WheelLeft", 'R', 2411.0),
    ("/Gear/WheelRight", 'L', 1150.0),
];

/// Left and right edges of the prims ending in `suffix` in `pose`'s image.
fn side_edges_px(pose: &super::resection::PinholePose, suffix: &str) -> (f64, f64) {
    let prims = sim_p38g::p38g_mesh_prims();
    super::landmarks::prim_points(&prims, suffix)
        .into_iter()
        .filter_map(|p| pose.project(p))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, r), [u, _]| {
            (l.min(u), r.max(u))
        })
}

/// The tyres are as wide as the photograph shows them: each side edge within 25 px of
/// the photo's (the port main tyre's outboard edge, bulged under load, is 23 px out;
/// the others within 8 px, about an inch).
#[test]
fn the_tyres_fill_their_outlines_in_the_photograph() {
    let photos = super::photos();
    let photo = photos.iter().find(|p| p.id.starts_with("p38_02")).unwrap();
    let pose = super::match_photo(photo).unwrap().solved.pose;
    for (suffix, edge, px) in TYRE_EDGES_02 {
        let (left, right) = side_edges_px(&pose, suffix);
        let model = if edge == 'L' { left } else { right };
        assert!(
            (model - px).abs() < 25.0,
            "{suffix} {edge}: photo {px:.0} px, model {model:.0} px"
        );
    }
}
