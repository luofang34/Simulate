//! Historical photographs matched to the P-38G: each photo's camera is resected from
//! airframe landmarks measured on it (`photomatch/photos.json`), the model is rendered
//! from that camera, and side-by-side, overlay and difference images are written for
//! comparison. The photographs themselves are not in the repository; each entry cites
//! its source and licence and the render step reads them from
//! `$SIMULATE_PHOTOMATCH_DIR`.

use glam::DVec3;
use serde_json::Value;

// Shared with other photo matches under `verify_shots` (cockpit photographs).
pub(crate) use backproject::{hit_mesh, hit_plane, ray_through, triangulate};
use landmarks::p38g_landmarks;
pub(crate) use resection::{PinholePose, Resection, cockpit_starts, resect, resect_from};

#[cfg(test)]
mod backproject;
#[cfg(test)]
mod compose;
#[cfg(test)]
mod features;
#[cfg(test)]
mod landmarks;
#[cfg(test)]
mod render;
#[cfg(test)]
mod resection;
#[cfg(test)]
mod tests;

const PHOTOS_JSON: &str = include_str!("photomatch/photos.json");

/// One photograph and the landmarks measured on it.
#[derive(Clone, Debug)]
struct Photo {
    id: String,
    file: String,
    size_px: [f64; 2],
    airborne: bool,
    gear_down: bool,
    /// Sun `(azimuth from the view direction, elevation)`, degrees: azimuth `0` lights
    /// the subject from behind the camera, `+90` from the camera's right.
    sun_deg: Option<(f64, f64)>,
    landmarks_px: Vec<(String, [f64; 2])>,
    /// Features identified on the photo, placed in 3D by [`features::place_features`].
    features_px: Vec<(String, [f64; 2])>,
}

/// A photo's solved camera, with the mesh landmarks it was solved on.
struct Match {
    photo: Photo,
    solved: Resection,
    used: Vec<(String, DVec3, [f64; 2])>,
}

fn photos() -> Vec<Photo> {
    let doc: Value = serde_json::from_str(PHOTOS_JSON).unwrap_or(Value::Null);
    let list = doc.get("photos").and_then(Value::as_array).cloned();
    list.unwrap_or_default().iter().filter_map(photo).collect()
}

fn photo(v: &Value) -> Option<Photo> {
    let text = |k: &str| v.get(k).and_then(Value::as_str).map(str::to_owned);
    let pair = |x: &Value| Some([x.get(0)?.as_f64()?, x.get(1)?.as_f64()?]);
    let flag = |k: &str| v.get(k).and_then(Value::as_bool);
    let sun = v.get("sun").and_then(|s| {
        Some((
            s.get("azimuth_from_view_deg")?.as_f64()?,
            s.get("elevation_deg")?.as_f64()?,
        ))
    });
    let points = |key: &str| -> Vec<(String, [f64; 2])> {
        v.get(key)
            .and_then(Value::as_object)
            .map(|o| {
                o.iter()
                    .filter_map(|(k, p)| Some((k.clone(), pair(p)?)))
                    .collect()
            })
            .unwrap_or_default()
    };
    let landmarks_px = points("landmarks_px");
    let features_px = points("features_px");
    Some(Photo {
        id: text("id")?,
        file: text("file")?,
        size_px: pair(v.get("size_px")?)?,
        airborne: flag("airborne")?,
        gear_down: flag("gear_down")?,
        sun_deg: sun,
        landmarks_px,
        features_px,
    })
}

/// Solves `photo`'s camera on the current mesh's landmarks; `None` when too few of its
/// measured landmarks exist on the mesh.
fn match_photo(photo: &Photo) -> Option<Match> {
    let mesh = p38g_landmarks();
    let used: Vec<(String, DVec3, [f64; 2])> = photo
        .landmarks_px
        .iter()
        .filter_map(|(id, px)| {
            let (_, at) = mesh.iter().find(|(m, _)| m == id)?;
            Some((id.clone(), *at, *px))
        })
        .collect();
    let pairs: Vec<(DVec3, [f64; 2])> = used.iter().map(|(_, p, px)| (*p, *px)).collect();
    let solved = resect(&pairs, photo.size_px, false)?;
    Some(Match {
        photo: photo.clone(),
        solved,
        used,
    })
}
