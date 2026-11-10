//! Airframe features identified in the photographs (markings, turbos, scoops, small
//! fittings), placed in 3D from the solved cameras and compared with where the model
//! has them, or reported missing where it has none.

use glam::DVec3;
use sim_p38g::{MarkingKind, MarkingSite, P38MeshPrim, P38Skin, p38g_mesh_prims};

use super::backproject::Ray;
use super::landmarks::{extreme, prim_points};
use super::{Match, hit_mesh, hit_plane, ray_through, triangulate};

/// Where the model carries a feature.
enum ModelFeature {
    /// The centroid, or the extreme along `toward`, of one prim's vertices inside an
    /// axis-aligned box (mesh axes, metres).
    Prim {
        prim: &'static str,
        within: ([f64; 3], [f64; 3]),
        toward: Option<[f64; 3]>,
    },
    /// A marking the baked skin paints on the side of the airframe with this sign of `x`.
    Marking { kind: MarkingKind, side: f64 },
}

/// Features the photographs are measured for, and where the model has each one.
const FEATURES: &[(&str, Option<ModelFeature>)] = &[
    ("serial_r", marking(MarkingKind::CallNumber, 1.0)),
    ("boom_star_r", marking(MarkingKind::BoomInsignia, 1.0)),
    ("boom_star_l", marking(MarkingKind::BoomInsignia, -1.0)),
    ("wing_star_l", marking(MarkingKind::WingInsignia, -1.0)),
    (
        "turbo_r",
        centroid(
            "/Powerplant/TurboBucketWheels",
            [2.0, -3.0, 0.0],
            [3.0, 0.0, 1.0],
        ),
    ),
    (
        "turbo_l",
        centroid(
            "/Powerplant/TurboBucketWheels",
            [-3.0, -3.0, 0.0],
            [-2.0, 0.0, 1.0],
        ),
    ),
    (
        "radiator_inlet_r",
        Some(ModelFeature::Prim {
            prim: "/Powerplant/RadiatorScoops",
            within: ([2.6, -6.0, -1.0], [3.4, 0.0, 1.0]),
            toward: Some([0.0, 1.0, 0.0]),
        }),
    ),
    ("canopy_mirror_base", None),
    ("deck_mast_base", None),
];

const fn centroid(prim: &'static str, lo: [f64; 3], hi: [f64; 3]) -> Option<ModelFeature> {
    Some(ModelFeature::Prim {
        prim,
        within: (lo, hi),
        toward: None,
    })
}

const fn marking(kind: MarkingKind, side: f64) -> Option<ModelFeature> {
    Some(ModelFeature::Marking { kind, side })
}

/// One feature placed from the photographs.
#[derive(Debug)]
pub(super) struct PlacedFeature {
    pub(super) id: &'static str,
    /// Where each photo's ray first meets the model surface: the placement for a
    /// marking or a flush part.
    pub(super) surface_hits: Vec<DVec3>,
    /// Where two or more photos' rays meet: the placement for a part standing off the
    /// surface, such as a turbo or a mast.
    pub(super) triangulated: Option<DVec3>,
    /// Where the first photo's ray crosses the aircraft's plane of symmetry: the
    /// placement for a single-photo centreline fitting the model lacks.
    pub(super) on_centreline: Option<DVec3>,
    /// Where the model has it; `None` when the model lacks it.
    pub(super) model: Option<DVec3>,
}

impl PlacedFeature {
    /// Mean of [`Self::surface_hits`].
    pub(super) fn on_surface(&self) -> Option<DVec3> {
        let n = u32::try_from(self.surface_hits.len())
            .ok()
            .filter(|&n| n > 0)?;
        Some(self.surface_hits.iter().sum::<DVec3>() / f64::from(n))
    }
}

/// Places every measured feature from `matches`' solved cameras.
pub(super) fn place_features(matches: &[Match]) -> Vec<PlacedFeature> {
    let prims = p38g_mesh_prims();
    let sites = P38Skin::new(&prims).map_or_else(|_| Vec::new(), |skin| skin.marking_sites());
    FEATURES
        .iter()
        .map(|(id, model)| {
            let rays: Vec<_> = matches
                .iter()
                .filter_map(|m| {
                    let (_, px) = m.photo.features_px.iter().find(|(f, _)| f == id)?;
                    Some(ray_through(&m.solved.pose, *px))
                })
                .collect();
            PlacedFeature {
                id,
                surface_hits: rays
                    .iter()
                    .filter_map(|r| hit_mesh(r, &prims).map(|(p, _)| p))
                    .collect(),
                triangulated: triangulate(&rays),
                on_centreline: rays
                    .first()
                    .and_then(|r| hit_plane(r, DVec3::ZERO, DVec3::X)),
                model: model.as_ref().and_then(|f| model_point(&prims, &sites, f)),
            }
        })
        .collect()
}

fn model_point(
    prims: &[P38MeshPrim],
    sites: &[MarkingSite],
    feature: &ModelFeature,
) -> Option<DVec3> {
    let (prim, within, toward) = match feature {
        ModelFeature::Prim {
            prim,
            within,
            toward,
        } => (prim, within, toward),
        ModelFeature::Marking { kind, side } => {
            let site = sites
                .iter()
                .find(|s| s.kind == *kind && f64::from(s.on_axis.x) * side > 0.0)?;
            // Looked at from well outside the airframe, along the marking's own axis.
            let facing = site.facing.as_dvec3();
            let sight = Ray {
                origin: site.on_axis.as_dvec3() + facing * SIGHT_STANDOFF_M,
                dir: -facing,
            };
            return hit_mesh(&sight, prims).map(|(p, _)| p);
        }
    };
    let (lo, hi) = (DVec3::from(within.0), DVec3::from(within.1));
    let inside: Vec<DVec3> = prim_points(prims, prim)
        .into_iter()
        .filter(|p| p.cmpge(lo).all() && p.cmple(hi).all())
        .collect();
    match toward {
        Some(toward) => extreme(&inside, DVec3::from(*toward)),
        None => {
            let count = u32::try_from(inside.len()).ok().filter(|&n| n > 0)?;
            Some(inside.iter().sum::<DVec3>() / f64::from(count))
        }
    }
}

/// How far outside the airframe a marking is sighted from: beyond a wing tip or a fin
/// from any marking's centre line.
const SIGHT_STANDOFF_M: f64 = 10.0;
