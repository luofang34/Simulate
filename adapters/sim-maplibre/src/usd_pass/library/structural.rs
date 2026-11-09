//! Where a drawn airframe sits in its flight model's structural frame, fitted to the
//! sourced landmarks its vehicle package ships, so the residual of each says which feature
//! of the model is off.
//!
//! Each landmark names a model feature, measured here on the mesh, and the station a
//! source gives it. The origin's `x` is the least-squares fit over all of them: the mean of
//! what each implies. Its height comes from the package's height reference.

use glam::{DVec3, Vec3};
use sim_pack::{MeshFeature, StructuralLandmarks};

use crate::{MapAdapterError, PartRole, UsdAircraftAsset};

/// Metres per inch, the unit of JSBSim structural frames.
const METRES_PER_INCH: f64 = 0.0254;
/// Width of the spanwise slices a leading edge is measured in.
const SPAN_SLICE_M: f64 = 0.1;

/// A drawn airframe's placement in its flight model's structural frame.
#[derive(Debug, Clone, PartialEq)]
pub struct StructuralDatum {
    origin_in: DVec3,
    residuals_aft_in: Vec<(String, f64)>,
}

impl StructuralDatum {
    /// Fits `asset` to `landmarks`, measuring each landmark's feature on the mesh.
    pub fn fit(
        landmarks: &StructuralLandmarks,
        asset: &UsdAircraftAsset,
    ) -> Result<Self, MapAdapterError> {
        let measured = landmarks
            .landmarks
            .iter()
            .map(|landmark| {
                let at =
                    measure(asset, &landmark.feature).ok_or_else(|| missing(&landmark.name))?;
                let implied =
                    landmarks.structural_station_in(landmark) + f64::from(at.y) / METRES_PER_INCH;
                Ok((landmark.name.clone(), implied))
            })
            .collect::<Result<Vec<_>, MapAdapterError>>()?;
        let x =
            measured.iter().map(|(_, implied)| implied).sum::<f64>() / measured.len().max(1) as f64;
        let height =
            measure(asset, &landmarks.height.feature).ok_or_else(|| missing("height reference"))?;
        let z = landmarks.height.structural_up_in - f64::from(height.z) / METRES_PER_INCH;
        Ok(Self {
            origin_in: DVec3::new(x, 0.0, z),
            residuals_aft_in: measured
                .into_iter()
                .map(|(name, implied)| (name, x - implied))
                .collect(),
        })
    }

    /// The model origin in the structural frame, inches (`x` aft, `y` right, `z` up).
    #[must_use]
    pub const fn origin_in(&self) -> DVec3 {
        self.origin_in
    }

    /// Per landmark, inches the model's feature sits aft of where its source puts it with
    /// the model placed at [`Self::origin_in`].
    #[must_use]
    pub fn residuals_aft_in(&self) -> &[(String, f64)] {
        &self.residuals_aft_in
    }
}

fn missing(name: &str) -> MapAdapterError {
    MapAdapterError::MissingLandmarkFeature {
        landmark: name.to_owned(),
    }
}

/// Where `feature` is on `asset`, model metres (`+X` right, `+Y` forward, `+Z` up).
fn measure(asset: &UsdAircraftAsset, feature: &MeshFeature) -> Option<Vec3> {
    match feature {
        MeshFeature::PropellerDiscCentre => {
            box_centre(asset, |p| p.role == PartRole::PropellerDisc)
        }
        MeshFeature::NoseAxleCentre => box_centre(asset, |p| p.role == PartRole::NoseWheel),
        MeshFeature::MainAxlesCentre => {
            let left = box_centre(asset, |p| p.role == PartRole::WheelLeft)?;
            let right = box_centre(asset, |p| p.role == PartRole::WheelRight)?;
            Some((left + right) * 0.5)
        }
        MeshFeature::PrimBoxCentre { prim_path } => {
            box_centre(asset, |p| p.prim_path == *prim_path)
        }
        MeshFeature::WingLeadingEdge {
            span_from_m,
            span_to_m,
        } => wing_leading_edge(asset, *span_from_m, *span_to_m),
    }
}

fn vertices<'a>(
    asset: &'a UsdAircraftAsset,
    pick: impl Fn(&crate::UsdMeshPart) -> bool + 'a,
) -> impl Iterator<Item = Vec3> + 'a {
    asset
        .parts()
        .iter()
        .filter(move |part| pick(part))
        .flat_map(|part| {
            let rest = part.rest_to_aircraft.as_mat4();
            part.vertices
                .iter()
                .map(move |v| rest.transform_point3(Vec3::from_array(v.position)))
        })
}

fn box_centre(
    asset: &UsdAircraftAsset,
    pick: impl Fn(&crate::UsdMeshPart) -> bool,
) -> Option<Vec3> {
    let (low, high) = vertices(asset, pick).fold(
        (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
        |(low, high), p| (low.min(p), high.max(p)),
    );
    low.is_finite().then(|| (low + high) * 0.5)
}

/// The median over spanwise slices of each slice's most forward wing vertex, at the
/// slices' median height.
fn wing_leading_edge(asset: &UsdAircraftAsset, from_m: f64, to_m: f64) -> Option<Vec3> {
    let wing: Vec<Vec3> = vertices(asset, |p| p.role == PartRole::WingStructure).collect();
    let slices = ((to_m - from_m) / SPAN_SLICE_M).round().max(1.0) as usize;
    let mut edges: Vec<Vec3> = (0..slices)
        .filter_map(|slice| {
            let inner = (from_m + slice as f64 * SPAN_SLICE_M) as f32;
            let outer = inner + SPAN_SLICE_M as f32;
            wing.iter()
                .copied()
                .filter(|p| (inner..outer).contains(&p.x.abs()))
                .max_by(|a, b| a.y.total_cmp(&b.y))
        })
        .collect();
    edges.sort_by(|a, b| a.y.total_cmp(&b.y));
    edges.get(edges.len() / 2).copied()
}

#[cfg(test)]
mod tests;
