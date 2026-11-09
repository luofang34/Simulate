//! Sourced structural landmarks a vehicle package ships (`geometry/landmarks.json`):
//! airframe features whose stations a handbook or drawing gives, each naming the feature
//! of the drawn model it matches, so a renderer can place the model in the flight model's
//! structural frame and say which features disagree with the sources.

use serde::{Deserialize, Serialize};

use crate::PackError;

/// Canonical asset path of a package's structural landmarks.
pub const STRUCTURAL_LANDMARKS_ASSET: &str = "geometry/landmarks.json";

/// How to find a landmark on the drawn model. Positions are in the model frame
/// (`+X` right, `+Y` forward, `+Z` up), metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "rule", rename_all = "snake_case")]
pub enum MeshFeature {
    /// The centre of the box around the propeller disc parts.
    PropellerDiscCentre,
    /// The midpoint of the left and right main wheels' box centres.
    MainAxlesCentre,
    /// The centre of the box around the nose wheel parts.
    NoseAxleCentre,
    /// The wing's leading edge: the median, over 0.1 m spanwise slices between
    /// `span_from_m` and `span_to_m` out from the centreline, of each slice's most forward
    /// wing vertex, so a fairing or fitting in one slice does not move it.
    WingLeadingEdge {
        /// Inboard end of the measured span, metres from the centreline.
        span_from_m: f64,
        /// Outboard end of the measured span, metres from the centreline.
        span_to_m: f64,
    },
    /// The centre of the box around the part at `prim_path`.
    PrimBoxCentre {
        /// The part's USD prim path.
        prim_path: String,
    },
}

/// One feature whose fore-and-aft station a source gives.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StructuralLandmark {
    /// What the feature is.
    pub name: String,
    /// The station the source gives, inches aft in the source's own frame.
    pub station_in: f64,
    /// Where the station comes from.
    pub source: String,
    /// The matching feature of the drawn model.
    pub feature: MeshFeature,
}

/// A height both frames agree on, which fixes the model's height in the structural frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeightReference {
    /// The matching feature of the drawn model.
    pub feature: MeshFeature,
    /// Its height in the structural frame, inches up.
    pub structural_up_in: f64,
    /// Where the height comes from.
    pub source: String,
}

/// A package's sourced structural landmarks.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StructuralLandmarks {
    /// Inches added to a source station to give the flight model's structural `x`.
    pub source_station_offset_in: f64,
    /// What the source frame is and why the offset is what it is.
    pub source_frame: String,
    /// The height reference.
    pub height: HeightReference,
    /// The fore-and-aft landmarks; the model's origin is fitted to all of them.
    pub landmarks: Vec<StructuralLandmark>,
}

impl StructuralLandmarks {
    /// Parses and validates a package's landmarks.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, PackError> {
        let landmarks: Self =
            serde_json::from_slice(bytes).map_err(|source| PackError::LandmarksParse {
                path: STRUCTURAL_LANDMARKS_ASSET.to_owned(),
                source,
            })?;
        landmarks.validate()?;
        Ok(landmarks)
    }

    /// A landmark's station in the flight model's structural frame, inches aft.
    #[must_use]
    pub fn structural_station_in(&self, landmark: &StructuralLandmark) -> f64 {
        landmark.station_in + self.source_station_offset_in
    }

    fn validate(&self) -> Result<(), PackError> {
        let invalid = |detail: String| PackError::InvalidLandmarks {
            path: STRUCTURAL_LANDMARKS_ASSET.to_owned(),
            detail,
        };
        if self.landmarks.is_empty() {
            return Err(invalid("at least one landmark is needed".to_owned()));
        }
        if !(self.source_station_offset_in.is_finite() && self.height.structural_up_in.is_finite())
        {
            return Err(invalid(
                "the station offset and height must be finite".to_owned(),
            ));
        }
        for landmark in &self.landmarks {
            if !landmark.station_in.is_finite() || landmark.name.trim().is_empty() {
                return Err(invalid(format!(
                    "landmark {:?} is incomplete",
                    landmark.name
                )));
            }
            if let MeshFeature::WingLeadingEdge {
                span_from_m,
                span_to_m,
            } = landmark.feature
                && !(span_from_m.is_finite() && span_to_m > span_from_m)
            {
                return Err(invalid(format!(
                    "landmark {:?} spans nothing",
                    landmark.name
                )));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
