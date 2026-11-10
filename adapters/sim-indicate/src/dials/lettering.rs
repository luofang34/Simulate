//! The faces a pack's dials are lettered in: each distinct face and style
//! the pack's instruments name gets a scene face slot, and the host binds
//! the slots to the typefaces the pack pins.

use indicate_instrument_glyphs::{Face, TextStyle, Typeface};
use indicate_instrument_scene::FaceSlot;
use syspack::FaceStyleSpec;

use super::DialError;

/// The face slots of one panel: slot `k` (from 1) is `styles[k - 1]`, and
/// slot 0 is Indicate's reference face.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PanelLettering {
    styles: Vec<FaceStyleSpec>,
}

impl PanelLettering {
    /// The slot lettering `spec`, adding one for a face and style not seen
    /// before; the reference face when the panel runs out of slots.
    pub(super) fn slot_for(&mut self, spec: &FaceStyleSpec) -> FaceSlot {
        let index = match self.styles.iter().position(|s| s == spec) {
            Some(index) => index,
            None => {
                self.styles.push(spec.clone());
                self.styles.len() - 1
            }
        };
        u8::try_from(index + 1).map_or(FaceSlot::DEFAULT, FaceSlot)
    }

    /// The slot lettering `face` in its plain style, if any dial of the panel
    /// uses it so: what a placard in the panel's face is lettered in.
    #[must_use]
    pub fn plain_slot(&self, face: &str) -> Option<FaceSlot> {
        let index = self.styles.iter().position(|s| {
            s.face == face
                && s.condense.is_none()
                && s.pen.is_none()
                && s.slant.is_none()
                && s.alternates.is_empty()
        })?;
        u8::try_from(index + 1).ok().map(FaceSlot)
    }

    /// The faces to bind, slot by slot from slot 0: each style lettered in
    /// the parsed typeface its spec names from `typefaces`, by pack id.
    pub fn faces<'a>(
        &self,
        typefaces: &[(&str, Typeface<'a>)],
    ) -> Result<Vec<Face<'a>>, DialError> {
        let mut faces = vec![Face::Reference];
        for spec in &self.styles {
            let refused = |detail: String| DialError::Lettering {
                face: spec.face.clone(),
                detail,
            };
            let typeface = typefaces
                .iter()
                .find(|(id, _)| *id == spec.face)
                .map(|(_, face)| *face)
                .ok_or_else(|| refused("the pack's typeface was not supplied".to_owned()))?;
            let mut alternates = 0_u32;
            for name in &spec.alternates {
                let tag = typeface
                    .alternate_tag(name)
                    .ok_or_else(|| refused(format!("the face has no alternate {name:?}")))?;
                alternates |= 1_u32
                    .checked_shl(u32::from(tag.saturating_sub(1)))
                    .unwrap_or(0);
            }
            let style = TextStyle {
                condense: spec.condense.map_or(1.0, |v| v as f32),
                pen: spec.pen.map(|v| v as f32),
                slant: spec.slant.map_or(0.0, |v| v as f32),
                alternates,
            };
            faces.push(Face::typeface(typeface, style).map_err(|e| refused(e.to_string()))?);
        }
        Ok(faces)
    }
}
