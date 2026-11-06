use crate::{ControlPreset, ControlPresetId};

/// One editable copy of every preset, so lookups cannot miss.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PresetTable {
    quad_cam: ControlPreset,
    quad_acro: ControlPreset,
    fixed_wing_gamepad: ControlPreset,
    fixed_wing_mode2: ControlPreset,
    hotas: ControlPreset,
}

impl PresetTable {
    pub(crate) fn builtin() -> Self {
        Self {
            quad_cam: ControlPresetId::QuadCamDroneMode2.builtin(),
            quad_acro: ControlPresetId::QuadAcroMode2.builtin(),
            fixed_wing_gamepad: ControlPresetId::FixedWingGamepad.builtin(),
            fixed_wing_mode2: ControlPresetId::FixedWingMode2.builtin(),
            hotas: ControlPresetId::FlightStickHotas.builtin(),
        }
    }

    pub(crate) fn get(&self, id: ControlPresetId) -> &ControlPreset {
        match id {
            ControlPresetId::QuadCamDroneMode2 => &self.quad_cam,
            ControlPresetId::QuadAcroMode2 => &self.quad_acro,
            ControlPresetId::FixedWingGamepad => &self.fixed_wing_gamepad,
            ControlPresetId::FixedWingMode2 => &self.fixed_wing_mode2,
            ControlPresetId::FlightStickHotas => &self.hotas,
        }
    }

    pub(crate) fn get_mut(&mut self, id: ControlPresetId) -> &mut ControlPreset {
        match id {
            ControlPresetId::QuadCamDroneMode2 => &mut self.quad_cam,
            ControlPresetId::QuadAcroMode2 => &mut self.quad_acro,
            ControlPresetId::FixedWingGamepad => &mut self.fixed_wing_gamepad,
            ControlPresetId::FixedWingMode2 => &mut self.fixed_wing_mode2,
            ControlPresetId::FlightStickHotas => &mut self.hotas,
        }
    }

    /// Replaces the stored preset with the same id; the id inside `preset`
    /// decides the slot.
    pub(crate) fn replace(&mut self, preset: ControlPreset) {
        let id = preset.id;
        *self.get_mut(id) = preset;
    }

    pub(crate) fn all(&self) -> Vec<ControlPreset> {
        ControlPresetId::ALL
            .iter()
            .map(|&id| self.get(id).clone())
            .collect()
    }
}
