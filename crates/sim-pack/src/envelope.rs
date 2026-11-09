use serde::{Deserialize, Serialize};

use crate::PackError;

/// Canonical asset path inside a fixed-wing system package declaring its
/// certificated airspeed limitations and V-speed schedule.
pub const AIRSPEED_ENVELOPE_ASSET: &str = "performance/airspeed_envelope.json";

/// Airspeed limitations and reference V-speeds (`kt`) shipped inside a
/// fixed-wing system package (`performance/airspeed_envelope.json`).
///
/// Structural and flap arc limits (`vs0_kt`, `vs1_kt`, `vfe_kt`, `vno_kt`,
/// `vne_kt`) represent fixed dial/tape markings at `reference_mass_kg`, while
/// [`Self::resolve_at_mass`] computes the operating maneuvering speed
/// $V_A(m) = V_{A,\text{ref}}\sqrt{m / m_{\text{ref}}}$ for the live aircraft
/// gross mass.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AirspeedEnvelope {
    /// ICAO or package aircraft type designator (for example `"C172"`).
    pub type_designator: String,
    /// Pilot's Operating Handbook or engineering source for the speeds.
    pub reference_document: String,
    /// Gross mass (`kg`) at which `va_ref_kt` and published stall/glide speeds apply.
    pub reference_mass_kg: f64,
    /// Positive design maneuvering load-factor limit (`g`, e.g. `+3.8`).
    pub positive_limit_g: f64,
    /// Negative design maneuvering load-factor limit (`g`, e.g. `-1.52`).
    pub negative_limit_g: f64,
    /// Power-off stall speed in landing configuration (`kt`, bottom of white arc, $V_{SO}$).
    pub vs0_kt: f32,
    /// Power-off stall speed in clean configuration (`kt`, bottom of green arc, $V_{S1}$).
    pub vs1_kt: f32,
    /// Maximum full-flap extended speed (`kt`, top of white arc, $V_{FE}$).
    pub vfe_kt: f32,
    /// Optional partial/takeoff-flap maximum speed (`kt`, e.g. `110 kt` for `10°` flaps).
    #[serde(default)]
    pub vfe_takeoff_kt: Option<f32>,
    /// Normal takeoff rotation speed (`kt`, $V_R$).
    #[serde(default)]
    pub vr_kt: Option<f32>,
    /// Sea-level best angle-of-climb speed (`kt`, $V_X$).
    #[serde(default)]
    pub vx_kt: Option<f32>,
    /// Sea-level best rate-of-climb speed (`kt`, $V_Y$).
    #[serde(default)]
    pub vy_kt: Option<f32>,
    /// Maximum-range power-off glide speed at `reference_mass_kg` (`kt`, $V_G$).
    #[serde(default)]
    pub vg_kt: Option<f32>,
    /// Design maneuvering speed at `reference_mass_kg` (`kt`, $V_A$).
    #[serde(default)]
    pub va_ref_kt: Option<f32>,
    /// Final approach reference speed (`kt`, $V_{REF}$).
    #[serde(default)]
    pub vref_kt: Option<f32>,
    /// Maximum structural cruising speed (`kt`, top of green arc, $V_{NO}$).
    pub vno_kt: f32,
    /// Never-exceed speed (`kt`, red line, $V_{NE}$).
    pub vne_kt: f32,
}

/// Airspeed envelope resolved at a concrete operating gross mass (`mass_kg`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedAirspeedEnvelope {
    /// Operating gross mass (`kg`) used to scale mass-dependent limits.
    pub mass_kg: f64,
    /// Bottom of white arc ($V_{SO}$, `kt`).
    pub vs0_kt: f32,
    /// Bottom of green arc ($V_{S1}$, `kt`).
    pub vs1_kt: f32,
    /// Top of white arc ($V_{FE}$, `kt`).
    pub vfe_kt: f32,
    /// Optional partial-flap limit ($V_{FE,\text{TO}}$, `kt`).
    pub vfe_takeoff_kt: Option<f32>,
    /// Rotation speed bug ($V_R$, `kt`).
    pub vr_kt: Option<f32>,
    /// Best angle-of-climb bug ($V_X$, `kt`).
    pub vx_kt: Option<f32>,
    /// Best rate-of-climb bug ($V_Y$, `kt`).
    pub vy_kt: Option<f32>,
    /// Published best-glide speed bug ($V_G$, `kt`).
    pub vg_kt: Option<f32>,
    /// Mass-adjusted maneuvering speed bug ($V_A(m)$, `kt`).
    pub va_kt: Option<f32>,
    /// Final approach reference bug ($V_{REF}$, `kt`).
    pub vref_kt: Option<f32>,
    /// Top of green arc ($V_{NO}$, `kt`).
    pub vno_kt: f32,
    /// Never-exceed red line ($V_{NE}$, `kt`).
    pub vne_kt: f32,
}

impl AirspeedEnvelope {
    /// Parses and validates an [`AirspeedEnvelope`] JSON payload fail-closed.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, PackError> {
        let envelope: Self =
            serde_json::from_slice(bytes).map_err(|source| PackError::AirspeedEnvelopeParse {
                path: AIRSPEED_ENVELOPE_ASSET.to_owned(),
                source,
            })?;
        envelope.validate()?;
        Ok(envelope)
    }

    /// Validates physical ordering and finiteness of all declared speeds and limits.
    pub fn validate(&self) -> Result<(), PackError> {
        if self.type_designator.trim().is_empty() {
            return Err(invalid("type_designator must not be empty"));
        }
        if !self.reference_mass_kg.is_finite() || self.reference_mass_kg <= 0.0 {
            return Err(invalid("reference_mass_kg must be finite and positive"));
        }
        if !self.positive_limit_g.is_finite() || self.positive_limit_g < 1.0 {
            return Err(invalid("positive_limit_g must be finite and >= 1.0"));
        }
        if !self.negative_limit_g.is_finite() || self.negative_limit_g > 0.0 {
            return Err(invalid("negative_limit_g must be finite and <= 0.0"));
        }
        self.validate_band_ladder()?;
        self.validate_reference_bugs()
    }

    fn validate_band_ladder(&self) -> Result<(), PackError> {
        let ladder = [
            self.vs0_kt,
            self.vs1_kt,
            self.vfe_kt,
            self.vno_kt,
            self.vne_kt,
        ];
        if !ladder.iter().all(|v| v.is_finite() && *v > 0.0) {
            return Err(invalid(
                "vs0_kt, vs1_kt, vfe_kt, vno_kt, and vne_kt must be finite and > 0",
            ));
        }
        let ordered = self.vs0_kt <= self.vs1_kt
            && self.vs1_kt < self.vfe_kt
            && self.vfe_kt <= self.vno_kt
            && self.vno_kt < self.vne_kt;
        if !ordered {
            return Err(invalid(
                "band ladder must satisfy vs0 <= vs1 < vfe <= vno < vne",
            ));
        }
        if let Some(vfe_to) = self.vfe_takeoff_kt
            && (!vfe_to.is_finite() || vfe_to < self.vfe_kt || vfe_to > self.vno_kt)
        {
            return Err(invalid("vfe_takeoff_kt must lie within [vfe_kt, vno_kt]"));
        }
        Ok(())
    }

    fn validate_reference_bugs(&self) -> Result<(), PackError> {
        for (name, bug) in [
            ("vr_kt", self.vr_kt),
            ("vx_kt", self.vx_kt),
            ("vy_kt", self.vy_kt),
            ("vg_kt", self.vg_kt),
            ("va_ref_kt", self.va_ref_kt),
            ("vref_kt", self.vref_kt),
        ] {
            if let Some(v) = bug
                && (!v.is_finite() || v <= self.vs0_kt || v > self.vno_kt)
            {
                return Err(invalid(&format!(
                    "{name} ({v}) must lie in (vs0_kt, vno_kt]"
                )));
            }
        }
        if let (Some(vx), Some(vy)) = (self.vx_kt, self.vy_kt)
            && vx > vy
        {
            return Err(invalid("vx_kt must not exceed vy_kt"));
        }
        if let Some(va) = self.va_ref_kt
            && va <= self.vs1_kt
        {
            return Err(invalid("va_ref_kt must exceed clean stall speed vs1_kt"));
        }
        Ok(())
    }

    /// Computes mass-adjusted maneuvering speed $V_A(m) = V_{A,\text{ref}}\sqrt{m / m_{\text{ref}}}$.
    #[must_use]
    pub fn va_at_mass_kt(&self, mass_kg: f64) -> Option<f32> {
        let va_ref = f64::from(self.va_ref_kt?);
        if !mass_kg.is_finite() || mass_kg <= 0.0 {
            return self.va_ref_kt;
        }
        let scale = (mass_kg / self.reference_mass_kg).sqrt();
        let scaled = (va_ref * scale) as f32;
        Some(scaled.clamp(self.vs1_kt + 1.0, self.vno_kt))
    }

    /// Computes mass-adjusted best-glide speed $V_G(m) = V_G\sqrt{m / m_{\text{ref}}}$.
    #[must_use]
    pub fn vg_at_mass_kt(&self, mass_kg: f64) -> Option<f32> {
        let vg_ref = f64::from(self.vg_kt?);
        if !mass_kg.is_finite() || mass_kg <= 0.0 {
            return self.vg_kt;
        }
        let scaled = (vg_ref * (mass_kg / self.reference_mass_kg).sqrt()) as f32;
        Some(scaled.clamp(self.vs1_kt + 1.0, self.vno_kt))
    }

    /// Resolves the envelope at operating gross mass `mass_kg` (`kg`).
    #[must_use]
    pub fn resolve_at_mass(&self, mass_kg: f64) -> ResolvedAirspeedEnvelope {
        let effective_mass = if mass_kg.is_finite() && mass_kg > 0.0 {
            mass_kg
        } else {
            self.reference_mass_kg
        };
        ResolvedAirspeedEnvelope {
            mass_kg: effective_mass,
            vs0_kt: self.vs0_kt,
            vs1_kt: self.vs1_kt,
            vfe_kt: self.vfe_kt,
            vfe_takeoff_kt: self.vfe_takeoff_kt,
            vr_kt: self.vr_kt,
            vx_kt: self.vx_kt,
            vy_kt: self.vy_kt,
            vg_kt: self.vg_kt,
            va_kt: self.va_at_mass_kt(effective_mass),
            vref_kt: self.vref_kt,
            vno_kt: self.vno_kt,
            vne_kt: self.vne_kt,
        }
    }
}

fn invalid(detail: &str) -> PackError {
    PackError::InvalidAirspeedEnvelope {
        path: AIRSPEED_ENVELOPE_ASSET.to_owned(),
        detail: detail.to_owned(),
    }
}
