//! Pitot-static position error calibration (`PEC`) tables shipped inside a
//! fixed-wing system package (`performance/pitot_static_calibration.json`).
//!
//! Aerodynamic forces act on calibrated/equivalent airspeed (`KCAS` / `KEAS`),
//! whereas cockpit airspeed indicators, Garmin G3X Touch `KIAS` tapes (STC
//! `SA01899WI`), and POH V-speed arcs read indicated airspeed (`KIAS`) corrupted
//! by static-port pressure coefficient $C_p(\alpha, \delta_f)$.

use serde::{Deserialize, Serialize};

use crate::PackError;

/// Canonical asset path inside a fixed-wing system package declaring its
/// flap-indexed pitot-static position error (`KIAS` ↔ `KCAS`) curves.
pub const PITOT_STATIC_CALIBRATION_ASSET: &str = "performance/pitot_static_calibration.json";

/// Exact conversion factor from knots to metres per second (`1852 / 3600`).
const MPS_PER_KT: f64 = 1_852.0 / 3_600.0;

/// Single paired `(KIAS, KCAS)` calibration knot from a POH airspeed calibration table.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AirspeedCalibrationPoint {
    /// Indicated airspeed (`kt`, cockpit dial/tape reading with normal static source).
    pub kias: f64,
    /// Calibrated airspeed (`kt`, position-error-corrected airspeed).
    pub kcas: f64,
}

/// Monotonic `KIAS` ↔ `KCAS` curve for one flap setting (`flap_norm` in `[0.0, 1.0]`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PitotStaticCurve {
    /// Physical flap deflection in degrees (informational, e.g. `0.0`, `10.0`, `30.0`).
    pub flap_deg: f64,
    /// Normalized flap position in `[0.0, 1.0]` matching `fcs/flap-pos-norm`.
    pub flap_norm: f64,
    /// Strictly increasing `(kias, kcas)` sample points covering the speed envelope.
    pub points: Vec<AirspeedCalibrationPoint>,
}

/// Flap-indexed pitot-static position error calibration shipped with an aircraft package.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PitotStaticCalibration {
    /// ICAO or package aircraft type designator (for example `"C172"`).
    pub type_designator: String,
    /// Engineering or POH source (for example `"1998 Cessna 172S POH Figure 5-1 & 5-3"`).
    pub reference_document: String,
    /// Flap curves ordered by strictly ascending `flap_norm`.
    pub curves: Vec<PitotStaticCurve>,
}

impl PitotStaticCalibration {
    /// Parses and validates a [`PitotStaticCalibration`] JSON payload fail-closed.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, PackError> {
        let cal: Self =
            serde_json::from_slice(bytes).map_err(|source| PackError::CalibrationParse {
                path: PITOT_STATIC_CALIBRATION_ASSET.to_owned(),
                source,
            })?;
        cal.validate()?;
        Ok(cal)
    }

    /// Validates finiteness and strict monotonicity across flap settings and speed knots.
    pub fn validate(&self) -> Result<(), PackError> {
        if self.type_designator.trim().is_empty() {
            return Err(invalid("type_designator must not be empty"));
        }
        if self.curves.is_empty() {
            return Err(invalid("curves must contain at least one flap setting"));
        }
        let mut prev_flap: Option<f64> = None;
        for curve in &self.curves {
            validate_curve(curve, prev_flap)?;
            prev_flap = Some(curve.flap_norm);
        }
        Ok(())
    }

    /// Converts calibrated airspeed (`KCAS`, knots) to indicated airspeed (`KIAS`, knots)
    /// at normalized flap deflection `flap_norm` (`[0.0, 1.0]`).
    #[must_use]
    pub fn kcas_to_kias(&self, flap_norm: f64, kcas: f64) -> f64 {
        if !kcas.is_finite() || kcas <= 0.0 {
            return 0.0;
        }
        self.interpolate_flap(flap_norm, |curve| eval_curve_kcas_to_kias(curve, kcas))
            .unwrap_or(kcas)
    }

    /// Converts indicated airspeed (`KIAS`, knots) to calibrated airspeed (`KCAS`, knots)
    /// at normalized flap deflection `flap_norm` (`[0.0, 1.0]`).
    #[must_use]
    pub fn kias_to_kcas(&self, flap_norm: f64, kias: f64) -> f64 {
        if !kias.is_finite() || kias <= 0.0 {
            return 0.0;
        }
        self.interpolate_flap(flap_norm, |curve| eval_curve_kias_to_kcas(curve, kias))
            .unwrap_or(kias)
    }

    /// Converts calibrated airspeed (`m/s`) to indicated airspeed (`m/s`) at `flap_norm`.
    #[must_use]
    pub fn cas_to_ias_m_s(&self, flap_norm: f64, cas_m_s: f64) -> f64 {
        if !cas_m_s.is_finite() || cas_m_s <= 0.0 {
            return 0.0;
        }
        let kcas = cas_m_s / MPS_PER_KT;
        self.kcas_to_kias(flap_norm, kcas) * MPS_PER_KT
    }

    /// Maps physical flap deflection (`deg`) onto `[0.0, 1.0]` using the package's max flap angle.
    #[must_use]
    pub fn flap_norm_from_deg(&self, flap_deg: f64) -> f64 {
        if !flap_deg.is_finite() || flap_deg <= 0.0 {
            return 0.0;
        }
        let max_deg = self
            .curves
            .last()
            .map_or(30.0, |curve| curve.flap_deg.max(1.0));
        (flap_deg / max_deg).clamp(0.0, 1.0)
    }

    /// Converts calibrated airspeed (`m/s`) to indicated airspeed (`m/s`) at physical `flap_deg`.
    #[must_use]
    pub fn cas_to_ias_at_flap_deg_m_s(&self, flap_deg: f64, cas_m_s: f64) -> f64 {
        self.cas_to_ias_m_s(self.flap_norm_from_deg(flap_deg), cas_m_s)
    }

    fn interpolate_flap(
        &self,
        flap_norm: f64,
        eval: impl Fn(&PitotStaticCurve) -> f64,
    ) -> Option<f64> {
        let clamped = if flap_norm.is_finite() {
            flap_norm.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let first = self.curves.first()?;
        if clamped <= first.flap_norm {
            return Some(eval(first));
        }
        for pair in self.curves.windows(2) {
            if let [lo, hi] = pair
                && clamped <= hi.flap_norm
            {
                let span = hi.flap_norm - lo.flap_norm;
                let t = if span > f64::EPSILON {
                    ((clamped - lo.flap_norm) / span).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let v_lo = eval(lo);
                let v_hi = eval(hi);
                return Some(v_lo + (v_hi - v_lo) * t);
            }
        }
        self.curves.last().map(eval)
    }
}

fn validate_curve(curve: &PitotStaticCurve, prev_flap: Option<f64>) -> Result<(), PackError> {
    if !curve.flap_deg.is_finite() || !curve.flap_norm.is_finite() {
        return Err(invalid("flap_deg and flap_norm must be finite"));
    }
    if !(0.0..=1.0).contains(&curve.flap_norm) {
        return Err(invalid("flap_norm must lie within [0.0, 1.0]"));
    }
    if let Some(prev) = prev_flap
        && curve.flap_norm <= prev
    {
        return Err(invalid(
            "curves must be ordered by strictly ascending flap_norm",
        ));
    }
    if curve.points.len() < 2 {
        return Err(invalid("each PEC curve must have at least 2 points"));
    }
    let mut prev_pt: Option<AirspeedCalibrationPoint> = None;
    for pt in &curve.points {
        if !pt.kias.is_finite() || !pt.kcas.is_finite() || pt.kias < 0.0 || pt.kcas < 0.0 {
            return Err(invalid(
                "PEC points must have finite non-negative kias and kcas",
            ));
        }
        if let Some(prev) = prev_pt
            && (pt.kias <= prev.kias || pt.kcas <= prev.kcas)
        {
            return Err(invalid(
                "PEC points must be strictly increasing in both kias and kcas",
            ));
        }
        prev_pt = Some(*pt);
    }
    Ok(())
}

fn eval_curve_kcas_to_kias(curve: &PitotStaticCurve, kcas: f64) -> f64 {
    let Some(first) = curve.points.first() else {
        return kcas;
    };
    if kcas <= first.kcas {
        return if first.kcas > f64::EPSILON {
            (kcas / first.kcas) * first.kias
        } else {
            kcas
        };
    }
    for pair in curve.points.windows(2) {
        if let [lo, hi] = pair
            && kcas <= hi.kcas
        {
            let span = hi.kcas - lo.kcas;
            let t = if span > f64::EPSILON {
                (kcas - lo.kcas) / span
            } else {
                0.0
            };
            return lo.kias + (hi.kias - lo.kias) * t;
        }
    }
    if let Some(last) = curve.points.last() {
        return (kcas + (last.kias - last.kcas)).max(0.0);
    }
    kcas
}

fn eval_curve_kias_to_kcas(curve: &PitotStaticCurve, kias: f64) -> f64 {
    let Some(first) = curve.points.first() else {
        return kias;
    };
    if kias <= first.kias {
        return if first.kias > f64::EPSILON {
            (kias / first.kias) * first.kcas
        } else {
            kias
        };
    }
    for pair in curve.points.windows(2) {
        if let [lo, hi] = pair
            && kias <= hi.kias
        {
            let span = hi.kias - lo.kias;
            let t = if span > f64::EPSILON {
                (kias - lo.kias) / span
            } else {
                0.0
            };
            return lo.kcas + (hi.kcas - lo.kcas) * t;
        }
    }
    if let Some(last) = curve.points.last() {
        return (kias + (last.kcas - last.kias)).max(0.0);
    }
    kias
}

fn invalid(detail: &str) -> PackError {
    PackError::InvalidCalibration {
        path: PITOT_STATIC_CALIBRATION_ASSET.to_owned(),
        detail: detail.to_owned(),
    }
}
