//! The declared parameters of an `ace-rs` design that its flight can measure,
//! converted to SI from the units the design states.

use serde_json::Value;

use super::AceBridgeError;
use crate::calibration::glide::measure_reference_geometry;
use crate::calibration::results::ReferenceGeometry;
use crate::calibration::solver::build_fdm;
use crate::{LoadedSyspack, PackError};

const M_PER_FT: f64 = 0.3048;
const KG_PER_LB: f64 = 0.453_592_37;
const KWH_PER_J: f64 = 1.0 / 3.6e6;

/// The parameters of an `ace-rs` `vehicle.json` that the comparison reads,
/// in SI. Each is the declared value, never a default.
#[derive(Debug, Clone, PartialEq)]
pub struct AceVehicleInputs {
    /// `id` of the design, which an `ace-export` package carries as its own.
    pub id: String,
    /// Display name of the design.
    pub name: String,
    /// `engine.count`.
    pub engine_count: usize,
    /// `wing.area` (`m^2`).
    pub wing_area_m2: f64,
    /// `wing.span` (`m`).
    pub wing_span_m: f64,
    /// `aero.cd0`.
    pub cd0: f64,
    /// `aero.oswald`.
    pub oswald: f64,
    /// `propeller.efficiency`, when the design has propellers.
    pub propeller_efficiency: Option<f64>,
    /// `engine.bsfc` (`kg/kWh`), when the design burns fuel in a shaft engine.
    pub bsfc_kg_per_kwh: Option<f64>,
}

#[derive(Clone, Copy)]
enum Dimension {
    Ratio,
    Length,
    Area,
    SpecificFuel,
}

impl Dimension {
    fn name(self) -> &'static str {
        match self {
            Self::Ratio => "dimensionless",
            Self::Length => "length",
            Self::Area => "area",
            Self::SpecificFuel => "specific fuel consumption",
        }
    }

    /// Factor from `unit` to this dimension's SI unit (`kg/kWh` for fuel).
    fn factor(self, unit: &str) -> Option<f64> {
        match (self, unit) {
            (Self::Ratio, "1" | "dimensionless") => Some(1.0),
            (Self::Length, "m") => Some(1.0),
            (Self::Length, "ft") => Some(M_PER_FT),
            (Self::Area, "m^2") => Some(1.0),
            (Self::Area, "ft^2") => Some(M_PER_FT * M_PER_FT),
            (Self::SpecificFuel, "kg/kWh") => Some(1.0),
            (Self::SpecificFuel, "kg/J") => Some(1.0 / KWH_PER_J),
            (Self::SpecificFuel, "lb/(hp*h)") => Some(KG_PER_LB / 0.745_699_872),
            _ => None,
        }
    }
}

impl AceVehicleInputs {
    /// Reads the design at `path` (named in errors) from `bytes`.
    pub fn from_vehicle_json(path: &str, bytes: &[u8]) -> Result<Self, AceBridgeError> {
        let doc: Value = serde_json::from_slice(bytes).map_err(|source| AceBridgeError::Json {
            path: path.to_owned(),
            source,
        })?;
        let field = |field: &'static str| {
            doc.get(field)
                .and_then(Value::as_str)
                .map(str::to_owned)
                .ok_or(AceBridgeError::MissingField {
                    path: path.to_owned(),
                    field,
                })
        };
        let id = field("id")?;
        let name = field("name")?;
        let params = doc
            .get("parameters")
            .ok_or_else(|| AceBridgeError::MissingField {
                path: path.to_owned(),
                field: "parameters",
            })?;
        let reader = Reader { id: &id, params };
        let count = reader.required("engine.count", Dimension::Ratio)?;
        if count < 1.0 || count.fract() != 0.0 || count > 64.0 {
            return Err(AceBridgeError::InvalidEngineCount {
                vehicle_id: id,
                count,
            });
        }
        Ok(Self {
            engine_count: count as usize,
            wing_area_m2: reader.required("wing.area", Dimension::Area)?,
            wing_span_m: reader.required("wing.span", Dimension::Length)?,
            cd0: reader.required("aero.cd0", Dimension::Ratio)?,
            oswald: reader.required("aero.oswald", Dimension::Ratio)?,
            propeller_efficiency: reader.optional("propeller.efficiency", Dimension::Ratio)?,
            bsfc_kg_per_kwh: reader.optional("engine.bsfc", Dimension::SpecificFuel)?,
            id,
            name,
        })
    }

    /// Checks that `package` was exported from this design and flies its
    /// engine count, before any flight is spent on it.
    pub fn check_package(&self, package: &LoadedSyspack) -> Result<(), PackError> {
        let mut fdm = build_fdm(package)?;
        let reference = measure_reference_geometry(&mut fdm)?;
        Ok(self.check_flown(&package.manifest().id, &reference)?)
    }

    /// Checks a package identity and flown geometry against this design.
    pub(super) fn check_flown(
        &self,
        package_id: &str,
        reference: &ReferenceGeometry,
    ) -> Result<(), AceBridgeError> {
        if package_id != self.id {
            return Err(AceBridgeError::PackageMismatch {
                vehicle_id: self.id.clone(),
                package_id: package_id.to_owned(),
            });
        }
        if reference.engine_count != self.engine_count {
            return Err(AceBridgeError::EngineCountMismatch {
                vehicle_id: self.id.clone(),
                package_id: package_id.to_owned(),
                declared: self.engine_count,
                flown: reference.engine_count,
            });
        }
        Ok(())
    }
}

struct Reader<'a> {
    id: &'a str,
    params: &'a Value,
}

impl Reader<'_> {
    fn required(&self, name: &str, dimension: Dimension) -> Result<f64, AceBridgeError> {
        self.optional(name, dimension)?
            .ok_or_else(|| AceBridgeError::MissingParameter {
                vehicle_id: self.id.to_owned(),
                parameter: name.to_owned(),
            })
    }

    /// The parameter's value in SI, or `None` when the design gives it no
    /// numeric value. The value's own unit wins over the declared one.
    fn optional(&self, name: &str, dimension: Dimension) -> Result<Option<f64>, AceBridgeError> {
        let Some(param) = self.params.get(name) else {
            return Ok(None);
        };
        let Some(quantity) = param.get("value") else {
            return Ok(None);
        };
        let Some(value) = quantity.get("value").and_then(Value::as_f64) else {
            return Ok(None);
        };
        let unit = quantity
            .get("unit")
            .or_else(|| param.get("unit"))
            .and_then(Value::as_str)
            .unwrap_or("");
        let factor = dimension
            .factor(unit)
            .ok_or_else(|| AceBridgeError::UnsupportedUnit {
                vehicle_id: self.id.to_owned(),
                parameter: name.to_owned(),
                unit: unit.to_owned(),
                dimension: dimension.name(),
            })?;
        Ok(Some(value * factor))
    }
}
