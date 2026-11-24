//! Serializable antenna descriptions. Gain evaluation is `sim-analysis`'s `AntennaPattern`.

use serde::{Deserialize, Serialize};

/// An antenna whose gain is a closed-form function of the angle off its `+Z` boresight.
///
/// Every variant is axisymmetric about boresight; patterns that are not (measured or
/// file-backed) implement `AntennaPattern` directly instead of adding a variant here.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum AntennaSpec {
    /// Unit gain in every direction.
    Isotropic,
    /// Circular parabolic reflector.
    ParabolicDish(ParabolicDishSpec),
    /// Planar patch-style radiator with a cosine-power main lobe.
    CosinePowerPatch(CosinePowerPatchSpec),
}

/// Circular parabolic reflector: boresight gain $\eta(\pi D/\lambda)^2$ and a Gaussian
/// main lobe $-12(\theta/\theta_{3\mathrm{dB}})^2$ dB with $\theta_{3\mathrm{dB}} = 70\lambda/D$ degrees.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParabolicDishSpec {
    /// Aperture diameter (`m`).
    pub diameter_m: f64,
    /// Aperture efficiency $\eta \in (0, 1]$.
    pub aperture_efficiency: f64,
    /// Gain floor below the peak (`dB`, `> 0`) that stands in for the side-lobe envelope once
    /// the Gaussian main lobe has fallen past it.
    pub sidelobe_floor_below_peak_db: f64,
}

/// Patch-style antenna: $G(\theta) = G_0 \cos^q\theta$ for $\theta < 90^\circ$, floored.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CosinePowerPatchSpec {
    /// Boresight gain (`dBi`).
    pub peak_gain_dbi: f64,
    /// Cosine exponent $q \ge 0$; `0` is a constant gain over the front hemisphere.
    pub cosine_exponent: f64,
    /// Gain behind the ground plane and wherever the cosine law falls below it (`dBi`).
    pub back_hemisphere_floor_dbi: f64,
}
