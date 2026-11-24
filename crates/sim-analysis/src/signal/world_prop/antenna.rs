//! ITU-R reference radiation patterns (`world-propagation`) behind [`AntennaPattern`].

use glam::DVec3;
use sim_spacetime::{acos_clamped, sqrt};
use world_propagation::antenna::{AntennaPattern as WorldAntennaPattern, F1245, S465, S580};
use world_propagation::{Frequency, PropagationError};

use crate::signal::AntennaPattern;

/// How ITU-R S.465 / S.580 sidelobe coordination envelopes behave inside the undefined main beam (`off_axis_deg < min_angle_deg`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum MainBeamPolicy {
    /// Fail closed: return `f64::NAN` from [`AntennaPattern::gain_dbi`] inside `min_angle_deg`, matching [`WorldAntennaPattern::gain_dbi`].
    #[default]
    Refuse,
    /// Hold the sidelobe envelope boundary gain at `min_angle_deg` inside the main beam.
    HoldMinAngle,
    /// Use an explicit boresight peak gain (`dBi`) inside the main beam (`< min_angle_deg`).
    ConstantBoresightPeak(f64),
}

fn off_axis_deg(direction_in_antenna_frame: DVec3) -> f64 {
    let len = sqrt(direction_in_antenna_frame.length_squared());
    if len <= 1.0e-15 {
        0.0
    } else {
        acos_clamped(direction_in_antenna_frame.z / len)
            .to_degrees()
            .clamp(0.0, 180.0)
    }
}

/// ITU-R S.465-6 FSS earth-station reference sidelobe pattern (`2..=31 GHz`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct S465Pattern {
    diameter_m: f64,
    main_beam_policy: MainBeamPolicy,
}

impl S465Pattern {
    /// Constructs a validated ITU-R S.465-6 pattern, checking `diameter_m` at `validation_frequency_hz`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] if `diameter_m` is non-positive or `validation_frequency_hz` is outside `2..=31 GHz`.
    pub fn new(diameter_m: f64, validation_frequency_hz: f64) -> Result<Self, PropagationError> {
        let freq = Frequency::from_hz(validation_frequency_hz)?;
        let _check = S465::new(diameter_m, freq)?;
        Ok(Self {
            diameter_m,
            main_beam_policy: MainBeamPolicy::Refuse,
        })
    }

    /// Configures how angles inside the undefined main beam (`< min_angle_deg`) behave when evaluated through [`AntennaPattern`].
    #[must_use]
    pub const fn with_main_beam_policy(mut self, policy: MainBeamPolicy) -> Self {
        self.main_beam_policy = policy;
        self
    }

    /// Evaluates the exact ITU-R S.465-6 envelope, returning [`PropagationError::AngleOutsidePattern`] inside `min_angle_deg`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] on out-of-band frequencies (`2..=31 GHz`) or angles inside `min_angle_deg`.
    pub fn try_gain_dbi(
        &self,
        direction_in_antenna_frame: DVec3,
        carrier_frequency_hz: f64,
    ) -> Result<f64, PropagationError> {
        let model = S465::new(self.diameter_m, Frequency::from_hz(carrier_frequency_hz)?)?;
        model.gain_dbi(off_axis_deg(direction_in_antenna_frame))
    }
}

impl AntennaPattern for S465Pattern {
    fn gain_dbi(&self, direction_in_antenna_frame: DVec3, carrier_frequency_hz: f64) -> f64 {
        let Ok(freq) = Frequency::from_hz(carrier_frequency_hz) else {
            return f64::NAN;
        };
        let Ok(model) = S465::new(self.diameter_m, freq) else {
            return f64::NAN;
        };
        let phi = off_axis_deg(direction_in_antenna_frame);
        match model.gain_dbi(phi) {
            Ok(gain) => gain,
            Err(PropagationError::AngleOutsidePattern { .. }) => match self.main_beam_policy {
                MainBeamPolicy::Refuse => f64::NAN,
                MainBeamPolicy::HoldMinAngle => {
                    model.gain_dbi(model.min_angle_deg()).unwrap_or(f64::NAN)
                }
                MainBeamPolicy::ConstantBoresightPeak(peak) => peak,
            },
            Err(_) => f64::NAN,
        }
    }

    fn peak_gain_dbi(&self, carrier_frequency_hz: f64) -> f64 {
        if let MainBeamPolicy::ConstantBoresightPeak(peak) = self.main_beam_policy {
            return peak;
        }
        let Ok(freq) = Frequency::from_hz(carrier_frequency_hz) else {
            return f64::NAN;
        };
        S465::new(self.diameter_m, freq)
            .and_then(|model| model.gain_dbi(model.min_angle_deg()))
            .unwrap_or(f64::NAN)
    }
}

/// ITU-R S.580-6 design-objective earth-station sidelobe pattern (`2..=31 GHz`, $D/\lambda \ge 50$).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct S580Pattern {
    diameter_m: f64,
    main_beam_policy: MainBeamPolicy,
}

impl S580Pattern {
    /// Constructs a validated ITU-R S.580-6 pattern, checking `diameter_m` and $D/\lambda \ge 50$ at `validation_frequency_hz`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] if `diameter_m` is invalid, `validation_frequency_hz` is outside `2..=31 GHz`, or $D/\lambda < 50$.
    pub fn new(diameter_m: f64, validation_frequency_hz: f64) -> Result<Self, PropagationError> {
        let freq = Frequency::from_hz(validation_frequency_hz)?;
        let _check = S580::new(diameter_m, freq)?;
        Ok(Self {
            diameter_m,
            main_beam_policy: MainBeamPolicy::Refuse,
        })
    }

    /// Configures how angles inside the undefined main beam (`< min_angle_deg`) behave when evaluated through [`AntennaPattern`].
    #[must_use]
    pub const fn with_main_beam_policy(mut self, policy: MainBeamPolicy) -> Self {
        self.main_beam_policy = policy;
        self
    }

    /// Evaluates the exact ITU-R S.580-6 envelope, returning [`PropagationError::AngleOutsidePattern`] inside `min_angle_deg`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] on out-of-band frequencies, $D/\lambda < 50$, or angles inside `min_angle_deg`.
    pub fn try_gain_dbi(
        &self,
        direction_in_antenna_frame: DVec3,
        carrier_frequency_hz: f64,
    ) -> Result<f64, PropagationError> {
        let model = S580::new(self.diameter_m, Frequency::from_hz(carrier_frequency_hz)?)?;
        model.gain_dbi(off_axis_deg(direction_in_antenna_frame))
    }
}

impl AntennaPattern for S580Pattern {
    fn gain_dbi(&self, direction_in_antenna_frame: DVec3, carrier_frequency_hz: f64) -> f64 {
        let Ok(freq) = Frequency::from_hz(carrier_frequency_hz) else {
            return f64::NAN;
        };
        let Ok(model) = S580::new(self.diameter_m, freq) else {
            return f64::NAN;
        };
        let phi = off_axis_deg(direction_in_antenna_frame);
        match model.gain_dbi(phi) {
            Ok(gain) => gain,
            Err(PropagationError::AngleOutsidePattern { .. }) => match self.main_beam_policy {
                MainBeamPolicy::Refuse => f64::NAN,
                MainBeamPolicy::HoldMinAngle => {
                    model.gain_dbi(model.min_angle_deg()).unwrap_or(f64::NAN)
                }
                MainBeamPolicy::ConstantBoresightPeak(peak) => peak,
            },
            Err(_) => f64::NAN,
        }
    }

    fn peak_gain_dbi(&self, carrier_frequency_hz: f64) -> f64 {
        if let MainBeamPolicy::ConstantBoresightPeak(peak) = self.main_beam_policy {
            return peak;
        }
        let Ok(freq) = Frequency::from_hz(carrier_frequency_hz) else {
            return f64::NAN;
        };
        S580::new(self.diameter_m, freq)
            .and_then(|model| model.gain_dbi(model.min_angle_deg()))
            .unwrap_or(f64::NAN)
    }
}

/// ITU-R F.1245-3 point-to-point fixed-wireless antenna pattern (`1..=86 GHz`, `0..=180°`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct F1245Pattern {
    diameter_m: f64,
    max_gain_dbi: Option<f64>,
}

impl F1245Pattern {
    /// Constructs a validated ITU-R F.1245-3 pattern deriving maximum gain from `diameter_m` and `validation_frequency_hz`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] on invalid `diameter_m` or frequency outside `1..=86 GHz`.
    pub fn new(diameter_m: f64, validation_frequency_hz: f64) -> Result<Self, PropagationError> {
        let freq = Frequency::from_hz(validation_frequency_hz)?;
        let _check = F1245::new(diameter_m, freq)?;
        Ok(Self {
            diameter_m,
            max_gain_dbi: None,
        })
    }

    /// Constructs a validated ITU-R F.1245-3 pattern with an explicit boresight `max_gain_dbi`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] if `max_gain_dbi` is not above the first side-lobe level or inputs are out of range.
    pub fn with_max_gain(
        diameter_m: f64,
        validation_frequency_hz: f64,
        max_gain_dbi: f64,
    ) -> Result<Self, PropagationError> {
        let freq = Frequency::from_hz(validation_frequency_hz)?;
        let _check = F1245::with_max_gain(diameter_m, freq, max_gain_dbi)?;
        Ok(Self {
            diameter_m,
            max_gain_dbi: Some(max_gain_dbi),
        })
    }

    fn model_at(&self, carrier_frequency_hz: f64) -> Result<F1245, PropagationError> {
        let freq = Frequency::from_hz(carrier_frequency_hz)?;
        match self.max_gain_dbi {
            Some(g_max) => F1245::with_max_gain(self.diameter_m, freq, g_max),
            None => F1245::new(self.diameter_m, freq),
        }
    }

    /// Evaluates the ITU-R F.1245-3 gain pattern with typed [`PropagationError`] diagnostics.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] if `carrier_frequency_hz` or aperture gain headroom is out of bounds.
    pub fn try_gain_dbi(
        &self,
        direction_in_antenna_frame: DVec3,
        carrier_frequency_hz: f64,
    ) -> Result<f64, PropagationError> {
        self.model_at(carrier_frequency_hz)?
            .gain_dbi(off_axis_deg(direction_in_antenna_frame))
    }
}

impl AntennaPattern for F1245Pattern {
    fn gain_dbi(&self, direction_in_antenna_frame: DVec3, carrier_frequency_hz: f64) -> f64 {
        self.try_gain_dbi(direction_in_antenna_frame, carrier_frequency_hz)
            .unwrap_or(f64::NAN)
    }

    fn peak_gain_dbi(&self, carrier_frequency_hz: f64) -> f64 {
        self.model_at(carrier_frequency_hz)
            .map_or(f64::NAN, |model| model.max_gain_dbi())
    }
}
