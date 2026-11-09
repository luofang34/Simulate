use super::{Microbolometer, MicrobolometerFrame, SceneTemperature};
use crate::RenderError;

#[cfg(test)]
mod tests;

/// A scene held for part of a frame: the input of
/// [`Microbolometer::capture_lagged`].
#[derive(Debug, Clone, Copy)]
pub struct SceneInterval<'a> {
    /// The brightness temperature reaching each pixel during the interval.
    pub scene: &'a SceneTemperature,
    /// How long the scene is held, seconds.
    pub duration_s: f64,
}

/// The temperature each pixel's membrane has reached, kelvin: the state a
/// microbolometer carries from frame to frame. One per sensor instance, so
/// a rerun from a fresh state repeats exactly.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ThermalLag {
    detector_k: Vec<f64>,
}

impl ThermalLag {
    /// A detector that has not seen a scene yet; its first frame starts
    /// settled on the first scene it is given.
    pub fn new() -> Self {
        Self::default()
    }

    /// The temperature each pixel reached at the last readout, row-major;
    /// empty before the first frame. Simulation state, not sensor output.
    pub fn detector_kelvin(&self) -> &[f64] {
        &self.detector_k
    }
}

impl Microbolometer {
    /// Reads out frame `sequence` at the end of `intervals`, which cover the
    /// time since the previous readout in order.
    ///
    /// Each pixel follows the scene through a first-order lag of
    /// [`MicrobolometerConfig::time_constant_s`](super::MicrobolometerConfig::time_constant_s):
    /// over an interval of length `Δ` at scene temperature `T`, the detector
    /// moves from `D` to `T + (D − T)·exp(−Δ/τ)`. So a scene that moves
    /// within the frame leaves an exponential trail behind it, and one that
    /// changes from frame to frame is followed late. Give the scene at
    /// sub-steps closer than `τ` for a smooth trail. The frame's truth is the
    /// scene of the last interval, the one in view at readout.
    ///
    /// Fails if there is no interval, a duration is negative or not finite,
    /// a scene does not fit the detector, or `lag` holds another detector's
    /// state. On a failure `lag` is unchanged.
    pub fn capture_lagged(
        &self,
        lag: &mut ThermalLag,
        intervals: &[SceneInterval<'_>],
        sequence: u64,
    ) -> Result<MicrobolometerFrame, RenderError> {
        let Some(last) = intervals.last() else {
            return Err(self.lag_error("no scene intervals".to_owned()));
        };
        for interval in intervals {
            self.check_scene(interval.scene)?;
            if !(interval.duration_s.is_finite() && interval.duration_s >= 0.0) {
                return Err(self.lag_error(format!("interval of {} s", interval.duration_s)));
            }
        }
        let pixels = self.gain.len();
        if !lag.detector_k.is_empty() && lag.detector_k.len() != pixels {
            return Err(self.lag_error(format!(
                "lag state of {} pixels for {pixels}",
                lag.detector_k.len()
            )));
        }
        if lag.detector_k.is_empty() {
            lag.detector_k = intervals
                .first()
                .map(|first| first.scene.kelvin.iter().map(|&k| f64::from(k)).collect())
                .unwrap_or_default();
        }
        for interval in intervals {
            let retained = (-interval.duration_s / self.config.time_constant_s).exp();
            for (detector, &scene) in lag.detector_k.iter_mut().zip(&interval.scene.kelvin) {
                let scene = f64::from(scene);
                *detector = scene + (*detector - scene) * retained;
            }
        }
        Ok(self.frame(&lag.detector_k, last.scene, sequence))
    }

    fn lag_error(&self, detail: String) -> RenderError {
        RenderError::InvalidSensor {
            sensor: "microbolometer",
            detail,
        }
    }
}
