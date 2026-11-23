//! Radio link description: carrier, endpoints, losses, and requirement.

use serde::{Deserialize, Serialize};

use crate::{
    ModelNotApplicableReason,
    rf::{AntennaSpec, PropagationModelSpec, ReceiverNoise},
};

/// How an antenna is aimed.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum AntennaPointing {
    /// Boresight is the mount frame's `+Z`; the vehicle attitude decides where it looks.
    FixedInMount,
    /// A gimbal or steered beam holds boresight on the far end, off by `pointing_error_rad`
    /// along the antenna `+X` axis (a pessimistic, deterministic choice for patterns that are not axisymmetric).
    TrackingTarget {
        /// Residual angle between boresight and the far end (`rad`, `>= 0`).
        pointing_error_rad: f64,
    },
}

/// Which way a request's platform and station communicate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LinkDirection {
    /// The platform transmits to the station.
    Downlink,
    /// The station transmits to the platform.
    Uplink,
}

/// The transmitting end of a link.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RfTransmitterSpec {
    /// Power delivered to the antenna feed before line loss (`W`, `> 0`).
    pub transmit_power_w: f64,
    /// Antenna pattern in the mount frame.
    pub antenna: AntennaSpec,
    /// Aiming.
    pub pointing: AntennaPointing,
    /// Waveguide, diplexer and radome loss between amplifier and antenna (`dB`, `>= 0`).
    pub line_loss_db: f64,
}

/// The receiving end of a link.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RfReceiverSpec {
    /// Antenna pattern in the mount frame.
    pub antenna: AntennaSpec,
    /// Aiming.
    pub pointing: AntennaPointing,
    /// Loss between antenna and the reference plane of the noise figure (`dB`, `>= 0`).
    pub line_loss_db: f64,
    /// Noise floor.
    pub noise: ReceiverNoise,
}

/// One radio link: the source endpoint of the geometric analysis transmits, the target receives.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RfLinkSpec {
    /// Carrier frequency at the transmitter (`Hz`, `> 0`).
    pub carrier_frequency_hz: f64,
    /// Noise bandwidth for the Shannon cross-check (`Hz`, `> 0`).
    pub bandwidth_hz: f64,
    /// Information bit rate (`bit/s`, `> 0`).
    pub data_rate_bps: f64,
    /// $E_b/N_0$ the modem needs for its target error rate (`dB`).
    pub required_eb_n0_db: f64,
    /// Loss from imperfect polarization alignment (`dB`, `>= 0`).
    pub polarization_mismatch_loss_db: f64,
    /// Modem and filtering loss applied to $E_b/N_0$ (`dB`, `>= 0`).
    pub implementation_loss_db: f64,
    /// Transmitting end.
    pub transmitter: RfTransmitterSpec,
    /// Receiving end.
    pub receiver: RfReceiverSpec,
    /// Extra propagation losses, summed.
    pub propagation: Vec<PropagationModelSpec>,
}

fn require(ok: bool, what: &str) -> Result<(), ModelNotApplicableReason> {
    if ok {
        Ok(())
    } else {
        Err(ModelNotApplicableReason::InvalidRfLinkParameters {
            reason: what.to_owned(),
        })
    }
}

fn positive(x: f64) -> bool {
    x.is_finite() && x > 0.0
}

fn non_negative(x: f64) -> bool {
    x.is_finite() && x >= 0.0
}

impl AntennaSpec {
    fn validate(&self) -> Result<(), ModelNotApplicableReason> {
        match *self {
            Self::Isotropic => Ok(()),
            Self::ParabolicDish(d) => require(
                positive(d.diameter_m)
                    && positive(d.aperture_efficiency)
                    && d.aperture_efficiency <= 1.0
                    && positive(d.sidelobe_floor_below_peak_db),
                "parabolic dish needs diameter > 0, 0 < efficiency <= 1, floor depth > 0",
            ),
            Self::CosinePowerPatch(p) => require(
                p.peak_gain_dbi.is_finite()
                    && non_negative(p.cosine_exponent)
                    && p.back_hemisphere_floor_dbi.is_finite(),
                "patch needs finite gains and cosine exponent >= 0",
            ),
        }
    }
}

fn validate_pointing(p: &AntennaPointing) -> Result<(), ModelNotApplicableReason> {
    match *p {
        AntennaPointing::FixedInMount => Ok(()),
        AntennaPointing::TrackingTarget { pointing_error_rad } => require(
            non_negative(pointing_error_rad) && pointing_error_rad < core::f64::consts::PI,
            "pointing error must lie in [0, pi)",
        ),
    }
}

impl RfLinkSpec {
    /// Checks every parameter is finite and physically admissible.
    pub fn validate(&self) -> Result<(), ModelNotApplicableReason> {
        require(
            positive(self.carrier_frequency_hz)
                && positive(self.bandwidth_hz)
                && positive(self.data_rate_bps),
            "carrier frequency, bandwidth and data rate must be > 0",
        )?;
        require(
            self.required_eb_n0_db.is_finite()
                && non_negative(self.polarization_mismatch_loss_db)
                && non_negative(self.implementation_loss_db)
                && non_negative(self.transmitter.line_loss_db)
                && non_negative(self.receiver.line_loss_db),
            "losses must be finite and >= 0",
        )?;
        require(
            positive(self.transmitter.transmit_power_w),
            "transmit power must be > 0 W",
        )?;
        self.transmitter.antenna.validate()?;
        self.receiver.antenna.validate()?;
        validate_pointing(&self.transmitter.pointing)?;
        validate_pointing(&self.receiver.pointing)?;
        match self.receiver.noise {
            ReceiverNoise::SystemTemperature {
                system_noise_temperature_k,
            } => require(positive(system_noise_temperature_k), "Tsys must be > 0 K"),
            ReceiverNoise::GainOverTemperature { g_over_t_db_per_k } => {
                require(g_over_t_db_per_k.is_finite(), "G/T must be finite")
            }
        }?;
        for model in &self.propagation {
            let PropagationModelSpec::TroposphericCosecant {
                zenith_attenuation_db,
                min_elevation_rad,
            } = *model;
            require(
                non_negative(zenith_attenuation_db)
                    && positive(min_elevation_rad)
                    && min_elevation_rad <= core::f64::consts::FRAC_PI_2,
                "troposphere needs zenith attenuation >= 0 and min elevation in (0, pi/2]",
            )?;
        }
        Ok(())
    }
}
