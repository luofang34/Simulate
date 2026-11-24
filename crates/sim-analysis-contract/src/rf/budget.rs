//! Link-budget results. Every term is stated in the unit its name carries so a table can be
//! printed straight from the fields.

use serde::{Deserialize, Serialize};

use crate::{EndpointLinkReport, RfLinkSpec, TdbEpoch};

/// One extra propagation loss that entered a budget.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PropagationLossTerm {
    /// Model name.
    pub model: String,
    /// Loss (`dB`, `>= 0`).
    pub loss_db: f64,
}

/// A complete link budget at one epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkBudget {
    /// Slant range between transmit at emission and receive at reception (`m`).
    pub slant_range_m: f64,
    /// Range rate (`m/s`, positive when receding).
    pub range_rate_m_s: f64,
    /// One-way light time when the signal-time model is `LightTime` (`s`).
    pub light_time_s: Option<f64>,
    /// First-order Doppler shift $-f\dot\rho/c$ (`Hz`).
    pub doppler_shift_hz: f64,
    /// Carrier as received (`Hz`).
    pub received_frequency_hz: f64,
    /// Transmit power (`dBW`).
    pub transmit_power_dbw: f64,
    /// Transmit line loss (`dB`).
    pub transmit_line_loss_db: f64,
    /// Transmit antenna gain toward the receiver (`dBi`).
    pub transmit_antenna_gain_dbi: f64,
    /// Angle between transmit boresight and the receiver (`rad`).
    pub transmit_off_axis_rad: f64,
    /// $P_t - L_{line} + G_t$ (`dBW`).
    pub eirp_dbw: f64,
    /// $20\log_{10}(4\pi\rho f/c)$ (`dB`).
    pub free_space_path_loss_db: f64,
    /// Extra propagation losses.
    pub propagation_losses: Vec<PropagationLossTerm>,
    /// Polarization mismatch (`dB`).
    pub polarization_mismatch_loss_db: f64,
    /// EIRP less every path loss, before the receive antenna (`dBW`).
    pub received_isotropic_power_dbw: f64,
    /// Receive antenna gain toward the transmitter (`dBi`).
    pub receive_antenna_gain_dbi: f64,
    /// Angle between receive boresight and the transmitter (`rad`).
    pub receive_off_axis_rad: f64,
    /// Receive line loss (`dB`).
    pub receive_line_loss_db: f64,
    /// Carrier power at the receiver reference plane (`dBW`).
    pub received_power_dbw: f64,
    /// $T_{sys}$ (`K`).
    pub system_noise_temperature_k: f64,
    /// $G/T$ toward the transmitter, line loss included (`dB/K`).
    pub gain_over_temperature_db_per_k: f64,
    /// $N_0 = kT_{sys}$ (`dBW/Hz`).
    pub noise_spectral_density_dbw_hz: f64,
    /// $C/N_0$ (`dB-Hz`).
    pub c_over_n0_dbhz: f64,
    /// $10\log_{10}R_b$ (`dB-Hz`).
    pub data_rate_dbhz: f64,
    /// Implementation loss (`dB`).
    pub implementation_loss_db: f64,
    /// Achieved $E_b/N_0$ (`dB`).
    pub eb_n0_db: f64,
    /// Required $E_b/N_0$ (`dB`).
    pub required_eb_n0_db: f64,
    /// `eb_n0_db - required_eb_n0_db` (`dB`).
    pub margin_db: f64,
    /// Shannon capacity $B\log_2(1 + C/N)$ over the link bandwidth (`bit/s`).
    pub shannon_capacity_bps: f64,
}

/// Which end of the link a condition concerns.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LinkEnd {
    /// The transmitting (source) end.
    Transmitter,
    /// The receiving (target) end.
    Receiver,
}

/// Why no budget was produced at an epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BudgetUnavailableReason {
    /// A geometric restriction (occultation, mask, field of view, range) holds.
    NotAccessible,
    /// A propagation model is not valid for this geometry.
    PropagationModelInapplicable {
        /// Model name.
        model: String,
        /// Why it does not apply.
        reason: String,
    },
    /// The two ends coincide, so range and path loss are undefined.
    DegenerateGeometry,
    /// A fixed-mount directional antenna needs an attitude source that was not supplied.
    AntennaOrientationUnavailable {
        /// End lacking orientation.
        end: LinkEnd,
    },
}

/// Budget (or the reason for none) at one grid epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkBudgetSample {
    /// Grid epoch (`TDB`).
    pub epoch: TdbEpoch,
    /// Budget, present only when the link is geometrically accessible and every model applies.
    pub budget: Option<LinkBudget>,
    /// Why `budget` is absent.
    pub unavailable: Option<BudgetUnavailableReason>,
}

/// Geometric access report together with the link budget on every grid epoch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkBudgetReport {
    /// The link evaluated.
    pub link: RfLinkSpec,
    /// Access samples and windows; its scope states that a budget was evaluated.
    pub access: EndpointLinkReport,
    /// One entry per `access.samples` entry, in order.
    pub samples: Vec<LinkBudgetSample>,
}
