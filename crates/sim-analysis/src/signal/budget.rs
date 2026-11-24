//! The link-budget arithmetic for one epoch.

use glam::DVec3;
use sim_analysis_contract::{
    AntennaPointing, BOLTZMANN_CONSTANT_J_K, BudgetUnavailableReason, LinkBudget, LinkEnd,
    PropagationLossTerm, ReceiverNoise, RfLinkSpec,
};
use sim_spacetime::{acos_clamped, ln, sin_cos, sqrt};

use super::{
    antenna::AntennaPattern,
    path::{PathGeometry, PropagationLoss, doppler_shift_hz, free_space_path_loss_db},
    units::{db_from_ratio, ratio_from_db},
};

/// Everything the budget of one epoch is computed from.
pub struct LinkBudgetInputs<'a> {
    /// Radio link description.
    pub link: &'a RfLinkSpec,
    /// Transmit antenna pattern.
    pub transmit_antenna: &'a dyn AntennaPattern,
    /// Receive antenna pattern.
    pub receive_antenna: &'a dyn AntennaPattern,
    /// Extra propagation losses.
    pub propagation: &'a [&'a dyn PropagationLoss],
    /// Path geometry.
    pub path: &'a PathGeometry,
    /// Unit direction from transmitter to receiver in the transmit mount frame.
    pub transmit_look_in_mount: Option<DVec3>,
    /// Unit direction from receiver to transmitter in the receive mount frame.
    pub receive_look_in_mount: Option<DVec3>,
}

/// Direction in the antenna frame that gain is evaluated at.
fn aim(
    pointing: AntennaPointing,
    antenna: &dyn AntennaPattern,
    look: Option<DVec3>,
    end: LinkEnd,
) -> Result<DVec3, BudgetUnavailableReason> {
    match pointing {
        AntennaPointing::TrackingTarget { pointing_error_rad } => {
            let (s, c) = sin_cos(pointing_error_rad);
            Ok(DVec3::new(s, 0.0, c))
        }
        AntennaPointing::FixedInMount => match look {
            Some(v) => Ok(v),
            None if !antenna.requires_orientation() => Ok(DVec3::Z),
            None => Err(BudgetUnavailableReason::AntennaOrientationUnavailable { end }),
        },
    }
}

fn off_axis_rad(direction: DVec3) -> f64 {
    let len = sqrt(direction.length_squared());
    if len <= 1.0e-15 {
        0.0
    } else {
        acos_clamped(direction.z / len)
    }
}

fn system_temperature_k(link: &RfLinkSpec, rx_peak_gain_dbi: f64) -> f64 {
    match link.receiver.noise {
        ReceiverNoise::SystemTemperature {
            system_noise_temperature_k,
        } => system_noise_temperature_k,
        ReceiverNoise::GainOverTemperature { g_over_t_db_per_k } => {
            ratio_from_db(rx_peak_gain_dbi - link.receiver.line_loss_db - g_over_t_db_per_k)
        }
    }
}

fn sum_propagation(
    models: &[&dyn PropagationLoss],
    path: &PathGeometry,
) -> Result<Vec<PropagationLossTerm>, BudgetUnavailableReason> {
    models
        .iter()
        .map(|m| {
            m.loss_db(path)
                .map(|loss_db| PropagationLossTerm {
                    model: m.name(),
                    loss_db,
                })
                .map_err(|e| BudgetUnavailableReason::PropagationModelInapplicable {
                    model: m.name(),
                    reason: e.to_string(),
                })
        })
        .collect()
}

/// Evaluates the budget, or says why there is none.
pub fn compute_link_budget(
    inputs: &LinkBudgetInputs<'_>,
) -> Result<LinkBudget, BudgetUnavailableReason> {
    let LinkBudgetInputs { link, path, .. } = *inputs;
    let f_hz = path.carrier_frequency_hz;
    if !(path.slant_range_m.is_finite() && path.slant_range_m > 0.0) {
        return Err(BudgetUnavailableReason::DegenerateGeometry);
    }
    let tx = &link.transmitter;
    let rx = &link.receiver;
    let tx_dir = aim(
        tx.pointing,
        inputs.transmit_antenna,
        inputs.transmit_look_in_mount,
        LinkEnd::Transmitter,
    )?;
    let rx_dir = aim(
        rx.pointing,
        inputs.receive_antenna,
        inputs.receive_look_in_mount,
        LinkEnd::Receiver,
    )?;
    let tx_gain_dbi = inputs.transmit_antenna.gain_dbi(tx_dir, f_hz);
    let rx_gain_dbi = inputs.receive_antenna.gain_dbi(rx_dir, f_hz);
    let propagation_losses = sum_propagation(inputs.propagation, path)?;

    let transmit_power_dbw = db_from_ratio(tx.transmit_power_w);
    let eirp_dbw = transmit_power_dbw - tx.line_loss_db + tx_gain_dbi;
    let fspl_db = free_space_path_loss_db(path.slant_range_m, f_hz);
    let extra_db: f64 = propagation_losses.iter().map(|t| t.loss_db).sum();
    let rip_dbw = eirp_dbw - fspl_db - extra_db - link.polarization_mismatch_loss_db;
    let received_power_dbw = rip_dbw + rx_gain_dbi - rx.line_loss_db;

    let t_sys_k = system_temperature_k(link, inputs.receive_antenna.peak_gain_dbi(f_hz));
    let t_sys_db = db_from_ratio(t_sys_k);
    let n0_dbw_hz = db_from_ratio(BOLTZMANN_CONSTANT_J_K) + t_sys_db;
    let c_over_n0_dbhz = received_power_dbw - n0_dbw_hz;
    let data_rate_dbhz = db_from_ratio(link.data_rate_bps);
    let eb_n0_db = c_over_n0_dbhz - data_rate_dbhz - link.implementation_loss_db;
    let c_over_n = ratio_from_db(c_over_n0_dbhz - db_from_ratio(link.bandwidth_hz));
    let doppler_hz = doppler_shift_hz(f_hz, path.range_rate_m_s);
    Ok(LinkBudget {
        slant_range_m: path.slant_range_m,
        range_rate_m_s: path.range_rate_m_s,
        light_time_s: path.light_time_s,
        doppler_shift_hz: doppler_hz,
        received_frequency_hz: f_hz + doppler_hz,
        transmit_power_dbw,
        transmit_line_loss_db: tx.line_loss_db,
        transmit_antenna_gain_dbi: tx_gain_dbi,
        transmit_off_axis_rad: off_axis_rad(tx_dir),
        eirp_dbw,
        free_space_path_loss_db: fspl_db,
        propagation_losses,
        polarization_mismatch_loss_db: link.polarization_mismatch_loss_db,
        received_isotropic_power_dbw: rip_dbw,
        receive_antenna_gain_dbi: rx_gain_dbi,
        receive_off_axis_rad: off_axis_rad(rx_dir),
        receive_line_loss_db: rx.line_loss_db,
        received_power_dbw,
        system_noise_temperature_k: t_sys_k,
        gain_over_temperature_db_per_k: rx_gain_dbi - rx.line_loss_db - t_sys_db,
        noise_spectral_density_dbw_hz: n0_dbw_hz,
        c_over_n0_dbhz,
        data_rate_dbhz,
        implementation_loss_db: link.implementation_loss_db,
        eb_n0_db,
        required_eb_n0_db: link.required_eb_n0_db,
        margin_db: eb_n0_db - link.required_eb_n0_db,
        shannon_capacity_bps: link.bandwidth_hz * ln(1.0 + c_over_n) / core::f64::consts::LN_2,
    })
}
