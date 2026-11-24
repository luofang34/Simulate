//! Unit tests of the signal layer, grouped by physical domain.

mod antenna;
mod budget;
mod path;

use sim_analysis_contract::{
    AntennaPointing, AntennaSpec, ParabolicDishSpec, PropagationModelSpec, ReceiverNoise,
    RfLinkSpec, RfReceiverSpec, RfTransmitterSpec,
};
use sim_spacetime::TdbEpoch;

use super::PathGeometry;

/// A 10 W, 2.2 GHz link between isotropic antennas with a 290 K receiver.
fn isotropic_link() -> RfLinkSpec {
    RfLinkSpec {
        carrier_frequency_hz: 2.2e9,
        bandwidth_hz: 1.0e6,
        data_rate_bps: 1.0e6,
        required_eb_n0_db: 10.0,
        polarization_mismatch_loss_db: 0.0,
        implementation_loss_db: 0.0,
        transmitter: RfTransmitterSpec {
            transmit_power_w: 10.0,
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::FixedInMount,
            line_loss_db: 0.0,
        },
        receiver: RfReceiverSpec {
            antenna: AntennaSpec::Isotropic,
            pointing: AntennaPointing::FixedInMount,
            line_loss_db: 0.0,
            noise: ReceiverNoise::SystemTemperature {
                system_noise_temperature_k: 290.0,
            },
        },
        propagation: Vec::<PropagationModelSpec>::new(),
    }
}

fn dish(diameter_m: f64, efficiency: f64) -> ParabolicDishSpec {
    ParabolicDishSpec {
        diameter_m,
        aperture_efficiency: efficiency,
        sidelobe_floor_below_peak_db: 30.0,
    }
}

fn path(range_m: f64, frequency_hz: f64) -> PathGeometry {
    PathGeometry::from_path(
        TdbEpoch::J2000,
        frequency_hz,
        glam::DVec3::ZERO,
        glam::DVec3::new(range_m, 0.0, 0.0),
        range_m,
        0.0,
    )
}
