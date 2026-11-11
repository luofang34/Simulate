use aviate_core::mixer::fixed_wing::FixedWingMixer;
use aviate_hal_io::SystemCommand;
use glam::{DQuat, DVec3};
use sim_atmosphere::GeoidStandardAtmosphere;
use sim_aviate::{DirectActuatorCmd, build_direct_fixed_wing_kernel, c172_airframe};
use sim_core::{TICKS_PER_SECOND, TickSpan};
use sim_devices::SensedTruth;
use sim_geo::{Geodetic, UtcTime};

use super::*;
use crate::{C172_RATE_HZ, C172Airframe, C172Start, ChainError, SharedAir, c172_sensor_suite};

/// A C172 whose reported truth is moved and turned a little more every
/// cycle, while its devices sense the unmoved aeroplane.
struct Perturbed {
    inner: C172Airframe,
    shift: f64,
}

impl Airframe for Perturbed {
    fn period(&self) -> TickSpan {
        self.inner.period()
    }

    fn state(&self) -> VehicleState {
        let state = self.inner.state();
        VehicleState {
            position_ecef: state.position_ecef + DVec3::splat(self.shift),
            velocity_ecef: state.velocity_ecef + DVec3::X * self.shift,
            attitude: state.attitude * DQuat::from_rotation_z(self.shift * 1e-3),
            angular_rate_body: state.angular_rate_body,
        }
    }

    fn sensed(&self) -> Result<SensedTruth, ChainError> {
        self.inner.sensed()
    }

    fn fly(&mut self, command: &DirectActuatorCmd) -> Result<(), ChainError> {
        self.shift += 7.0;
        self.inner.fly(command)
    }
}

fn board<A: Airframe>(
    wrap: impl FnOnce(C172Airframe) -> A,
) -> HostedBoard<A, FixedWingController, FixedWingMixer> {
    let origin = Geodetic::from_degrees(41.70, -1.07, 2050.0);
    let start = C172Start {
        position: origin,
        heading_rad: 1.5,
        speed_m_s: 60.0,
        period: TickSpan::from_ticks(TICKS_PER_SECOND / C172_RATE_HZ),
        utc: UtcTime::from_unix_seconds(1_781_525_040.0),
    };
    let inner =
        C172Airframe::trimmed(&start, SharedAir::new(GeoidStandardAtmosphere::new())).unwrap();
    let kernel = build_direct_fixed_wing_kernel(&c172_airframe()).unwrap();
    HostedBoard::new(
        wrap(inner),
        &c172_sensor_suite(2026.5, 3),
        kernel,
        origin,
        2026.5,
    )
    .unwrap()
}

/// The avionics read the flight controller, never the airframe: a truth
/// that moves away from what the devices sense leaves every report
/// bit-identical.
#[test]
fn the_avionics_never_see_truth() {
    let mut plain = board(|inner| inner);
    let mut perturbed = board(|inner| Perturbed { inner, shift: 3.0 });
    let mut reported = 0;
    for cycle in 0..(20 * C172_RATE_HZ) {
        if cycle == 5 * C172_RATE_HZ {
            plain.send_command(SystemCommand::Arm);
            perturbed.send_command(SystemCommand::Arm);
        }
        plain.step().unwrap();
        perturbed.step().unwrap();
        assert_ne!(plain.true_state(), perturbed.true_state());
        assert_eq!(plain.avionics(), perturbed.avionics(), "cycle {cycle}");
        reported += usize::from(plain.avionics().estimate.is_some());
    }
    let last = plain.avionics();
    assert!(reported > 0 && last.air.is_some() && last.specific_force_body_m_s2.is_some());
    assert!(last.armed, "{last:?}");
}
