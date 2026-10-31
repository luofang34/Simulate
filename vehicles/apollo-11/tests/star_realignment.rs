//! A coding agent sees only what the crew port shows, finds two stars through
//! the Alignment Optical Telescope, and restores a tumbled platform.

use glam::{DQuat, DVec3};
use mbd_core::VacuumAero;
use sim_apollo::{
    AOT_DETENT_PANEL, AOT_DETENTS, InertialSiteSample, LM_IMU, OpticalSightBlock,
    OpticalSightConfig, OpticalSightInput, SightMark, StarSighting, ThreeGimbalImuBlock,
    ThreeGimbalImuConfig, aot_mount, build_apollo_lm_tree, realign_imu_from_two_star_marks,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    Block, CrewAction, CrewObservation, CrewStationPort, RigidBody6DofBlock, RigidBodyConfig,
    RigidBodyInput, SimInstant, ThrusterWrench, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, NavStar, UtcTime, nav_star_by_octal_id};

const EPOCH_UNIX_S: f64 = -14_182_940.0;
const HZ: u64 = 50;
const SIGHT: u8 = 1;
const SLEW_ATTEMPTS: u32 = 5;

/// What the crew port shows of the sight.
#[cfg(test)]
struct View {
    octals: [u8; 8],
    offsets: [[f64; 2]; 8],
    count: usize,
    centered: bool,
}

#[cfg(test)]
struct Lander {
    vehicle: RigidBody6DofBlock,
    imu: ThreeGimbalImuBlock,
    aot: OpticalSightBlock,
    port: CrewStationPort,
    truth: DQuat,
}

#[cfg(test)]
impl Lander {
    fn new() -> Self {
        let body = BodyEllipsoid::MOON_IAU;
        let period = TickSpan::period_of_hz(HZ).unwrap();
        let position = body.geodetic_to_fixed(&Geodetic::from_degrees(0.0, 23.0, 110_000.0));
        let initial = VehicleState {
            position_ecef: position,
            velocity_ecef: DVec3::new(0.0, 0.0, 1_600.0),
            attitude: DQuat::IDENTITY,
            angular_rate_body: DVec3::new(0.0, 0.0, 8f64.to_radians()),
        };
        let vehicle = RigidBody6DofBlock::new(
            RigidBodyConfig {
                body,
                period,
                epoch: UtcTime::from_unix_seconds(EPOCH_UNIX_S),
                initial,
            },
            build_apollo_lm_tree().unwrap(),
            Box::new(VacuumAero),
            Box::new(VacuumAtmosphere),
        )
        .unwrap();
        let start = sample(&vehicle).site_to_inertial;
        let imu = ThreeGimbalImuBlock::new(ThreeGimbalImuConfig::nominal(period, start)).unwrap();
        let aot = OpticalSightBlock::new(OpticalSightConfig::alignment_optical_telescope(
            SIGHT,
            period,
            aot_mount(),
        ))
        .unwrap();
        Self {
            vehicle,
            imu,
            aot,
            port: CrewStationPort::new(),
            truth: start,
        }
    }

    fn tick(&mut self, input: &RigidBodyInput) {
        self.vehicle.step(input).unwrap();
        let now = self.vehicle.now();
        let at_imu = sample(&self.vehicle);
        self.truth = at_imu.site_to_inertial;
        self.imu.step(&at_imu).unwrap();
        let actions = self.port.take_due(now);
        self.aot
            .step(&OpticalSightInput {
                now,
                actions,
                navbase_to_inertial: self.truth,
                navbase_to_platform: self.imu.output().navbase_to_platform,
            })
            .unwrap();
        self.aot.publish_view(now, &mut self.port).unwrap();
    }

    fn idle(&mut self) {
        self.tick(&RigidBodyInput::default());
    }

    /// Fires the RCS to null the body rate.
    fn null_rates(&mut self) {
        let inertia = self.vehicle.mass_properties().inertia_cg_body;
        let moment = inertia * (self.vehicle.output().angular_rate_body * -2.0);
        let rcs = ThrusterWrench {
            application_point_body_m: self.vehicle.mass_properties().cg_body_m,
            force_body_n: DVec3::ZERO,
            moment_body_nm: moment,
        };
        self.tick(&RigidBodyInput {
            thrusters: vec![rcs],
            ..Default::default()
        });
    }

    fn next_instant(&self) -> SimInstant {
        self.vehicle.now().advanced(self.vehicle.period())
    }

    fn crew(&mut self, action: CrewAction) {
        self.port.submit(self.next_instant(), action).unwrap();
    }

    fn view(&self) -> View {
        self.port
            .latest_observations()
            .into_iter()
            .find_map(|seen| match seen.observation {
                CrewObservation::OpticalSightView {
                    visible_star_octals,
                    visible_star_offsets_rad,
                    visible_count,
                    target_centered,
                    ..
                } => Some(View {
                    octals: visible_star_octals,
                    offsets: visible_star_offsets_rad,
                    count: usize::from(visible_count),
                    centered: target_centered,
                }),
                _ => None,
            })
            .expect("the sight publishes every step")
    }

    fn select_detent(&mut self, detent: i8) {
        self.crew(CrewAction::SetSwitch {
            panel_id: AOT_DETENT_PANEL,
            switch_id: u16::from(SIGHT),
            state: detent,
        });
        self.idle();
    }

    fn slew(&mut self, shaft_rate_rad_s: f64, trunnion_rate_rad_s: f64) {
        self.crew(CrewAction::OpticalSlew {
            sight_id: SIGHT,
            shaft_rate_rad_s,
            trunnion_rate_rad_s,
        });
        self.idle();
    }

    /// Reads the star's offset and slews by it, one period at the rate that
    /// covers it, then stops to look again. `false` if the reticle cannot
    /// reach the star or it leaves the field.
    fn center_on(&mut self, octal: u8) -> bool {
        for _ in 0..SLEW_ATTEMPTS {
            let view = self.view();
            let Some(index) = view.octals[..view.count]
                .iter()
                .position(|seen| *seen == octal)
            else {
                return false;
            };
            let [shaft, trunnion] = view.offsets[index];
            if view.centered && shaft.hypot(trunnion) < 2e-5 {
                return true;
            }
            self.slew(shaft * HZ as f64, trunnion * HZ as f64);
            self.slew(0.0, 0.0);
        }
        false
    }

    fn mark(&mut self) -> SightMark {
        self.crew(CrewAction::OpticalMark { sight_id: SIGHT });
        self.idle();
        *self.aot.marks().last().expect("a mark was just taken")
    }

    /// Walks the detents marking stars until two lie far enough apart.
    fn mark_two_stars(&mut self) -> [(SightMark, &'static NavStar); 2] {
        let mut marked: Vec<(SightMark, &'static NavStar)> = Vec::new();
        for detent in 0..AOT_DETENTS {
            self.select_detent(detent);
            let view = self.view();
            for octal in &view.octals[..view.count] {
                let star = nav_star_by_octal_id(*octal).expect("the view shows catalog stars");
                let far_enough = marked.iter().all(|(_, first)| {
                    first.direction_j2000.angle_between(star.direction_j2000) > 0.5
                });
                if far_enough && self.center_on(*octal) {
                    marked.push((self.mark(), star));
                }
                if let [first, second] = marked[..] {
                    return [first, second];
                }
            }
        }
        panic!("only {} stars found in six detents", marked.len());
    }
}

#[cfg(test)]
fn sample(vehicle: &RigidBody6DofBlock) -> InertialSiteSample {
    let elapsed = vehicle.now().duration_since(SimInstant::START).as_seconds();
    let utc = UtcTime::from_unix_seconds(EPOCH_UNIX_S).advanced_by(elapsed);
    InertialSiteSample::of(vehicle, LM_IMU, &InertialTransform::moon_at(utc)).unwrap()
}

#[test]
fn an_agent_on_the_crew_port_realigns_a_tumbled_imu_from_two_stars() {
    let mut lander = Lander::new();
    while !lander.imu.is_gimbal_locked() {
        lander.idle();
    }
    for _ in 0..3 * HZ {
        lander.idle();
    }
    for _ in 0..8 * HZ {
        lander.null_rates();
    }
    assert!(lander.vehicle.output().angular_rate_body.length() < 1e-5);

    lander.imu.cage_to_zero();
    let caged = lander.imu.output();
    assert!(caged.platform_valid && !caged.gimbal_locked);
    let misaligned = caged
        .indicated_navbase_to_inertial
        .angle_between(lander.truth);
    assert!(misaligned > 0.1, "{misaligned}");

    let [(first_mark, first_star), (second_mark, second_star)] = lander.mark_two_stars();
    assert!(second_mark.at > first_mark.at);
    assert_ne!(first_star.apollo_octal_id, second_star.apollo_octal_id);

    let realignment = realign_imu_from_two_star_marks(
        &StarSighting::from_mark(&first_mark, first_star),
        &StarSighting::from_mark(&second_mark, second_star),
    )
    .unwrap();
    assert!(realignment.star_angle_difference_rad.abs() < 1e-5);
    lander.imu.fine_align(realignment.platform_to_inertial);
    lander.idle();

    let imu = lander.imu.output();
    let error = imu
        .indicated_navbase_to_inertial
        .angle_between(lander.truth);
    assert!(error < 1e-4, "{error}");
    let physical = imu.platform_to_inertial;
    assert!(physical.angle_between(realignment.platform_to_inertial) < 1e-4);
}
