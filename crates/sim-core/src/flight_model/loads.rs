//! The loads on a vehicle at one accepted state, read out for telemetry and
//! inertial sensing: what the elements apply, summed in body axes.

use glam::{DQuat, DVec3};
use mbd_aero::AeroElement;
use mbd_core::{ComponentId, WrenchAtSite};
use mbd_multibody::{
    ContactPointState, DockingCapture, DockingState, ForceElement, Kinematics, ModelState,
    MultibodyError, PortAnchor, PortKinematics, SurfaceQuery, Tether, TetherAnchor, TetherEnds,
    WheelState, WinchTetherState, Workspace,
};

use crate::flight_model::coupled_mechanism::CoupledMechanism;
use crate::flight_model::elements::{VehicleElements, aero_lens};
use crate::flight_model::environment::FlightEnvironment;
use crate::flight_model::integration::{StepSetup, Stepper};
use crate::flight_model::planet_frame::{RootMount, fixed_to_instantaneous_inertial};

/// Aerodynamic load on a whole vehicle, in its body axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroLoad {
    /// Total force, body axes, N.
    pub force_body_n: DVec3,
    /// Total moment about the origin of the body axes, N·m.
    pub moment_origin_body_nm: DVec3,
    /// Largest stagnation-point heat flux over the vehicle's elements, W/m².
    pub stagnation_heat_flux_w_m2: f64,
}

impl AeroLoad {
    /// No load.
    pub const ZERO: Self = Self {
        force_body_n: DVec3::ZERO,
        moment_origin_body_nm: DVec3::ZERO,
        stagnation_heat_flux_w_m2: 0.0,
    };

    /// The moment about `point_body_m`, N·m.
    #[must_use]
    pub fn moment_about(&self, point_body_m: DVec3) -> DVec3 {
        self.moment_origin_body_nm - point_body_m.cross(self.force_body_n)
    }
}

/// What the elements of a vehicle apply at one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LoadReadout {
    /// Aerodynamic load.
    pub(crate) aero: AeroLoad,
    /// Every force that is not gravity and not internal to the vehicle,
    /// body axes, N.
    pub(crate) external_force_body_n: DVec3,
    /// The part of it that the ground and the water carry, body axes, N.
    pub(crate) support_force_body_n: DVec3,
    /// Whether any footpad or wheel carries load.
    pub(crate) any_contact: bool,
    /// Buoyant force on the hull, N.
    pub(crate) buoyancy_n: f64,
    /// Inertial angular acceleration of the root, body axes, rad/s².
    pub(crate) alpha_body: DVec3,
    /// Whether the solve behind these loads treated the vehicle as one
    /// rigid body.
    pub(crate) rigid_fast_path: bool,
}

/// The blocks of the flat state not yet handed to an element.
struct Blocks<'a> {
    state: &'a [f64],
    modes: &'a [f64],
}

impl<'a> Blocks<'a> {
    fn take(&mut self, states: usize, modes: usize) -> (&'a [f64], &'a [f64]) {
        let (state, rest) = self.state.split_at_checked(states).unwrap_or((&[], &[]));
        let (mode, rest_modes) = self.modes.split_at_checked(modes).unwrap_or((&[], &[]));
        self.state = rest;
        self.modes = rest_modes;
        (state, mode)
    }
}

fn lens<'e>(element: &impl ForceElement<FlightEnvironment<'e>>) -> (usize, usize) {
    (element.state_len(), element.mode_len())
}

/// Body-from-link rotation and link origin in body axes.
fn link_in_body(
    kinematics: &Kinematics<'_>,
    mount: &RootMount,
    component: ComponentId,
) -> Result<(DQuat, DVec3), MultibodyError> {
    let pose = kinematics.link_pose_in_root(component)?;
    Ok((
        mount.body_from_link * pose.rotation_root_from_link,
        mount.origin_body_m + mount.body_from_link * pose.position_root_m,
    ))
}

fn wrench_force_body(
    kinematics: &Kinematics<'_>,
    mount: &RootMount,
    wrench: &WrenchAtSite,
) -> Result<DVec3, MultibodyError> {
    if !kinematics.is_attached(wrench.site) {
        return Ok(DVec3::ZERO);
    }
    let (rotation, _) = link_in_body(kinematics, mount, wrench.site)?;
    Ok(rotation * wrench.force_local_n)
}

struct Reader<'a, 'e> {
    kinematics: &'a Kinematics<'a>,
    environment: &'a FlightEnvironment<'e>,
    mount: &'a RootMount,
    time_s: f64,
}

impl Reader<'_, '_> {
    fn aero(
        &self,
        elements: &[AeroElement],
        blocks: &mut Blocks<'_>,
    ) -> Result<AeroLoad, MultibodyError> {
        let mut total = AeroLoad::ZERO;
        for element in elements {
            let (states, modes) = aero_lens(element);
            let (state, mode) = blocks.take(states, modes);
            if !self.kinematics.is_attached(element.component) {
                continue;
            }
            let flow = element.flow(self.kinematics, self.environment, self.time_s)?;
            let wrench = element.loads(&flow, state, mode)?.wrench;
            let (rotation, origin) = link_in_body(self.kinematics, self.mount, element.component)?;
            let force = rotation * wrench.force_link_n;
            let station = origin + rotation * wrench.reference_point_link_m;
            total.force_body_n += force;
            total.moment_origin_body_nm +=
                rotation * wrench.moment_ref_link_nm + station.cross(force);
            total.stagnation_heat_flux_w_m2 = total
                .stagnation_heat_flux_w_m2
                .max(wrench.stagnation_heat_flux_w_m2);
        }
        Ok(total)
    }

    /// Force of the footpads and wheels in body axes, and whether any
    /// carries load.
    fn ground(
        &self,
        elements: &VehicleElements,
        blocks: &mut Blocks<'_>,
    ) -> Result<(DVec3, bool), MultibodyError> {
        let mut force = DVec3::ZERO;
        let mut any = false;
        for contact in &elements.contacts {
            let (state, mode) = blocks.take(3, 1);
            if !self.kinematics.is_attached(contact.component) {
                continue;
            }
            let point = self
                .kinematics
                .point(contact.component, contact.offset_local_m)?;
            let surface = self
                .environment
                .surface_at(point.position_world_m, self.time_s)?;
            let typed = ContactPointState::from_blocks(state, mode);
            let out = contact.evaluate(&typed, &point, &surface);
            any |= out.normal_force_n > 0.0;
            force += wrench_force_body(self.kinematics, self.mount, &out.wrench(contact, &point))?;
        }
        for wheel in &elements.wheels {
            let (state, _) = blocks.take(1, 0);
            let spec = &wheel.spec;
            if !self.kinematics.is_attached(spec.component) {
                continue;
            }
            let hub = self
                .kinematics
                .point(spec.component, spec.hub_offset_local_m)?;
            let surface = self
                .environment
                .surface_at(hub.position_world_m, self.time_s)?;
            let typed = WheelState {
                spin_rate_rad_s: state.first().copied().unwrap_or(0.0),
            };
            let out = spec.evaluate(
                &typed,
                &wheel.command,
                &hub,
                &surface,
                wheel.spin_inertia_kg_m2,
            )?;
            any |= out.contact_load_n > 0.0;
            force += wrench_force_body(self.kinematics, self.mount, &out.wrench(spec, &hub))?;
        }
        Ok((force, any))
    }

    /// Force of the water on the hull in body axes, and its buoyant part.
    fn water(&self, elements: &VehicleElements) -> Result<(DVec3, f64), MultibodyError> {
        let Some(hull) = &elements.hull else {
            return Ok((DVec3::ZERO, 0.0));
        };
        if !self.kinematics.is_attached(hull.component()) {
            return Ok((DVec3::ZERO, 0.0));
        }
        let totals = hull.totals(self.kinematics, self.environment, self.time_s)?;
        let (rotation, _) = link_in_body(self.kinematics, self.mount, hull.component())?;
        Ok((rotation * totals.force_local_n, totals.total_buoyancy_n))
    }
}

/// Force a tether applies to the links of this vehicle, world axes, N.
fn tether_force_world(
    kinematics: &Kinematics<'_>,
    tether: &Tether,
    state: &WinchTetherState,
) -> Result<DVec3, MultibodyError> {
    let end = |anchor: &TetherAnchor| match *anchor {
        TetherAnchor::Link {
            component,
            offset_local_m,
        } => kinematics
            .point(component, offset_local_m)
            .map(|p| (p.position_world_m, p.velocity_world_m_s, true)),
        TetherAnchor::World {
            position_world_m,
            velocity_world_m_s,
        } => Ok((position_world_m, velocity_world_m_s, false)),
    };
    let (anchor_a_world_m, vel_a_world_m_s, a_on_vehicle) = end(&tether.anchor_a)?;
    let (anchor_b_world_m, vel_b_world_m_s, b_on_vehicle) = end(&tether.anchor_b)?;
    let out = tether.spec.evaluate(
        state,
        &tether.command,
        &TetherEnds {
            anchor_a_world_m,
            vel_a_world_m_s,
            anchor_b_world_m,
            vel_b_world_m_s,
        },
    );
    let on = |applies: bool, force: DVec3| if applies { force } else { DVec3::ZERO };
    Ok(on(a_on_vehicle, out.force_on_a_world_n) + on(b_on_vehicle, out.force_on_b_world_n))
}

/// Force a docking ring applies to the links of this vehicle, world axes, N.
fn docking_force_world(
    kinematics: &Kinematics<'_>,
    ring: &DockingCapture,
    state: &DockingState,
) -> Result<DVec3, MultibodyError> {
    let port = |anchor: &PortAnchor| match *anchor {
        PortAnchor::World(port) => Ok((port, false)),
        PortAnchor::Link {
            component,
            offset_local_m,
        } => kinematics.point(component, offset_local_m).map(|p| {
            let port = PortKinematics {
                position_world_m: p.position_world_m,
                rotation_world: p.rotation_quat(),
                velocity_world_m_s: p.velocity_world_m_s,
                angular_vel_world_rad_s: p.angular_vel_world_rad_s,
            };
            (port, true)
        }),
    };
    let (a, a_on_vehicle) = port(&ring.port_a)?;
    let (b, b_on_vehicle) = port(&ring.port_b)?;
    let out = ring.spec.evaluate(state, &a, &b, ring.axis_a_local);
    let on = |applies: bool, force: DVec3| if applies { force } else { DVec3::ZERO };
    Ok(on(a_on_vehicle, out.world_wrench_on_a.force)
        + on(b_on_vehicle, out.world_wrench_on_b.force))
}

/// Force the coupled mechanisms apply to the vehicle from outside it, in
/// body axes: tethers and docking rings. Rails and hinges act between two
/// of its own links.
fn coupled_force_body(
    reader: &Reader<'_, '_>,
    mechanisms: &[CoupledMechanism],
    blocks: &mut Blocks<'_>,
) -> Result<DVec3, MultibodyError> {
    let (kinematics, mount) = (reader.kinematics, reader.mount);
    let mut world = DVec3::ZERO;
    for mechanism in mechanisms {
        let (states, modes) = lens(mechanism);
        let (state, mode) = blocks.take(states, modes);
        world += match &mechanism.carried_on(reader.environment, reader.time_s) {
            CoupledMechanism::Tether(tether, _) => tether_force_world(
                kinematics,
                tether,
                &WinchTetherState::from_blocks(state, mode),
            )?,
            CoupledMechanism::Docking(ring, _) => {
                docking_force_world(kinematics, ring, &DockingState::from_blocks(state, mode))?
            }
            CoupledMechanism::Rail(..) | CoupledMechanism::Hinge(..) => DVec3::ZERO,
        };
    }
    Ok(mount.attitude(kinematics.base()).conjugate() * world)
}

impl StepSetup<'_, '_, VehicleElements> {
    /// Reads the loads at `state`, whose base is in fixed axes, taken as
    /// the start of the environment's step. `rates` is scratch.
    pub(crate) fn read_loads(
        &self,
        mount: &RootMount,
        state: &ModelState,
        work: &mut Workspace,
        rates: &mut Vec<f64>,
    ) -> Result<LoadReadout, MultibodyError> {
        let mut in_world = state.clone();
        in_world.set_base(&fixed_to_instantaneous_inertial(
            &state.base(),
            &self.environment.body,
        ));
        self.loads_at(mount, &in_world, 0.0, work, rates)
    }

    /// Advances `state` by one period as [`StepSetup::advance`] does and
    /// reads the loads at the state it reaches, at the end of the step, so
    /// that what the far ends of its mechanisms did over the step is in
    /// them.
    pub(crate) fn advance_and_read(
        &self,
        mount: &RootMount,
        stepper: &mut Stepper,
        state: &mut ModelState,
        work: &mut Workspace,
        rates: &mut Vec<f64>,
    ) -> Result<LoadReadout, MultibodyError> {
        self.advance_then(stepper, state, work, |ended, work| {
            self.loads_at(mount, ended, self.dt_s, work, rates)
        })
    }

    pub(crate) fn loads_at(
        &self,
        mount: &RootMount,
        state: &ModelState,
        time_s: f64,
        work: &mut Workspace,
        rates: &mut Vec<f64>,
    ) -> Result<LoadReadout, MultibodyError> {
        let elements = self.elements;
        rates.clear();
        rates.resize(state.as_slice().len(), 0.0);
        self.model.derivative(
            time_s,
            state.as_slice(),
            state.modes(),
            elements,
            self.environment,
            work,
            rates,
        )?;
        let alpha_body = mount.body_from_link * work.base_angular_accel_root_rad_s2();
        let rigid_fast_path = work.used_rigid_fast_path();

        let kinematics = self.model.kinematics(state, work)?;
        let reader = Reader {
            kinematics: &kinematics,
            environment: self.environment,
            mount,
            time_s,
        };
        let mut blocks = Blocks {
            state: state.auxiliary(),
            modes: state.modes(),
        };
        let aero = reader.aero(&elements.aero, &mut blocks)?;
        let (ground, any_contact) = reader.ground(elements, &mut blocks)?;
        let (water, buoyancy_n) = reader.water(elements)?;
        let mut held = DVec3::ZERO;
        for wrench in &elements.wrenches {
            held += wrench_force_body(&kinematics, mount, wrench)?;
        }
        held += coupled_force_body(&reader, &elements.mechanisms, &mut blocks)?;
        Ok(LoadReadout {
            aero,
            external_force_body_n: aero.force_body_n + held + ground + water,
            support_force_body_n: ground + water,
            any_contact,
            buoyancy_n,
            alpha_body,
            rigid_fast_path,
        })
    }
}
