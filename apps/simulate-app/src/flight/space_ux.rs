//! Spaceflight navigation UX (`SpaceReferenceFrame`, gimbal-free `FDAI / Navball`,
//! 3D `Orrery` projection, `AerospikePlumeVisual`), `WarpGovernor` bottleneck reporting,
//! and live light-time `DSN` line-of-sight status.

use glam::{DQuat, DVec3};
use sim_analysis::{
    OccultationEnvironmentModel, OccultingBodySpec, OccultingShape, check_signal_path_occultation,
};
use sim_core::ControlInputs;
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_orbital::{
    DerivativeOrder, NaifId, SynodicFrame, VesselWarpConstraintInput, WarpStatus,
    conservative_hazard_time_to_go_s, evaluate_global_warp_status,
};
use sim_render::{
    AerospikePlumeVisual, FdaiNavballState, OrreryBodyNode, ProjectedOrreryNode,
    evaluate_aerospike_plume_visual, evaluate_fdai_navball, project_orrery_scene,
};
use sim_spacetime::{SPEED_OF_LIGHT_M_S, surface_station_state_icrf};

use super::Flight;
use super::mode::Mode;
use super::space_world::SpaceWorld;

/// Reference frame in which spaceflight state and trajectories are reported.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SpaceReferenceFrame {
    /// Non-rotating `ICRF / J2000` axes centred on the active central body.
    BodyInertial,
    /// Rotating body-fixed axes (`ITRF93`, `MOON_ME`, `IAU_MARS`) of the central body.
    BodyFixed,
    /// Rotating Sun–Earth synodic frame (`+X` Sun to Earth, `+Z` orbital angular momentum).
    SunEarthSynodic,
}

impl SpaceReferenceFrame {
    /// Short HUD/title label for this reference frame.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::BodyInertial => "ICRF",
            Self::BodyFixed => "FIXED",
            Self::SunEarthSynodic => "SYNODIC",
        }
    }

    /// Next reference frame in the operator cycle (`ICRF -> FIXED -> SYNODIC -> ICRF`).
    #[must_use]
    pub const fn next(self) -> Self {
        match self {
            Self::BodyInertial => Self::BodyFixed,
            Self::BodyFixed => Self::SunEarthSynodic,
            Self::SunEarthSynodic => Self::BodyInertial,
        }
    }
}

/// Live one-way light-time and line-of-sight status from the spaceflight ownship to
/// Earth's `DSS-14 Goldstone` deep-space station (`35.426° N, -116.89° E`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpaceLinkStatus {
    /// Whether the signal path is clear of planetary occultation and above Goldstone's `10°` mask.
    pub accessible: bool,
    /// One-way light travel time $\tau = \rho / c$ (`s`).
    pub light_time_s: f64,
    /// Light-time-retarded slant range $\rho$ (`m`).
    pub slant_range_m: f64,
    /// Elevation angle of the arriving signal at DSS-14 Goldstone (`rad`).
    pub elevation_at_station_rad: f64,
    /// Blocking body along the retarded signal path (`None` when unoccluded).
    pub occulted_by: Option<NaifId>,
}

impl Flight {
    /// Active [`SpaceReferenceFrame`] when a spaceflight is flying.
    #[must_use]
    pub fn space_reference_frame(&self) -> Option<SpaceReferenceFrame> {
        match &self.mode {
            Mode::Space(world) => Some(world.reference_frame()),
            Mode::Free { .. } | Mode::Scenario(_) => None,
        }
    }

    /// Selects the [`SpaceReferenceFrame`] for a spaceflight.
    pub fn set_space_reference_frame(&mut self, frame: SpaceReferenceFrame) {
        if let Mode::Space(world) = &mut self.mode {
            world.set_reference_frame(frame);
        }
    }

    /// Cycles the active [`SpaceReferenceFrame`] when a spaceflight is flying.
    pub fn cycle_space_reference_frame(&mut self) -> Option<SpaceReferenceFrame> {
        let next = self.space_reference_frame()?.next();
        self.set_space_reference_frame(next);
        Some(next)
    }

    /// Ownship position (`m`) and velocity (`m/s`) expressed in the active [`SpaceReferenceFrame`].
    #[must_use]
    pub fn space_state_in_reference_frame(&self) -> Option<(DVec3, DVec3)> {
        let Mode::Space(world) = &self.mode else {
            return None;
        };
        let state = world.vehicle();
        match world.reference_frame() {
            SpaceReferenceFrame::BodyFixed => Some((state.position_ecef, state.velocity_ecef)),
            SpaceReferenceFrame::BodyInertial => self.ownship_rel_central_icrf(world),
            SpaceReferenceFrame::SunEarthSynodic => {
                let (pos_rel, vel_rel) = self.ownship_rel_central_icrf(world)?;
                let at = self.epoch.tdb_at(self.now());
                let eph = self.solar.ephemeris();
                let central_rel_earth = if world.central_body() == NaifId::EARTH {
                    (DVec3::ZERO, DVec3::ZERO)
                } else {
                    let s = eph
                        .geometric_state_icrf(
                            world.central_body(),
                            NaifId::EARTH,
                            at,
                            DerivativeOrder::Velocity,
                        )
                        .ok()?;
                    (s.pos_m, s.vel_m_s)
                };
                let synodic =
                    SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, at, eph).ok()?;
                Some(synodic.icrf_rel_secondary_to_synodic(
                    central_rel_earth.0 + pos_rel,
                    central_rel_earth.1 + vel_rel,
                ))
            }
        }
    }

    /// Gimbal-free quaternion `FDAI / Navball` state (`Prograde`, `Retrograde`, `Radial`,
    /// `Normal`, `Target`, `ManeuverBurn`) for the spaceflight ownship.
    #[must_use]
    pub fn space_fdai_navball(&self) -> Option<FdaiNavballState> {
        let Mode::Space(world) = &self.mode else {
            return None;
        };
        let (pos_icrf, vel_icrf) = self.ownship_rel_central_icrf(world)?;
        let at = self.epoch.tdb_at(self.now());
        let rot = self
            .solar
            .orientation()
            .body_rotation(world.central_body(), at)
            .ok()?;
        let q_f2i = DQuat::from_mat3(&rot.r_icrf_to_fixed.transpose());
        let q_body_to_icrf = (q_f2i * world.vehicle().attitude).normalize();
        let burn_dir = world.nose_icrf().unwrap_or(vel_icrf);
        Some(evaluate_fdai_navball(
            q_body_to_icrf,
            pos_icrf,
            vel_icrf,
            vel_icrf,
            burn_dir,
        ))
    }

    /// Projects the Sun, Earth, Moon, Mars, and the spaceflight ownship into the 3D `Orrery` view
    /// relative to the active central body.
    #[must_use]
    pub fn space_orrery_projection(
        &self,
        camera_pos_rel_focus_m: DVec3,
        q_cam_to_icrf: DQuat,
        fov_y_rad: f64,
        aspect: f64,
    ) -> Option<Vec<ProjectedOrreryNode>> {
        let Mode::Space(world) = &self.mode else {
            return None;
        };
        let focus = world.central_body();
        let at = self.epoch.tdb_at(self.now());
        let eph = self.solar.ephemeris();
        let (own_pos, _) = self.ownship_rel_central_icrf(world)?;
        let mut nodes = Vec::with_capacity(5);
        for (body, soi_m) in [
            (NaifId::SUN, 0.0),
            (NaifId::EARTH, 924_000_000.0),
            (NaifId::MOON, 66_100_000.0),
            (NaifId::MARS, 577_000_000.0),
        ] {
            let pos = if body == focus {
                DVec3::ZERO
            } else {
                eph.geometric_state_icrf(body, focus, at, DerivativeOrder::Position)
                    .ok()?
                    .pos_m
            };
            nodes.push(OrreryBodyNode {
                id: i64::from(body.0),
                pos_rel_focus_icrf_m: pos,
                soi_radius_m: soi_m,
            });
        }
        nodes.push(OrreryBodyNode {
            id: i64::try_from(SpaceWorld::ownship_id().0).unwrap_or(0),
            pos_rel_focus_icrf_m: own_pos,
            soi_radius_m: 0.0,
        });
        if let Some(jwst) = world.jwst_l2_state() {
            let primary_rel_focus = if jwst.primary_body == focus {
                DVec3::ZERO
            } else {
                eph.geometric_state_icrf(jwst.primary_body, focus, at, DerivativeOrder::Position)
                    .ok()?
                    .pos_m
            };
            nodes.push(OrreryBodyNode {
                id: i64::try_from(SpaceWorld::jwst_l2_id().0).unwrap_or(0),
                pos_rel_focus_icrf_m: primary_rel_focus + jwst.pos_rel_icrf_m,
                soi_radius_m: 0.0,
            });
        }
        Some(project_orrery_scene(
            &nodes,
            camera_pos_rel_focus_m,
            q_cam_to_icrf,
            fov_y_rad,
            aspect,
        ))
    }

    /// Mach- and ambient-pressure-adaptive aerospike plume state for `Raketa`.
    #[must_use]
    pub fn space_aerospike_plume(&self) -> Option<AerospikePlumeVisual> {
        let Mode::Space(world) = &self.mode else {
            return None;
        };
        let ell = world.world().body();
        let alt_m = ell
            .fixed_to_geodetic(world.vehicle().position_ecef)
            .height_m;
        let pressure_pa = match world.central_body() {
            NaifId::EARTH if alt_m < 86_000.0 => earth_isa_pressure_density(alt_m).0,
            NaifId::MARS if alt_m < 80_000.0 => 610.0 * (-alt_m.max(0.0) / 11_100.0).exp(),
            _ => 0.0,
        };
        Some(evaluate_aerospike_plume_visual(
            world.throttle_fraction(),
            pressure_pa,
        ))
    }

    /// Global [`WarpStatus`] and bottleneck reason for the spaceflight ownship.
    #[must_use]
    pub fn space_warp_status(
        &self,
        requested_warp: f64,
        controls: ControlInputs,
    ) -> Option<WarpStatus> {
        let Mode::Space(world) = &self.mode else {
            return None;
        };
        let state = world.vehicle();
        let ell = world.world().body();
        let geo = ell.fixed_to_geodetic(state.position_ecef);
        let alt_m = geo.height_m.max(0.0);
        let q_inf_pa = if world.central_body() == NaifId::EARTH && alt_m < 100_000.0 {
            let rho = earth_isa_pressure_density(alt_m).1;
            0.5 * rho * state.velocity_ecef.length_squared()
        } else {
            0.0
        };
        let coasting = world.coasting(controls);
        let tgo_s = if coasting {
            f64::INFINITY
        } else {
            conservative_hazard_time_to_go_s(alt_m.max(10_000.0), 0.0, 39.2, None)
        };
        let input = VesselWarpConstraintInput {
            vessel_id: SpaceWorld::ownship_id(),
            is_pinned_surface_rest: false,
            is_active_ground_contact: false,
            q_inf_pa,
            in_proximity_group: false,
            hazard_tgo_s: tgo_s,
            on_reduced_manifold: coasting,
        };
        Some(evaluate_global_warp_status(requested_warp, 0.25, &[input]))
    }

    /// Evaluates the live light-time line-of-sight link from the spaceflight ownship to
    /// Earth's `DSS-14 Goldstone` deep-space station.
    #[must_use]
    pub fn space_dsn_link_status(&self) -> Option<SpaceLinkStatus> {
        let Mode::Space(world) = &self.mode else {
            return None;
        };
        let state = world.vehicle();
        self.space_dsn_link_from_body_fixed(
            world.central_body(),
            state.position_ecef,
            state.velocity_ecef,
        )
    }

    /// Evaluates the live light-time line-of-sight link from a body-fixed state on or
    /// around `body` to Earth's `DSS-14 Goldstone` deep-space station.
    #[must_use]
    pub fn space_dsn_link_from_body_fixed(
        &self,
        body: NaifId,
        pos_fixed_m: DVec3,
        vel_fixed_m_s: DVec3,
    ) -> Option<SpaceLinkStatus> {
        let receive = self.epoch.tdb_at(self.now());
        let eph = self.solar.ephemeris();
        let ori = self.solar.orientation();
        let goldstone = Geodetic::from_degrees(35.426, -116.89, 0.0);
        let earth_ssb = eph
            .geometric_state_icrf(
                NaifId::EARTH,
                NaifId::SSB,
                receive,
                DerivativeOrder::Position,
            )
            .ok()?
            .pos_m;
        let (dsn_rel_earth, _, dsn_enu) = surface_station_state_icrf(
            NaifId::EARTH,
            BodyEllipsoid::WGS84,
            &goldstone,
            receive,
            ori,
        )
        .ok()?;
        let dst_ssb = earth_ssb + dsn_rel_earth;
        let rot_earth = ori.body_rotation(NaifId::EARTH, receive).ok()?;
        let r_enu_to_icrf = rot_earth.r_icrf_to_fixed.transpose() * dsn_enu.enu_to_ecef();

        let rot_body = ori.body_rotation(body, receive).ok()?;
        let (own_rel_central, own_vel_central) =
            rot_body.fixed_to_icrf_pv(pos_fixed_m, vel_fixed_m_s);
        let central_ssb = eph
            .geometric_state_icrf(body, NaifId::SSB, receive, DerivativeOrder::Velocity)
            .ok()?;
        let own_ssb_now = central_ssb.pos_m + own_rel_central;
        let own_vel_ssb = central_ssb.vel_m_s + own_vel_central;
        let mut light_time_s = (dst_ssb - own_ssb_now).length() / SPEED_OF_LIGHT_M_S;
        let mut src_ssb = own_ssb_now;
        for _ in 0..4 {
            src_ssb = own_ssb_now - own_vel_ssb * light_time_s;
            light_time_s = (dst_ssb - src_ssb).length() / SPEED_OF_LIGHT_M_S;
        }
        let emit = receive.advanced_seconds(-light_time_s);
        let rho = src_ssb - dst_ssb;
        let slant_range_m = rho.length();
        let rho_enu = r_enu_to_icrf.transpose() * rho;
        let el_rad = (rho_enu.z / slant_range_m.max(1.0)).clamp(-1.0, 1.0).asin();
        let env = solar_system_occultation_env();
        let occ = check_signal_path_occultation(
            emit,
            receive,
            None,
            Some(NaifId::EARTH),
            src_ssb,
            dst_ssb,
            &env,
            eph,
            ori,
        )
        .ok()?
        .map(|(b, _)| b);
        let min_el_rad = 10.0_f64.to_radians();
        Some(SpaceLinkStatus {
            accessible: occ.is_none() && el_rad >= min_el_rad,
            light_time_s,
            slant_range_m,
            elevation_at_station_rad: el_rad,
            occulted_by: occ,
        })
    }

    fn ownship_rel_central_icrf(&self, world: &SpaceWorld) -> Option<(DVec3, DVec3)> {
        let at = self.epoch.tdb_at(self.now());
        let rot = self
            .solar
            .orientation()
            .body_rotation(world.central_body(), at)
            .ok()?;
        let state = world.vehicle();
        Some(rot.fixed_to_icrf_pv(state.position_ecef, state.velocity_ecef))
    }
}

fn solar_system_occultation_env() -> OccultationEnvironmentModel {
    OccultationEnvironmentModel {
        bodies: vec![
            OccultingBodySpec {
                body: NaifId::EARTH,
                shape: OccultingShape::Ellipsoid(BodyEllipsoid::WGS84),
            },
            OccultingBodySpec {
                body: NaifId::MOON,
                shape: OccultingShape::Sphere {
                    radius_m: 1_737_400.0,
                },
            },
            OccultingBodySpec {
                body: NaifId::MARS,
                shape: OccultingShape::Ellipsoid(BodyEllipsoid::MARS_IAU),
            },
            OccultingBodySpec {
                body: NaifId::SUN,
                shape: OccultingShape::Sphere {
                    radius_m: 695_700_000.0,
                },
            },
        ],
    }
}

fn earth_isa_pressure_density(alt_m: f64) -> (f64, f64) {
    let clamped_m = alt_m.max(0.0);
    let h_gp = sim_atmosphere::geometric_to_geopotential_m(clamped_m.min(32_000.0));
    if let Ok(isa) = sim_atmosphere::isa_at_geopotential(h_gp) {
        if clamped_m <= 32_000.0 {
            return (isa.pressure_pa, isa.density_kg_m3);
        }
        let decay = (-(clamped_m - 32_000.0) / 6_700.0).exp();
        return (isa.pressure_pa * decay, isa.density_kg_m3 * decay);
    }
    (0.0, 0.0)
}
