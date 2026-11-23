//! What a vessel's surroundings do to it in orbit besides gravity, against the closed
//! forms: drag decay, gravity-gradient libration, and the torque on a residual dipole.

use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use sim_atmosphere::{AtmosphereError, AtmosphereField, AtmosphereSample, GasComposition};
use sim_bodies::{BodyCatalog, BodyDescriptor};
use sim_geo::{BodyEllipsoid, Geodetic, MagneticField, MagneticFieldError, UtcTime};
use sim_orbital::{
    BodyGravitySpec, BuiltinSolarSystem2026_2028, CoastControlLaw, ContentHash,
    ContinuousControlLaw, ContinuousPlantState, Dop853Tolerances, DragSurface, DynamicalMode,
    EnvironmentContext, EnvironmentLaw, EnvironmentLoadSpec, EnvironmentLoads, ForceModel,
    FrameCode, GravityConstantsCatalog, GravityGradient, MassPropertiesModel, NaifId,
    OrientationError, PropellantMode, ReactionWheelMode, ResidualDipole, SurfaceNavGrade, TdbEpoch,
    TdbSpanNs, propagate_plant_interval,
};
use sim_spacetime::{BodyOrientationCatalog, BodyRotationState};

#[cfg(test)]
const MU: f64 = 3.986_004_418e14;
#[cfg(test)]
const RADIUS_M: f64 = 6_378_137.0;

/// A globe that does not turn: its fixed axes are the inertial ones, so a closed form
/// in inertial space is the whole answer.
#[cfg(test)]
const GLOBE: BodyEllipsoid = BodyEllipsoid {
    semi_major_m: RADIUS_M,
    flattening: 0.0,
    mu_m3_s2: MU,
    omega_rad_s: 0.0,
    j2: 0.0,
};

#[cfg(test)]
fn epoch() -> TdbEpoch {
    TdbEpoch::new(845_000_000, 0)
}

#[cfg(test)]
struct Still;

#[cfg(test)]
impl BodyOrientationCatalog for Still {
    fn content_hash(&self) -> ContentHash {
        ContentHash([3; 32])
    }

    fn coverage(&self, _body: NaifId, _min_grade: SurfaceNavGrade) -> Vec<TdbSpanNs> {
        Vec::new()
    }

    fn body_rotation(
        &self,
        _body: NaifId,
        _at: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError> {
        Ok(BodyRotationState {
            r_icrf_to_fixed: DMat3::IDENTITY,
            omega_body_rad_s: DVec3::ZERO,
            alpha_body_rad_s2: DVec3::ZERO,
            frame_code: FrameCode::ITRF93,
            grade: SurfaceNavGrade::PrecisionGeodetic,
        })
    }
}

/// Density falling by `e` every `scale_height_m` from `density_kg_m3` at `height_m`.
#[derive(Debug)]
#[cfg(test)]
struct ExponentialAir {
    density_kg_m3: f64,
    height_m: f64,
    scale_height_m: f64,
}

#[cfg(test)]
impl ExponentialAir {
    fn density_at(&self, height_m: f64) -> f64 {
        self.density_kg_m3 * (-(height_m - self.height_m) / self.scale_height_m).exp()
    }
}

#[cfg(test)]
impl AtmosphereField for ExponentialAir {
    fn sample(&self, at: &Geodetic, _utc: UtcTime) -> Result<AtmosphereSample, AtmosphereError> {
        Ok(AtmosphereSample {
            temperature_k: 1_000.0,
            pressure_pa: 0.0,
            density_kg_m3: self.density_at(at.height_m),
            speed_of_sound_m_s: 0.0,
            wind_ned_m_s: DVec3::ZERO,
            composition: GasComposition::earth_dry_air(),
        })
    }
}

#[derive(Debug)]
#[cfg(test)]
struct UniformField(DVec3);

#[cfg(test)]
impl MagneticField for UniformField {
    fn field_fixed_nt(&self, _fixed: DVec3, _year: f64) -> Result<DVec3, MagneticFieldError> {
        Ok(self.0)
    }
}

/// The globe alone, described by `body`.
#[cfg(test)]
fn world(body: BodyDescriptor) -> ForceModel {
    let catalog = Arc::new(BodyCatalog::new().with(body));
    let constants = GravityConstantsCatalog::from_specs([BodyGravitySpec {
        naif_id: NaifId::EARTH,
        mu_m3_s2: MU,
        equatorial_radius_m: RADIUS_M,
        polar_radius_m: RADIUS_M,
        harmonics: None,
    }])
    .with_body_catalog(catalog);
    ForceModel::new(
        constants,
        vec![NaifId::EARTH],
        Vec::new(),
        DynamicalMode::SelfConsistentNBody,
        0,
    )
    .expect("model")
}

#[cfg(test)]
fn globe() -> BodyDescriptor {
    BodyDescriptor::new(NaifId::EARTH, "globe", GLOBE).expect("globe")
}

#[cfg(test)]
struct Flight<'a> {
    solar: &'a BuiltinSolarSystem2026_2028,
    model: ForceModel,
    mass: MassPropertiesModel,
    loads: EnvironmentLoads,
    state: ContinuousPlantState,
    at: TdbEpoch,
}

#[cfg(test)]
impl Flight<'_> {
    fn law<'a>(&'a self, inner: &'a CoastControlLaw) -> EnvironmentLaw<'a> {
        EnvironmentLaw {
            inner,
            loads: &self.loads,
            context: EnvironmentContext::of(&self.model, &self.solar.ephemeris, &Still, &self.mass),
        }
    }

    fn advance(&mut self, seconds: f64) {
        let tolerances = Dop853Tolerances {
            rtol: 1.0e-12,
            atol_att_rad: 1.0e-11,
            atol_omega_rad_s: 1.0e-12,
            ..Dop853Tolerances::default()
        };
        let until = self.at.advanced_seconds(seconds);
        let out = propagate_plant_interval(
            &self.state,
            self.at,
            until,
            None,
            &tolerances,
            &self.mass,
            PropellantMode::InfiniteConstantMass,
            ReactionWheelMode::ConserveMomentum {
                h_max_nms: 0.0,
                tau_max_nm: 0.0,
            },
            &self.model,
            &self.solar.ephemeris,
            &Still,
            &self.law(&CoastControlLaw),
        )
        .expect("propagate");
        self.state = out.final_state;
        self.at = until;
    }
}

/// A circular orbit of radius `r` in the inertial `xy` plane, starting on `+x`.
#[cfg(test)]
fn circular(r: f64, q_body_to_icrf: DQuat, omega_body_rad_s: DVec3) -> ContinuousPlantState {
    ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(r, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, (MU / r).sqrt(), 0.0),
        q_body_to_icrf,
        omega_body_rad_s,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    }
}

#[cfg(test)]
fn loads(specs: impl IntoIterator<Item = EnvironmentLoadSpec>) -> EnvironmentLoads {
    specs.into_iter().collect()
}

/// A circular orbit in an exponential atmosphere decays at `da/dt = -ρ √(μ a) / β`.
#[test]
fn drag_decays_a_circular_orbit_at_the_analytic_rate() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let air = ExponentialAir {
        density_kg_m3: 2.8e-12,
        height_m: 400_000.0,
        scale_height_m: 58_000.0,
    };
    let beta = 50.0;
    let a0 = RADIUS_M + 400_000.0;
    let rate = |a: f64| -air.density_at(a - RADIUS_M) * (MU * a).sqrt() / beta;
    let body = globe().with_atmosphere("exponential", Arc::new(ExponentialAir { ..air }));
    let mut flight = Flight {
        solar: &solar,
        model: world(body),
        mass: MassPropertiesModel::diagonal(500.0, [100.0, 100.0, 100.0], [0.0; 3]),
        loads: loads([EnvironmentLoadSpec::Drag(DragSurface {
            ballistic_coefficient_kg_m2: beta,
        })]),
        state: circular(a0, DQuat::IDENTITY, DVec3::ZERO),
        at: epoch(),
    };
    let (span_s, chunk_s) = (10.0 * 5_553.0, 60.0);
    let mut expected = a0;
    let mut flown_s = 0.0;
    while flown_s < span_s {
        flight.advance(chunk_s);
        // Classical Runge-Kutta on the decay law.
        let k1 = rate(expected);
        let k2 = rate(expected + 0.5 * chunk_s * k1);
        let k3 = rate(expected + 0.5 * chunk_s * k2);
        let k4 = rate(expected + chunk_s * k3);
        expected += chunk_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        flown_s += chunk_s;
    }
    let (r, v) = (flight.state.pos_rel_icrf_m, flight.state.vel_rel_icrf_m_s);
    let semi_major = -MU / (v.length_squared() - 2.0 * MU / r.length());
    let (decay, predicted) = (a0 - semi_major, a0 - expected);
    assert!(
        (decay / predicted - 1.0).abs() < 0.01,
        "{decay} vs {predicted}"
    );
}

/// `start` flown for `span_s` in the built-in solar system with `loads`.
#[cfg(test)]
fn fly_real_earth(
    solar: &BuiltinSolarSystem2026_2028,
    model: &ForceModel,
    start: &ContinuousPlantState,
    loads: &EnvironmentLoads,
    span_s: f64,
) -> ContinuousPlantState {
    let mass = MassPropertiesModel::diagonal(500.0, [100.0, 100.0, 100.0], [0.0; 3]);
    let law = EnvironmentLaw {
        inner: &CoastControlLaw,
        loads,
        context: EnvironmentContext::of(model, &solar.ephemeris, &solar.orientation, &mass),
    };
    propagate_plant_interval(
        start,
        epoch(),
        epoch().advanced_seconds(span_s),
        None,
        &Dop853Tolerances::default(),
        &mass,
        PropellantMode::InfiniteConstantMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        model,
        &solar.ephemeris,
        &solar.orientation,
        &law,
    )
    .expect("propagate")
    .final_state
}

/// The catalog's Earth carries a thermosphere, so a vessel at 300 km loses height in
/// orbital propagation at the rate its air there gives, without the flight tier.
#[test]
fn the_earths_thermosphere_decays_a_low_orbit() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let model = ForceModel::standard_solar_system(solar.constants.clone()).expect("model");
    let earth = model
        .body_catalog()
        .get(NaifId::EARTH)
        .expect("earth")
        .clone();
    let rotation = solar
        .orientation
        .body_rotation(NaifId::EARTH, epoch())
        .expect("rotation");
    // An equatorial orbit keeps one height above the ellipsoid, so one density.
    let r = earth.ellipsoid.semi_major_m + 300_000.0;
    let to_icrf = rotation.r_icrf_to_fixed.transpose();
    // Circular under the oblate field, which pulls harder over the equator than the
    // point mass does.
    let speed = (r * earth
        .ellipsoid
        .gravity_fixed(DVec3::new(r, 0.0, 0.0))
        .length())
    .sqrt();
    let start = ContinuousPlantState {
        pos_rel_icrf_m: to_icrf * DVec3::new(r, 0.0, 0.0),
        vel_rel_icrf_m_s: to_icrf * DVec3::new(0.0, speed, 0.0),
        ..circular(r, DQuat::IDENTITY, DVec3::ZERO)
    };
    let beta = 50.0;
    let dragged = loads([EnvironmentLoadSpec::Drag(DragSurface {
        ballistic_coefficient_kg_m2: beta,
    })]);
    let period_s = std::f64::consts::TAU * (r.powi(3) / earth.ellipsoid.mu_m3_s2).sqrt();
    let fly = |loads: &EnvironmentLoads| {
        let end = fly_real_earth(&solar, &model, &start, loads, period_s);
        let mu = earth.ellipsoid.mu_m3_s2;
        -mu / (end.vel_rel_icrf_m_s.length_squared() - 2.0 * mu / end.pos_rel_icrf_m.length())
    };
    let decay = fly(&EnvironmentLoads::default()) - fly(&dragged);
    let air = earth
        .atmosphere()
        .sample(
            &Geodetic::from_degrees(0.0, 0.0, 300_000.0),
            epoch().approximate_utc(),
        )
        .expect("air");
    // The air turns with the Earth, so the vessel meets it slower than it flies.
    let through_air = speed - earth.ellipsoid.omega_rad_s * r;
    let analytic = air.density_kg_m3 * through_air * through_air * r * r * speed
        / (beta * earth.ellipsoid.mu_m3_s2)
        * period_s;
    assert!(
        (decay / analytic - 1.0).abs() < 0.1,
        "{decay} vs {analytic}"
    );
}

/// A body whose long axis hangs along the local vertical swings about it in the orbit
/// plane with period `2π / (n √(3 (I_x − I_z) / I_y))`.
#[test]
fn a_dumbbell_librates_in_the_gravity_gradient_at_the_analytic_period() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let r = 7_000_000.0_f64;
    let n = (MU / r.powi(3)).sqrt();
    let (transverse, axial) = (1_000.0, 50.0);
    // Body x along track, y along the orbit normal, z up the local vertical, then
    // pitched about the normal.
    let hanging = DQuat::from_mat3(&DMat3::from_cols(DVec3::Y, DVec3::Z, DVec3::X));
    let pitch0 = 0.02_f64;
    let mut flight = Flight {
        solar: &solar,
        model: world(globe()),
        mass: MassPropertiesModel::diagonal(800.0, [transverse, transverse, axial], [0.0; 3]),
        loads: loads([EnvironmentLoadSpec::GravityGradient(GravityGradient)]),
        state: circular(
            r,
            DQuat::from_rotation_z(pitch0) * hanging,
            DVec3::new(0.0, n, 0.0),
        ),
        at: epoch(),
    };
    let pitch = |state: &ContinuousPlantState| {
        let long_axis = state.q_body_to_icrf * DVec3::Z;
        let up = state.pos_rel_icrf_m.normalize();
        up.cross(long_axis).z.atan2(up.dot(long_axis))
    };
    let (step_s, mut t_s, mut last) = (5.0, 0.0, pitch0);
    let mut crossings = Vec::new();
    while crossings.len() < 3 {
        flight.advance(step_s);
        t_s += step_s;
        let now = pitch(&flight.state);
        if now.signum() != last.signum() {
            crossings.push(t_s - step_s * now / (now - last));
        }
        last = now;
        assert!(t_s < 20_000.0, "no libration: pitch {now}");
    }
    let period = crossings[2] - crossings[0];
    let analytic = std::f64::consts::TAU / (n * (3.0 * (transverse - axial) / transverse).sqrt());
    assert!(
        (period / analytic - 1.0).abs() < 2.0e-3,
        "{period} vs {analytic}"
    );
}

/// The torque on a residual dipole is `m × B`, and on a spinning body it turns the
/// spin axis about the field at `m B / L`, against the field's sense.
#[test]
fn a_residual_dipole_feels_m_cross_b_and_precesses_a_spinning_body() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let field_nt = DVec3::new(0.0, 0.0, 40_000.0);
    let body = globe().with_magnetic_field("uniform", Arc::new(UniformField(field_nt)));
    let moment = DVec3::new(0.0, 0.0, 200.0);
    let (spin_rad_s, axial) = (1.0, 50.0);
    // The spin axis leans 30° from the field.
    let lean = DQuat::from_rotation_y(30.0_f64.to_radians());
    let mut flight = Flight {
        solar: &solar,
        model: world(body),
        mass: MassPropertiesModel::diagonal(300.0, [80.0, 80.0, axial], [0.0; 3]),
        loads: loads([EnvironmentLoadSpec::MagneticDipole(ResidualDipole {
            moment_body_a_m2: moment.to_array(),
        })]),
        state: circular(7_000_000.0, lean, DVec3::new(0.0, 0.0, spin_rad_s)),
        at: epoch(),
    };
    let wrench = flight
        .law(&CoastControlLaw)
        .evaluate_stage_checked(0.0, epoch(), &flight.state)
        .expect("stage");
    let field_body_t = lean.inverse() * field_nt * 1.0e-9;
    let expected = moment.cross(field_body_t);
    assert!((wrench.ext_torque_body_nm - expected).length() < 1e-15 * expected.length().max(1.0));
    assert!((expected.length() - 200.0 * 4.0e-5 * 0.5).abs() < 1e-12);
    assert_eq!(wrench.env_force_body_n, DVec3::ZERO);

    let azimuth = |state: &ContinuousPlantState| {
        let axis = state.q_body_to_icrf * DVec3::Z;
        axis.y.atan2(axis.x)
    };
    let span_s = 1_000.0;
    flight.advance(span_s);
    let turned = azimuth(&flight.state);
    let analytic = -(moment.z * field_nt.z * 1.0e-9) / (axial * spin_rad_s) * span_s;
    assert!(
        (turned / analytic - 1.0).abs() < 0.01,
        "{turned} vs {analytic}"
    );
    let tilt = (flight.state.q_body_to_icrf * DVec3::Z)
        .z
        .acos()
        .to_degrees();
    assert!((tilt - 30.0).abs() < 0.05, "{tilt}");
}

/// A load that needs a body is nothing about an origin the catalog does not describe,
/// and a vessel with no loads keeps the bare law's bits.
#[test]
fn loads_need_their_body_and_an_empty_set_changes_nothing() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let mut flight = Flight {
        solar: &solar,
        model: world(globe()),
        mass: MassPropertiesModel::diagonal(500.0, [100.0, 200.0, 300.0], [0.0; 3]),
        loads: loads([
            EnvironmentLoadSpec::Drag(DragSurface {
                ballistic_coefficient_kg_m2: 50.0,
            }),
            EnvironmentLoadSpec::GravityGradient(GravityGradient),
        ]),
        state: circular(7_000_000.0, DQuat::from_rotation_z(0.4), DVec3::ZERO),
        at: epoch(),
    };
    let stage = |flight: &Flight<'_>| {
        flight
            .law(&CoastControlLaw)
            .evaluate_stage_checked(0.0, epoch(), &flight.state)
            .expect("stage")
    };
    // No air on the bare globe, but its gravity gradient torques the lopsided body.
    let about_globe = stage(&flight);
    assert_eq!(about_globe.env_force_body_n, DVec3::ZERO);
    assert!(about_globe.ext_torque_body_nm.length() > 1e-5);
    flight.state.primary_body = NaifId::SSB;
    flight.state.pos_rel_icrf_m = DVec3::new(1.5e11, 0.0, 0.0);
    assert_eq!(stage(&flight).ext_torque_body_nm, DVec3::ZERO);
    flight.loads = EnvironmentLoads::default();
    assert_eq!(stage(&flight), Default::default());
}
