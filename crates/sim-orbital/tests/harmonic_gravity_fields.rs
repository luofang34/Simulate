//! The orbital tier's harmonic gravity through the shared field type: the default `J2`
//! against its closed form, a degree-36 Earth against `J2` in low orbit, and the hash
//! that tells two fields apart.

use glam::{DMat3, DQuat, DVec3};
use std::sync::Arc;

use sim_bodies::{BodyCatalog, BodyDescriptor, HarmonicGravity};
use sim_geo::{BodyEllipsoid, GravityField};
use sim_orbital::{
    BodyGravitySpec, BuiltinSolarSystem2026_2028, CoastControlLaw, ContentHash,
    ContinuousPlantState, Dop853Tolerances, DynamicalMode, ForceModel, FrameCode,
    GravityConstantsCatalog, MassPropertiesModel, NaifId, OrientationError, PropellantMode,
    ReactionWheelMode, SurfaceNavGrade, TdbEpoch, TdbSpanNs, propagate_plant_interval,
};
use sim_spacetime::{BodyOrientationCatalog, BodyRotationState};

#[cfg(test)]
const EARTH_SPIN_RAD_S: f64 = BodyEllipsoid::WGS84.omega_rad_s;

#[cfg(test)]
fn epoch() -> TdbEpoch {
    TdbEpoch::new(845_000_000, 0)
}

/// A body turning at a constant rate about a fixed pole, so the only time dependence of
/// its field in inertial space is that rotation and the Jacobi integral is exact.
#[cfg(test)]
struct UniformSpin;

#[cfg(test)]
impl BodyOrientationCatalog for UniformSpin {
    fn content_hash(&self) -> ContentHash {
        ContentHash([7; 32])
    }

    fn coverage(&self, _body: NaifId, _min_grade: SurfaceNavGrade) -> Vec<TdbSpanNs> {
        Vec::new()
    }

    fn body_rotation(
        &self,
        _body: NaifId,
        at: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError> {
        let angle = EARTH_SPIN_RAD_S * at.signed_offset_seconds_from(epoch());
        Ok(BodyRotationState {
            r_icrf_to_fixed: DMat3::from_rotation_z(-angle),
            omega_body_rad_s: DVec3::new(0.0, 0.0, EARTH_SPIN_RAD_S),
            alpha_body_rad_s2: DVec3::ZERO,
            frame_code: FrameCode::ITRF93,
            grade: SurfaceNavGrade::PrecisionGeodetic,
        })
    }
}

/// Earth alone, with `field` as its gravitation beyond the central term.
#[cfg(test)]
fn earth_only(field: HarmonicGravity, max_degree: u32) -> (ForceModel, f64) {
    let mu = BodyEllipsoid::WGS84.mu_m3_s2;
    let constants = GravityConstantsCatalog::from_specs([BodyGravitySpec {
        naif_id: NaifId::EARTH,
        mu_m3_s2: mu,
        equatorial_radius_m: BodyEllipsoid::WGS84.semi_major_m,
        polar_radius_m: BodyEllipsoid::WGS84.semi_minor_m(),
        harmonics: Some(field),
    }]);
    let model = ForceModel::new(
        constants,
        vec![NaifId::EARTH],
        vec![NaifId::EARTH],
        DynamicalMode::SelfConsistentNBody,
        max_degree,
    )
    .expect("model");
    (model, mu)
}

#[cfg(test)]
fn j2_field() -> HarmonicGravity {
    HarmonicGravity::reference(&BodyEllipsoid::WGS84).expect("J2")
}

/// The default orbital field must be the flight tier's closed-form `J2`: a vessel
/// handed between the tiers then feels one gravity.
#[test]
fn the_default_harmonic_acceleration_is_the_closed_form_j2() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let (model, mu) = earth_only(j2_field(), 8);
    let body = BodyEllipsoid::WGS84;
    let mut worst = 0.0_f64;
    for k in 0..400 {
        let t = f64::from(k);
        let dir = DVec3::new((t * 0.37).sin(), (t * 0.91).cos(), (t * 0.13).sin() * 1.3);
        let pos = dir.normalize() * body.semi_major_m * (1.02 + 0.001 * t);
        let accel = model
            .relative_gravitational_accel_icrf(
                NaifId::EARTH,
                pos,
                epoch(),
                &solar.ephemeris,
                &UniformSpin,
            )
            .expect("accel");
        // At the reference epoch the fixed and inertial axes coincide.
        let closed = body.gravity_fixed(pos);
        let central = -mu * pos / pos.length().powi(3);
        let oblateness = (closed - central).length();
        worst = worst.max((accel - closed).length() / oblateness);
    }
    assert!(worst < 1.0e-12, "{worst:e} of the J2 term");
}

#[cfg(test)]
struct Flight {
    model: ForceModel,
    mu: f64,
    field: HarmonicGravity,
    state: ContinuousPlantState,
    at: TdbEpoch,
}

#[cfg(test)]
impl Flight {
    /// A circular low orbit at 400 km, inclined 28.5°.
    fn start(field: HarmonicGravity, max_degree: u32) -> Self {
        let (model, mu) = earth_only(field.clone(), max_degree);
        let radius = BodyEllipsoid::WGS84.semi_major_m + 400_000.0;
        let speed = (mu / radius).sqrt();
        let (sin_i, cos_i) = 28.5_f64.to_radians().sin_cos();
        let state = ContinuousPlantState {
            primary_body: NaifId::EARTH,
            pos_rel_icrf_m: DVec3::new(radius, 0.0, 0.0),
            vel_rel_icrf_m_s: DVec3::new(0.0, speed * cos_i, speed * sin_i),
            q_body_to_icrf: DQuat::IDENTITY,
            omega_body_rad_s: DVec3::ZERO,
            propellant_mass_kg: 0.0,
            h_rw_body_nms: DVec3::ZERO,
        };
        let field = field
            .capped(0, max_degree as usize, max_degree as usize)
            .expect("cap");
        Self {
            model,
            mu,
            field,
            state,
            at: epoch(),
        }
    }

    fn advance(&mut self, solar: &BuiltinSolarSystem2026_2028, seconds: f64) {
        let tolerances = Dop853Tolerances {
            rtol: 1.0e-13,
            atol_pos_m: 1.0e-7,
            atol_vel_m_s: 1.0e-10,
            ..Dop853Tolerances::default()
        };
        let until = self.at.advanced_seconds(seconds);
        let out = propagate_plant_interval(
            &self.state,
            self.at,
            until,
            None,
            &tolerances,
            &MassPropertiesModel::diagonal(500.0, [100.0, 100.0, 100.0], [0.0; 3]),
            PropellantMode::InfiniteConstantMass,
            ReactionWheelMode::ConserveMomentum {
                h_max_nms: 0.0,
                tau_max_nm: 0.0,
            },
            &self.model,
            &solar.ephemeris,
            &UniformSpin,
            &CoastControlLaw,
        )
        .expect("propagate");
        self.state = out.final_state;
        self.at = until;
    }

    /// Specific orbital energy in inertial axes, and the Jacobi integral of the
    /// rotating field, J/kg.
    fn energy_and_jacobi(&self) -> (f64, f64) {
        let (r, v) = (self.state.pos_rel_icrf_m, self.state.vel_rel_icrf_m_s);
        let rotation = UniformSpin
            .body_rotation(NaifId::EARTH, self.at)
            .expect("rot");
        let fixed = rotation.r_icrf_to_fixed * r;
        // The field carries its own central term; the model flies `mu` instead.
        let own_central = self.field.gm_m3_s2() / fixed.length();
        let non_central = self.field.sample(fixed, false).expect("u").potential_m2_s2 - own_central;
        let energy = 0.5 * v.length_squared() - self.mu / r.length() - non_central;
        let spin = DVec3::new(0.0, 0.0, EARTH_SPIN_RAD_S);
        (energy, energy - spin.dot(r.cross(v)))
    }

    fn node_rad(&self) -> f64 {
        let h = self.state.pos_rel_icrf_m.cross(self.state.vel_rel_icrf_m_s);
        h.x.atan2(-h.y)
    }
}

/// Mean ascending-node longitude over one orbit of 60 s samples, with the largest
/// departures of the energy and of the Jacobi integral from `reference`.
#[cfg(test)]
fn orbit_window(
    flight: &mut Flight,
    solar: &BuiltinSolarSystem2026_2028,
    reference: (f64, f64),
    worst: &mut (f64, f64),
    path: &mut Vec<DVec3>,
) -> f64 {
    let samples = 93;
    let mut node_sum = 0.0;
    for _ in 0..samples {
        flight.advance(solar, 60.0);
        path.push(flight.state.pos_rel_icrf_m);
        node_sum += flight.node_rad();
        let (energy, jacobi) = flight.energy_and_jacobi();
        worst.0 = worst.0.max((energy - reference.0).abs());
        worst.1 = worst.1.max((jacobi - reference.1).abs());
    }
    node_sum / f64::from(samples)
}

#[cfg(test)]
struct Outcome {
    node_rate_rad_s: f64,
    energy_swing: f64,
    jacobi_swing: f64,
    reference_energy: f64,
    first_orbit: Vec<DVec3>,
}

#[cfg(test)]
fn fly_a_day(field: HarmonicGravity, max_degree: u32) -> Outcome {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let mut flight = Flight::start(field, max_degree);
    let reference = flight.energy_and_jacobi();
    let mut worst = (0.0, 0.0);
    let (mut first_orbit, mut last_orbit) = (Vec::new(), Vec::new());
    let first = orbit_window(&mut flight, &solar, reference, &mut worst, &mut first_orbit);
    let first_mid_s = flight.at.signed_offset_seconds_from(epoch()) - 46.5 * 60.0;
    for _ in 0..(23 * 60) {
        flight.advance(&solar, 60.0);
        let (energy, jacobi) = flight.energy_and_jacobi();
        worst.0 = worst.0.max((energy - reference.0).abs());
        worst.1 = worst.1.max((jacobi - reference.1).abs());
    }
    let last = orbit_window(&mut flight, &solar, reference, &mut worst, &mut last_orbit);
    let last_mid_s = flight.at.signed_offset_seconds_from(epoch()) - 46.5 * 60.0;
    Outcome {
        node_rate_rad_s: (last - first) / (last_mid_s - first_mid_s),
        energy_swing: worst.0,
        jacobi_swing: worst.1,
        reference_energy: reference.0,
        first_orbit,
    }
}

/// A day at 400 km and 28.5° with `J2` alone and with EGM96 to degree 36.
///
/// `J2` regresses the node at the first-order rate. The higher zonals change that rate
/// by the `J4` secular term; the tesserals move the orbit by hundreds of metres within
/// one revolution and exchange energy with the Earth's rotation, so inertial energy
/// swings while the Jacobi integral of the rotating field stays put.
#[test]
fn degree_36_gravity_departs_from_j2_as_theory_says_and_keeps_its_jacobi_integral() {
    let j2 = fly_a_day(j2_field(), 2);
    let detailed = fly_a_day(HarmonicGravity::egm96_degree_36().expect("EGM96"), 36);

    let body = BodyEllipsoid::WGS84;
    let a = body.semi_major_m + 400_000.0;
    let n = (body.mu_m3_s2 / a.powi(3)).sqrt();
    let (sin_i, cos_i) = 28.5_f64.to_radians().sin_cos();
    let ratio2 = (body.semi_major_m / a).powi(2);
    let j2_rate = -1.5 * body.j2 * n * ratio2 * cos_i;
    let j4 = -3.0 * detailed_c40();
    let j4_rate = (15.0 / 32.0) * j4 * n * ratio2 * ratio2 * cos_i * (8.0 - 14.0 * sin_i * sin_i);

    let separation = j2
        .first_orbit
        .iter()
        .zip(&detailed.first_orbit)
        .map(|(a, b)| (*a - *b).length())
        .fold(0.0_f64, f64::max);
    let extra_rate = detailed.node_rate_rad_s - j2.node_rate_rad_s;
    assert!((j2.node_rate_rad_s / j2_rate - 1.0).abs() < 0.01);
    assert!((extra_rate / j4_rate - 1.0).abs() < 0.3, "{extra_rate:e}");
    assert!((100.0..5_000.0).contains(&separation), "{separation}");
    let scale = detailed.reference_energy.abs();
    assert!(detailed.jacobi_swing / scale < 1.0e-10);
    assert!(j2.jacobi_swing / scale < 1.0e-10);
    assert!(j2.energy_swing / scale < 1.0e-10);
    assert!(detailed.energy_swing > 100.0 * detailed.jacobi_swing);
}

#[cfg(test)]
fn detailed_c40() -> f64 {
    HarmonicGravity::egm96_degree_36()
        .expect("EGM96")
        .coefficient(4, 0)
        .0
}

/// Two catalogs that differ in one coefficient, or only in how far a field is summed,
/// fly different trajectories and must not share a manifest hash.
#[test]
fn the_constants_hash_covers_coefficients_and_truncation() {
    let hash_of = |field: HarmonicGravity| earth_only(field, 36).0.constants().content_hash();
    let detailed = HarmonicGravity::egm96_degree_36().expect("EGM96");
    let whole = hash_of(detailed.clone());
    assert_eq!(
        whole,
        hash_of(HarmonicGravity::egm96_degree_36().expect("EGM96"))
    );
    assert_ne!(
        whole,
        hash_of(detailed.clone().capped(0, 35, 35).expect("cap"))
    );
    assert_ne!(whole, hash_of(detailed.capped(0, 36, 35).expect("cap")));
    assert_ne!(whole, hash_of(j2_field()));
    let (eight, _) = earth_only(HarmonicGravity::egm96_degree_36().expect("EGM96"), 8);
    let (twelve, _) = earth_only(HarmonicGravity::egm96_degree_36().expect("EGM96"), 12);
    assert_ne!(eight.content_hash(), twelve.content_hash());
}

/// A body given a detailed field in the catalog is flown with all of it in orbit: the
/// degree its descriptor states is the degree both tiers read.
#[test]
fn an_opted_in_field_is_flown_whole_by_the_orbital_tier() {
    let detailed = HarmonicGravity::egm96_degree_36().expect("EGM96");
    let earth = BodyDescriptor::earth()
        .expect("earth")
        .with_gravity(detailed.clone());
    let bodies = BodyCatalog::standard_solar_system()
        .expect("bodies")
        .with(earth);
    let solar =
        BuiltinSolarSystem2026_2028::load_verified_with_bodies(Arc::new(bodies)).expect("solar");
    let model = ForceModel::standard_solar_system(solar.constants.clone()).expect("model");
    assert_eq!(model.max_harmonic_degree(), 36);
    let without = ForceModel::new(
        solar.constants.clone(),
        model.attracting_bodies().to_vec(),
        vec![NaifId::MOON, NaifId::MARS],
        DynamicalMode::PrescribedEphemeris,
        36,
    )
    .expect("model");
    let accel = |model: &ForceModel, pos: DVec3| {
        model
            .relative_gravitational_accel_icrf(
                NaifId::EARTH,
                pos,
                epoch(),
                &solar.ephemeris,
                &solar.orientation,
            )
            .expect("accel")
    };
    let rotation = solar
        .orientation
        .body_rotation(NaifId::EARTH, epoch())
        .expect("rotation");
    let central =
        HarmonicGravity::point_mass(detailed.gm_m3_s2(), detailed.radius_m()).expect("central");
    let mut worst = 0.0_f64;
    for k in 0..60 {
        let t = f64::from(k);
        let dir = DVec3::new((t * 0.37).sin(), (t * 0.91).cos(), (t * 0.13).sin() * 1.3);
        let pos = dir.normalize() * (6_778_000.0 + 20_000.0 * t);
        let fixed = rotation.r_icrf_to_fixed * pos;
        let beyond_central = detailed.attraction_fixed(fixed) - central.attraction_fixed(fixed);
        let flown = accel(&model, pos) - accel(&without, pos);
        let expected = rotation.r_icrf_to_fixed.transpose() * beyond_central;
        worst = worst.max((flown - expected).length() / expected.length());
    }
    assert!(worst < 1.0e-9, "{worst:e} of the non-central field");
}
