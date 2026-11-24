use std::sync::Arc;

use glam::DVec3;
use navigate_contract::{SourceEpoch, SourceId, WrappingSequence};
use sim_atmosphere::{geometric_to_geopotential_m, isa_at_geopotential};
use sim_bodies::{
    AnomalyOverlay, GridGeometry, HarmonicMagnetism, MagnetismWithAnomaly, OutsideCoverage,
};
use sim_devices::{GaussMarkov, ImuGrade, MagnetometerSpec, PitotStatic, PressureErrorSpec};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, MagneticField, UtcTime};
use sim_navigate::{
    MagneticAnomalyConfig, MagneticAnomalyMap, MasterClockBridge, SimulatedImu,
    SimulatedMagnetometer,
};

pub const DECIMAL_YEAR: f64 = 2026.5;
pub const EPOCH_UNIX_S: f64 = 1_773_972_000.0;
pub const MAG_SOURCE: SourceId = SourceId::new(74);

#[derive(Debug, Clone, Copy)]
pub enum MagneticAidingMode {
    None,
    MainFieldOnly,
    CrustalOverlay { bias_offset_t: f64 },
}

pub struct Sensors {
    pub imu: SimulatedImu,
    pub static_port: PitotStatic,
    pub pitot: PitotStatic,
    pub mag_sensor: SimulatedMagnetometer,
    pub mag_map: MagneticAnomalyMap,
    pub baro_variance_m2: f64,
    pub baro_seq: WrappingSequence,
    pub utc: UtcTime,
}

pub fn corridor_crustal_overlay(outside: OutsideCoverage) -> AnomalyOverlay {
    let geometry = GridGeometry {
        south_deg: 38.90,
        west_deg: -75.72,
        latitude_step_deg: 0.002,
        longitude_step_deg: 0.002,
        rows: 141,
        columns: 161,
    };
    let origin = Geodetic::from_degrees(39.00, -75.60, 0.0);
    let origin_ecef = BodyEllipsoid::WGS84.geodetic_to_fixed(&origin);
    let frame = LocalFrame::at(&origin);
    // Roughly aligned with Earth's main field over Delaware so total-field
    // scalar anomaly maps cleanly onto both North and East gradients.
    let unit = DVec3::new(0.40, -0.08, 0.91).normalize();
    let mut values = Vec::with_capacity(geometry.rows * geometry.columns * 3);
    for row in 0..geometry.rows {
        let lat_deg = geometry.south_deg + row as f64 * geometry.latitude_step_deg;
        for col in 0..geometry.columns {
            let lon_deg = geometry.west_deg + col as f64 * geometry.longitude_step_deg;
            let p = BodyEllipsoid::WGS84
                .geodetic_to_fixed(&Geodetic::from_degrees(lat_deg, lon_deg, 0.0));
            let ned = frame.ecef_to_ned_vector(p - origin_ecef);
            let scalar_nt = 900.0 * ((ned.x + 0.6 * ned.y) / 2_000.0).sin()
                + 750.0 * ((0.7 * ned.x - ned.y) / 1_900.0).cos();
            let vec = unit * scalar_nt;
            values.extend_from_slice(&[vec.x, vec.y, vec.z]);
        }
    }
    AnomalyOverlay::new(geometry, values, &BodyEllipsoid::WGS84, outside).unwrap()
}

pub fn pressure_altitude_m(pressure_pa: f64) -> f64 {
    let target = pressure_pa.ln();
    let residual = |h: f64| {
        isa_at_geopotential(geometric_to_geopotential_m(h))
            .unwrap()
            .pressure_pa
            .ln()
            - target
    };
    let (mut a, mut b) = (0.0, 1_000.0);
    let (mut ra, mut rb) = (residual(a), residual(b));
    for _ in 0..60 {
        if (b - a).abs() < 1e-9 || rb == 0.0 {
            return b;
        }
        let next = b - rb * (b - a) / (rb - ra);
        (a, ra) = (b, rb);
        b = next;
        rb = residual(b);
    }
    b
}

fn build_magnetic_pair(
    mode: MagneticAidingMode,
    seed: u64,
    clock: MasterClockBridge,
    world_overlay: &AnomalyOverlay,
    survey_overlay: &AnomalyOverlay,
) -> (SimulatedMagnetometer, MagneticAnomalyMap) {
    let main_field: Arc<dyn MagneticField> = Arc::new(HarmonicMagnetism::igrf14().unwrap());
    let (world_mag, overlays, bias_offset_t) = match mode {
        MagneticAidingMode::None | MagneticAidingMode::MainFieldOnly => {
            (Arc::clone(&main_field), Vec::new(), 0.0)
        }
        MagneticAidingMode::CrustalOverlay { bias_offset_t } => (
            Arc::new(MagnetismWithAnomaly {
                base: Arc::clone(&main_field),
                overlays: vec![world_overlay.clone()],
            }) as Arc<dyn MagneticField>,
            vec![survey_overlay.clone()],
            bias_offset_t,
        ),
    };
    let mag_spec = MagnetometerSpec {
        white_sigma_ut: 0.002,
        hard_iron_sigma_ut: 0.0,
        scale_factor_sigma: 0.0,
    };
    let mag_sensor = SimulatedMagnetometer::new(
        mag_spec,
        0x3A6F_u64.wrapping_add(seed),
        DECIMAL_YEAR,
        world_mag,
    )
    .unwrap();
    let map_cfg = MagneticAnomalyConfig {
        decimal_year: DECIMAL_YEAR,
        valid_radius_m: 600.0,
        variance_t2: 1.6e-17,
        bias_offset_t,
        gradient_step_m: 25.0,
    };
    let mut mag_map = MagneticAnomalyMap::new(
        main_field,
        overlays,
        map_cfg,
        clock,
        MAG_SOURCE,
        SourceEpoch::new(0),
    )
    .unwrap();
    mag_map.set_map_revision(Some(1), bias_offset_t).unwrap();
    (mag_sensor, mag_map)
}

pub fn build_sensors(
    mode: MagneticAidingMode,
    seed: u64,
    clock: MasterClockBridge,
    world_overlay: &AnomalyOverlay,
    survey_overlay: &AnomalyOverlay,
) -> Sensors {
    let imu = SimulatedImu::new(
        ImuGrade::Tactical.spec(),
        0x5_7A2_u64.wrapping_add(seed),
        Arc::new(BodyEllipsoid::WGS84),
    )
    .unwrap();
    let static_spec = PressureErrorSpec {
        white_sigma_pa: 3.0,
        bias: GaussMarkov {
            tau_s: 600.0,
            sigma: 20.0,
        },
    };
    let pitot_spec = PressureErrorSpec {
        white_sigma_pa: 3.0,
        bias: GaussMarkov {
            tau_s: 600.0,
            sigma: 10.0,
        },
    };
    let static_port = PitotStatic::static_only(static_spec, 0xBA20_u64.wrapping_add(seed)).unwrap();
    let pitot = PitotStatic::with_pitot(
        static_spec,
        pitot_spec,
        DVec3::X,
        0x5049_544F_u64.wrapping_add(seed),
    )
    .unwrap();
    let (mag_sensor, mag_map) =
        build_magnetic_pair(mode, seed, clock, world_overlay, survey_overlay);
    let isa = isa_at_geopotential(geometric_to_geopotential_m(1_000.0)).unwrap();
    let white_m = static_spec.white_sigma_pa / (isa.density_kg_m3 * 9.806_65);
    Sensors {
        imu,
        static_port,
        pitot,
        mag_sensor,
        mag_map,
        baro_variance_m2: white_m * white_m,
        baro_seq: WrappingSequence::new(0),
        utc: UtcTime::from_unix_seconds(EPOCH_UNIX_S),
    }
}
