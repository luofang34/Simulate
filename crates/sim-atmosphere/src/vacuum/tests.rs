use super::*;

#[test]
fn vacuum_is_empty_at_any_position_and_instant() {
    let utc = UtcTime::from_unix_seconds(-14_182_940.0);
    for height_m in [-3_000.0, 0.0, 15_000.0, 400_000_000.0] {
        let position = Geodetic::from_degrees(0.67416, 23.47314, height_m);
        let sample = VacuumAtmosphere.sample(&position, utc).unwrap();
        assert_eq!(sample.pressure_pa, 0.0);
        assert_eq!(sample.density_kg_m3, 0.0);
        assert_eq!(sample.speed_of_sound_m_s, 0.0);
        assert_eq!(sample.wind_ned_m_s, DVec3::ZERO);
        assert_eq!(sample.temperature_k, COSMIC_BACKGROUND_K);
    }
}
