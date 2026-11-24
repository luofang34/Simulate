use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use mbd_core::GasSpecies;
use sim_geo::{Geodetic, UtcTime};
use world_core::GeodeticPosition;
use world_thermosphere::{Effect, MsisInput, Nrlmsise00, Switch, Switches};

use super::{Nrlmsise00Activity, Nrlmsise00Atmosphere};
use crate::{
    ApHistory, AtmosphereContent, AtmosphereError, AtmosphereField, CalendarDate,
    GeomagneticActivity, IndexQuality, SpaceWeatherDay, SpaceWeatherSource, SpaceWeatherTable,
};

fn utc_of(year: i32, month: u8, day: u8, ut_seconds: f64) -> UtcTime {
    let date = CalendarDate::new(year, month, day).expect("valid date");
    let unix_s = (date.days_since_unix_epoch() as f64) * 86_400.0 + ut_seconds;
    UtcTime::from_unix_seconds(unix_s)
}

fn sw_day(date: CalendarDate, ap_val: f64, daily_ap: f64, f107: f64) -> SpaceWeatherDay {
    SpaceWeatherDay {
        date,
        ap: [Some(ap_val); 8],
        daily_ap: Some(daily_ap),
        f107_observed: Some(f107),
        f107_adjusted: Some(f107),
        f107_observed_center81: Some(f107),
        f107_adjusted_center81: Some(f107),
        quality: IndexQuality::Definitive,
        source: SpaceWeatherSource::Gfz,
    }
}

fn content_digest(model: &Nrlmsise00Atmosphere) -> u64 {
    let mut hasher = DefaultHasher::new();
    model.visit_content(&mut |bytes| bytes.hash(&mut hasher));
    hasher.finish()
}

#[test]
fn matches_world_thermosphere_oracle_and_sea_level_bounds() {
    let model = Nrlmsise00Atmosphere::fixed(150.0, 150.0, GeomagneticActivity::DailyAp(4.0));
    let utc = utc_of(2026, 6, 21, 29_000.0);
    let pos = Geodetic::from_degrees(60.0, -70.0, 400_000.0);
    let sample = model.sample(&pos, utc).expect("400 km sample");
    let raw = model.sample_thermosphere(&pos, utc).expect("raw state");

    let lst_h = MsisInput::apparent_local_solar_time_h(29_000.0, -70.0);
    let oracle = Nrlmsise00::STANDARD
        .evaluate(&MsisInput {
            position: GeodeticPosition::wgs84(60.0, -70.0, 400_000.0),
            day_of_year: 172,
            ut_seconds: 29_000.0,
            local_solar_time_h: lst_h,
            f107_previous_day: 150.0,
            f107_81_day_mean: 150.0,
            geomagnetic: GeomagneticActivity::DailyAp(4.0),
        })
        .expect("oracle");

    assert!((sample.density_kg_m3 - oracle.drag_mass_density_kg_m3).abs() < 1e-24);
    assert!((sample.temperature_k - oracle.temperature_k).abs() < 1e-12);
    assert!((raw.mass_density_kg_m3 - oracle.mass_density_kg_m3).abs() < 1e-24);

    let ground = model
        .sample(&Geodetic::from_degrees(45.0, 0.0, 0.0), utc)
        .expect("ground");
    assert!((250.0..=315.0).contains(&ground.temperature_k));
    assert!((1.0..=1.5).contains(&ground.density_kg_m3));

    let below = model.sample(&Geodetic::from_degrees(0.0, 0.0, -10.0), utc);
    assert!(matches!(
        below,
        Err(AtmosphereError::OutOfRange { floor_m, .. }) if floor_m == 0.0
    ));
}

#[test]
fn captures_species_stratification_and_anomalous_oxygen_drag_density() {
    let model = Nrlmsise00Atmosphere::quiet();
    let utc = utc_of(2026, 6, 21, 50_400.0);

    let e_region = model
        .sample(&Geodetic::from_degrees(0.0, 0.0, 120_000.0), utc)
        .expect("120 km");
    assert!(
        e_region.composition.mole_fraction(GasSpecies::N2)
            > e_region.composition.mole_fraction(GasSpecies::AtomicO)
    );

    let f_region = model
        .sample(&Geodetic::from_degrees(0.0, 0.0, 350_000.0), utc)
        .expect("350 km");
    assert!(
        f_region.composition.mole_fraction(GasSpecies::AtomicO) > 0.70,
        "AtomicO mole fraction at 350 km = {}",
        f_region.composition.mole_fraction(GasSpecies::AtomicO)
    );

    let exobase = model
        .sample(&Geodetic::from_degrees(0.0, 0.0, 850_000.0), utc)
        .expect("850 km");
    assert!(
        exobase.composition.mole_fraction(GasSpecies::He)
            > exobase.composition.mole_fraction(GasSpecies::N2)
    );

    let drag_model = Nrlmsise00Atmosphere::quiet().with_drag_density(true);
    let thermal_model = Nrlmsise00Atmosphere::quiet().with_drag_density(false);
    let p_800 = Geodetic::from_degrees(30.0, 0.0, 800_000.0);
    let d_drag = drag_model.sample(&p_800, utc).expect("drag").density_kg_m3;
    let d_therm = thermal_model
        .sample(&p_800, utc)
        .expect("thermal")
        .density_kg_m3;
    assert!(d_drag > d_therm, "drag={d_drag} thermal={d_therm}");
}

#[test]
fn solar_cycle_geomagnetic_storm_and_diurnal_bulge_match_physics() {
    let utc_14lst = utc_of(2026, 3, 20, 50_400.0); // 14:00 UT at 0 deg lon -> 14:00 LST
    let pos = Geodetic::from_degrees(0.0, 0.0, 400_000.0);

    let solar_min = Nrlmsise00Atmosphere::fixed(70.0, 70.0, GeomagneticActivity::DailyAp(4.0));
    let solar_max = Nrlmsise00Atmosphere::fixed(220.0, 220.0, GeomagneticActivity::DailyAp(4.0));
    let s_min = solar_min.sample(&pos, utc_14lst).expect("solar min");
    let s_max = solar_max.sample(&pos, utc_14lst).expect("solar max");
    assert!(s_max.density_kg_m3 > s_min.density_kg_m3 * 3.0);
    assert!(s_max.temperature_k > s_min.temperature_k + 250.0);

    let storm = Nrlmsise00Atmosphere::fixed(150.0, 150.0, GeomagneticActivity::DailyAp(200.0));
    let quiet = Nrlmsise00Atmosphere::quiet();
    assert!(
        storm.sample(&pos, utc_14lst).expect("storm").density_kg_m3
            > quiet.sample(&pos, utc_14lst).expect("quiet").density_kg_m3 * 1.4
    );

    // Diurnal bulge: 14:00 LST (afternoon bulge) vs 04:00 LST (pre-dawn trough) at 0 deg lon.
    let utc_04lst = utc_of(2026, 3, 20, 14_400.0);
    let afternoon = quiet.sample(&pos, utc_14lst).expect("14 LST");
    let predawn = quiet.sample(&pos, utc_04lst).expect("04 LST");
    assert!(afternoon.density_kg_m3 > predawn.density_kg_m3);
    assert!(afternoon.temperature_k > predawn.temperature_k);
}

#[test]
fn space_weather_table_lookup_and_content_hashing_cover_all_knobs() {
    let base_date = CalendarDate::new(2026, 6, 18).expect("date");
    let days: Vec<_> = (0..7)
        .map(|offset| {
            let d = base_date.add_days(offset).expect("day");
            let ap = if offset >= 4 { 110.0 } else { 4.0 };
            sw_day(d, ap, ap, 145.0)
        })
        .collect();
    let table = SpaceWeatherTable::from_days(days).expect("table");
    let sw_atmo = Nrlmsise00Atmosphere::from_space_weather(table);

    let pos = Geodetic::from_degrees(20.0, 10.0, 350_000.0);
    let quiet_sample = sw_atmo
        .sample(&pos, utc_of(2026, 6, 21, 36_000.0))
        .expect("quiet day");
    let storm_sample = sw_atmo
        .sample(&pos, utc_of(2026, 6, 23, 36_000.0))
        .expect("storm day");
    assert!(storm_sample.density_kg_m3 > quiet_sample.density_kg_m3 * 1.3);

    // Missing date outside table coverage returns SpaceWeather error.
    let oob = sw_atmo.sample(&pos, utc_of(2026, 8, 1, 0.0));
    assert!(matches!(oob, Err(AtmosphereError::SpaceWeather(_))));

    // Content hash sensitivity across every configuration knob.
    let base_fixed = Nrlmsise00Atmosphere::quiet();
    let alt_flux = Nrlmsise00Atmosphere::fixed(155.0, 150.0, GeomagneticActivity::DailyAp(4.0));
    let alt_hist = Nrlmsise00Atmosphere::new(Nrlmsise00Activity::Fixed {
        f107_previous_day: 150.0,
        f107_81_day_mean: 150.0,
        geomagnetic: GeomagneticActivity::History(ApHistory {
            daily: 4.0,
            current: 4.0,
            hours_3_before: 4.0,
            hours_6_before: 4.0,
            hours_9_before: 4.0,
            mean_12_to_33_hours_before: 4.0,
            mean_36_to_57_hours_before: 4.0,
        }),
    });
    let alt_switch = base_fixed
        .clone()
        .with_switches(Switches::STANDARD.with(Effect::Diurnal, Switch::Off));
    let alt_mode = base_fixed.clone().with_drag_density(false);

    let digests = [
        content_digest(&base_fixed),
        content_digest(&alt_flux),
        content_digest(&alt_hist),
        content_digest(&alt_switch),
        content_digest(&alt_mode),
        content_digest(&sw_atmo),
    ];
    for (i, a) in digests.iter().enumerate() {
        for b in digests.iter().skip(i + 1) {
            assert_ne!(a, b);
        }
    }
}
