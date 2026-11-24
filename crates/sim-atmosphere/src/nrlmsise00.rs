//! NRLMSISE-00 empirical atmosphere (`world-thermosphere`) behind
//! [`AtmosphereField`] and [`AtmosphereContent`], supporting fixed solar/geomagnetic
//! indices as well as dynamic daily/3-hourly [`SpaceWeatherTable`] evaluation.

use glam::DVec3;
use mbd_core::{AmbientGas, GasSpecies};
use sim_geo::{Geodetic, UtcTime};
use world_core::{GeodeticPosition, WorldInstant};
use world_thermosphere::space_weather::{
    CalendarDate, IndexQuality, SpaceWeatherDay, SpaceWeatherSource, SpaceWeatherTable,
};
use world_thermosphere::{
    Effect, GeomagneticActivity, MsisInput, Nrlmsise00, Switch, Switches, ThermosphereError,
    ThermosphereState,
};

use crate::content::{feed_f64s, feed_kind};
use crate::{
    AtmosphereContent, AtmosphereError, AtmosphereField, AtmosphereSample, VacuumAtmosphere,
    geometric_to_geopotential_m,
};

#[cfg(test)]
mod tests;

const QUIET_F107_SFU: f64 = 150.0;
const QUIET_DAILY_AP: f64 = 4.0;
const SECONDS_PER_DAY: f64 = 86_400.0;

/// Solar and geomagnetic activity configuration for [`Nrlmsise00Atmosphere`].
#[derive(Clone, Debug, PartialEq)]
pub enum Nrlmsise00Activity {
    /// Constant solar radio flux (`F10.7`) and geomagnetic activity (`Ap` or 3-hourly history).
    Fixed {
        /// Observed 10.7 cm solar radio flux of the previous day, s.f.u.
        f107_previous_day: f64,
        /// Centered 81-day mean observed 10.7 cm solar radio flux, s.f.u.
        f107_81_day_mean: f64,
        /// Geomagnetic activity (daily `Ap` or 3-hourly `ApHistory`).
        geomagnetic: GeomagneticActivity,
    },
    /// Time-varying solar and 3-hourly geomagnetic indices synthesized from a table.
    SpaceWeather(SpaceWeatherTable),
}

/// Empirical NRLMSISE-00 atmosphere from sea level (`0 m`) into the exosphere.
///
/// By default (`use_drag_density = true`), [`AtmosphereSample::density_kg_m3`]
/// returns [`ThermosphereState::drag_mass_density_kg_m3`] (NRLMSISE-00 `GTD7D`),
/// which includes the non-thermal anomalous oxygen component appreciable above
/// ~500 km for orbital drag. Static thermal pressure, sound speed, and
/// [`AtmosphereSample::composition`] are derived from the seven resolved thermal
/// neutral species (`N2`, `O2`, `AtomicO`, `Ar`, `He`, `H`, `AtomicN`).
#[derive(Clone, Debug, PartialEq)]
pub struct Nrlmsise00Atmosphere {
    model: Nrlmsise00,
    activity: Nrlmsise00Activity,
    use_drag_density: bool,
}

impl Default for Nrlmsise00Atmosphere {
    fn default() -> Self {
        Self::quiet()
    }
}

impl Nrlmsise00Atmosphere {
    /// Constructs an NRLMSISE-00 atmosphere driven by `activity` with standard switches
    /// and effective drag mass density enabled (`GTD7D`).
    #[must_use]
    pub const fn new(activity: Nrlmsise00Activity) -> Self {
        Self {
            model: Nrlmsise00::STANDARD,
            activity,
            use_drag_density: true,
        }
    }

    /// Fixed-activity NRLMSISE-00 atmosphere (`F10.7` and `Ap` held constant).
    #[must_use]
    pub const fn fixed(
        f107_previous_day: f64,
        f107_81_day_mean: f64,
        geomagnetic: GeomagneticActivity,
    ) -> Self {
        Self::new(Nrlmsise00Activity::Fixed {
            f107_previous_day,
            f107_81_day_mean,
            geomagnetic,
        })
    }

    /// Reference quiet solar/geomagnetic baseline (`F10.7 = 150 sfu`, `Ap = 4`).
    #[must_use]
    pub const fn quiet() -> Self {
        Self::fixed(
            QUIET_F107_SFU,
            QUIET_F107_SFU,
            GeomagneticActivity::DailyAp(QUIET_DAILY_AP),
        )
    }

    /// Time-varying NRLMSISE-00 atmosphere backed by a [`SpaceWeatherTable`].
    #[must_use]
    pub const fn from_space_weather(table: SpaceWeatherTable) -> Self {
        Self::new(Nrlmsise00Activity::SpaceWeather(table))
    }

    /// Overrides the 23 NRLMSISE-00 model term [`Switches`].
    #[must_use]
    pub const fn with_switches(mut self, switches: Switches) -> Self {
        self.model = Nrlmsise00::new(switches);
        self
    }

    /// Selects effective drag density (`true` -> `drag_mass_density_kg_m3`, `GTD7D`)
    /// vs thermal mass density (`false` -> `mass_density_kg_m3`, `GTD7`).
    #[must_use]
    pub const fn with_drag_density(mut self, use_drag_density: bool) -> Self {
        self.use_drag_density = use_drag_density;
        self
    }

    /// Active term switches.
    #[must_use]
    pub const fn switches(&self) -> Switches {
        self.model.switches()
    }

    /// Active solar/geomagnetic configuration.
    #[must_use]
    pub const fn activity(&self) -> &Nrlmsise00Activity {
        &self.activity
    }

    /// Whether [`ThermosphereState::drag_mass_density_kg_m3`] is reported in samples.
    #[must_use]
    pub const fn uses_drag_density(&self) -> bool {
        self.use_drag_density
    }

    /// Evaluates the underlying NRLMSISE-00 model directly, returning all raw species
    /// number densities (including anomalous oxygen) and exospheric temperature.
    ///
    /// # Errors
    ///
    /// Returns [`AtmosphereError::OutOfRange`] below `0 m` or on non-finite coordinates,
    /// [`AtmosphereError::SpaceWeather`] if a table lookup fails, or
    /// [`AtmosphereError::Thermosphere`] on invalid inputs or model refusal.
    pub fn sample_thermosphere(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<ThermosphereState, AtmosphereError> {
        let input = build_msis_input(&self.activity, position, utc)?;
        self.model
            .evaluate(&input)
            .map_err(|err| map_thermosphere_error(err, position.height_m))
    }
}

impl AtmosphereField for Nrlmsise00Atmosphere {
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let state = self.sample_thermosphere(position, utc)?;
        let density_kg_m3 = if self.use_drag_density {
            state.drag_mass_density_kg_m3
        } else {
            state.mass_density_kg_m3
        };
        let thermal_species = [
            (GasSpecies::N2, state.number_density.molecular_nitrogen_m3),
            (GasSpecies::O2, state.number_density.molecular_oxygen_m3),
            (GasSpecies::AtomicO, state.number_density.atomic_oxygen_m3),
            (GasSpecies::Ar, state.number_density.argon_m3),
            (GasSpecies::He, state.number_density.helium_m3),
            (GasSpecies::H, state.number_density.atomic_hydrogen_m3),
            (GasSpecies::AtomicN, state.number_density.atomic_nitrogen_m3),
        ];
        let Ok(gas) =
            AmbientGas::from_partial_number_densities(state.temperature_k, &thermal_species)
        else {
            return Ok(VacuumAtmosphere::SAMPLE);
        };
        if gas.is_vacuum() {
            return Ok(AtmosphereSample {
                density_kg_m3,
                ..VacuumAtmosphere::SAMPLE
            });
        }
        Ok(AtmosphereSample {
            temperature_k: state.temperature_k,
            pressure_pa: gas.pressure_pa,
            density_kg_m3,
            speed_of_sound_m_s: gas.sound_speed_m_s,
            wind_ned_m_s: DVec3::ZERO,
            composition: gas.composition,
        })
    }
}

impl AtmosphereContent for Nrlmsise00Atmosphere {
    fn visit_content(&self, f: &mut dyn FnMut(&[u8])) {
        feed_kind(f, "nrlmsise00");
        f(&[u8::from(self.use_drag_density)]);
        let switches = self.model.switches();
        for effect in Effect::ALL {
            let code = match switches.get(effect) {
                Switch::Off => 0_u8,
                Switch::On => 1_u8,
                Switch::CrossTermsOnly => 2_u8,
            };
            f(&[code]);
        }
        match &self.activity {
            Nrlmsise00Activity::Fixed {
                f107_previous_day,
                f107_81_day_mean,
                geomagnetic,
            } => {
                feed_kind(f, "fixed");
                feed_f64s(f, &[*f107_previous_day, *f107_81_day_mean]);
                match geomagnetic {
                    GeomagneticActivity::DailyAp(ap) => {
                        f(&[0_u8]);
                        feed_f64s(f, &[*ap]);
                    }
                    GeomagneticActivity::History(h) => {
                        f(&[1_u8]);
                        feed_f64s(
                            f,
                            &[
                                h.daily,
                                h.current,
                                h.hours_3_before,
                                h.hours_6_before,
                                h.hours_9_before,
                                h.mean_12_to_33_hours_before,
                                h.mean_36_to_57_hours_before,
                            ],
                        );
                    }
                }
            }
            Nrlmsise00Activity::SpaceWeather(table) => {
                feed_kind(f, "space-weather-table");
                let days = table.days();
                f(&(days.len() as u64).to_le_bytes());
                for day in days {
                    feed_space_weather_day(f, day);
                }
            }
        }
    }
}

fn feed_space_weather_day(f: &mut dyn FnMut(&[u8]), day: &SpaceWeatherDay) {
    f(&day.date.year.to_le_bytes());
    f(&[day.date.month, day.date.day]);
    for slot in day.ap {
        feed_opt_f64(f, slot);
    }
    feed_opt_f64(f, day.daily_ap);
    feed_opt_f64(f, day.f107_observed);
    feed_opt_f64(f, day.f107_adjusted);
    feed_opt_f64(f, day.f107_observed_center81);
    feed_opt_f64(f, day.f107_adjusted_center81);
    let quality_code = match day.quality {
        IndexQuality::MonthlyPredicted => 0_u8,
        IndexQuality::Predicted => 1_u8,
        IndexQuality::Interpolated => 2_u8,
        IndexQuality::Provisional => 3_u8,
        IndexQuality::Definitive => 4_u8,
    };
    let source_code = match day.source {
        SpaceWeatherSource::Gfz => 0_u8,
        SpaceWeatherSource::NoaaDailySolarData => 1_u8,
        SpaceWeatherSource::NoaaF107Json => 2_u8,
        SpaceWeatherSource::CelestrakCsv => 3_u8,
        SpaceWeatherSource::CelestrakLegacy => 4_u8,
        SpaceWeatherSource::Merged => 5_u8,
    };
    f(&[quality_code, source_code]);
}

fn feed_opt_f64(f: &mut dyn FnMut(&[u8]), value: Option<f64>) {
    match value {
        None => f(&[0_u8]),
        Some(v) => {
            f(&[1_u8]);
            f(&v.to_bits().to_le_bytes());
        }
    }
}

fn build_msis_input(
    activity: &Nrlmsise00Activity,
    position: &Geodetic,
    utc: UtcTime,
) -> Result<MsisInput, AtmosphereError> {
    if !position.height_m.is_finite()
        || !position.lat_rad.is_finite()
        || !position.lon_rad.is_finite()
    {
        return Err(AtmosphereError::OutOfRange {
            geopotential_m: position.height_m,
            floor_m: 0.0,
            ceiling_m: f64::INFINITY,
        });
    }
    let geodetic = GeodeticPosition::wgs84(
        position.lat_rad.to_degrees(),
        position.lon_rad.to_degrees(),
        position.height_m,
    );
    let secs = utc.unix_seconds();
    if !secs.is_finite() {
        return Err(AtmosphereError::Thermosphere(
            ThermosphereError::InvalidInput {
                quantity: "ut_seconds",
                value: secs,
            },
        ));
    }
    let (whole_secs, sub_micros) = split_unix_seconds(secs);
    match activity {
        Nrlmsise00Activity::Fixed {
            f107_previous_day,
            f107_81_day_mean,
            geomagnetic,
        } => {
            let (date, sec_of_day) = CalendarDate::from_unix_seconds(whole_secs)?;
            let ut_seconds = (sec_of_day + (sub_micros as f64) * 1.0e-6).min(SECONDS_PER_DAY);
            Ok(MsisInput {
                position: geodetic,
                day_of_year: date.day_of_year(),
                ut_seconds,
                local_solar_time_h: MsisInput::apparent_local_solar_time_h(
                    ut_seconds,
                    geodetic.longitude_deg,
                ),
                f107_previous_day: *f107_previous_day,
                f107_81_day_mean: *f107_81_day_mean,
                geomagnetic: *geomagnetic,
            })
        }
        Nrlmsise00Activity::SpaceWeather(table) => {
            let unix_micros = whole_secs
                .saturating_mul(1_000_000)
                .saturating_add(sub_micros);
            let instant = WorldInstant::new(0, unix_micros);
            Ok(table.msis_input_at_instant(geodetic, instant)?)
        }
    }
}

fn split_unix_seconds(unix_seconds: f64) -> (i64, i64) {
    let floor_s = libm::floor(unix_seconds);
    let mut whole_s = floor_s as i64;
    let mut sub_micros = libm::round((unix_seconds - floor_s) * 1_000_000.0) as i64;
    if sub_micros >= 1_000_000 {
        whole_s = whole_s.saturating_add(1);
        sub_micros = sub_micros.saturating_sub(1_000_000);
    }
    (whole_s, sub_micros)
}

fn map_thermosphere_error(err: ThermosphereError, height_m: f64) -> AtmosphereError {
    match err {
        ThermosphereError::BelowModel { lowest_km, .. } => AtmosphereError::OutOfRange {
            geopotential_m: geometric_to_geopotential_m(height_m),
            floor_m: lowest_km * 1_000.0,
            ceiling_m: f64::INFINITY,
        },
        other => AtmosphereError::Thermosphere(other),
    }
}
