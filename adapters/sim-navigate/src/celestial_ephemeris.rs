//! Bridge between `sim-spacetime` [`EphemerisCatalog`] / [`BodyOrientationCatalog`]
//! and `navigate_celestial::Ephemeris` + `hifitime::Epoch`.

use glam::DVec3;
use navigate_celestial::{Duration, Ephemeris as NavEphemeris, Epoch, FrameId, State, TimeScale};
use sim_spacetime::{
    BodyOrientationCatalog, DerivativeOrder, EphemerisCatalog, EphemerisError, FrameCode, NaifId,
    OrientationError, TdbEpoch, default_iau_frame_code,
};

/// Converts a `sim_spacetime::TdbEpoch` into a `navigate_celestial::Epoch` (`hifitime::Epoch`) on [`TimeScale::TDB`].
#[must_use]
pub fn tdb_to_hifitime_epoch(epoch: TdbEpoch) -> Epoch {
    let j2000_tdb = Epoch::from_tdb_seconds(0.0);
    let ns = epoch.to_j2000_nanos_floor();
    j2000_tdb + Duration::from_total_nanoseconds(ns)
}

/// Converts a `navigate_celestial::Epoch` (`hifitime::Epoch`) into [`TdbEpoch`] (transforming to [`TimeScale::TDB`] first).
#[must_use]
pub fn tdb_from_hifitime_epoch(epoch: Epoch) -> TdbEpoch {
    let tdb = epoch.to_time_scale(TimeScale::TDB);
    let delta = tdb - Epoch::from_tdb_seconds(0.0);
    TdbEpoch::from_j2000_nanos(delta.total_nanoseconds())
}

/// Error returned by [`CelestialObserverAdapter`] or [`PureEphemerisNavAdapter`] during `navigate-celestial` sight reduction.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum CelestialAdapterError {
    /// Underlying ephemeris lookup failed.
    #[error("ephemeris error: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Underlying orientation lookup failed.
    #[error("orientation error: {0}")]
    Orientation(#[from] OrientationError),
    /// Observer `State.frame.orientation_naif_id` did not match `J2000` or the center body's fixed frame.
    #[error("unrecognized observer frame orientation {orientation} on center {center}")]
    UnrecognizedFrame {
        /// Frame center NAIF ID.
        center: i32,
        /// Frame orientation NAIF code.
        orientation: i32,
    },
}

/// Combined ephemeris + orientation adapter implementing [`navigate_celestial::Ephemeris`]
/// for observers expressed in either `J2000 (1)` or body-fixed frames (`ITRF93`, `MOON_ME`, `IAU_*`).
pub struct CelestialObserverAdapter<'a, E: EphemerisCatalog, O: BodyOrientationCatalog> {
    /// Geometric SPK ephemeris provider.
    pub ephemeris: &'a E,
    /// Body orientation provider.
    pub orientation: &'a O,
}

impl<E: EphemerisCatalog, O: BodyOrientationCatalog> NavEphemeris
    for CelestialObserverAdapter<'_, E, O>
{
    type Error = CelestialAdapterError;

    fn barycentric_state(&self, state: State) -> Result<State, Self::Error> {
        let tdb = tdb_from_hifitime_epoch(state.epoch);
        let center_id = NaifId(state.frame.center_naif_id);
        let center_icrf = self.ephemeris.geometric_state_icrf(
            center_id,
            NaifId::SSB,
            tdb,
            DerivativeOrder::Velocity,
        )?;
        let raw_p = DVec3::from_array(state.position_m);
        let raw_v = DVec3::from_array(state.velocity_m_s);

        let (rel_p_icrf, rel_v_icrf) = if state.frame.orientation_naif_id == FrameCode::J2000.0 {
            (raw_p, raw_v)
        } else {
            let rot = self.orientation.body_rotation(center_id, tdb)?;
            let code = FrameCode(state.frame.orientation_naif_id);
            if code != rot.frame_code && code != default_iau_frame_code(center_id) {
                return Err(CelestialAdapterError::UnrecognizedFrame {
                    center: state.frame.center_naif_id,
                    orientation: state.frame.orientation_naif_id,
                });
            }
            rot.fixed_to_icrf_pv(raw_p, raw_v)
        };

        Ok(State {
            position_m: (center_icrf.pos_m + rel_p_icrf).to_array(),
            velocity_m_s: (center_icrf.vel_m_s + rel_v_icrf).to_array(),
            epoch: state.epoch,
            frame: FrameId::SSB_J2000,
        })
    }

    fn body_state(&self, body: i32, epoch: Epoch) -> Result<State, Self::Error> {
        let tdb = tdb_from_hifitime_epoch(epoch);
        let s = self.ephemeris.geometric_state_icrf(
            NaifId(body),
            NaifId::SSB,
            tdb,
            DerivativeOrder::Velocity,
        )?;
        Ok(State {
            position_m: s.pos_m.to_array(),
            velocity_m_s: s.vel_m_s.to_array(),
            epoch,
            frame: FrameId::SSB_J2000,
        })
    }
}

/// Pure `J2000` ephemeris adapter implementing [`navigate_celestial::Ephemeris`] for any [`EphemerisCatalog`].
pub struct PureEphemerisNavAdapter<'a, E: EphemerisCatalog> {
    /// Geometric SPK ephemeris provider.
    pub ephemeris: &'a E,
}

impl<E: EphemerisCatalog> NavEphemeris for PureEphemerisNavAdapter<'_, E> {
    type Error = EphemerisError;

    fn barycentric_state(&self, state: State) -> Result<State, Self::Error> {
        if state.frame.orientation_naif_id != FrameCode::J2000.0 {
            return Err(EphemerisError::UnsupportedObserverOrientation(
                state.frame.orientation_naif_id,
            ));
        }
        let tdb = tdb_from_hifitime_epoch(state.epoch);
        let center_icrf = self.ephemeris.geometric_state_icrf(
            NaifId(state.frame.center_naif_id),
            NaifId::SSB,
            tdb,
            DerivativeOrder::Velocity,
        )?;
        let pos = center_icrf.pos_m + DVec3::from_array(state.position_m);
        let vel = center_icrf.vel_m_s + DVec3::from_array(state.velocity_m_s);
        Ok(State {
            position_m: pos.to_array(),
            velocity_m_s: vel.to_array(),
            epoch: state.epoch,
            frame: FrameId::SSB_J2000,
        })
    }

    fn body_state(&self, body: i32, epoch: Epoch) -> Result<State, Self::Error> {
        let tdb = tdb_from_hifitime_epoch(epoch);
        let s = self.ephemeris.geometric_state_icrf(
            NaifId(body),
            NaifId::SSB,
            tdb,
            DerivativeOrder::Velocity,
        )?;
        Ok(State {
            position_m: s.pos_m.to_array(),
            velocity_m_s: s.vel_m_s.to_array(),
            epoch,
            frame: FrameId::SSB_J2000,
        })
    }
}
