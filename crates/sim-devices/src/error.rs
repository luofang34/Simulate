use mbd_core::MbdError;
use sim_core::SimInstant;

/// Why a device refused an operation.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum DeviceError {
    /// A frame fails its protocol's rules.
    #[error("frame is not valid on this bus")]
    InvalidFrame {
        /// What the protocol objected to.
        #[source]
        source: MbdError,
    },
    /// A frame belongs to some other channel or protocol.
    #[error("frame {frame} does not belong on ARINC 429 channel {channel}")]
    WrongChannel {
        /// This bus's channel.
        channel: u8,
        /// The offending frame.
        frame: String,
    },
    /// A word was queued behind a later instant, so the transmitter would
    /// have to start in the past.
    #[error("word queued at {at:?}, before the previous request at {previous:?}")]
    OutOfOrder {
        /// The offending request.
        at: SimInstant,
        /// The latest earlier request.
        previous: SimInstant,
    },
    /// An inertial error statistic is negative or not finite.
    #[error("{triad} {field} of {value} is not a usable error statistic")]
    InvalidErrorModel {
        /// The sensor triad, `accel` or `gyro`.
        triad: &'static str,
        /// The offending field.
        field: &'static str,
        /// Its value.
        value: f64,
    },
    /// An error statistic or a sampling period of a device is not usable.
    #[error("{device} {field} of {value} is not usable")]
    InvalidStatistic {
        /// The device.
        device: &'static str,
        /// The offending field.
        field: &'static str,
        /// Its value.
        value: f64,
    },
    /// The geomagnetic model has no field for the position or date.
    #[error(
        "no magnetic field at latitude {lat_deg} deg, longitude {lon_deg} deg in {decimal_year}"
    )]
    NoMagneticField {
        /// Latitude asked for.
        lat_deg: f64,
        /// Longitude asked for.
        lon_deg: f64,
        /// Date asked for, decimal years.
        decimal_year: f64,
        /// The model's reason.
        #[source]
        source: world_geomag::GeomagError,
    },
    /// An ideal inertial increment is not finite, or its interval is not
    /// finite and positive.
    #[error("inertial increment over {dt_s} s is not usable")]
    InvalidIncrement {
        /// The interval, seconds.
        dt_s: f64,
    },
}
