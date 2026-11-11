//! The shareable simulation link, version 1.
//!
//! One URL describes one navigation simulation. Everything sits in the URL
//! fragment, so opening the link sends none of it to a server:
//!
//! ```text
//! <base>#v=1&r=<route>&h=<m>&hd=msl|hae&start=<start>&t=<UTC>&veh=<vehicle>
//!       &spd=<m/s>&seed=<n>&nav=<sets>&fly=<set>&out=<outages>&cam=<camera>
//!       &go=1&mb=<MB>
//! ```
//!
//! | key | value |
//! |---|---|
//! | `v` | the format version, `1` |
//! | `r` | waypoints joined by `;`, each `lat,lon[,height[,turn[,name]]]`: decimal degrees north and east with at most 6 decimals; height in metres in the `hd` datum, empty for `h`; turn `o` fly-over, `b` or empty fly-by; name 1 to 32 of `[A-Za-z0-9_-]` |
//! | `h` | flight height in metres for waypoints that give none |
//! | `hd` | height datum: `msl` or `hae` |
//! | `start` | `lat,lon[,height[,heading_deg]]`; without it the flight begins over waypoint 1 heading for waypoint 2 |
//! | `t` | start time, UTC, `YYYYMMDDTHHMMSSZ` |
//! | `veh` | `c172` or `x500` |
//! | `spd` | cruise speed, m/s; absent for the vehicle's own |
//! | `seed` | master seed of every simulated sensor, 0 to 2^64-1 |
//! | `nav` | navigation sets joined by `;`, each a `.`-joined list of `ins`, `baro`, `gnss`, `stars`, `visual`; `ins` in every set, no means twice in a set, no two sets with the same means |
//! | `fly` | the set that steers the flight; absent for the first |
//! | `out` | outages joined by `;`, one entry per sensor, each `sensor:start-end[,start-end...]` in seconds from the start, increasing and not overlapping; an empty end runs to the end of the flight; sensors `gnss`, `baro`, `stars`, `cam` |
//! | `cam` | terrain camera `WxH,fov_deg,depression_deg,rate_hz`; the field of view is horizontal and depression 90 looks straight down |
//! | `go` | `1` builds the map package and starts without asking |
//! | `mb` | storage budget of the map package, MB, 1 to 65536 |
//!
//! [`decode_url`] and [`decode_fragment`] refuse anything else with a
//! [`Refusal`] naming the parameter, the character and the reason; nothing
//! malformed falls back to a default. A [`SimulationLink`] holds every number
//! as a scaled integer ([`Micro`], [`Milli`]), so
//! [`SimulationLink::fragment`] and the decoders round-trip exactly.

mod decode;
mod encode;
mod fixed;
mod link;
mod refusal;

pub use decode::{decode_fragment, decode_url, resolve_escapes};
pub use fixed::{Fixed, Micro, Milli, NumberFault};
pub use link::{
    Camera, HeightDatum, MAX_LINK_CHARS, MAX_WAYPOINTS, Means, NavSet, Outage, Sensor,
    SimulationLink, Span, Start, StartTime, Turn, Vehicle, Waypoint,
};
pub use refusal::{Parameter, Reason, Refusal};
