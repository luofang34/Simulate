//! The shareable simulation link, version 2.
//!
//! One URL describes one navigation simulation. Everything sits in the URL
//! fragment, so opening the link sends none of it to a server:
//!
//! ```text
//! <base>#v=2&r=<legs>&h=<m>&hd=msl|hae&start=<start>&t=<UTC>&veh=<vehicle>
//!       &spd=<m/s>&seed=<n>&nav=<sets>&fly=<set>&out=<outages>&spoof=<spoof>
//!       &cam=<camera>&go=1&mb=<MB>
//! ```
//!
//! | key | value |
//! |---|---|
//! | `v` | the format version, `2`; a version 1 link is refused as retired |
//! | `r` | 2 to 300 legs joined by `;`, each `CODE[,fix]{,attribute}` (below) |
//! | `h` | flight height in metres where no altitude constraint says otherwise |
//! | `hd` | height datum of every height in metres or feet: `msl` or `hae` |
//! | `start` | `lat,lon[,height[,heading_deg]]`; without it the flight begins over the first fix heading for the second |
//! | `t` | start time, UTC, `YYYYMMDDTHHMMSSZ` |
//! | `veh` | `c172` or `x500` |
//! | `spd` | cruise speed, m/s; absent for the vehicle's own |
//! | `seed` | master seed of every simulated sensor, 0 to 2^64-1 |
//! | `nav` | navigation sets joined by `;`, each a `.`-joined list of `ins`, `baro`, `gnss`, `stars`, `visual`; `ins` in every set, no means twice in a set, no two sets with the same means |
//! | `fly` | the set that steers the flight; absent for the first |
//! | `out` | outages joined by `;`, one entry per sensor, each `sensor:start-end[,start-end...]` in seconds from the start, increasing and not overlapping; an empty end runs to the end of the flight; sensors `gnss`, `baro`, `stars`, `cam` |
//! | `spoof` | a spoofer dragging the GNSS receiver's fixes, `gnss:rate,bearing:start-end[,start-end...]`: m/s above 0 to 100, degrees from true north 0 to under 360, then spans as `out` writes them; from a span's start the reported position leaves the true one from no offset at that rate, the reported velocity with it, and a span that begins where the one before ends carries the drag on |
//! | `cam` | terrain camera `WxH,fov_deg,depression_deg,rate_hz`; the field of view is horizontal and depression 90 looks straight down |
//! | `go` | `1` builds the map package and starts without asking |
//! | `mb` | storage budget of the map package, MB, 1 to 65536 |
//!
//! A leg is an ARINC 424 path terminator ([`PathTerminator`]), the fix it
//! ends at or starts from when it has one, then attributes, each a key
//! letter and its value ([`Key`]). [`shape`] says which keys each code needs
//! and takes. A fix is `NAME:lat/lon`, `lat/lon`, or a bare `NAME` for the
//! latest earlier fix of that name; names are 1 to 32 of `[A-Za-z0-9_-]`.
//!
//! | key | value |
//! |---|---|
//! | `o` `m` | fly-over; first leg of the missed approach |
//! | `r` | roles joined by `.`: `IAF IF FACF FAF SDF MAP MAHF FEP RWY` |
//! | `t` | turn direction `L` or `R` |
//! | `a` | altitude `1500` at, `1500+` at or above, `1500-` at or below, `1500..3000` between |
//! | `g` | second altitude: `G` glide path, `I` intercept or `V` on the vertical angle, then a height |
//! | `v` `x` | vertical angle in degrees, negative descending; threshold crossing height |
//! | `s` | speed, written as `a` is: `60` m/s or `120kt` |
//! | `c` `h` `b` | course, heading, bearing from the navaid: degrees then `T` true or `M` magnetic |
//! | `d` | distance: along the course for `FC`, the excursion limit for `PI`, else from the navaid |
//! | `n` `z` | the navaid `b` and `d` are measured from; the `RF` arc centre (both fixes) |
//! | `l` | hold leg length with its unit: `60s`, `1min`, `4nm`, `7400m` |
//! | `p` | required navigation performance, a distance |
//!
//! Heights are metres, `ft`, or `FL` flight levels; distances metres or
//! `nm`. For example
//! `r=IF,ENO:39.231/-75.516,rIAF,a3000ft+;TF,RAIDR:38.8/-75.3,rFAF,a1800ft;HM,RAIDR,c220T,tR,l1min`.
//!
//! [`decode_url`] and [`decode_fragment`] refuse anything else with a
//! [`Refusal`] naming the parameter, the leg, the character and the
//! reason; nothing malformed falls back to a default. A [`SimulationLink`]
//! holds every number as a scaled integer with its unit, so
//! [`SimulationLink::fragment`] and the decoders round-trip exactly.

mod decode;
mod encode;
mod fixed;
mod link;
mod refusal;

pub use decode::{decode_fragment, decode_url, resolve_escapes};
pub use fixed::{Fixed, Micro, Milli, NumberFault};
pub use link::{
    Bearing, Camera, Distance, DistanceUnit, Duration, EVERY_LEG, Fix, Height, HeightDatum,
    HeightUnit, HoldLength, Key, Leg, Limit, MAX_LEGS, MAX_LINK_CHARS, Means, NavSet, North,
    Outage, PathTerminator, Role, SecondAltitude, SecondAltitudeKind, Sensor, Shape,
    SimulationLink, Span, Speed, SpeedUnit, Spoof, Start, StartTime, TimeUnit, TurnDirection,
    Vehicle, distance_from_navaid, shape,
};
pub use refusal::{Parameter, Reason, Refusal};
