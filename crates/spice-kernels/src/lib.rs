//! Pure-Rust NAIF SPICE binary kernels: a DAF container builder and reader,
//! and SPK ephemeris segments that can be written, read back and evaluated
//! without CSPICE.
//!
//! # Layers
//!
//! - [`Daf`] is the Double Precision Array File container: file record,
//!   comment area, summary and name records, and the arrays themselves. It
//!   knows nothing about what the arrays mean.
//! - [`SpkFile`] is a `DAF/SPK` ephemeris kernel made of [`SpkSegment`]s.
//!   Segments of type 9 (Lagrange interpolation, unequal time steps) and type
//!   13 (Hermite interpolation, unequal time steps) are built from sampled
//!   states ([`SampledStates`]) and evaluated in-crate. Segments of every other
//!   type are carried verbatim as [`SpkData::Raw`], so a kernel holding them
//!   still reads and writes without loss.
//!
//! # Conventions
//!
//! Everything is in SPICE's native units: epochs are TDB seconds past J2000,
//! positions are kilometres, velocities kilometres per second, bodies and
//! frames are NAIF integer codes. The crate converts nothing.
//!
//! Writing is deterministic: a file carries no timestamp or host detail, so
//! equal inputs give byte-identical output. Files are written little-endian
//! (`LTL-IEEE`) and read in either IEEE byte order.
//!
//! # Features
//!
//! `std` (default) adds the file-system helpers
//! `SpkFile::read_file_blocking` and `SpkFile::write_file_blocking`. Without
//! it the crate is `no_std` + `alloc` and works on `&[u8]` and `Vec<u8>`.
//!
//! # Example
//!
//! ```
//! use spice_kernels::{FRAME_J2000, SampledStates, SpkFile, SpkSegment};
//!
//! # fn main() -> Result<(), spice_kernels::Error> {
//! let epochs = vec![0.0, 60.0, 120.0, 180.0];
//! let states = epochs
//!     .iter()
//!     .map(|t| [7000.0 + t, 0.0, 0.0, 1.0, 0.0, 0.0])
//!     .collect();
//! let samples = SampledStates::new(epochs, states, 2)?;
//! let segment = SpkSegment::hermite("probe", -77, 399, FRAME_J2000, samples)?;
//!
//! let mut file = SpkFile::new("example")?;
//! file.push(segment);
//! let bytes = file.to_bytes()?;
//!
//! let read = SpkFile::parse(&bytes)?;
//! let state = read.state(-77, 399, 90.0)?;
//! assert!((state[0] - 7090.0).abs() < 1e-9);
//! # Ok(())
//! # }
//! ```
#![no_std]

extern crate alloc;
#[cfg(feature = "std")]
extern crate std;

mod ck;
mod daf;
mod error;
mod spk;

pub use ck::{
    CkData, CkFile, CkInterval, CkPointing, CkPointingInstance, CkSegment, CkType2, CkType3,
    matrix_to_quaternion, quaternion_to_matrix,
};
pub use daf::{Daf, DafArray, Endian};
pub use error::Error;
pub use spk::{
    ChebyshevPositionRecord, ChebyshevPositions, ChebyshevStateRecord, ChebyshevStates,
    FRAME_J2000, MAX_HERMITE_WINDOW, MAX_LAGRANGE_WINDOW, SampledStates, SpkData, SpkFile,
    SpkSegment,
};
