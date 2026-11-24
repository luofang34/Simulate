//! Written kernels read back unchanged, writing is byte-deterministic, and a
//! kernel of segment types the crate does not model passes through intact.

use spice_kernels::{
    CkFile, CkInterval, CkPointingInstance, CkSegment, CkType2, CkType3, Daf, Error, FRAME_J2000,
    SampledStates, SpkData, SpkFile, SpkSegment,
};

/// A DE440s excerpt (type 2 segments) written by Simulate's kernel tooling.
const EXCERPT: &[u8] = include_bytes!("data/de440s_excerpt_1969.bsp");

const EPOCH: f64 = 8.0e8;

/// States with no short binary representation at unequal epochs, so that a
/// lossy round trip could not go unnoticed.
fn samples(count: usize) -> (Vec<f64>, Vec<[f64; 6]>) {
    let epochs: Vec<f64> = (0..count)
        .map(|k| EPOCH + 40.1 * k as f64 + 0.3 * (k % 3) as f64)
        .collect();
    let states = epochs
        .iter()
        .map(|&et| {
            let t = (et - EPOCH) / 7.0;
            [
                7000.1 + t,
                -t / 3.0,
                0.7 * t * t,
                1.0 / 7.0,
                -1.0 / 21.0,
                0.2 * t,
            ]
        })
        .collect();
    (epochs, states)
}

/// A kernel of `count` segments alternating between the two sampled types,
/// with differing lengths and windows.
fn kernel(count: usize) -> Result<SpkFile, Error> {
    let mut file = SpkFile::new("spice-kernels round trip")?;
    file.set_comment_lines(vec![
        "Written by the spice-kernels round-trip test.".into(),
        String::new(),
        "Units: km, km/s, TDB seconds past J2000.".into(),
    ])?;
    for index in 0..count {
        let window = 2 + index % 5;
        let (epochs, states) = samples(40 + 17 * index);
        let samples = SampledStates::new(epochs, states, window)?;
        let name = format!("segment {index}");
        let target = -100 - i32::try_from(index).unwrap_or(0);
        let segment = if index % 2 == 0 {
            SpkSegment::lagrange(&name, target, 399, FRAME_J2000, samples)?
        } else {
            SpkSegment::hermite(&name, target, 399, FRAME_J2000, samples)?
        };
        file.push(segment);
    }
    Ok(file)
}

#[test]
fn written_kernel_reads_back_with_identical_segments() -> Result<(), Error> {
    // Thirty segments need two summary records.
    let file = kernel(30)?;
    let bytes = file.to_bytes()?;
    let read = SpkFile::parse(&bytes)?;
    assert_eq!(read.segments(), file.segments());
    assert_eq!(read.comment_lines(), file.comment_lines());
    assert_eq!(read.internal_name(), "spice-kernels round trip");
    for (written, got) in file.segments().iter().zip(read.segments()) {
        let (SpkData::Lagrange(a) | SpkData::Hermite(a)) = written.data() else {
            panic!("unexpected segment data {:?}", written.data());
        };
        let (SpkData::Lagrange(b) | SpkData::Hermite(b)) = got.data() else {
            panic!("unexpected segment data {:?}", got.data());
        };
        assert_eq!(written.data().data_type(), got.data().data_type());
        assert_eq!(a.window(), b.window());
        let bits = |values: &[f64]| values.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(a.epochs()), bits(b.epochs()));
        assert_eq!(
            bits(a.states().as_flattened()),
            bits(b.states().as_flattened())
        );
        assert_eq!(bits(&written.coverage()), bits(&got.coverage()));
    }
    // Reading and writing again changes nothing.
    assert_eq!(read.to_bytes()?, bytes);
    Ok(())
}

#[test]
fn writing_is_byte_deterministic() -> Result<(), Error> {
    let first = kernel(7)?.to_bytes()?;
    let second = kernel(7)?.to_bytes()?;
    assert_eq!(first, second);
    assert_eq!(first.len() % 1024, 0);
    Ok(())
}

#[test]
fn narrowed_coverage_survives_the_round_trip() -> Result<(), Error> {
    let (epochs, states) = samples(20);
    let coverage = [EPOCH + 90.0, EPOCH + 600.0];
    let samples = SampledStates::new(epochs, states, 4)?;
    let segment = SpkSegment::hermite("narrow", -9, 399, 17, samples)?.with_coverage(coverage)?;
    let mut file = SpkFile::new("narrow")?;
    file.push(segment);
    let read = SpkFile::parse(&file.to_bytes()?)?;
    let got = &read.segments()[0];
    assert_eq!(got.coverage(), coverage);
    assert_eq!((got.target(), got.center(), got.frame()), (-9, 399, 17));
    assert!(matches!(
        read.state(-9, 399, EPOCH),
        Err(Error::NoCoverage { .. })
    ));
    assert!(read.state(-9, 399, EPOCH + 90.0).is_ok());
    Ok(())
}

#[cfg(feature = "std")]
#[test]
fn file_helpers_write_and_read_the_same_kernel() -> Result<(), Box<dyn std::error::Error>> {
    let file = kernel(3)?;
    let directory = tempfile::tempdir()?;
    let path = directory.path().join("round_trip.bsp");
    file.write_file_blocking(&path)?;
    assert_eq!(std::fs::read(&path)?, file.to_bytes()?);
    assert_eq!(SpkFile::read_file_blocking(&path)?, file);
    let missing = SpkFile::read_file_blocking(directory.path().join("absent.bsp"));
    assert!(matches!(missing, Err(Error::Io { .. })));
    Ok(())
}

#[test]
fn excerpt_of_a_planetary_ephemeris_passes_through_unchanged() -> Result<(), Error> {
    let original = SpkFile::parse(EXCERPT)?;
    assert_eq!(original.segments().len(), 11);
    assert!(
        original
            .segments()
            .iter()
            .all(|s| s.data().data_type() == 2)
    );

    let rewritten = original.to_bytes()?;
    let read = SpkFile::parse(&rewritten)?;
    assert_eq!(read.internal_name(), original.internal_name());
    for (before, after) in original.segments().iter().zip(read.segments()) {
        assert_eq!(before.name(), after.name());
        assert_eq!(
            (before.target(), before.center(), before.frame()),
            (after.target(), after.center(), after.frame())
        );
        assert_eq!(
            before.coverage().map(f64::to_bits),
            after.coverage().map(f64::to_bits)
        );
        let (SpkData::Type2(a), SpkData::Type2(b)) = (before.data(), after.data()) else {
            panic!("type 2 segments are decoded");
        };
        assert_eq!(a, b);
    }

    // The container level agrees too: same arrays at the same addresses, so
    // the only bytes that may differ are padding in the name record.
    let (before, after) = (Daf::parse(EXCERPT)?, Daf::parse(&rewritten)?);
    assert_eq!(before, after);
    assert_eq!(rewritten.len(), EXCERPT.len());
    let differing: Vec<usize> = (0..EXCERPT.len())
        .filter(|&i| EXCERPT[i] != rewritten[i])
        .collect();
    assert!(
        differing.iter().all(|&i| (2048..3072).contains(&i)),
        "{differing:?}"
    );
    assert!(original.state(301, 3, -960_910_900.0).is_ok());
    Ok(())
}

#[test]
fn ck_file_round_trips_and_is_byte_deterministic() -> Result<(), Error> {
    let mut ck = CkFile::new("ck round trip")?;
    ck.set_comment_lines(vec!["CK Type 2 and Type 3".into()])?;
    for index in 0..30_i32 {
        let inst = -1000 - index;
        if index % 2 == 0 {
            let t2 = CkType2::new(vec![CkInterval {
                start_sclk: 100.0,
                stop_sclk: 200.0,
                seconds_per_tick: 0.01,
                quaternion: [1.0, 0.0, 0.0, 0.0],
                angular_velocity: [0.001, -0.002, 0.003],
            }])?;
            ck.push(CkSegment::type2(
                &format!("ck2_{index}"),
                inst,
                FRAME_J2000,
                t2,
            )?);
        } else {
            let t3 = CkType3::single_interval(
                vec![
                    CkPointingInstance {
                        sclk: 100.0,
                        quaternion: [1.0, 0.0, 0.0, 0.0],
                        angular_velocity: Some([0.0, 0.0, 0.01]),
                    },
                    CkPointingInstance {
                        sclk: 200.0,
                        quaternion: [0.0, 0.0, 0.0, 1.0],
                        angular_velocity: Some([0.0, 0.0, 0.01]),
                    },
                ],
                true,
            )?;
            ck.push(CkSegment::type3(
                &format!("ck3_{index}"),
                inst,
                FRAME_J2000,
                t3,
            )?);
        }
    }
    let bytes = ck.to_bytes()?;
    assert_eq!(bytes, ck.to_bytes()?);
    let read = CkFile::parse(&bytes)?;
    assert_eq!(read, ck);
    assert_eq!(read.to_bytes()?, bytes);
    Ok(())
}

#[cfg(feature = "std")]
#[test]
fn ck_file_helpers_write_and_read_the_same_kernel() -> Result<(), Box<dyn std::error::Error>> {
    let mut ck = CkFile::new("ck file io")?;
    ck.push(CkSegment::type2(
        "seg0",
        -77_000,
        FRAME_J2000,
        CkType2::new(vec![CkInterval {
            start_sclk: 0.0,
            stop_sclk: 100.0,
            seconds_per_tick: 1.0,
            quaternion: [1.0, 0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.01, 0.0],
        }])?,
    )?);
    let dir = tempfile::tempdir()?;
    let path = dir.path().join("test.bc");
    ck.write_file_blocking(&path)?;
    assert_eq!(CkFile::read_file_blocking(&path)?, ck);
    Ok(())
}
