use glam::{DMat3, DQuat, DVec3};

/// Smallest separation between two stars that fixes an attitude, rad: 10°.
const MIN_STAR_SEPARATION_RAD: f64 = 10.0 * std::f64::consts::PI / 180.0;

/// One star mark: where a known star was seen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarSighting {
    /// Unit line of sight in stable-member axes when marked.
    pub line_of_sight_platform: DVec3,
    /// Catalog direction of the star in J2000.
    pub star_direction_j2000: DVec3,
}

/// The result of a two-star alignment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuRealignment {
    /// Where the marks put the stable member: stable-member-to-J2000.
    pub platform_to_inertial: DQuat,
    /// Marked separation minus catalog separation, rad: the crew's check that
    /// the right stars were marked, displayed on the DSKY in Apollo.
    pub star_angle_difference_rad: f64,
}

/// Why two marks cannot fix an attitude.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum AlignmentError {
    /// The stars are too close together, in sky or as marked, to fix the
    /// rotation about the line between them.
    #[error("stars {separation_rad} rad apart are closer than {minimum_rad} rad")]
    StarsTooClose {
        /// Separation found.
        separation_rad: f64,
        /// Smallest usable separation.
        minimum_rad: f64,
    },
    /// A direction is not a unit vector.
    #[error("sighting direction {direction} is not a unit vector")]
    NotUnit {
        /// The offending direction.
        direction: DVec3,
    },
}

/// An orthonormal frame from two directions, splitting their disagreement
/// evenly: a first axis along one alone would load all of it on the second.
fn triad(first: DVec3, second: DVec3) -> Result<DMat3, AlignmentError> {
    let separation_rad = first.cross(second).length().atan2(first.dot(second));
    if separation_rad < MIN_STAR_SEPARATION_RAD {
        return Err(AlignmentError::StarsTooClose {
            separation_rad,
            minimum_rad: MIN_STAR_SEPARATION_RAD,
        });
    }
    let bisector = (first + second).normalize();
    let normal = first.cross(second).normalize();
    Ok(DMat3::from_cols(bisector, normal, bisector.cross(normal)))
}

fn require_unit(direction: DVec3) -> Result<(), AlignmentError> {
    if direction.is_normalized() {
        return Ok(());
    }
    Err(AlignmentError::NotUnit { direction })
}

/// TRIAD attitude determination from two star marks: the stable-member-to-J2000
/// rotation that best carries the marked lines of sight onto the catalog
/// directions.
///
/// The lines of sight are in stable-member axes, so the vehicle may turn
/// between the marks as long as the stable member holds still; a marked
/// separation that disagrees with the catalog shows up in
/// [`ImuRealignment::star_angle_difference_rad`].
pub fn realign_imu_from_two_star_marks(
    first: &StarSighting,
    second: &StarSighting,
) -> Result<ImuRealignment, AlignmentError> {
    for direction in [
        first.line_of_sight_platform,
        second.line_of_sight_platform,
        first.star_direction_j2000,
        second.star_direction_j2000,
    ] {
        require_unit(direction)?;
    }
    let marked = triad(first.line_of_sight_platform, second.line_of_sight_platform)?;
    let catalog = triad(first.star_direction_j2000, second.star_direction_j2000)?;
    let rotation = catalog * marked.transpose();
    let angle = |a: DVec3, b: DVec3| a.cross(b).length().atan2(a.dot(b));
    Ok(ImuRealignment {
        platform_to_inertial: DQuat::from_mat3(&rotation).normalize(),
        star_angle_difference_rad: angle(
            first.line_of_sight_platform,
            second.line_of_sight_platform,
        ) - angle(
            first.star_direction_j2000,
            second.star_direction_j2000,
        ),
    })
}
