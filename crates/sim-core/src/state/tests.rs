use super::*;

/// A vehicle at rest on the Earth turns with it: its gyroscopes sense the
/// Earth's spin, `Ω` in all, `Ω·sin φ` of it about the local vertical.
#[test]
fn a_vehicle_at_rest_senses_the_earths_spin() {
    let wgs84 = BodyEllipsoid::WGS84;
    for (lat_deg, heading_rad) in [(41.7, 1.2), (-33.9, 0.0), (0.0, 2.5), (78.2, -0.7)] {
        let position = Geodetic::from_degrees(lat_deg, 10.0, 300.0);
        let rest = VehicleState::level_flight(&position, heading_rad, 0.0);
        let sensed = rest.inertial_rate_body(&wgs84);
        assert!(
            (sensed.length() - wgs84.omega_rad_s).abs() < 1e-15,
            "{sensed}"
        );
        let up = LocalFrame::at(&position).up();
        let vertical = (rest.attitude * sensed).dot(up);
        let expected = wgs84.omega_rad_s * lat_deg.to_radians().sin();
        assert!(
            (vertical - expected).abs() < 1e-12,
            "{lat_deg}: {vertical} {expected}"
        );
        let turning = VehicleState {
            angular_rate_body: DVec3::new(0.1, -0.2, 0.3),
            ..rest
        };
        let added = turning.inertial_rate_body(&wgs84) - turning.angular_rate_body;
        assert!((added - sensed).length() < 1e-15);
    }
}
