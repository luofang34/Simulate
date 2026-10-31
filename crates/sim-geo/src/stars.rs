use glam::DVec3;

#[cfg(test)]
mod tests;

/// One star the Apollo guidance computers could be told to sight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavStar {
    /// The code entered on the DSKY, displayed there in octal: `0o01..=0o45`.
    pub apollo_octal_id: u8,
    /// Name on the Apollo star charts, including the crew's Navi, Regor, and
    /// Dnoces.
    pub name: &'static str,
    /// Unit vector toward the star at the J2000 epoch, without proper motion.
    pub direction_j2000: DVec3,
    /// Apparent visual magnitude; the primary's for a multiple star.
    pub visual_magnitude: f32,
}

const fn star(
    apollo_octal_id: u8,
    name: &'static str,
    [x, y, z]: [f64; 3],
    visual_magnitude: f32,
) -> NavStar {
    NavStar {
        apollo_octal_id,
        name,
        direction_j2000: DVec3::new(x, y, z),
        visual_magnitude,
    }
}

/// The 37 Apollo navigation stars, ordered by code. Directions and magnitudes
/// are SIMBAD's ICRS J2000 entries; the names and codes match the star table
/// flown in the AGC.
// Dnoces's magnitude is 3.14, which the lint mistakes for an approximate pi.
#[allow(clippy::approx_constant)]
pub static APOLLO_NAV_STARS: [NavStar; 37] = [
    star(
        0o01,
        "Alpheratz",
        [0.873268269792276, 0.031974229029160, 0.486189446257315],
        2.06,
    ),
    star(
        0o02,
        "Diphda",
        [0.933977367981934, 0.179811379049112, -0.308794663266703],
        2.01,
    ),
    star(
        0o03,
        "Navi",
        [0.474230198146198, 0.119798026016588, 0.872212217369572],
        2.39,
    ),
    star(
        0o04,
        "Achernar",
        [0.492722332369658, 0.223804587928003, -0.840913913314901],
        0.46,
    ),
    star(
        0o05,
        "Polaris",
        [0.010126980049571, 0.007899138194321, 0.999917520544001],
        2.02,
    ),
    star(
        0o06,
        "Acamar",
        [0.543326144349385, 0.535144033848836, -0.646852041739397],
        3.18,
    ),
    star(
        0o07,
        "Menkar",
        [0.698256141421668, 0.712286451089838, 0.071318809305597],
        2.53,
    ),
    star(
        0o10,
        "Mirfak",
        [0.404980100349651, 0.501551701183512, 0.764484799954003],
        1.79,
    ),
    star(
        0o11,
        "Aldebaran",
        [0.343903615246871, 0.894973261901489, 0.284171011719247],
        0.86,
    ),
    star(
        0o12,
        "Rigel",
        [0.195052031455532, 0.970362621908236, -0.142657236158714],
        0.13,
    ),
    star(
        0o13,
        "Capella",
        [0.130500275993577, 0.682315882686928, 0.719315448324835],
        0.08,
    ),
    star(
        0o14,
        "Canopus",
        [-0.063222652197488, 0.602741937611867, -0.795427591231916],
        -0.74,
    ),
    star(
        0o15,
        "Sirius",
        [-0.187455228821880, 0.939217528795477, -0.287629919846065],
        -1.46,
    ),
    star(
        0o16,
        "Procyon",
        [-0.418111427133012, 0.903819481817536, 0.091066891831649],
        0.37,
    ),
    star(
        0o17,
        "Regor",
        [-0.362956060079649, 0.572299724014244, -0.735347485441188],
        1.83,
    ),
    star(
        0o20,
        "Dnoces",
        [-0.471125577060395, 0.474394888326306, 0.743633095396770],
        3.14,
    ),
    star(
        0o21,
        "Alphard",
        [-0.777932669839674, 0.610046314965379, -0.150546520362571],
        1.97,
    ),
    star(
        0o22,
        "Regulus",
        [-0.864502016652606, 0.457866218624312, 0.207351848427381],
        1.4,
    ),
    star(
        0o23,
        "Denebola",
        [-0.966729466607082, 0.046183202366180, 0.251597397070779],
        2.13,
    ),
    star(
        0o24,
        "Gienah",
        [-0.951229871056582, -0.065708132528107, -0.301403672388622],
        2.58,
    ),
    star(
        0o25,
        "Acrux",
        [-0.449405206509022, -0.052391981940156, -0.891790356861390],
        1.28,
    ),
    star(
        0o26,
        "Spica",
        [-0.914080176493796, -0.356352757528708, -0.193572061885894],
        0.97,
    ),
    star(
        0o27,
        "Alkaid",
        [-0.581459568692221, -0.294801902315980, 0.758285308025374],
        1.86,
    ),
    star(
        0o30,
        "Menkent",
        [-0.685294159492604, -0.422760920294641, -0.592996727846751],
        2.05,
    ),
    star(
        0o31,
        "Arcturus",
        [-0.783787057871449, -0.526986912810060, 0.328576690652487],
        -0.05,
    ),
    star(
        0o32,
        "Alphecca",
        [-0.529171790163498, -0.719641474219205, 0.449547956372586],
        2.24,
    ),
    star(
        0o33,
        "Antares",
        [-0.344816020327007, -0.826411748758821, -0.445135410453080],
        0.91,
    ),
    star(
        0o34,
        "Atria",
        [-0.109614194130243, -0.340718147306459, -0.933753646601303],
        1.88,
    ),
    star(
        0o35,
        "Rasalhague",
        [-0.106538880652570, -0.970236839654393, 0.217462506852911],
        2.07,
    ),
    star(
        0o36,
        "Vega",
        [0.125096463599930, -0.769413127916007, 0.626381922939430],
        0.03,
    ),
    star(
        0o37,
        "Nunki",
        [0.214096644721417, -0.870572205642777, -0.443019933503338],
        2.067,
    ),
    star(
        0o40,
        "Altair",
        [0.459221265861156, -0.874842416102532, 0.154164120238018],
        0.76,
    ),
    star(
        0o41,
        "Dabih",
        [0.558083556627468, -0.789588753991837, -0.255132011695228],
        3.08,
    ),
    star(
        0o42,
        "Peacock",
        [0.325588414399733, -0.441425335216574, -0.836143443338285],
        1.918,
    ),
    star(
        0o43,
        "Deneb",
        [0.455649054133710, -0.536182042184163, 0.710558060334464],
        1.25,
    ),
    star(
        0o44,
        "Enif",
        [0.817201419541613, -0.550245214874294, 0.171499397687320],
        2.39,
    ),
    star(
        0o45,
        "Fomalhaut",
        [0.837332030512427, -0.233587361743581, -0.494279288572371],
        1.16,
    ),
];

/// The star with Apollo code `code`, or `None` outside `0o01..=0o45`.
pub fn nav_star_by_octal_id(code: u8) -> Option<&'static NavStar> {
    APOLLO_NAV_STARS
        .iter()
        .find(|star| star.apollo_octal_id == code)
}

/// The navigation stars within `half_angle_rad` of `boresight_inertial`, a
/// J2000 direction of any length. A zero or non-finite boresight sees nothing.
pub fn stars_in_cone(
    boresight_inertial: DVec3,
    half_angle_rad: f64,
) -> impl Iterator<Item = &'static NavStar> {
    let boresight = boresight_inertial.try_normalize();
    APOLLO_NAV_STARS.iter().filter(move |star| {
        boresight.is_some_and(|axis| {
            // atan2 stays accurate for the narrow fields of a sextant, where
            // the cosine of the separation rounds to one.
            let separation = axis
                .cross(star.direction_j2000)
                .length()
                .atan2(axis.dot(star.direction_j2000));
            separation <= half_angle_rad
        })
    })
}
