//! The markings of P-38G-13-LO 43-2264, White 147 of the 339th FS, as on
//! 18 April 1943.

use super::markings::{Art, Colour, Marking, Place};

const PORT: f32 = -1.0;
const STARBOARD: f32 = 1.0;

/// Insignia of the period: the white star in a blue disc, no bars and no
/// surround (red centre deleted 15 May 1942, bars from 28 June 1943), on
/// the upper port and lower starboard wing only, the Air Corps rule since
/// 26 February 1941 (as summarised by Wikipedia, "United States military
/// aircraft national insignia"). The form is Spec 24102-K Amendment 3,
/// in force 28 May 1942 to 29 June 1943, its star's points touching the
/// circle (secondary sources quoting the spec and T.O. 07-1-1 of 10 July
/// 1942; the documents themselves were not reached). No size rule in force
/// before July 1943 was found: the 45 in wing diameter at WS 205 is
/// carried from the earlier model and is unsourced.
const WING_STAR: Art = Art::Insignia { diameter_in: 45.0 };
/// The boom insignia measured on 41-7631 (USAAF, RAF Goxhill 1942) against
/// its 36 in main tyre: about 25 in, centred about 24 in aft of the wing's
/// trailing edge.
const BOOM_STAR: Art = Art::Insignia { diameter_in: 25.0 };

/// White nose number, measured on 43-2242's "129" (USAAF 342-FH-3A43614):
/// about 11 in tall against the nose's section, its strokes about a
/// seventh of the height, starting just aft of the nose-cap break, high on
/// the olive side.
const NOSE_NUMBER: Art = Art::Numerals {
    text: "147",
    height_in: 11.0,
    stroke: 0.15,
    colour: Colour::White,
};

/// The radio call number, the serial less its first digit, on both sides
/// of the vertical tail in Identification Yellow No. 48 as block numerals
/// with a stroke of about a sixth of their height, sized only to read from
/// about 150 yd (T.O. 07-1-1 as quoted by secondary sources). Which fin
/// faces 147 carried it is not documented; the outboard faces are assumed.
/// Its size and place follow 41-7631: on 41-7631 about 9 in tall against the 82 in fin, run across
/// fin and rudder at mid-chord, its centre about 18 in above the boom's
/// centreline.
const CALL_NUMBER: Art = Art::Numerals {
    text: "32264",
    height_in: 9.0,
    stroke: 0.167,
    colour: Colour::Yellow,
};

/// Petit's two flags and the ship (see `victories`): placement and size
/// are assumptions, a row 3 in tall centred over the nose number.
const VICTORIES: Art = Art::Victories {
    flags: 2,
    ships: 1,
    height_in: 3.0,
};

/// Every marking, in painting order.
pub(crate) const MARKINGS: &[Marking] = &[
    Marking {
        art: WING_STAR,
        place: Place::Wing {
            side: PORT,
            upper: true,
            ws_in: 205.0,
            chord: 0.45,
        },
    },
    Marking {
        art: WING_STAR,
        place: Place::Wing {
            side: STARBOARD,
            upper: false,
            ws_in: 205.0,
            chord: 0.45,
        },
    },
    Marking {
        art: BOOM_STAR,
        place: Place::Boom {
            side: PORT,
            outboard: true,
            aft_of_te_in: 24.0,
            wl_in: 4.0,
        },
    },
    Marking {
        art: BOOM_STAR,
        place: Place::Boom {
            side: STARBOARD,
            outboard: true,
            aft_of_te_in: 24.0,
            wl_in: 4.0,
        },
    },
    Marking {
        art: NOSE_NUMBER,
        place: Place::Nose {
            side: PORT,
            fs_in: 48.0,
            wl_in: 9.5,
        },
    },
    Marking {
        art: NOSE_NUMBER,
        place: Place::Nose {
            side: STARBOARD,
            fs_in: 48.0,
            wl_in: 9.5,
        },
    },
    Marking {
        art: VICTORIES,
        place: Place::Nose {
            side: PORT,
            fs_in: 48.0,
            wl_in: 18.5,
        },
    },
    Marking {
        art: VICTORIES,
        place: Place::Nose {
            side: STARBOARD,
            fs_in: 48.0,
            wl_in: 18.5,
        },
    },
    Marking {
        art: CALL_NUMBER,
        place: Place::Fin {
            side: PORT,
            outboard: true,
            fs_in: 410.0,
            wl_in: 27.0,
        },
    },
    Marking {
        art: CALL_NUMBER,
        place: Place::Fin {
            side: STARBOARD,
            outboard: true,
            fs_in: 410.0,
            wl_in: 27.0,
        },
    },
];
