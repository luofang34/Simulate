//! The markings of P-38G-13-LO 43-2264, White 147 of the 339th FS, as on
//! 18 April 1943.

use super::markings::{Art, Colour, Marking, Place};

const PORT: f32 = -1.0;
const STARBOARD: f32 = 1.0;

/// Insignia of the period: the white star in a blue disc, no bars and no
/// surround (bars and red surround from 29 June 1943; red centre deleted
/// May 1942), on the upper port and lower starboard wing and both booms.
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

/// The radio call number, the serial less its first digit, in yellow on
/// the fins: on 41-7631 about 9 in tall against the 82 in fin, run across
/// fin and rudder at mid-chord, its centre about 18 in above the boom's
/// centreline.
const CALL_NUMBER: Art = Art::Numerals {
    text: "32264",
    height_in: 9.0,
    stroke: 0.16,
    colour: Colour::Yellow,
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
            wl_in: 13.0,
        },
    },
    Marking {
        art: NOSE_NUMBER,
        place: Place::Nose {
            side: STARBOARD,
            fs_in: 48.0,
            wl_in: 13.0,
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
