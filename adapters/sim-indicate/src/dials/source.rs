//! [`DialSource`]: a converted dial in owned storage, lent to Indicate as an
//! [`AnalogDialSpec`] whenever something draws, builds or poses it.

mod pose;

pub use pose::PointerValues;

use indicate_instrument_analog::{
    AnalogDialSpec, Channel, DialChannel, DialInput, DialPose, DialUnit, FailurePose, Legend,
    Lettering, Lighting, MAX_CARDS, MAX_POINTERS, MAX_TAPES, MarkArc, Numeral, OffFlag, Paint,
    PhysicalBuild, PointerBuild, PointerSpec, PointerStyle, Radial, ScaleLaw, ScalePoint,
    ScaleRing, SpecError, TickSet, WindowSpec, physical_build,
};
use indicate_instrument_descriptor::{DesignFrame, PanelDrawError};
use indicate_instrument_scene::{Rgba8, SceneWriter};

/// A channel that reads nothing: the primary of a dial none of whose
/// pointers Indicate's state carries yet. No engine has this index, so a
/// read shows no value rather than another quantity's.
const NO_LANE: DialChannel = DialChannel {
    input: DialInput::EngineRpm { engine: u16::MAX },
    unit: DialUnit::RevolutionsPerMinute,
};

/// How a pointer's angle follows its value.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Law {
    /// Where the scale's angles are measured from: degrees clockwise from
    /// twelve.
    pub(super) reference_deg: f32,
    pub(super) kind: LawKind,
}

/// The shape of a [`Law`].
#[derive(Debug, Clone, PartialEq)]
pub(super) enum LawKind {
    /// The pack's scale points.
    Points(Vec<ScalePoint>),
    /// One turn per this many dial units.
    Turns(f32),
}

impl Law {
    pub(super) fn scale(&self) -> ScaleLaw<'_> {
        match &self.kind {
            LawKind::Points(points) => ScaleLaw::Piecewise {
                reference_deg: self.reference_deg,
                points,
            },
            LawKind::Turns(per_turn) => ScaleLaw::Turns {
                reference_deg: self.reference_deg,
                value_per_turn: *per_turn,
            },
        }
    }
}

/// Where a pointer's value comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lane {
    /// A lane of Indicate's state, read from the resolved panel data.
    State(DialChannel),
    /// A quantity Indicate's state does not carry yet: the caller hands
    /// the mechanism's reading over in dial units.
    Direct,
}

/// One graduated scale.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct RingSource {
    pub(super) center: [f32; 2],
    pub(super) radius: f32,
    pub(super) law: Law,
    pub(super) ticks: Vec<TickSet>,
    pub(super) numerals: Vec<(f32, String)>,
    pub(super) numeral_radius: f32,
    pub(super) numeral_size: f32,
    pub(super) arcs: Vec<MarkArc>,
    pub(super) radials: Vec<Radial>,
    pub(super) paint: Paint,
}

/// One moving pointer.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct PointerSource {
    pub(super) lane: Lane,
    pub(super) law: Law,
    /// Mechanism stops on the pointer's value, dial units.
    pub(super) input_range: Option<[f32; 2]>,
    pub(super) pivot: [f32; 2],
    pub(super) style: PointerStyle,
    pub(super) stack_height_mm: f32,
    pub(super) on_failure: FailurePose,
}

/// A printed legend.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct LegendSource {
    pub(super) text: String,
    pub(super) at: [f32; 2],
    pub(super) size: f32,
    pub(super) paint: Paint,
}

/// Which paint a face is drawn in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum FacePaint {
    /// Every paint as printed.
    Printed,
    /// Luminous paint in white and everything else black, drawn under
    /// Indicate's night lighting: where the face glows.
    GlowMask,
}

impl FacePaint {
    pub(super) fn paint(self, paint: Paint) -> Paint {
        match (self, paint.luminous) {
            (Self::Printed, _) => paint,
            (Self::GlowMask, Some(_)) => Paint::luminous(paint.day, Rgba8::rgb(255, 255, 255)),
            (Self::GlowMask, None) => Paint::luminous(paint.day, Rgba8::rgb(0, 0, 0)),
        }
    }

    fn colour(self, colour: Rgba8) -> Rgba8 {
        match self {
            Self::Printed => colour,
            Self::GlowMask => Rgba8::rgb(0, 0, 0),
        }
    }
}

/// One instrument of a pack as an Indicate dial: one or more pointers, each
/// on its own scale and fed by its own lane, on one printed face.
#[derive(Debug, Clone, PartialEq)]
pub struct DialSource {
    pub(super) case_diameter_mm: f32,
    pub(super) face_radius_mm: f32,
    pub(super) face: Paint,
    pub(super) bezel: Rgba8,
    pub(super) rings: Vec<RingSource>,
    pub(super) legends: Vec<LegendSource>,
    pub(super) pointers: Vec<PointerSource>,
    /// A drum window and the state lane it reads, such as a Kollsman window.
    pub(super) window: Option<(WindowSpec<'static>, DialChannel)>,
    pub(super) off_flag: Option<OffFlag<'static>>,
    /// The face slots its numerals and legends are lettered in.
    pub(super) lettering: Lettering,
    /// Indicate's built-in gauge, lent as it is, for an instrument the
    /// vehicle's state feeds directly; everything above but the case, face
    /// size and lettering then goes unused.
    pub(super) builtin: Option<&'static AnalogDialSpec<'static>>,
}

impl DialSource {
    /// Everything but the borrowed slices, with the first state lane as the
    /// primary channel. A pack dial prints its marks on its rings and has no
    /// cards, tapes or overlays.
    fn frame(&self, mode: FacePaint) -> AnalogDialSpec<'static> {
        let primary = self.pointers.iter().find_map(|p| match p.lane {
            Lane::State(channel) => Some((channel, p.input_range)),
            Lane::Direct => None,
        });
        AnalogDialSpec {
            case_diameter_mm: self.case_diameter_mm,
            face_radius_mm: self.face_radius_mm,
            face: mode.paint(self.face),
            bezel: mode.colour(self.bezel),
            primary: primary.map_or(NO_LANE, |(channel, _)| channel),
            secondary: self.window.map(|(_, channel)| channel),
            extra: &[],
            input_range: primary.and_then(|(_, range)| range),
            rings: &[],
            legends: &[],
            pointers: &[],
            windows: &[],
            off_flag: self.off_flag,
            marks: &[],
            cards: &[],
            tapes: &[],
            overlays: &[],
            lettering: self.lettering,
            arc_legends: &[],
        }
    }

    /// Lends the whole dial, painted as `mode` says, to `f`.
    pub(super) fn with_spec<R>(
        &self,
        mode: FacePaint,
        f: impl FnOnce(&AnalogDialSpec<'_>) -> R,
    ) -> R {
        // A built-in gauge's own paints already glow under night lighting,
        // which is what the glow mask is drawn under.
        if let Some(builtin) = self.builtin_spec() {
            return f(&builtin);
        }
        let numerals: Vec<Vec<Numeral<'_>>> = self
            .rings
            .iter()
            .map(|ring| {
                ring.numerals
                    .iter()
                    .map(|(value, text)| Numeral {
                        value: *value,
                        text,
                    })
                    .collect()
            })
            .collect();
        let rings: Vec<ScaleRing<'_>> = self
            .rings
            .iter()
            .zip(&numerals)
            .map(|(ring, numerals)| ScaleRing {
                center: ring.center,
                radius: ring.radius,
                law: ring.law.scale(),
                ticks: &ring.ticks,
                numerals,
                numeral_radius: ring.numeral_radius,
                numeral_size: ring.numeral_size,
                arcs: &ring.arcs,
                radials: &ring.radials,
                paint: mode.paint(ring.paint),
            })
            .collect();
        let legends: Vec<Legend<'_>> = self
            .legends
            .iter()
            .map(|legend| Legend {
                text: &legend.text,
                at: legend.at,
                size: legend.size,
                paint: mode.paint(legend.paint),
            })
            .collect();
        let pointers: Vec<PointerSpec<'_>> = self
            .pointers
            .iter()
            .map(|p| pointer_spec(p, mode))
            .collect();
        let windows: Vec<WindowSpec<'static>> = self
            .window
            .iter()
            .map(|(window, _)| WindowSpec {
                paint: mode.paint(window.paint),
                drum: mode.colour(window.drum),
                ..*window
            })
            .collect();
        f(&AnalogDialSpec {
            rings: &rings,
            legends: &legends,
            pointers: &pointers,
            windows: &windows,
            ..self.frame(mode)
        })
    }

    /// Draws the whole dial at `pose` into `scene`, centred in `frame`: the
    /// flat instrument a 2D panel shows, from the same description the 3D
    /// gauge is built from.
    pub fn draw_dial(
        &self,
        pose: &DialPose,
        lighting: Lighting,
        frame: DesignFrame,
        scene: &mut SceneWriter<'_>,
    ) -> Result<(), PanelDrawError> {
        self.with_spec(FacePaint::Printed, |spec| {
            indicate_instrument_analog::draw_dial(spec, pose, lighting, frame, scene)
        })
    }

    /// Checks the dial as Indicate would before drawing it.
    pub fn check(&self) -> Result<(), SpecError> {
        self.with_spec(FacePaint::Printed, |spec| spec.check())
    }

    /// Where each pointer's value comes from, in pointer order; a built-in
    /// gauge reads every value from the state.
    pub fn lanes(&self) -> impl Iterator<Item = Lane> + '_ {
        let builtin = self.builtin_spec();
        let from_state = builtin.into_iter().flat_map(move |spec| {
            spec.pointers
                .iter()
                .filter_map(move |p| spec.channel(p.channel))
                .map(Lane::State)
        });
        self.pointers.iter().map(|p| p.lane).chain(from_state)
    }

    /// How far the dial's tape drum stands turned, in turns, for a tape
    /// showing `value`: a drum whose values rise to the right turns one way,
    /// one whose values rise to the left the other. `None` without a tape.
    #[must_use]
    pub fn tape_turns(&self, value: f32) -> Option<f32> {
        let tape = self.builtin.and_then(|b| b.tapes.first())?;
        let turns = value / tape.period;
        Some(match tape.sense {
            indicate_instrument_analog::TapeSense::RightIncreasing => turns,
            indicate_instrument_analog::TapeSense::LeftIncreasing => -turns,
        })
    }

    /// Whether the dial is Indicate's built-in gauge, fed from the state.
    #[must_use]
    pub const fn is_builtin(&self) -> bool {
        self.builtin.is_some()
    }

    /// The built-in gauge in this case's size and lettering, if the dial is one.
    pub(super) fn builtin_spec(&self) -> Option<AnalogDialSpec<'static>> {
        self.builtin.map(|builtin| AnalogDialSpec {
            case_diameter_mm: self.case_diameter_mm,
            face_radius_mm: self.face_radius_mm,
            lettering: self.lettering,
            ..*builtin
        })
    }

    /// What a 3D backend builds the gauge from, in millimetres. Its laws
    /// borrow nothing beyond this source.
    pub fn physical_build(&self) -> Result<PhysicalBuild<'_>, SpecError> {
        if let Some(builtin) = self.builtin_spec() {
            return physical_build(&builtin);
        }
        let built = self.with_spec(FacePaint::Printed, |spec| {
            physical_build(spec).map(|build| {
                // Everything but the laws is plain data; the laws are re-lent
                // from `self` below, since the spec's borrows end here.
                let pointers = build.pointers.map(|slot| {
                    slot.map(|p| {
                        (
                            p.outline_mm,
                            p.pivot_mm,
                            p.hub_radius_mm,
                            p.stack_height_mm,
                            p.paint,
                            p.channel,
                        )
                    })
                });
                (build.face_layers, pointers, build.windows)
            })
        })?;
        let (face_layers, pointers, windows) = built;
        let mut out = [None; MAX_POINTERS];
        for ((slot, built), source) in out.iter_mut().zip(pointers).zip(&self.pointers) {
            *slot = built.map(
                |(outline_mm, pivot_mm, hub_radius_mm, stack_height_mm, paint, channel)| {
                    PointerBuild {
                        outline_mm,
                        pivot_mm,
                        hub_radius_mm,
                        stack_height_mm,
                        paint,
                        channel,
                        law: source.law.scale(),
                    }
                },
            );
        }
        Ok(PhysicalBuild {
            lettering: self.lettering,
            case_diameter_mm: self.case_diameter_mm,
            face_radius_mm: self.face_radius_mm,
            face_layers,
            pointers: out,
            windows,
            cards: [None; MAX_CARDS],
            tapes: [None; MAX_TAPES],
            overlays: &[],
        })
    }

    /// Outside diameter of the case, millimetres.
    #[must_use]
    pub const fn case_diameter_mm(&self) -> f32 {
        self.case_diameter_mm
    }

    /// Radius of the visible face, millimetres.
    #[must_use]
    pub const fn face_radius_mm(&self) -> f32 {
        self.face_radius_mm
    }

    /// After how many wheels from the right the drum window prints its
    /// separator, such as a decimal point, if it has a drum and one.
    #[must_use]
    pub fn drum_separator(&self) -> Option<u8> {
        self.window
            .and_then(|(window, _)| window.separator)
            .map(|(after, _)| after)
    }

    /// Where the failure flag sits and how large its text is, face units.
    #[must_use]
    pub fn off_flag(&self) -> Option<([f32; 2], f32)> {
        self.off_flag.map(|flag| (flag.at, flag.size))
    }

    /// The face slot its numerals, drum digits included, are lettered in.
    #[must_use]
    pub const fn numerals_slot(&self) -> indicate_instrument_scene::FaceSlot {
        self.lettering.numerals
    }

    /// The glow of the dial's luminous paint, if any of it is.
    #[must_use]
    pub fn glow(&self) -> Option<Rgba8> {
        if let Some(builtin) = self.builtin {
            let shapes = builtin
                .cards
                .iter()
                .flat_map(|c| c.shapes)
                .chain(builtin.overlays.iter().flat_map(|o| o.shapes))
                .chain(builtin.marks);
            return builtin
                .rings
                .iter()
                .map(|r| r.paint)
                .chain(builtin.pointers.iter().map(|p| p.style.paint))
                .chain(builtin.tapes.iter().map(|t| t.paint))
                .chain(shapes.map(|m| m.paint))
                .find_map(|paint| paint.luminous);
        }
        self.rings
            .iter()
            .find_map(|r| r.paint.luminous)
            .or_else(|| self.pointers.iter().find_map(|p| p.style.paint.luminous))
    }
}

/// `pointer` as Indicate takes it, on the primary channel: drawing and
/// building read no channel, and posing gives each pointer its own reading.
fn pointer_spec(pointer: &PointerSource, mode: FacePaint) -> PointerSpec<'_> {
    PointerSpec {
        channel: Channel::Primary,
        law: pointer.law.scale(),
        pivot: pointer.pivot,
        style: PointerStyle {
            paint: mode.paint(pointer.style.paint),
            ..pointer.style
        },
        stack_height_mm: pointer.stack_height_mm,
        on_failure: pointer.on_failure,
    }
}
