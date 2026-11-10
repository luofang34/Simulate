# us-ms33558: provenance and licence

`us-ms33558.idtf` is a stroke typeface in Indicate's typeface format,
version 2. `cargo xtask gen-typeface` builds it from
`tools/xtask/src/typeface/us_ms33558.rs`, and a test fails when the
committed bytes and the source disagree.

## Authorship

The glyph geometry is original. It was drawn for this repository as one
centreline per character, plus a round pen. No font file, font outline, or
font data from any other source was copied, traced, or converted.

## Sources

The letterforms follow two US Government documents:

- **MS33558 (ASG), revision C**, *Numerals and Letters, Aircraft
  Instrument Dial, Standard Form of*. Approved 4 February 1955, revised
  26 January 1968. Notice 1 (11 April 1986) marks it Distribution
  Statement A: approved for public release, distribution unlimited.
  Source: https://everyspec.com/MS-Specs/MS3/MS33000-MS33999/MS33558C_13545/
- **MIL-M-18012B**, *Markings for Aircrew Station Displays*, 20 July 1964.
  It gives the dimensions that MS33558 omits. Source:
  https://everyspec.com/MIL-SPECS/MIL-SPECS-MIL-M/MIL-M-18012B_6496/

From MS33558 the face takes these forms: a plain bar "1", a closed "4", an
uncrossed "7", a round-topped "3", an "O" wider than "0", an "M" with
vertical sides, and a "G" with a spur.

From MIL-M-18012B it takes these proportions:

- A character is 3/5 of its height wide, measured over the outside of its
  strokes.
- "4" is one stroke wider.
- "1" and "I" are one stroke wide.
- "M" and "W" are a fifth wider.
- The stroke is 1/8 to 1/6 of the height. This face uses 1/7.

A test checks each of these proportions.

The standards define capitals and digits only. The face adds
`. , - + / = ( ) × °` and the space, drawn in the same construction. It
also has one alternate, `one.flag`: the flagged "1" seen on some 1940
dials.

## Licence

A work of the US Government is not protected by copyright in the United
States (17 U.S.C. §105). Distribution Statement A releases MS33558 for
public use. Typeface designs as such are not registrable for copyright in
the United States (37 CFR 202.1(e)). This is the project's reading of the
sources, not legal advice.

The geometry in this directory is the project's own work. It is
distributed under the repository's terms.
