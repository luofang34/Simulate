# usaaf-1943-fighter: provenance

The P-38G-13-LO main instrument panel: what each instrument is, where it sits
and how its dial is printed, with the sources each fact rests on and every
choice made where the sources are silent.

## Sources

- **G-13 board, 43-2285.** A surviving P-38G-13-LO instrument panel,
  recovered at Henderson Field in 1963: the original hole pattern and board
  outline, with instruments refitted by a collector. Cited by URL only; its
  photographs are the dealer's copyright and are not part of this pack.
  <https://aeroantique.com/products/p-38g-13-lo-lightening-instrument-panel-43-2285>
- **Lockheed P-38G cockpit photographs** (USAF, public domain):
  <https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_left_wing_061019-F-1234P-004.jpg>,
  <https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_right_wing_061019-F-1234P-003.jpg>.
  They agree with the G-13 board's layout: the engine group on the left,
  the flight group centre and right.
- **TM 1-413, Aircraft Instruments**, War Department, 2 February 1942:
  instrument ranges, luminous paint, fluorescent lighting, panel finish.
  <https://cdn.shopify.com/s/files/1/1343/9895/files/Aircraft_Instruments_1942.pdf>
- **Index of Army Aeronautical Equipment, Vol. 6 Instruments**, AAF Materiel
  Command, 1944: the Army types whose ranges match the G's dials.
  <https://cdn.shopify.com/s/files/1/1343/9895/files/1_Introduction_and_Index.pdf>
- **P-38H/J/L Pilot's Flight Operating Instructions** (01-75FF-1), for
  contrast and the fluorescent instrument lights on the control column.
  <http://www.airwar.ru/other/bibl/p-38pilot.pdf>

## Instruments

| Instrument | Range | Army type by range | Confidence |
|---|---|---|---|
| Airspeed | 50-500 mph, non-linear, angles measured on the G-13 dial | D-7 class | seen on the G-13 dial; type inferred |
| Altimeter | 0-50,000 ft, 28.1-31.0 inHg Kollsman | C-12 class | dial seen; range chosen for the G's ceiling |
| Rate of climb | +/-6,000 ft/min, zero at nine o'clock, re-measured angles | C-2 class | seen |
| Manifold pressure L/R | 10-50 inHg, 3-1/8 in | D-9 class | seen |
| Tachometer L/R | 0-3,500 rpm, 3-1/8 in | electric | seen |
| Engine gage unit L/R | oil 0-100 C, oil 0-200 psi, fuel 0-25 psi | - | seen on the G-13 board |
| Coolant temperature L/R | 60-150 C | - | holes seen; dials refitted |
| Fuel quantity FRONT, REAR | L and R pointers, US gal | - | seen; numerals unreadable |
| Suction | 0-10 inHg | F-4 class | seen |
| Hydraulic pressure (two) | 0-2,000 psi | - | seen; dials refitted |

## Choices where the sources are silent

- The engine gage units' sub-dial layout and arc directions are drawn here;
  only their ranges are sourced. The case is 76 mm in a 3-1/8 in hole.
- Fuel FRONT reads 0-90 gal and REAR 0-60 gal, the capacities of the main
  and reserve tanks in the P-38G flight model.
- The coolant gauges sweep 270 degrees and carry no arcs: no limits for the
  G's V-1710-51/-55 were found. The only marking on the panel is a red line
  at 3,000 rpm on the tachometers, inferred from the engine family.
- Fuel FRONT is placed 5 mm above its measured centre so its case clears
  the airspeed indicator's; positions are measured to about 5 %.
- Faces are printed 0.86 of the case diameter.
- Paint is luminous radium, drawn green-white; the G-13's aged refitted
  dials read orange.
- Lettering: the 1940-43 Kollsman/Pioneer condensed style with the flagged
  one on the airspeed indicator, altimeter and rate of climb; the regular
  MS33558 forms elsewhere. Which G instruments Kollsman made is not
  established. The airspeed indicator prints full numerals, 60 to 500.
- The gyro horizon, directional gyro, turn and bank indicator, magnetic
  compass, synchroscope and clock are Indicate's built-in period gauges
  (pack kind `Direct`), drawn as Indicate draws them and fed from the
  flight state with no modelled gyro or compass error. Their cases are
  measured on the G-13 board; their faces are not the G's own. The SLOW/FAST
  dial is read as a synchroscope and the small top-right dial as a clock;
  neither identification is verified.
- Instruments the pack cannot describe yet, and two whose identity is
  unresolved (the dual X-pointer dial in the right group and the small L/R
  dial below the altimeter), are empty cases on the vehicle's board.
