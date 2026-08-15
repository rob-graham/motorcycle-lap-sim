# Mallala reference track

## Purpose and provenance

`examples/tracks/mallala_reference.yaml` is the project's first real-world
validation circuit. It is **QGIS-DERIVED / APPROXIMATE REFERENCE GEOMETRY**, not
surveyed or survey-grade geometry. Georeferenced observations were digitised in
QGIS using GDA2020 / MGA Zone 54 (EPSG:7854), and a tangent-connected sequence
of straights and circular arcs was fitted and visually reviewed against aerial
imagery. The accepted reference is centreline fit v0.3.

The simulator file deliberately discards the absolute EPSG:7854 coordinates.
It starts at `(0 m, 0 m)` with heading `0 rad` in the simulator's local frame.
Its 23 primitives close to numerical roundoff and have a fitted centreline
length of approximately 2557.19 m. The nominal real-circuit description of
approximately 2.60--2.61 km is recorded only as context: the fitted geometry is
not scaled, stretched, or otherwise corrected to match it.

T3 uses three consecutive right-hand arcs to represent a corner that tightens
and then opens. T7 uses two consecutive right-hand arcs, first tight and then
larger-radius. Abrupt straight/arc and unequal-radius arc/arc curvature changes
are intentional. No clothoids or centreline smoothing are applied.

## Width model

The usable reference width is modelled symmetrically as 8 m generally (4 m on
each side of the centreline) and 10 m on the physical start/finish straight (5
m each side). Since the local start lies partway along that straight, its first
`S0a` and final `S0b` primitives both override the defaults. The change is a
step at primitive boundaries; no unmeasured taper is invented.

A `Track` stores its global half-widths as defaults and resolves immutable
`primitive_width_left_m` and `primitive_width_right_m` tuples, one value per
primitive. A primitive YAML entry may override either side independently:

```yaml
width_left_m: 4.0
width_right_m: 4.0
primitives:
  - type: straight
    length_m: 100.0
    width_left_m: 5.0
    width_right_m: 5.0
```

An omitted side inherits its global default. Every default and resolved value
must be numeric, finite, and greater than zero. At an exact primitive join,
sampling assigns the station to the next primitive, preserving the established
`searchsorted(..., side="right")` convention. There is no width interpolation.
The resulting per-sample arrays flow unchanged into boundary and racing-line
validation.

## Reproducible diagnostic and limitations

Run `python scripts/r6_mallala_reference_check.py` from the repository root to
solve the unoptimised centreline with the reference R6 at 2.0, 1.0, and 0.5 m
sampling and to write `mallala_reference.png`. The plot has equal axis scaling
and shows centreline, both boundaries, and start/finish. The default R6
curvature-transient proxy remains disabled. This is fixed-path numerical
validation, not Phase 8 racing-line optimisation.

The reference does not model elevation, banking, kerbs, runoff, white-line or
motorcycle width, width tapers, clothoids, or roll dynamics. It is not intended
for real lap-record calibration. GPS/KML/QGIS importing and arbitrary polyline
tracks are also outside this phase.
