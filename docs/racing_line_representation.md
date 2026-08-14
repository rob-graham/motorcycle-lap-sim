# Racing-line representation (Phase 4)

Phase 4 represents and validates a **supplied** dense racing line. It does not
select, search, or claim to optimise that line. Each centreline station has one
lateral offset `n`. Positive `n` is left of travel and `P = C + n N`, where
`N` is the sampled centreline's left unit normal. Thus positive offset on the
counter-clockwise test oval moves inside; negative offset moves right.

## Boundary constraint

The reference point must satisfy `-width_right <= n <= width_left` at every
sample. Optional non-negative margin `m` changes the limits to
`-(width_right-m)` and `width_left-m`. Impossible margins and out-of-range
offsets are errors; values are never clipped. This is a point reference model:
motorcycle width and rider clearance are deliberately not modelled.

When constructing a path from raw offsets, the default margin is zero. When an
existing `LateralOffsetProfile` is supplied, its recorded margin is preserved
unless an explicit margin override is requested. In both cases the offsets are
revalidated against the widths of the sampled track passed to the path builder;
a profile validated on one track cannot bypass another track's boundaries.

## Actual path geometry

Centreline `s` labels where an offset is specified. Racing-line `q` is the
cumulative Euclidean chord length of displaced samples, beginning at zero. The
full length includes the last-to-first chord; closed arrays omit the duplicate
endpoint. Thus `q` and generally total length are not copied from centreline
`s`.

Signed curvature uses the circumcircle through each periodic
previous/current/next triple: twice the signed cross product divided by the
product of the triangle's three side lengths. Counter-clockwise (left) triples
are positive, clockwise triples negative, and collinear triples yield zero.
Coincident neighbours are rejected. First and last samples use wrapped
neighbours.

Chordal length and local three-point curvature are resolution-dependent.
Chord length converges from below for a sampled circle; sharp offset changes or
primitive joins can produce locally large curvature. Perform spacing convergence
checks. No smoothing, hidden clipping, or optimisation is performed.

In particular, the Phase 3 centreline path uses the sampled track's analytic
primitive curvature and centreline `s`, whereas generated Phase 4 paths use
chordal `q` and geometric periodic three-point curvature. Finite-resolution lap
time differences are therefore expected, especially near primitive curvature
jumps. Check that the difference converges under spacing refinement before
interpreting a small lap-time difference.

```bash
python -m motorcycle_lap_sim.racing_line.diagnostics \
  examples/tracks/test_oval.yaml --spacing 1.0 --constant-offset 2.0 \
  --csv racing-line.csv --output-png racing-line.png
```

The generic output `SampledPath` passes directly to the fixed-path solver,
which has no dependency on `Track`.

## Alternative smooth planar representation (Phase 7)

The original `C(s)+n(s)N(s)` sampled representation and periodic three-point curvature remain available. Phase 7 separately interpolates exact offset guide points with a C2-periodic Cartesian cubic and uses analytic derivatives for curvature plus quadrature for actual arc length. Guide density and output sampling resolution are deliberately independent. This alternative is experimental and is not the optimiser default. See `smooth_planar_racing_line.md`.
