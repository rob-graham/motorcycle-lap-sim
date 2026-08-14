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

```bash
python -m motorcycle_lap_sim.racing_line.diagnostics \
  examples/tracks/test_oval.yaml --spacing 1.0 --constant-offset 2.0 \
  --csv racing-line.csv --output-png racing-line.png
```

The generic output `SampledPath` passes directly to the fixed-path solver,
which has no dependency on `Track`.
