# Phase 8: direct planar racing-line optimisation

Phase 8 is an alternative to, not a replacement for, the historical Phase 5
optimiser. Phase 5 optimises a latent periodic field and constructs its path
through Phase 4 offset geometry. Phase 8 instead makes every optimisation
variable a physical lateral offset in metres at a planar interpolation guide.
Those guides define the final Cartesian motorcycle trajectory directly.

## Geometry-aware path-model resolution

Uniform sparse guides cannot represent both a tight, high-angle corner and a
straight hundreds of metres long efficiently. Controls are generated within
every analytic track primitive. A straight of length `L` uses
`max(1, ceil(L / max_spacing_m))` subdivisions. A circular arc also takes the
maximum with `ceil(abs(turn_angle_rad) / max_arc_heading_change_rad)`. The start
of each subdivision is a station; every primitive boundary is retained, while
the duplicate closed endpoint is omitted. There are no hidden guides.

At station `s_i`, the guide is `C(s_i) + offset_i N(s_i)`. Positive offsets are
left of travel. Its local bounds are `-(width_right_i - margin)` and
`width_left_i - margin`; widths and calculations use SI units.

## Smooth Cartesian geometry and validity

Guides are interpolated by a non-uniform, C2-periodic Cartesian cubic spline
parameterised by analytic centreline station. The cyclic knot system uses each
actual interval and is solved for x and y with NumPy. First and second
derivatives, curvature and Gauss--Legendre geometric arc length derive from the
same spline. SciPy is not required.

Dense validation at common analytic centreline stations rejects rather than
clips between-guide corridor overshoot. It also requires a finite, strictly
positive dot product between spline derivative and track tangent and rejects a
zero or near-zero tangent. This prevents local reversal and loops without a
curvature-rate constraint. Phase 6 remains disabled by default and motorcycle
physics is unchanged.

All-zero controls put every guide on the analytic centreline, but the C2 spline
between them is not the track's piecewise-straight/circular-arc geometry. It is
the **zero-control planar baseline**, never the exact centreline. Diagnostics
evaluate the analytic fixed centreline separately.

## Two distinct sensitivity studies

Changing the control policy changes path-model order and refits different
geometry. Sampling a saved spline at 1.0, 0.5, or 0.25 m changes only the
fixed-path solver grid: guides, controls, spline, and integrated length remain
identical. Diagnostics separate control-policy and fixed-spline output studies.

The deterministic bounded coordinate search starts at zero, tries `+` then
`-`, uses local asymmetric physical bounds, and follows Phase 5 step reduction.
It is local, so no global optimum is claimed. A finer model is not automatically
more physically accurate. Material policy dependence is reported as
unconverged path-model sensitivity rather than tuned away. The model adds no
clothoids, centreline smoothing, roll dynamics, banking, elevation, kerbs,
calibration changes, or curvature-rate limit.

## Parallel poll evaluation

`PlanarOptimisationConfig.parallel_workers` optionally evaluates the independent
candidates in a complete best-improvement poll with a persistent process pool.
It defaults to `1`, which retains the original serial execution path and creates
no worker processes. Values greater than one use multiprocessing's `spawn`
context so the same module-level, picklable worker design is exercised on every
platform, including Windows. Immutable track, motorcycle, station, and sampling
context is installed once per worker; individual tasks contain only a control
vector.

Parallelism changes wall-clock scheduling only. Results are collected in the
original direction/sign order before the existing lap-time and order tie-break
is applied. Polls remain all-or-nothing with respect to the evaluation budget,
and the single pattern move retains its existing evaluation count and semantics.

## Optional fixed-path speed backend

`PlanarOptimisationConfig.speed_backend` explicitly selects `"python"` or
`"numba"`. Python remains the default and authoritative reference, so existing
callers and installations are unchanged. Install the optional backend with:

```text
python -m pip install -e ".[test,accelerated]"
```

Numba is imported lazily only when selected. If it is unavailable, optimisation
fails immediately with an installation hint rather than classifying candidates
as infeasible. `--speed-backend numba --workers 1` runs serially without a
process pool. Larger worker counts retain the persistent spawned process pool;
each worker resolves the cached Numba backend once during initialization.
Windows and other platforms therefore use process-level parallelism, not Numba
threads (`parallel=True` is not used). The final accepted Numba path is solved
once by the Python reference backend, and lap time and every speed sample must
agree within `1e-9` before the Python result becomes the reported profile.

The focused Mallala/reference, one-sweep throughput comparison is:

```text
python scripts/r6_phase8_planar_optimisation_check.py --track mallala --policy reference --max-sweeps 1 --max-evaluations 400 --workers 1 --speed-backend python
python scripts/r6_phase8_planar_optimisation_check.py --track mallala --policy reference --max-sweeps 1 --max-evaluations 400 --workers 1 --speed-backend numba
python scripts/r6_phase8_planar_optimisation_check.py --track mallala --policy reference --max-sweeps 1 --max-evaluations 400 --workers 8 --speed-backend numba
python scripts/r6_phase8_planar_optimisation_check.py --track mallala --policy reference --max-sweeps 1 --max-evaluations 400 --workers 16 --speed-backend numba
```

The diagnostic reports elapsed time, seconds per evaluation, and evaluations
per wall-clock second. These measurements are diagnostics, never test gates.

## Restarting from exported controls

The Phase 8 diagnostic can start one track and one control policy from the best
physical controls in an earlier `phase8_*_controls.csv` export. For example
(shown as a Windows-compatible single line):

```text
python scripts/r6_phase8_planar_optimisation_check.py --track mallala --policy reference --max-sweeps 100 --max-evaluations 12000 --workers 16 --initial-step-m 0.25 --initial-controls-csv phase8_mallala_controls.csv
```

This is a strict same-track, same-policy restart. The saved station layout and
bounds must match those generated with the current boundary margin; controls
are never reordered, interpolated, projected, or clipped. Controls and the
search step are separate pieces of restart state: the CSV restores only the
physical control vector, while `--initial-step-m` explicitly restores a known
search step (and defaults to `1.0` m when omitted). Evaluation count, sweep
count, and process-pool state begin fresh from the current
`PlanarOptimisationConfig`. The restart CSV format is unchanged.

## Warm-start checkpoints

Long targeted runs may pass `--checkpoint-controls-csv PATH`. After every
complete poll (and its optional pattern move), the parent process atomically
replaces this file in the strict controls format accepted by
`--initial-controls-csv`. For example:

```text
python scripts/r6_phase8_planar_optimisation_check.py --track mallala --policy reference --checkpoint-controls-csv mallala-checkpoint.csv
```

This is a **warm-start checkpoint**, not exact optimiser resume. Restarting
restores accepted controls, but does not restore the evaluation count or sweep
count and does not automatically restore the search step. The diagnostic logs
those values; supply its recorded step manually with `--initial-step-m` when
desired. It is therefore not claimed to be an exact uninterrupted continuation.
