# Phase 5 racing-line optimisation

Phase 5 composes the existing validated layers without changing their physics:

`SampledTrack → periodic controls → dense offsets → build_racing_line_path → SampledPath → solve_speed_profile → lap time`.

## Parameterisation and boundaries

A configurable, low-dimensional uniform periodic cubic B-spline supplies a C2
latent function. Wrapped control indices make periodicity intrinsic rather than
joining two independently calculated ends. Zero controls produce exactly zero
offset. At each station a shifted logistic maps the latent value strictly
inside the usable interval `[-(width_right-margin), width_left-margin]`; its
shift is selected so latent zero maps to offset zero even when widths differ.
Phase 4 validates the resulting dense profile independently. The explicit
margin concerns the simulated reference point only: it is not motorcycle width
or a validated safety clearance.

## Objective and deterministic search

The reported scalar objective is only solved lap time. There are no hidden
length, curvature, or smoothness penalties. Known geometry and solver validation
failures make a candidate infeasible; programming errors are not broadly
caught. A deterministic bounded coordinate pattern search starts at zero,
tries positive then negative moves for every control, accepts genuine lap-time
improvement, and reduces the step after an unproductive sweep. Immutable
configuration exposes control bounds, steps, reduction and tolerance, sweep and
evaluation limits, control count, and boundary margin. Termination reports the
minimum-step, sweep-limit, or evaluation-limit condition.

This is a **locally optimised racing line**, not proof of a global optimum. It
has one zero initialisation, no multistart, gradients, stochastic search, or
external optimisation dependency.

## Resolution validation

Optimisation and validation resolutions are intentionally distinct. The CLI
defaults to 1.0 m optimisation samples and re-evaluates both zero controls and
the same continuous best controls at 0.5 m. A coarse-grid improvement should
not be trusted unless it remains an improvement on the finer grid; exact gains
need not match because Phase 4 geometry is chordally discretised.

The example oval is artificial, symmetric, piecewise constant-curvature
geometry. It is useful for deterministic numerical checks but neither resembles
a complete circuit nor validates a visually traditional racing line or real
on-track performance.

```bash
python -m motorcycle_lap_sim.optimisation.diagnostics \
  examples/tracks/test_oval.yaml \
  examples/motorcycles/r6_2017plus_reference.yaml \
  --spacing 1.0 --validation-spacing 0.5 --controls 12 \
  --boundary-margin 0.25 --output-csv optimised-line.csv \
  --output-png optimised-line.png
```

## Phase 6 interoperability

Phase 6 leaves the Phase 5 periodic-cubic parameterisation and deterministic bounded pattern search unchanged. If the motorcycle enables the path-curvature transient proxy, each objective evaluation receives its effect through the fixed-path solver. Optimised controls should be re-evaluated (not re-optimised) at finer path spacings because curvature gradients are resolution-dependent. Optimisation CSV output includes gradient, actual rate, and transient ceiling.

## Phase 7 status

Phase 5 pattern search and its 12-DOF periodic offset parameterisation are unchanged and still build the established offset-sampled geometry by default. Diagnostics can map the same controls to uniformly spaced planar guide offsets for comparison, but Phase 7 neither optimises independent planar guide points nor switches the objective geometry.
