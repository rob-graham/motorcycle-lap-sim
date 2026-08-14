# Phase 7 smooth planar racing-line geometry

## Separation from the track

Track primitives remain deliberately understandable straights and constant-radius circular arcs. Their centreline curvature may jump at joins; Phase 7 does not smooth the track or add clothoids. The motorcycle line is a separate geometric object. Phase 7 offers a smooth planar line experimentally while the Phase 5 offset-sampled representation and pattern-search optimiser remain the defaults.

## Guide construction and spline

For `N` guides, exact analytic track geometry is evaluated at `s_i=iL/N`, independently of any output grid. A supplied geometric offset gives `G_i=C(s_i)+n_i N(s_i)`. Diagnostics may obtain `n_i` by evaluating the existing 12-DOF `PeriodicCubicParameterisation` and boundary-safe mapping, but the planar builder has no optimisation dependency.

Cartesian x and y use the same genuine periodic interpolating cubic. With `h=L/N`, its vector knot second derivatives solve

```
M[i-1] + 4 M[i] + M[i+1]
    = 6/h^2 (G[i+1] - 2 G[i] + G[i-1])
```

with cyclic indices. Standard second-derivative cubic segments interpolate the guides and make position, first derivative, and second derivative periodic and continuous (C2), including at start/finish. The implementation uses NumPy's dense linear solver and no SciPy.

## Derivatives, curvature, and length

The cubic is differentiated analytically. Signed path curvature is
`(x' y''-y' x'')/(x'^2+y'^2)^(3/2)`, so left turns are positive. Non-finite results and a tangent magnitude at or below `1e-10` are rejected.

Spline parameter `s` remains centreline station, not motorcycle-path distance `q`. Actual distance integrates `dq/ds=hypot(x',y')` with fixed five-point Gauss-Legendre quadrature, split at every spline knot. Output `q` starts at zero and strictly increases; the periodic endpoint is omitted while full length includes the final interval. Thus guide count defines the continuous curve, whereas 1.0, 0.5, or 0.25 m output spacing merely evaluates the same curve.

## Corridor validation

Guides first obey `-(width_right-margin) <= n <= width_left-margin`. Cubics can overshoot, so a configurable dense station grid evaluates both track and spline at a common centreline parameter `s`. The projected normal offset is checked without clipping. The result records projected offset, boundary clearance, and tangent deviation `(P-C).T`; tangent deviation is diagnostic only. This parameter-aligned check is intentionally not a nearest-centreline projection, which remains future work for self-near or more complex tracks.

## Validation and sensitivity

`python scripts/r6_phase7_planar_geometry_check.py` uses the recorded **Phase 5 deterministic local reference** (not a global optimum). It reports guide-count sensitivity for 24/36/48 guides, old-versus-smooth geometry, output-resolution convergence for one 48-guide curve, and the optional experimental `0.8 1/(m s)` Phase 6 transient limit. It also writes `phase7_planar_geometry.png`. Guide count is a representation choice and is not selected merely for lap time; material sensitivity must be reported rather than tuned away.

The optional transient model consumes ordinary `SampledPath` curvature diagnostics. It is not asserted to be physically correct, and the smooth result need not equal the disabled result.

### Recorded test-oval results

With 0.5 m output samples, guide counts 24/36/48 respectively produced lengths 371.3983/371.4525/371.4677 m, lap times 15.6283/15.6777/15.7519 s, minimum clearances 0.1507/0.1529/0.1546 m, and maximum tangent deviations 0.1286/0.0907/0.0708 m. Curvature ranges were `[0.00056,0.03869]`, `[-0.00388,0.03754]`, and `[-0.00668,0.03746]` 1/m; peak absolute `dκ/dq` was 0.00268/0.00411/0.00554 1/m². This is material guide-count sensitivity, so 48 is used as a clearly labelled diagnostic selection, not asserted to be converged or selected for speed.

For the same 48-guide spline, 1.0/0.5/0.25 m output grids gave identical quadrature length 371.467745 m and disabled lap times 15.76874/15.75188/15.74355 s. Peak `|dκ/dq|` was 0.00537/0.00554/0.00561 1/m² and peak `|dκ/dt|` was 0.10798/0.11080/0.11187 1/(m s). The experimental 0.8 limit was inactive because these rates stayed below it, so those lap times were identical.

The old offset-sampled lap times at 1.0/0.5/0.25 m were 15.81793/15.96884/16.60722 s. Smooth-minus-old deltas were -0.04918/-0.21696/-0.86366 s (-0.31/-1.36/-5.20%). Old peak `|dκ/dq|` grew from 0.01716 to 0.10515 1/m² while smooth remained near 0.0056. Thus the former 0.25 m curvature-gradient and lap-time sensitivity is substantially reduced, although the material fine-grid lap difference exposes the geometric effect rather than justifying either representation by speed.

## Deferred work and limitations

Phase 7 does **not** make planar geometry the optimiser default, introduce per-guide optimisation variables, change pattern search or motorcycle physics, add SciPy/global/multistart optimisation, add clothoid primitives, or model roll, steering-head dynamics, banking, elevation, or motorcycle width. Nearest-centreline corridor projection and optimisation using this alternative representation are deferred until validation warrants them.
