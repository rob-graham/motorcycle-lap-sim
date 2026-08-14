# System specification

## Scope and conventions

The project is a clean-sheet minimum-lap-time simulator. SI units are used
internally: metres, seconds, kilograms, radians, and derived SI units. Heading
is measured counter-clockwise from the positive x-axis. Assumptions and model
parameters must be explicit configuration, not constants hidden in algorithms.

The architecture deliberately separates four concepts:

1. **Track geometry — what physical region is available to ride on?** An
   analytic centreline, left and right widths, sampled differential geometry,
   and boundaries describe the permitted physical region. Track geometry does
   not choose a path or contain vehicle physics.
2. **Racing line — what path through that region does the motorcycle follow?**
   The Phase 4 `racing_line` package represents a supplied periodic path and
   validates it against track boundaries. It does not solve vehicle speed.
3. **Fixed-path motorcycle simulation — for a supplied path, what is the
   fastest physically feasible periodic speed profile?** The Phase 2
   `motorcycle` package exposes independently validated configuration,
   forces, and constraints; the `speed_solver` uses them to
   calculate speed, acceleration, lap time, and diagnostics for an
   immutable supplied path. This capability must be built and validated before
   optimisation.
4. **Racing-line optimisation — which permissible path minimises calculated
   lap time?** The Phase 5 `optimisation` package varies the racing line, calls
   the fixed-path solver through its public interface, and enforce track/path
   constraints. It must not duplicate geometry or motorcycle feasibility logic.

## Phase 1 interfaces

`track.primitives` provides analytic `Straight` and `CircularArc` geometry from
a `Pose`. `track.track` composes primitives, retains width and closure intent,
and reports closure errors without modifying geometry. `track.sampling`
produces immutable array-based samples approximately uniform in centreline arc
length. `track.boundaries` offsets samples along their normals. The positive
normal points left of travel. `plotting` depends on these APIs, while numerical
modules never depend on plotting.

Closed-track samples omit the duplicate endpoint by default: `s=0` is present
and `s=total_length` is absent. Callers may explicitly request the endpoint.
Open tracks include both ends. Primitive joins are represented only once where
a sampling location falls exactly on a join.

## Phase 2 motorcycle physics

The `motorcycle` package provides immutable YAML configuration, deterministic
engine interpolation, explicit gearing, resistance forces, longitudinal axle
loads, geometry-derived tip-over limits, lateral caps, and a generic friction
ellipse. Each formula is usable without track geometry or plotting. It does not
propagate speed or select gears.

## Module status

- `racing_line`: Phase 4 dense supplied-offset representation and boundary validation.
- `speed_solver`: Phase 3 fastest feasible periodic speed on one fixed path and
  transparent constraint diagnostics.
- `optimisation`: racing-line parameterisation and minimum-time optimisation.
- output/reporting modules for speed, acceleration, line, and diagnostics.

Interfaces will pass typed configuration and result objects. The track, racing
line, motorcycle model, speed solver, and optimiser remain independently
testable and contain no duplicated hidden logic or mutable global state.

## Phase 3 status

The solver calculates speed and lap time on a supplied fixed, closed path. Track-centreline sampling is an adapter only; racing-line optimisation remains outside the current system.

## Phase 4 status

The racing-line layer validates dense lateral offsets and constructs actual
coordinates, chordal distance, and periodic geometric curvature as a generic
`SampledPath`. The speed solver remains independent of track geometry. Phase 4
contains no objective function, control-vector parameterisation, or optimiser.

## Phase 5 status

The optimisation layer provides a C2 periodic cubic control parameterisation,
smooth asymmetric boundary-safe mapping, pure lap-time evaluation, and a
deterministic bounded coordinate pattern search. It returns a locally and
numerically optimised racing line and explicitly supports finer-resolution
re-evaluation. It makes no global-optimality claim and introduces no new
optimisation dependency.

## Phase 6 optional path-handling proxy

Motorcycle YAML may contain `handling.max_path_curvature_rate_1pmps`, a finite, strictly positive limit in 1/(m*s). Omitting `handling` disables it without a default. The fixed-path capability layer calculates the periodic curvature gradient and combines its speed ceiling with existing local ceilings. Racing-line optimisation remains formula-agnostic and sees only the resulting lap time. See [the curvature transient specification](curvature_transient_limit.md).

## Phase 7 geometry boundary

The track remains a piecewise straight/circular analytic definition and may have curvature jumps. Experimental Phase 7 converts supplied offset guides into a C2-periodic Cartesian spline, then adapts it to the same generic `SampledPath` consumed by the track-unaware fixed-path solver. Existing Phase 5 geometry remains the optimiser default pending validation.
