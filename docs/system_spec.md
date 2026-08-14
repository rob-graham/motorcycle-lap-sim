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
   A future `racing_line` package will represent a periodic path and validate
   it against track boundaries. It will not solve vehicle speed.
3. **Fixed-path motorcycle simulation — for a supplied path, what is the
   fastest physically feasible periodic speed profile?** Future `motorcycle`
   configuration and `speed_solver` modules will expose forces and constraints,
   then calculate speed, acceleration, lap time, and diagnostics for an
   immutable supplied path. This capability must be built and validated before
   optimisation.
4. **Racing-line optimisation — which permissible path minimises calculated
   lap time?** A future `optimisation` package will vary the racing line, call
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

## Future modules (not implemented in Phase 1)

- `racing_line`: path representation, interpolation, and boundary validation.
- `motorcycle`: explicit mass, geometry, tyres, aerodynamics, powertrain, and
  control-limit configuration plus clearly identified physical formulas.
- `speed_solver`: fastest feasible periodic speed on one fixed path and
  transparent constraint diagnostics.
- `optimisation`: racing-line parameterisation and minimum-time optimisation.
- output/reporting modules for speed, acceleration, line, and diagnostics.

Interfaces will pass typed configuration and result objects. The track, racing
line, motorcycle model, speed solver, and optimiser remain independently
testable and contain no duplicated hidden logic or mutable global state.
