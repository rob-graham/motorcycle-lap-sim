# Motorcycle Lap Simulation

A clean-sheet Python project for minimum-lap-time motorcycle simulation and
racing-line optimisation. Phase 5 adds a deterministic first locally optimised
minimum-lap-time racing line on top of the validated fixed-path architecture.

## Architecture and status

The numerical `track` package contains analytic centreline primitives, ordered
tracks, closure diagnostics, arc-length sampling, and boundary calculation.
The independent `motorcycle` package contains immutable validated configuration,
engine and powertrain calculations, forces, load transfer, and physical limits.
Plotting remains separate. The distinct `racing_line` package constructs a
generic `SampledPath`; the separate `optimisation` package varies a smooth,
periodic, boundary-safe line and minimises solved lap time; see
[the system specification](docs/system_spec.md). Internal calculations use SI.

## Install and test

```bash
python -m pip install -e '.[test]'
python -m pytest
```

The installed `pytest` console command is an optional shortcut. On Windows,
the Python Scripts directory containing installed console commands may not be
on `PATH`.

## Motorcycle diagnostics

From the repository root:

```bash
python -m motorcycle_lap_sim.motorcycle.diagnostics \
    examples/motorcycles/test_motorcycle.yaml
```

This runs independent motorcycle-model checks; it is not a lap speed
simulation.

## Plot the example oval

From the repository root:

```bash
python -m motorcycle_lap_sim.plotting.track_plot \
    examples/tracks/test_oval.yaml --output test_oval.png
```

The installed console command
`plot-example-track examples/tracks/test_oval.yaml --output test_oval.png` is an
optional shortcut, with the same Windows `PATH` caveat described above.

The YAML format records a start pose, distinct left/right widths, closure
intent, and an ordered list of straight and circular-arc primitives.

## Phase 3 fixed-path solver

Phase 3 calculates a periodic minimum-time speed profile on a supplied fixed closed path, currently demonstrated on the sampled track centreline. It does not optimise a racing line. See [the fixed-path solver](docs/fixed_path_solver.md).

Run the fixed-path diagnostics on the example inputs with a requested sampling
spacing (metres):

```bash
python -m motorcycle_lap_sim.speed_solver.diagnostics \
    examples/tracks/test_oval.yaml \
    examples/motorcycles/test_motorcycle.yaml \
    --spacing 1.0
```

Add `--csv speed-profile.csv` to export path coordinates and curvature, both
speed ceilings, the solved speed, lateral and longitudinal acceleration, gear,
and engine RPM. The console summary reports the sample count and path length,
lap time, speed statistics, peak accelerations, gear and RPM ranges, and solver
iteration/convergence information.

## Provisional 2017+ Yamaha YZF-R6 reference

The repository includes a provisional stock-like 2017+ R6 reference. It is not
an exact model-year reconstruction or an experimentally validated motorcycle;
see the [calibration and provenance record](docs/r6_2017plus_calibration.md).
Run its independent motorcycle and fixed-path diagnostics from the repository
root:

```bash
python -m motorcycle_lap_sim.motorcycle.diagnostics \
    examples/motorcycles/r6_2017plus_reference.yaml

python -m motorcycle_lap_sim.speed_solver.diagnostics \
    examples/tracks/test_oval.yaml \
    examples/motorcycles/r6_2017plus_reference.yaml \
    --spacing 1.0 --csv r6_2017plus_test_oval.csv
```

The inline torque curve is a smooth stock-like rear-wheel estimate. The
reported lap time consequently remains provisional and must not be interpreted
as experimental validation.

## Phase 4 supplied racing line

Phase 4 accepts one lateral offset per sampled centreline station, validates it
against track widths, and calculates displaced coordinates, chordal arc length,
and periodic signed curvature. Positive offset is left of travel. See the
[racing-line representation](docs/racing_line_representation.md).

```bash
python -m motorcycle_lap_sim.racing_line.diagnostics \
    examples/tracks/test_oval.yaml --spacing 1.0 --constant-offset 2.0 \
    --csv racing-line.csv --output-png racing-line.png
```

The `+2 m` example is deterministic manual geometry for inspection, not an
optimal line.

## Phase 5 local racing-line optimisation

Phase 5 uses a low-dimensional periodic cubic parameterisation and deterministic
bounded pattern search, without SciPy. Its result is numerical and local, not a
claim of global optimality. See [racing-line optimisation](docs/racing_line_optimisation.md).

```bash
python -m motorcycle_lap_sim.optimisation.diagnostics \
  examples/tracks/test_oval.yaml \
  examples/motorcycles/r6_2017plus_reference.yaml \
  --spacing 1.0 --validation-spacing 0.5 --controls 12 \
  --boundary-margin 0.25 --output-csv optimised-line.csv \
  --output-png optimised-line.png
```

## Phase 6: path-curvature transient proxy

Phase 6 adds an optional, deterministic path-curvature transient speed ceiling to the fixed-path solver, while retaining the Phase 5 racing-line optimiser. It is a path-handling proxy—not a validated steering-dynamics model. See [`docs/curvature_transient_limit.md`](docs/curvature_transient_limit.md) for the formula, units, assumptions, diagnostics, and resolution guidance.
