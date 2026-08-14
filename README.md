# Motorcycle Lap Simulation

A clean-sheet Python project for minimum-lap-time motorcycle simulation and
racing-line optimisation. **Phase 3 adds a validated periodic fixed-path speed
solver and lap time calculation; racing-line optimisation remains out of scope.**

## Architecture and status

The numerical `track` package contains analytic centreline primitives, ordered
tracks, closure diagnostics, arc-length sampling, and boundary calculation.
The independent `motorcycle` package contains immutable validated configuration,
engine and powertrain calculations, forces, load transfer, and physical limits.
Plotting remains separate. Future phases will add distinct racing-line,
fixed-path speed-solver, and optimisation packages; see
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

## Fixed-path speed diagnostics

From the repository root, run the Phase 3 solver on the example oval:

```bash
python -m motorcycle_lap_sim.speed_solver.diagnostics \
    examples/tracks/test_oval.yaml \
    examples/motorcycles/test_motorcycle.yaml
```

Use `--spacing METRES` to change the track sampling interval (the default is
1 metre), and `--csv PATH` to write the solved distance and speed columns.

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
