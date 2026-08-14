# Motorcycle Lap Simulation

A clean-sheet Python project for minimum-lap-time motorcycle simulation and
racing-line optimisation. **Phase 1 only defines and samples track geometry; it
does not simulate motorcycle speed, dynamics, or optimise a racing line.**

## Architecture and status

The numerical `track` package contains analytic centreline primitives, ordered
tracks, closure diagnostics, arc-length sampling, and boundary calculation.
Plotting is a separate, optional consumer of those numerical results. Future
phases will add distinct racing-line, motorcycle, fixed-path speed-solver, and
optimisation packages; see [the system specification](docs/system_spec.md).
All internal geometry uses metres, radians, and inverse metres.

## Install and test

```bash
python -m pip install -e '.[test]'
python -m pytest
```

The installed `pytest` console command is an optional shortcut. On Windows,
the Python Scripts directory containing installed console commands may not be
on `PATH`.

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
