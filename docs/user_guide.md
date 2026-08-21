# User guide

This guide covers the supported direct-planar racing-line command. Calculations use SI units and the result is a deterministic **local** optimisation from the configured starting line—not proof of the globally fastest line.

## A. Five-minute first run

The supplied oval is deliberately small. Its `test_motorcycle.yaml` is synthetic validation data, **not a model of a real motorcycle**.

### Windows PowerShell

From the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e .
.\motorcycle-lap-sim.cmd --help
.\motorcycle-lap-sim.cmd optimise examples\tracks\test_oval.yaml examples\motorcycles\test_motorcycle.yaml --policy fine --max-sweeps 1 --max-evaluations 100 --output oval_controls.csv
```

### macOS/Linux/POSIX shell

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e .
motorcycle-lap-sim --help
motorcycle-lap-sim optimise examples/tracks/test_oval.yaml examples/motorcycles/test_motorcycle.yaml --policy fine --max-sweeps 1 --max-evaluations 100 --output oval_controls.csv
```

### Bounded smoke example

The command above deliberately uses the feasible `fine` basis and bounds the run to one complete poll because a full optimisation is not reasonably quick. Termination on its sweep/evaluation limit is **not a converged optimisation**. The summary reports inputs, settings, lap-time change, search termination and boundary clearance. For substantive work, choose a supported policy and budgets deliberately; the command defaults remain the existing `reference` policy and existing numerical settings.

`oval_controls.csv` has the strict header:

```text
index,control_s_m,best_offset_m,lower_bound_m,upper_bound_m
```

Rows are ordered controls around the lap. Station and offset values are metres; positive offset is left of travel. Bounds are the usable local limits after the configured boundary margin.

If `--output` is omitted, the command writes `<track-stem>_controls.csv` in the current working directory; for example, `test_oval.yaml` produces `test_oval_controls.csv`.

## B. Mallala and provisional R6 example

```powershell
.\motorcycle-lap-sim.cmd optimise examples\tracks\mallala_reference.yaml examples\motorcycles\r6_2017plus_reference.yaml --output mallala_new_controls.csv
```

The Mallala analytic geometry is approximate and not survey-grade. The R6 configuration is provisional rather than a fully identified motorcycle/rider model. A fresh generic local optimisation is not expected to reproduce, and does not replace, the separately controlled retained Phase 11 representative. That accepted representative remains the provenance-controlled input for the specialised downstream run-off workflow.

## C. Bring your own track and motorcycle

Copy the supplied YAML examples and edit only supported fields. Track files describe a closed analytic sequence of straights and circular arcs, travel direction, and left/right widths; see the [system specification](system_spec.md), [smooth planar representation](smooth_planar_racing_line.md), and [Mallala reference notes](mallala_reference_track.md). Motorcycle files supply explicit model parameters and limits; see the [motorcycle model](motorcycle_model.md). All quantities use the documented SI units. A motorcycle configuration is model input, not automatic vehicle identification or calibration.

```powershell
.\motorcycle-lap-sim.cmd optimise my_track.yaml my_motorcycle.yaml --output my_controls.csv
```

The optimiser depends on its starting line, control policy and defaults. `--policy {coarse,reference,fine}` changes the physical control basis; `reference` is the default. Run `motorcycle-lap-sim optimise --help` for advanced options including step, tolerance, search budgets, margin, sampling, workers and backend. These expose the existing method without changing its defaults.

### Optional Numba acceleration

Python is the authoritative default. Install and select the optional validated backend with:

```bash
python -m pip install -e '.[accelerated]'
motorcycle-lap-sim optimise TRACK.yaml MOTORCYCLE.yaml --speed-backend numba
```

If Numba is unavailable or disagrees with authoritative validation, the existing error behaviour is retained; the command does not silently fall back.

## General optimisation versus retained run-off export

These are deliberately separate workflows:

```text
GENERAL
track YAML + motorcycle YAML
            |
            v
 motorcycle-lap-sim optimise
            |
            v
        controls.csv

RETAINED MALLALA LOWSIDE
accepted retained Mallala controls
            |
            v
 motorcycle-lap-sim export runoff
            |
            v
 versioned runoff-bundle
```

A CSV made by generic `optimise` is **not automatically accepted input** to `export runoff`. The latter is a retained Mallala workflow with provenance and acceptance checks; those checks are not weakened to connect the commands.

## Troubleshooting and further documentation

- **Command/executable not found:** confirm installation used the intended `.venv`; activate it on POSIX, or use the repo-root `.cmd` launcher on Windows.
- **Virtual environment not installed:** create it with the commands above, then install the editable package using that environment's Python.
- **Input rejected:** verify the file exists and follows the supported YAML examples and technical specifications. Numerical, geometry and optimisation errors are reported rather than suppressed.
- **Output path error:** ensure its parent directory already exists and is writable.

This is the practical user guide. Developers and engineering reviewers should also read the [project context](project_context.md), [system specification](system_spec.md), [direct-planar method](direct_planar_racing_line_optimisation.md), and [development scope review](development_scope_review.md).
