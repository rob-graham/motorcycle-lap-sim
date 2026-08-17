# Motorcycle Lap Simulation

A clean-sheet Python project for minimum-lap-time motorcycle simulation, racing-line optimisation, and Mallala R6 validation development.

The repository has progressed beyond the original Phase 8 documentation. It now contains the frozen Mallala baseline, telemetry ingestion/registration/comparison tools, a simple switchable finite-roll sensitivity model, roll-aware re-optimisation diagnostics, and later optimisation-assurance experiments.

For review, start with:

- [simulation project context and source hierarchy](docs/project_context.md);
- [system specification and implemented phase status](docs/system_spec.md);
- [Phase 9 Mallala numerical baseline freeze](docs/phase9_baseline_freeze.md); and
- [Mallala R6 telemetry integrity assessment](docs/mallala_r6_telemetry_integrity.md).

The project-level roadmap and the repository serve different purposes: the roadmap defines intended development direction; repository code, tests, cases, and documentation define implemented behaviour.

## Architecture

The numerical code deliberately separates:

- `track` — analytic track geometry, widths, sampling, and boundaries;
- `path` / `racing_line` — supplied and spline-defined motorcycle paths;
- `motorcycle` — immutable configuration, powertrain, forces, physical limits, and optional simple handling-response limits;
- `speed_solver` — fastest feasible periodic speed on a supplied path;
- `optimisation` — deterministic racing-line search using the fixed-path solver;
- `telemetry` — import, quality, registration, map matching, and validation utilities; and
- `plotting` / scripts — diagnostics and engineering review outputs.

Internal calculations use SI units. Measured telemetry is not a hidden dependency of the physics solver.

## Install and test

```bash
python -m pip install -e '.[test]'
python -m pytest
```

For optional Numba acceleration:

```bash
python -m pip install -e '.[test,accelerated]'
```

The Python fixed-path solver remains the authoritative reference backend. The optional Numba backend is a computational acceleration and is checked against the Python result for accepted paths.

## Development/validation claim boundary

The repository is a development and engineering-analysis tool. The current Mallala/R6 work is case-specific and provisional:

- the Mallala geometry is an approximate development reference, not survey-grade;
- the 2017+ R6 configuration is provisional and not a fully identified rider/motorcycle model;
- the finite-roll parameter is a replaceable sensitivity scenario unless separately calibrated;
- measured rider speed and line are validation evidence, not the optimisation target; and
- no output claims regulatory approval, homologation, certification, or insurance acceptance.

A total lap-time match by itself is not a validation criterion. Spatial speed, line, acceleration, transition behaviour, active constraints, and data quality must also be considered.

## Core phase history

### Phases 1-3: track, motorcycle and fixed-path solver

The repository began with analytic track primitives and boundaries, an independently testable motorcycle model, and a periodic fixed-path minimum-time solver. See [fixed-path solver](docs/fixed_path_solver.md).

Example fixed-path diagnostic:

```bash
python -m motorcycle_lap_sim.speed_solver.diagnostics \
    examples/tracks/test_oval.yaml \
    examples/motorcycles/test_motorcycle.yaml \
    --spacing 1.0
```

### Phases 4-5: supplied and locally optimised racing lines

Phase 4 added supplied lateral-offset paths. Phase 5 added smooth periodic parameterisation and deterministic local racing-line optimisation. The result is a numerical local optimum, not a global-optimality claim.

See [racing-line representation](docs/racing_line_representation.md) and [racing-line optimisation](docs/racing_line_optimisation.md).

### Phase 6: optional curvature-transient proxy

An optional `max_path_curvature_rate_1pmps` handling proxy can add a curvature-transient speed ceiling. It remains a simple sensitivity feature rather than a validated steering-dynamics model. See [curvature transient limit](docs/curvature_transient_limit.md).

### Phases 7-8: smooth planar paths and direct physical controls

Phase 7 added a C2-periodic Cartesian path spline. Phase 7.5 integrated the approximate Mallala reference and variable primitive widths. Phase 8 added direct physical lateral controls, continuous corridor checks, warm starts, deterministic best-improvement search, optional parallel polling, and fixed-grid re-evaluation.

See [smooth planar racing line](docs/smooth_planar_racing_line.md), [Mallala reference track](docs/mallala_reference_track.md), and [direct planar optimisation](docs/direct_planar_racing_line_optimisation.md).

## Current Mallala baseline and validation work

### Frozen ideal-response baseline

The retained representative Mallala baseline uses the 52-control Phase 8 line stored in:

`cases/mallala_r6/baseline/phase8_reference_controls.csv`

Its canonical identities and regression values are recorded in [Phase 9 baseline freeze](docs/phase9_baseline_freeze.md). Reproduce the saved geometry without re-running optimisation using:

```bash
python scripts/r6_phase9_baseline_check.py
```

Fresh local optimisation is not expected to reproduce the historical path exactly; numerical change control is based on the committed saved geometry.

### Mallala R6 telemetry

The `telemetry` package and diagnostic scripts support AiM/Excel ingestion, lap extraction, quality checks, rigid 2D registration, map matching, cross-lap/peer comparison, repeatability checks, and speed comparison.

The source workbook is not committed; its identity is recorded separately in the case manifest. See [Mallala R6 telemetry integrity](docs/mallala_r6_telemetry_integrity.md).

Historical repository script names use `phase10` for some telemetry work even though the current project roadmap groups baseline, telemetry and initial roll-response work under **Phase 9**. Reviewers should use [project context](docs/project_context.md) and [system specification](docs/system_spec.md), not infer roadmap meaning from script numbering alone.

### Level-1 finite-roll sensitivity

The simulator now includes planar demanded lean and a simple trajectory-driven roll-transition constraint. The implemented finite-roll sensitivity uses an explicit `max_roll_rate_radps` in motorcycle handling configuration and can be disabled to reproduce the ideal-response baseline.

This is deliberately not a full rider/steering dynamics model. Values used in diagnostics, including `0.8 rad/s`, are scenarios unless separately supported by calibration evidence.

Relevant implementation and diagnostics include:

- `src/motorcycle_lap_sim/motorcycle/roll.py`;
- `scripts/r6_phase9f_roll_aware_optimisation.py`;
- `scripts/r6_phase9f_spatial_comparison.py`;
- `scripts/r6_phase9g_roll_rate_components.py`;
- `scripts/r6_phase9g_sector_diagnostics.py`; and
- `scripts/r6_phase10_trajectory_export.py`.

The intended comparison separates:

1. frozen ideal-response line;
2. the same line with finite roll response;
3. a line re-optimised with finite roll response; and
4. measured Mallala R6 behaviour.

That separation prevents a physics change, path adaptation, and measurement discrepancy from being conflated.

## Calibration boundary

Substantial R6 parameter fitting is intentionally restrained until the roll/telemetry discrepancy is understood. Mass, power, drag, grip/utilisation, gearing/radius corrections, edge margin, and handling response can compensate for each other.

Any calibration work should therefore use a small bounded identifiable subset, retain documented defaults, use hold-out laps, and report local/sector metrics. The valid claim is a **Mallala R6 case calibration/validation** where supported, not general motorcycle-model validation.

## Optimisation-assurance diagnostics

The repository also contains later `r6_phase11a_*`, `r6_phase11b_*`, and `r6_phase11c_*` scripts. These are diagnostic experiments assessing warm-start and basin/convergence limitations of the existing deterministic pattern search. They are not a production replacement optimiser and do not prove global optimality.

Their main review value is to show that generic-start spread can arise from limited search convergence. It should not be mislabelled as physical uncertainty, and progressively larger brute-force search budgets are not the primary Mallala validation objective.

## Provisional 2017+ Yamaha YZF-R6 reference

The repository includes a stock-like 2017+ R6 reference configuration. It is not an exact model-year reconstruction or experimentally validated motorcycle. See [calibration and provenance record](docs/r6_2017plus_calibration.md).

Independent diagnostics:

```bash
python -m motorcycle_lap_sim.motorcycle.diagnostics \
    examples/motorcycles/r6_2017plus_reference.yaml
```

## Future project interfaces

The wider track-layout project roadmap proposes later formal coaching-event extraction, robust line/envelope outputs, GIS/georeferencing, LiDAR/DEM-derived `TrackSurface` / `z(s,n)` support, grade/banking in the lap solver, and a separate run-off package consuming versioned simulation outputs.

Those are future interfaces, not current capability claims unless implemented code and tests explicitly support them.
