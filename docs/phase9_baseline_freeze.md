# Phase 9 Mallala numerical baseline freeze

## Status

Phase 8 racing-line development is closed for the present development sequence. This document freezes the numerical/software baseline to be preserved while Phase 10 telemetry, validation, and roll-response work is added.

The frozen repository commit is:

`300fd58fff97a46d8152422cb1d19077af091a44`

The freeze uses:

- track: `examples/tracks/mallala_reference.yaml` (Mallala centreline fit v0.3);
- motorcycle: `examples/motorcycles/r6_2017plus_reference.yaml`;
- Phase 8 diagnostic: `scripts/r6_phase8_planar_optimisation_check.py`;
- ideal handling response, with the provisional curvature-transient proxy disabled unless a comparison explicitly says otherwise; and
- SI units internally.

The track is the existing approximate, simulator-local Mallala reference geometry. It is not survey-grade and must not be silently rescaled to the nominal circuit length.

## Reproduction commands

Run the full test suite first:

```bash
python -m pytest
```

Then reproduce the existing Phase 8 reference diagnostic:

```bash
python scripts/r6_phase8_planar_optimisation_check.py \
  --track mallala \
  --policy reference \
  --speed-backend python
```

For backend equivalence/speed work, the optional Numba backend may be run separately, but its result is a cross-check rather than a different baseline definition.

The Phase 8 extra-fine diagnostic policy remains available explicitly:

```bash
python scripts/r6_phase8_planar_optimisation_check.py \
  --track mallala \
  --policy extra-fine \
  --speed-backend numba \
  --workers <N>
```

The extra-fine policy is 50 m maximum station spacing / 20 degree maximum arc-heading change and produces 96 Mallala controls. It is deliberately not a normal optimisation default.

## Frozen result package requirements

A retained baseline run must record, without suppressing warnings:

- repository commit;
- Python/package environment;
- track and motorcycle input file hashes;
- selected control policy;
- cold-start or restart status;
- speed backend and worker count;
- initial step, maximum sweeps, and maximum evaluations;
- number of controls;
- initial and final lap time;
- evaluation and sweep counts;
- termination reason;
- path length;
- minimum continuous boundary clearance;
- minimum forward progress;
- speed/gear/RPM and acceleration summary;
- fixed-spline output-resolution checks at 1.0, 0.5, and 0.25 m; and
- exported controls and Phase 8 plots.

The current project context records the ideal-response Mallala result as approximately 1:09. The exact converged extra-fine cold-start scalar/log is not stored in the repository at this freeze commit, so this document intentionally does not invent a more precise number. The original Phase 8 run log/result package should be retained alongside this freeze when available.

## Optimisation-assurance finding retained from Phase 8

The converged extra-fine cold-start exercise is retained as an optimisation-assurance finding, not as a reason to redesign the optimiser immediately.

It demonstrated that increasing control count and driving the existing deterministic local pattern search toward exhaustive local convergence is computationally expensive and, by itself, is not a practical strategy for establishing that the physically relevant racing line is globally or robustly optimal. More controls can improve representation freedom while simultaneously increasing local-search cost and sensitivity to the starting basin.

Therefore:

1. retain the current deterministic pattern search as the reference optimiser for the immediate telemetry/physics work;
2. preserve the extra-fine result and its run settings as evidence about optimisation behaviour;
3. do not make an even denser/exhaustive local search the default next step; and
4. defer systematic multistart, cross-resolution ranking, measured-line starts, perturbation starts, and any independent benchmark optimiser to the optimisation-assurance phase after the initial Mallala model/telemetry comparison is working.

This finding does not prove that the Phase 8 line is the global optimum. It records why brute-force densification of the same local method is not the preferred immediate development direction.

## Change-control rule

Phase 10 changes must be switchable or otherwise traceable so that the frozen ideal-response case can still be reproduced. If a later result differs with Phase 10 features disabled, treat that as a numerical regression until a documented intentional change explains it.
