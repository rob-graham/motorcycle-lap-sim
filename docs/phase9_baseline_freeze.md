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

The track is the existing approximate, simulator-local Mallala reference geometry. It is not survey-grade, but engineering review considers it a good representation of Mallala for the present validation work. It must not be silently rescaled to force agreement with telemetry.

## Representative 52-control reference policy

The 52-control `reference` policy is the representative Phase 9 racing-line baseline for Phase 10 comparison because, on engineering review, it is considered more plausible/representative than the more aggressively flexible 96-control result. That judgement is a project decision, not a proof that 52 controls are physically optimal.

The recovered final reference control artifact is retained as:

`cases/mallala_r6/baseline/phase8_reference_controls.csv`

The artifact supplied for the freeze is identified as the **69.354897583 s** reference result and contains 52 controls. The recovered controls include two values exactly at their lower local bounds; that fact is retained rather than hidden and should be considered when interpreting later geometry/physics comparisons.

### Earlier 70.243539391 s restart seed

The retained historical log `phase8_mallala_12000_workers16.txt` records an earlier 52-control run with:

- zero-control lap: 73.992511085 s;
- optimised lap at the 1.0 m grid: 70.243539391 s;
- evaluations: 11,879;
- sweeps: 38; and
- termination: `maximum evaluations reached`.

Its same-geometry fixed-spline re-evaluations were:

- 1.00 m: 70.243539391 s;
- 0.50 m: 70.211070356 s; and
- 0.25 m: 70.194001531 s.

That run was budget-limited and its control vector was subsequently used as a restart/warm-start seed. It is retained separately as:

`cases/mallala_r6/baseline/phase8_reference_restart_seed_70p243539391.csv`

It must not be confused with the recovered 69.354897583 s final reference artifact.

## Secondary optimisation-assurance artifact: 96-control extra-fine policy

The 96-control extra-fine result is **not** the primary racing-line baseline. It is retained as a sensitivity/optimisation-assurance artifact because it demonstrates what happens when substantially more path freedom and much more local search are applied.

The final retained extra-fine sequence reached:

- control count: 96;
- final 1.0 m-grid lap: 69.843701478 s;
- final restart improvement: 0.020994560 s from 69.864696038 s;
- evaluations in the final restart: 23,575;
- sweeps in the final restart: 41;
- termination: `minimum step reached`;
- final step: 0.03125 m;
- path length: 2530.631558111 m;
- minimum continuous boundary clearance: 0.000015037 m; and
- minimum forward progress: 0.877497717.

The same geometry re-evaluated at finer output spacing gave:

- 1.00 m: 69.843701478 s;
- 0.50 m: 69.804007378 s; and
- 0.25 m: 69.783636921 s.

The 96-control search was reached through multiple long restart stages. The important finding is not simply its calculated lap time. It is slower than the recovered 52-control 69.354897583 s artifact despite substantially greater path freedom and search burden. The extra freedom also produced a more aggressive line, very small boundary clearances, continuing path-model/output-resolution sensitivity, and a large search burden. Therefore, more controls plus exhaustive local refinement are not by themselves a practical optimisation-assurance strategy.

The extra-fine diagnostic remains useful for comparison against later roll-aware optimisation, checking whether new physics changes the incentive for high-frequency path shaping, optimisation-assurance/multistart work, and identifying possible geometry/solver exploitation. It must not silently replace the 52-control reference line as the representative Phase 10 baseline.

## Verification status

On 16 August 2026 the complete repository test suite passed after the telemetry dependency was installed:

`185 passed`

This satisfies the software-test portion of the Phase 9 numerical freeze. The numerical baseline still carries explicit qualifications about local optimisation, finite output resolution, approximate Mallala geometry, provisional R6 parameters, and near-boundary line choices.

## Reproduction commands

Run the full test suite first:

```bash
python -m pytest
```

For numerical regression checks, re-evaluate the saved 52-control geometry rather than re-optimising it. A fresh optimisation is not expected to reproduce the historical local-search path unless all settings and start conditions are duplicated.

The Phase 8 extra-fine diagnostic policy remains available explicitly. It is 50 m maximum station spacing / 20 degree maximum arc-heading change and produces 96 Mallala controls. It is deliberately not a normal optimisation default.

## Frozen result package requirements

A retained baseline run should record, without suppressing warnings, repository commit, Python/package environment, track and motorcycle input hashes, selected control policy, cold-start/restart status, speed backend and workers, search settings, control count, initial/final lap time, evaluations/sweeps, termination reason, path length, minimum boundary clearance, minimum forward progress, speed/gear/RPM and acceleration summary, fixed-spline output-resolution checks, exported controls, and plots.

## Optimisation-assurance finding retained from Phase 8

The extra-fine exercise is retained as an optimisation-assurance finding, not as a reason to redesign the optimiser immediately. Increasing control count and driving the deterministic local pattern search toward exhaustive local convergence is computationally expensive and, by itself, does not establish a globally or robustly optimal physically relevant racing line.

Therefore:

1. retain the recovered 52-control 69.354897583 s result as the representative Phase 10 ideal-response baseline;
2. retain the current deterministic pattern search as the reference optimiser for the immediate telemetry/physics work;
3. preserve the 96-control result and its run settings as optimisation-assurance evidence;
4. do not make an even denser/exhaustive local search the default next step; and
5. defer systematic multistart, cross-resolution ranking, measured-line starts, perturbation starts, and any independent benchmark optimiser to the later optimisation-assurance phase.

This finding does not prove that either Phase 8 line is the global optimum. It records why brute-force densification of the same local method is not the preferred immediate development direction.

## Change-control rule

Phase 10 changes must be switchable or otherwise traceable so that the frozen ideal-response case can still be reproduced. Compare new physics against the saved 52-control geometry first. If a later result differs with Phase 10 features disabled, treat that as a numerical regression until a documented intentional change explains it.
