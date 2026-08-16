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

## Primary frozen racing-line baseline: 52-control reference policy

The 52-control `reference` policy is the primary Phase 9 racing-line baseline for Phase 10 comparison. It is retained because it is the established Phase 8 reference model and, on engineering review, is considered a more plausible/representative racing line than the more aggressively flexible 96-control result. That judgement is a project decision, not a proof that 52 controls are physically optimal.

Retained control vector:

`cases/mallala_r6/baseline/phase8_reference_controls.csv`

Recorded Phase 8 run metrics:

- control count: 52;
- zero-control lap: 73.992511085 s;
- optimised lap at the 1.0 m optimisation/output grid: 70.243539391 s;
- improvement from zero controls: 3.748971694 s;
- evaluations: 11,879;
- sweeps: 38;
- termination: `maximum evaluations reached`;
- optimisation wall time: 3690.274636 s with 16 workers in the retained run;
- path length: 2516.347858745 m;
- minimum continuous boundary clearance: 0.000069340 m;
- minimum forward progress: 0.770882441;
- speed range: 9.894198 to 58.325656 m/s; and
- control extrema: -3.75 to +3.50 m.

Fixed-spline re-evaluation of the same geometry gave:

- 1.00 m: 70.243539391 s;
- 0.50 m: 70.211070356 s; and
- 0.25 m: 70.194001531 s.

This output-resolution dependence is retained as part of the baseline evidence. The 52-control run also terminated on its evaluation budget rather than minimum step, so it must not be described as a mathematically converged optimum.

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

The 96-control search was reached through multiple long restart stages. The important finding is not simply that it is about 0.40 s faster than the retained 52-control result at the 1.0 m grid. The extra freedom produced a more aggressive line, very small boundary clearances, continuing path-model/output-resolution sensitivity, and a large search burden. Therefore, more controls plus exhaustive local refinement are not by themselves a practical optimisation-assurance strategy.

The extra-fine diagnostic remains useful for:

- comparison against later roll-aware optimisation;
- checking whether new physics changes the incentive for high-frequency path shaping;
- optimisation-assurance/multistart work; and
- identifying possible geometry/solver exploitation.

It must not silently replace the 52-control reference line as the representative Phase 10 baseline merely because its calculated lap is faster.

## Verification status

On 16 August 2026 the complete repository test suite passed after the telemetry dependency was installed:

`185 passed`

This satisfies the software-test portion of the Phase 9 numerical freeze. The retained numerical baseline still carries the explicit qualifications above about local optimisation, finite output resolution, approximate Mallala geometry, provisional R6 parameters, and near-boundary line choices.

## Reproduction commands

Run the full test suite first:

```bash
python -m pytest
```

To reproduce the retained 52-control reference configuration from its saved controls, use the Phase 8 diagnostic with `--policy reference` and the retained controls CSV. Re-evaluation rather than re-optimisation is preferred when checking later numerical regressions; a fresh optimisation is not expected to reproduce the exact saved search history unless all search settings and start conditions are duplicated.

The Phase 8 extra-fine diagnostic policy remains available explicitly. It is 50 m maximum station spacing / 20 degree maximum arc-heading change and produces 96 Mallala controls. It is deliberately not a normal optimisation default.

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

## Optimisation-assurance finding retained from Phase 8

The converged extra-fine exercise is retained as an optimisation-assurance finding, not as a reason to redesign the optimiser immediately.

It demonstrated that increasing control count and driving the existing deterministic local pattern search toward exhaustive local convergence is computationally expensive and, by itself, is not a practical strategy for establishing that the physically relevant racing line is globally or robustly optimal. More controls can improve representation freedom while simultaneously increasing local-search cost and sensitivity to the starting basin.

Therefore:

1. retain the 52-control reference result as the representative Phase 10 ideal-response baseline;
2. retain the current deterministic pattern search as the reference optimiser for the immediate telemetry/physics work;
3. preserve the 96-control result and its run settings as optimisation-assurance evidence;
4. do not make an even denser/exhaustive local search the default next step; and
5. defer systematic multistart, cross-resolution ranking, measured-line starts, perturbation starts, and any independent benchmark optimiser to the optimisation-assurance phase after the initial Mallala model/telemetry comparison is working.

This finding does not prove that either Phase 8 line is the global optimum. It records why brute-force densification of the same local method is not the preferred immediate development direction.

## Change-control rule

Phase 10 changes must be switchable or otherwise traceable so that the frozen ideal-response case can still be reproduced. When validating new physics, compare against the saved 52-control geometry first. If a later result differs with Phase 10 features disabled, treat that as a numerical regression until a documented intentional change explains it.
