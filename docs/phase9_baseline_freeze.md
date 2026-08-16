# Phase 9 Mallala numerical baseline freeze

## Status

Phase 8 racing-line development is closed for the present development sequence. This document freezes the numerical/software baseline to be preserved while Phase 10 telemetry, validation, and roll-response work is added.

The provenance is intentionally split into distinct references:

- `software_source_commit`: `300fd58fff97a46d8152422cb1d19077af091a44` — the pre-Phase-9 source state associated with the historical Phase 8 work;
- `artifact_first_committed_in`: `67fc56c1508ef1ad7fd267d4ff3ea23a92923a7b` — the merge that first committed the recovered controls and this freeze documentation;
- `baseline_package_commit`: the Phase 9/10 integration-hardening branch state containing the recovered controls, frozen input hashes, executable fixed-geometry reproduction command and numerical regression checks; and
- `historical_execution_commit`: unknown — the final external run log for the recovered 69.354897583 s control artifact was not retained, so no source commit is invented for it.

Machine-readable provenance, hashes, numerical values and tolerances are retained in:

`cases/mallala_r6/baseline/phase9_baseline_manifest.json`

The freeze uses:

- track: `examples/tracks/mallala_reference.yaml` (Mallala centreline fit v0.3);
- motorcycle: `examples/motorcycles/r6_2017plus_reference.yaml`;
- recovered controls: `cases/mallala_r6/baseline/phase8_reference_controls.csv`;
- executable fixed-geometry check: `scripts/r6_phase9_baseline_check.py`;
- ideal handling response, with the provisional curvature-transient proxy disabled unless a comparison explicitly says otherwise; and
- SI units internally.

The track is the existing approximate, simulator-local Mallala reference geometry. It is not survey-grade, but engineering review considers it a good representation of Mallala for the present validation work. It must not be silently rescaled to force agreement with telemetry.

## Representative 52-control reference policy

The 52-control `reference` policy is the representative Phase 9 racing-line baseline for Phase 10 comparison because, on engineering review, it is considered more plausible/representative than the more aggressively flexible 96-control result. That judgement is a project decision, not a proof that 52 controls are physically optimal.

The recovered final reference control artifact is retained as:

`cases/mallala_r6/baseline/phase8_reference_controls.csv`

Its SHA-256 is:

`2290d07de682fa0ced7701d6cfb6f8459a9e0a96bfd662b0f37c931b8ea5d368`

The historical external run identified this artifact as **69.354897583 s**. The CSV body itself contains stations, controls and bounds, not the historical final-run log. On 16 August 2026 the repository fixed-geometry reproduction command independently evaluated the same committed artifact and reproduced **69.354897583 s exactly at 1.00 m output spacing**. The historical label is therefore now also the executable repository regression value, while the historical execution commit and unrecovered convergence/evaluation metadata remain unknown.

The recovered controls include two values exactly at their lower local bounds; that fact is retained rather than hidden and should be considered when interpreting later geometry/physics comparisons.

## Executable numerical baseline

With the canonical committed inputs and Python speed backend, the fixed saved geometry evaluates to:

| Output spacing | Lap time (s) | Path length (m) | Minimum boundary clearance (m) | Offset min/max (m) | Curvature min/max (1/m) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1.00 m | 69.354897583 | 2510.660863823 | 0.000014708 | -3.749985292 / 4.749689886 | -0.101362936 / 0.027468564 |
| 0.50 m | 69.321493766 | 2510.660863823 | 0.000014708 | -3.749985292 / 4.749689886 | -0.101793650 / 0.027609254 |
| 0.25 m | 69.305349182 | 2510.660863823 | 0.000014708 | -3.749985292 / 4.749689886 | -0.101472758 / 0.027597204 |

The canonical input identities are:

- controls SHA-256: `2290d07de682fa0ced7701d6cfb6f8459a9e0a96bfd662b0f37c931b8ea5d368`;
- track SHA-256: `f419ee6e6e48f92b1d884223052f2db4411e6ea2a75c7e9a1e9e1c10f04cb787`;
- motorcycle SHA-256: `1cd7d74a272b2dd8b42339fd0e85e46f3dc67485696aba772e901bdc27922832`.

The executable check fails closed if those canonical hashes change or if lap time, path length, clearance, projected offsets or curvature extrema exceed the explicit tolerances recorded in the manifest/script. These tolerances are numerical-regression tolerances, not estimates of physical model uncertainty.

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

It must not be confused with the verified 69.354897583 s final reference artifact.

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

The 96-control search was reached through multiple long restart stages. The important finding is not simply its calculated lap time. It is slower than the verified 52-control 69.354897583 s artifact despite substantially greater path freedom and search burden. The extra freedom also produced a more aggressive line, very small boundary clearances, continuing path-model/output-resolution sensitivity, and a large search burden. Therefore, more controls plus exhaustive local refinement are not by themselves a practical optimisation-assurance strategy.

The extra-fine diagnostic remains useful for comparison against later roll-aware optimisation, checking whether new physics changes the incentive for high-frequency path shaping, optimisation-assurance/multistart work, and identifying possible geometry/solver exploitation. It must not silently replace the 52-control reference line as the representative Phase 10 baseline.

## Verification status

On 16 August 2026 the integration-hardening branch full suite passed:

`207 passed in 55.13 s`

The same validation session established the executable Phase 9 numerical baseline above. The test suite now includes an end-to-end fixed-geometry regression check that loads the committed controls, validates their identity/stations/bounds, verifies track and motorcycle hashes, and evaluates all three documented output spacings.

## Reproduction commands

Run the full test suite:

```bash
python -m pytest -q
```

Then evaluate and verify the saved geometry without running optimisation:

```bash
python scripts/r6_phase9_baseline_check.py
```

For the canonical controls/track/motorcycle and Python backend the command must end with:

```text
historical_reference_label_status=reproduced
executable_baseline_regression_status=passed
```

A fresh optimisation is not expected to reproduce the historical local-search path unless all settings and start conditions are duplicated. Numerical change control is therefore based on re-evaluating the frozen saved geometry, not rerunning the optimiser.

## Frozen result package requirements

A retained baseline run should record, without suppressing warnings, repository/source provenance, Python/package environment, track and motorcycle input hashes, selected control policy, cold-start/restart status, speed backend and workers, search settings, control count, initial/final lap time, evaluations/sweeps, termination reason, path length, minimum boundary clearance, minimum forward progress, speed/gear/RPM and acceleration summary, fixed-spline output-resolution checks, exported controls, and plots.

Not all of that metadata is recoverable for the historical 69.354897583 s external optimisation run. The freeze therefore distinguishes historical optimisation provenance from the independently verified executable fixed-geometry repository regression rather than fabricating missing fields.

## Optimisation-assurance finding retained from Phase 8

The extra-fine exercise is retained as an optimisation-assurance finding, not as a reason to redesign the optimiser immediately. Increasing control count and driving the deterministic local pattern search toward exhaustive local convergence is computationally expensive and, by itself, does not establish a globally or robustly optimal physically relevant racing line.

Therefore:

1. retain the verified 52-control 69.354897583 s fixed-geometry case as the representative Phase 10 ideal-response executable baseline;
2. retain the current deterministic pattern search as the reference optimiser for the immediate telemetry/physics work;
3. preserve the 96-control result and its run settings as optimisation-assurance evidence;
4. do not make an even denser/exhaustive local search the default next step; and
5. defer systematic multistart, cross-resolution ranking, measured-line starts, perturbation starts, and any independent benchmark optimiser to the later optimisation-assurance phase.

This finding does not prove that either Phase 8 line is the global optimum. It records why brute-force densification of the same local method is not the preferred immediate development direction.

## Change-control rule

Phase 10 changes must be switchable or otherwise traceable so that the frozen ideal-response case remains reproducible. Compare new physics against the saved 52-control geometry first. Any later difference outside the documented regression tolerances with Phase 10 features disabled is a numerical regression until a documented intentional change explains it.
