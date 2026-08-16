# Phase 9 Mallala numerical baseline freeze

## Status

Phase 8 racing-line development is closed for the present development sequence. This document freezes the numerical/software baseline to be preserved while Phase 10 telemetry, validation, and roll-response work is added.

The provenance is intentionally split into distinct references:

- `software_source_commit`: `300fd58fff97a46d8152422cb1d19077af091a44` — the pre-Phase-9 source state associated with the historical Phase 8 work;
- `artifact_first_committed_in`: `67fc56c1508ef1ad7fd267d4ff3ea23a92923a7b` — the merge that first committed the recovered controls and this freeze documentation;
- `baseline_package_commit`: `f8917f6be7ac06776ccfeab28a7418636958e2bd` — a source state containing the recovered controls, track/motorcycle inputs and executable fixed-geometry reproduction command; and
- `historical_execution_commit`: unknown — the final external run log for the recovered 69.354897583 s control artifact was not retained, so no source commit is invented for it.

Machine-readable provenance and SHA-256 identities are retained in:

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

The artifact supplied for the freeze is identified by the recovered historical run label **69.354897583 s** and contains 52 controls. The CSV body contains stations, controls and bounds; it does not contain the final lap time or the complete final-run convergence/evaluation metadata. Therefore **69.354897583 s remains a recovered historical result label until the fixed-geometry repository check reproduces it**. This qualification is deliberate and must not be replaced by invented provenance.

The recovered controls include two values exactly at their lower local bounds; that fact is retained rather than hidden and should be considered when interpreting later geometry/physics comparisons.

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

On 16 August 2026 the complete repository test suite passed after the telemetry dependency was installed. Later Phase 10 integration hardening increased the suite further; the current full-suite result must be recorded with the pull request that completes this baseline reproduction work.

The software-test result alone does not establish the recovered 69.354897583 s label as an executable numerical regression. That status depends on the command below and the values subsequently recorded in the manifest.

## Reproduction commands

Run the full test suite first:

```bash
python -m pytest -q
```

Then evaluate the saved geometry without running optimisation:

```bash
python scripts/r6_phase9_baseline_check.py
```

The command:

1. verifies the exact five-column CSV schema;
2. requires exactly 52 controls;
3. regenerates the reference-policy stations (100 m maximum spacing / 45 degree maximum arc-heading change);
4. regenerates the 0.25 m-margin control bounds and checks every stored station/bound/control against them;
5. verifies the recovered controls SHA-256;
6. prints SHA-256 identities for the track and motorcycle inputs;
7. evaluates the same saved geometry at 1.00, 0.50 and 0.25 m output spacing; and
8. prints lap time and geometric metrics without performing a fresh optimisation.

A fresh optimisation is not expected to reproduce the historical local-search path unless all settings and start conditions are duplicated.

Until the first execution of this command is reviewed, `phase9_baseline_manifest.json` intentionally records `status: pending_first_executable_reproduction` and does not populate executable lap-time tolerances. If the 1.0 m result reproduces 69.354897583 s, that value can become the executable regression target with an explicit tolerance. If it does not, the manifest and tests must freeze the actually reproducible current-input result separately while retaining 69.354897583 s only as the recovered historical label.

## Frozen result package requirements

A retained baseline run should record, without suppressing warnings, repository/source provenance, Python/package environment, track and motorcycle input hashes, selected control policy, cold-start/restart status, speed backend and workers, search settings, control count, initial/final lap time, evaluations/sweeps, termination reason, path length, minimum boundary clearance, minimum forward progress, speed/gear/RPM and acceleration summary, fixed-spline output-resolution checks, exported controls, and plots.

Not all of that metadata is recoverable for the historical 69.354897583 s external run. The freeze therefore distinguishes historical provenance from executable repository regression rather than fabricating missing fields.

## Optimisation-assurance finding retained from Phase 8

The extra-fine exercise is retained as an optimisation-assurance finding, not as a reason to redesign the optimiser immediately. Increasing control count and driving the deterministic local pattern search toward exhaustive local convergence is computationally expensive and, by itself, does not establish a globally or robustly optimal physically relevant racing line.

Therefore:

1. retain the recovered 52-control artifact as the representative Phase 10 ideal-response geometry;
2. retain 69.354897583 s as its historical run label unless/until the executable check verifies that exact value;
3. retain the current deterministic pattern search as the reference optimiser for the immediate telemetry/physics work;
4. preserve the 96-control result and its run settings as optimisation-assurance evidence;
5. do not make an even denser/exhaustive local search the default next step; and
6. defer systematic multistart, cross-resolution ranking, measured-line starts, perturbation starts, and any independent benchmark optimiser to the later optimisation-assurance phase.

This finding does not prove that either Phase 8 line is the global optimum. It records why brute-force densification of the same local method is not the preferred immediate development direction.

## Change-control rule

Phase 10 changes must be switchable or otherwise traceable so that the frozen ideal-response case can still be evaluated. Compare new physics against the saved 52-control geometry first. Once the executable baseline values and tolerances are recorded in the manifest, any later difference with Phase 10 features disabled is a numerical regression until a documented intentional change explains it.
