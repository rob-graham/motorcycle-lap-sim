# Phase 10 Mallala R6 closure decision

**Decision date:** 2026-08-17  
**Classification:** Mallala R6 case validation / calibration deferred  
**Status:** Phase 10 closed for the present development sequence

## Purpose

Phase 10 was intended to follow the Phase 9 telemetry and finite-roll discrepancy work with a deliberately small, identifiable calibration exercise and an untouched hold-out validation. The available Mallala R6 session has now been exercised end-to-end on current `main` and provides useful case-validation evidence, but it does not support sufficiently identifiable multi-parameter calibration without a material risk of fitting unknown rider, motorcycle, track-geometry, and logger effects to one session.

The engineering decision is therefore to close Phase 10 with **substantial parameter calibration deferred** rather than force a lap-time match.

This is not a claim that the R6 reference motorcycle is calibrated. Existing motorcycle and finite-roll inputs remain explicit, replaceable, provisional parameters unless their provenance says otherwise.

## Runtime verification on current main

The closure run was performed on Windows in the project `.venv-numba` environment with Python 3.13.7 and NumPy 2.2.6.

### Full test suite

```text
python -m pytest
236 passed
```

### Frozen Phase 9 numerical baseline

```text
python scripts/r6_phase9_baseline_check.py
```

The canonical text identities reproduced:

- controls SHA-256: `5727dd1326c7892682f1d7dc1b78a67cede5c7b1c769577a6d26ae9ad564bf83`
- track SHA-256: `a213f9f15a3797ddb73f4a2a5969f0a1afa8b7dfccc4c057dd0c1e14e4e67959`
- motorcycle SHA-256: `8c3ed9d3ac13b483dd441e6d9b500ada573cd4e5679581c614768117e5f63aee`

The saved 52-control geometry reproduced the frozen Python-backend results:

- 1.00 m output spacing: `69.354897583 s`
- 0.50 m output spacing: `69.321493766 s`
- 0.25 m output spacing: `69.305349182 s`

The command ended with:

```text
historical_reference_label_status=reproduced
executable_baseline_regression_status=passed
```

### Telemetry ingestion

```text
python scripts/r6_phase10_telemetry_check.py "<R6 Mallala P4.xlsx>"
```

The supplied `Updated` sheet reproduced the expected session structure:

- 9,580 samples;
- 20 Hz nominal sampling (`0.05 s` median interval);
- five complete laps plus incomplete Lap 6;
- complete-lap sample-covered times: `73.60`, `74.20`, `73.55`, `72.65`, and `72.10 s`.

### Rigid registration and map matching

```text
python scripts/r6_phase10_registration_check.py "<R6 Mallala P4.xlsx>" --envelope-csv phase10_mallala_envelope_closure.csv
```

Key results:

- registration converged: `true`;
- iterations: `66`;
- inliers: `5912/7322`;
- RMS residual: `2.576299699 m`;
- median residual: `2.244757518 m`;
- p95 residual: `4.436600847 m`;
- all five complete laps had `backward_steps=0`;
- envelope bins: `256`;
- bins with every selected lap: `219`;
- median measured offset outside the nominal model corridor in `39/219` eligible bins;
- maximum median corridor excess: `3.955719350 m` at chainage `1902.907608296 m`.

The registered-envelope interpretation remains that consistent measured offsets outside the nominal corridor indicate local approximate-reference-geometry mismatch, not automatic rider off-track classification. No scale correction is applied to force agreement.

The closure envelope SHA-256 was:

`dcb16e7a4c62feedabddc4ecde40d8a5ae4cefbb5fa4adab756f109386faf205`

The generated envelope is a runtime artifact and is not required to be committed to the repository.

### Frozen ideal-response speed comparison

```text
python scripts/r6_phase10_speed_validation.py phase10_mallala_envelope_closure.csv --expected-complete-lap-bins 219
```

Results on the frozen 1.00 m line:

- simulated fixed-line lap: `69.354897583 s`;
- eligible complete-lap bins: `219/256`;
- mean simulation minus measured median speed: `+1.936217980 m/s`;
- median simulation minus measured median speed: `+1.554882298 m/s`;
- mean absolute speed error: `2.149122670 m/s`;
- RMS speed error: `2.792046088 m/s`;
- p95 absolute speed error: `5.644222181 m/s`;
- simulation within measured p10-p90 envelope: `35/219` bins;
- simulation above measured p90: `167/219` bins;
- simulation below measured p10: `17/219` bins.

This is a broad, spatially structured discrepancy. Because the simulated and measured trajectories differ geometrically, it is not uniquely attributable to motorcycle parameters.

### Fixed-line Level-1 finite-roll sensitivity

```text
python scripts/r6_phase10_roll_sweep.py phase10_mallala_envelope_closure.csv --expected-complete-lap-bins 219
```

The command does not fit measured roll-rate telemetry and does not calibrate a parameter. It applies the simple constant maximum roll-rate Level-1 sensitivity model to the same frozen line.

| Case | Lap time (s) | Mean sim - measured median (m/s) | MAE (m/s) | RMS (m/s) | Within p10-p90 bins |
| --- | ---: | ---: | ---: | ---: | ---: |
| Unconstrained | 69.354897583 | +1.936217980 | 2.149122670 | 2.792046088 | 35/219 |
| 0.5 rad/s | 76.893519731 | -1.243528214 | 2.940984525 | 3.674139865 | 30/219 |
| 0.7 rad/s | 73.027447317 | +0.495735995 | 1.898031177 | 2.620780984 | 52/219 |
| 0.9 rad/s | 71.334030101 | +1.183907333 | 1.869846183 | 2.504548782 | 43/219 |

The finite-roll sensitivity explains a substantial, physically plausible portion of the ideal-response/measured discrepancy without changing power, mass, drag, tyre utilisation, gearing, or track geometry. However, no single roll-rate value is preferred by all comparison metrics: `0.7 rad/s` gives the smaller mean bias and more bins inside the measured band, while `0.9 rad/s` gives the smaller MAE and RMS error. This is evidence against treating the roll-rate constant as a lap-time tuning knob.

## Engineering interpretation

The current simulator is a minimum-time / high-performance scenario model. The measured data represent one human rider, one motorcycle/session, an incompletely characterised logger installation, and an approximate non-survey track model. A total lap-time match is therefore not an appropriate calibration target by itself.

The available evidence establishes that:

1. the frozen ideal-response numerical baseline is reproducible;
2. the measured-versus-model speed discrepancy is real and spatially structured;
3. a simple finite-roll response materially changes the predicted lap and moves important comparison metrics toward the measured session;
4. the remaining discrepancy cannot be uniquely assigned to mass, power, drag, grip/utilisation, gearing/radius, rider edge margin, roll dynamics, approximate track geometry, or telemetry uncertainty; and
5. the present model behaviour is reasonable for continued development as a high-performance scenario model, while its provisional parameters remain replaceable.

## Calibration decision

No bounded multi-parameter calibration is performed in this phase.

The current dataset is insufficient to identify a small parameter subset strongly enough to justify fitting it while preserving the distinction between rider behaviour, motorcycle capability, track geometry, and telemetry effects. Simultaneously fitting mass, torque/power, drag, tyre utilisation, gearing/radius, edge margin, or roll response would create strong compensation and overfitting risk.

Accordingly:

- the R6 reference configuration remains provisional rather than calibrated;
- `max_roll_rate_radps` remains a sensitivity/scenario parameter rather than an identified R6/rider constant;
- existing defaults are not modified to force total lap-time agreement;
- the approximate Mallala geometry is not rescaled or reshaped merely to improve telemetry agreement; and
- no general motorcycle, rider, or circuit validation claim is made.

## Hold-out status

The previously documented split is retained for future re-entry:

- **Lap 5** remains the first calibration/development candidate;
- **Lap 4** remains reserved as the first untouched hold-out candidate; and
- **Laps 1-3** remain additional out-of-fit comparisons.

Because no performance-parameter calibration was performed, the hold-out has not been consumed by a fitted-model test. Lap 4 should remain unused for future fitting unless a later documented review changes the split for a stronger data-quality reason.

## Re-entry criteria for calibration

Reopen Phase 10 calibration only when enough new evidence exists to make a small parameter subset meaningfully identifiable. Suitable triggers include one or more of:

- measured combined motorcycle/rider mass and setup information;
- better measured or traceable tyre, gearing/effective-radius, power/torque, aerodynamic, or CG inputs;
- improved survey-grade/georeferenced Mallala geometry;
- clarified logger orientation/filtering and stronger roll/lean channel interpretation;
- additional sessions or riders that separate repeatable motorcycle/track effects from one-rider behaviour; or
- a specific local discrepancy that sensitivity analysis shows can be attributed to one bounded parameter family without compensating changes elsewhere.

Any future calibration must preserve a calibration/hold-out split, use bounded documented parameters, avoid fitting to total lap time alone, and report local/sector metrics before the hold-out is evaluated without further fitting.

## Phase transition

With the Phase 9/10 runtime workflow reproducible and the remaining validation gaps explicitly recorded, the project may proceed to the roadmap's Phase 11 optimisation-assurance and robust-line work.

The retained Phase 11A multistart and Phase 11B hierarchical warm-start results remain bounded evidence of warm-start/search limitations. They do not establish global optimality and do not by themselves justify reviving superseded Phase 11C/11D experiments. Any new optimisation work must satisfy the documented re-entry criteria: explicit engineering need, bounded benchmark, common-grid ranking, runtime reporting, deterministic reproduction, and meaningful tests.
