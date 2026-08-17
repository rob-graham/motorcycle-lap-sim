# Simulation project context and source hierarchy

**Status date:** 2026-08-17  
**Repository:** `rob-graham/motorcycle-lap-sim`

This document brings the simulation-relevant project context from the external project index and roadmap into the repository so that code and pull-request reviews do not depend on chat history or separately supplied files.

It is derived from:

- `PROJECT_INDEX_v1.1.md` (project index version 1.1, status 2026-08-16); and
- `motorcycle_racing_line_project_approach_v2.1.md` (authoritative project approach version 2.1, status 2026-08-16).

The repository remains the source of truth for **implemented behaviour**. This document records project direction, evidence hierarchy, modelling boundaries, and the intended relationship between the simulator and the wider track-layout project.

## 1. Source hierarchy

When sources overlap, use the following order for simulation work:

1. **Current project direction and development decisions:** `motorcycle_racing_line_project_approach_v2.1.md`.
2. **Implemented simulator behaviour:** current repository code, tests, configuration, cases, and documentation.
3. **Measured Mallala validation data:** `R6 Mallala P4.xlsx`.
4. **Published standards/reference documents:** supplied Motorcycling Australia road-race track standards, subject to separate verification of current applicability.
5. **Published technical precedent:** the supplied Wanneroo safety assessment, as precedent rather than governing authority.
6. **Internal run-off calculations:** provisional working material, not accepted external criteria.

The roadmap says where the software should go; this repository says what it actually does.

## 2. Project purpose and constraints relevant to this repository

The simulator is being developed as part of a motorcycle track-layout and safety-design workflow. The immediate objective is to develop, test, document, and internally review a technically defensible method before approaching Motorcycling Australia, insurers, certifiers, or other agencies.

Important project constraints are:

- SI units are used internally.
- Important default parameters may be used during development, but must be explicit, versioned, traceable, and replaceable.
- Mallala and the available R6 telemetry are the first validation case.
- The R6 motorcycle, rider, logger installation, and session are incompletely characterised; results are therefore case-specific and provisional.
- Measured telemetry is validation evidence, not an optimisation target to be fitted blindly.
- A total lap-time match is not sufficient validation. Local speed, trajectory, acceleration, braking, lean/roll transition, gear/RPM, and spatial consistency matter.
- The simulator should remain a minimum-time / high-performance scenario model, not a predictor of one particular rider.
- No repository output currently claims regulatory approval, homologation, certification, or insurance acceptance.

## 3. Architecture boundary

The simulation work should preserve the existing modular separation of:

- track geometry and boundaries;
- racing-line/path geometry;
- motorcycle configuration and physical limits;
- fixed-path minimum-time speed solution;
- racing-line optimisation;
- telemetry import, registration, map matching, and validation;
- plotting and diagnostics.

Measured data must not become a hidden dependency of the physics solver.

The wider project also contains separate future workstreams for three-dimensional track-surface/LiDAR processing, GIS export, coaching-event extraction, and run-off calculations. The run-off calculation is intended to remain a separate package with a defined, versioned input contract from this simulator.

## 4. Roadmap phases relevant to the simulator

### Phase 9 - Baseline freeze, Mallala telemetry validation and initial roll-response model

The project approach defines Phase 9 as the immediate Mallala physics/validation milestone. Its purpose is to understand the approximately 1:09 ideal-response simulation versus approximately 1:12 measured R6 laps before materially calibrating uncertain motorcycle parameters.

The intended sequence is:

- **9A baseline freeze:** preserve a reproducible ideal-response Mallala line and fixed-path result with new handling features disabled;
- **9B telemetry ingestion/map matching:** import and clean the R6 workbook, detect laps, register/map-match GPS data, and compare simulation and measurement by chainage;
- **9C roll-channel assessment:** assess sign, bias, filtering, timing, and physical plausibility of the logged roll-rate signal before using it as a model constraint;
- **9D Level-1 demanded-lean / roll-rate model:** implement a switchable, physically interpretable planar lean/roll-rate constraint;
- **9E fixed-line roll sensitivity:** apply finite roll response to the frozen line without re-optimising, isolating the physics effect;
- **9F roll-aware re-optimisation:** allow the racing-line optimiser to respond to finite roll capability; and
- **9G discrepancy decomposition:** compare ideal, fixed-line roll, roll-aware optimum, and measured R6 behaviour spatially and by sector.

The Level-1 planar demand is based on steady lean

`phi_d = atan(v^2 * kappa / g)`

with a trajectory-driven roll-rate demand. The implemented production sensitivity constraint presently uses the curvature-transition component of demanded roll rate. It is deliberately a simple Level-1 model, not a full steering/rider dynamics model. The constant `max_roll_rate_radps` is a replaceable scenario parameter, not an identified R6/rider constant.

A more complex dynamic lean/roll model is explicitly deferred unless the simpler model leaves a systematic, decision-relevant discrepancy that suitable telemetry can identify.

### Phase 10 - Mallala parameter calibration and hold-out validation

The roadmap places substantial motorcycle-parameter calibration **after** the Phase 9 roll-response assessment to reduce the risk of compensating for missing handling physics by incorrectly changing power, grip, mass, drag, or related parameters.

If calibration is pursued, it should:

- use a deliberately small identifiable parameter subset;
- use bounded parameters and documented defaults;
- preserve a calibration/hold-out split;
- re-run hold-out laps without further fitting;
- avoid using roll-response parameters merely as lap-time tuning knobs; and
- report local and sector validation metrics, not just total lap time.

Candidate post-roll parameters include combined mass, torque/power scale, drag area, longitudinal/lateral utilisation, rolling-radius/gearing correction, and rider edge margin. These parameters can compensate for one another, so they must not all be fitted simultaneously.

The correct classification is **Mallala R6 case calibration/validation**, not general motorcycle or track validation.

### Phase 11 - Optimisation assurance and robust line generation

The roadmap contains a later assurance phase because a rider-facing or safety-analysis line should not be accepted merely because one local optimiser run produced it. It calls for multiple starts, common-grid ranking, resolution and margin sensitivity, and eventually an independent benchmark method.

Repository Phase 11A-C scripts are **diagnostic assurance experiments**, not a replacement production optimiser and not evidence of global optimality. They were used to demonstrate practical warm-start/search limitations of the current deterministic coordinate-pattern method. They should not be allowed to distract from the primary Phase 9/10 validation work.

### Later simulation-facing phases

The roadmap subsequently proposes:

- formal coaching/event extraction from solved speed/line/acceleration/lean/gear state;
- optional advanced roll dynamics only if evidence requires it;
- a reusable `TrackSurface` / `z(s,n)` LiDAR/DEM interface;
- three-dimensional lap simulation with grade and banking; and
- versioned GIS/run-off exports.

These are future interfaces, not claims about current implemented capability unless repository code and tests say otherwise.

## 5. Current repository status relevant to review

As of the status date, the repository includes substantially more than the original Phase 8 documentation described:

- a frozen Mallala 52-control numerical baseline and executable regression check;
- telemetry import/quality tooling, rigid registration, map matching, peer/repeatability diagnostics, and speed-comparison utilities;
- a Level-1 demanded-lean/roll-rate implementation and optional finite-roll speed constraint;
- fixed-line finite-roll sensitivity and roll-aware racing-line re-optimisation scripts;
- sector/spatial diagnostics for comparing ideal, finite-roll, and measured behaviour;
- trajectory export with speed, accelerations, lean, roll demand/ceiling, limits, gear and RPM;
- optimisation-assurance diagnostics using multiple starts and alternative warm-start/search representations; and
- an optional Numba fixed-path backend while retaining the Python solver as the authoritative reference.

Detailed numerical baseline provenance is in [`phase9_baseline_freeze.md`](phase9_baseline_freeze.md). Telemetry source-quality findings are in [`mallala_r6_telemetry_integrity.md`](mallala_r6_telemetry_integrity.md). The implemented architecture is summarised in [`system_spec.md`](system_spec.md).

## 6. Current evidence and interpretation boundaries

For review, retain these distinctions:

- The frozen ideal-response Mallala result is a numerical/software baseline, not proof of physical accuracy.
- The approximate Mallala track is adequate for present development comparison but is not survey-grade and must not be silently rescaled to force telemetry agreement.
- The provisional R6 configuration is not a fully identified motorcycle/rider model.
- Finite roll response is a sensitivity/model-development feature. A value such as `0.8 rad/s` must be labelled provisional/uncalibrated unless separately supported by evidence.
- Re-optimising with roll active may improve minimum calculated lap time without improving agreement with a particular measured rider line; this is not automatically a failure of the minimum-time model.
- Optimisation spread from centreline or perturbed starts is evidence about search convergence/warm-start dependence unless credible local convergence has been established; it must not be presented as physical model uncertainty.
- The measured rider trajectory and speed are evidence for validation and plausibility, not a target that the optimiser should reproduce exactly.

## 7. Review and change-control expectations

Reviews should be able to answer four separate questions:

1. **Numerical regression:** with optional new physics disabled, does the frozen baseline remain reproducible?
2. **Physics effect:** with a new model enabled on a fixed path, what changes and where?
3. **Optimisation response:** after re-optimisation, what additional change comes from path adaptation rather than the physics feature itself?
4. **Validation evidence:** do spatial and channel-level comparisons with the Mallala telemetry become more physically plausible, and what discrepancies remain?

Do not collapse these questions into one total-lap-time comparison.

Long-running optimisation diagnostics must record initial state, controls/policy, spacing, boundary margin, backend/workers, evaluation/sweep limits, termination reason, final step, and final fixed-grid re-evaluation. Saved controls and configuration hashes are preferable to relying on rerunning a local optimiser to reproduce a historical path.

## 8. Interfaces to future track-design work

The simulator is expected eventually to provide rider-facing and safety-analysis outputs such as braking onset/release, turn-in, apex regions, throttle pickup, exit, maximum speed, gear/RPM, lean and roll-transition zones.

For run-off analysis, event output should support multiple departure scenarios rather than assuming the nominal braking point is always worst. Expected future seeds include missed braking/upright overrun, braking-zone departures, entry lowside, exit highside, forced-line/envelope cases, and regular tangentials for standards comparison.

These future outputs should include provenance, scenario identity, quality/confidence flags, and local/GIS coordinates once georeferencing is available.
