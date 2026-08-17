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
- coaching/event extraction; and
- plotting and diagnostics.

Measured data must not become a hidden dependency of the physics solver. Coaching/event extraction consumes solved trajectory state; it does not alter the physics or optimiser.

The wider project also contains separate future workstreams for three-dimensional track-surface/LiDAR processing, GIS export, and run-off calculations. The run-off calculation remains intended as a separate package. Its simulator input contract is intentionally not frozen until Phase 12A event locations have been reviewed around Mallala.

## 4. Roadmap phases relevant to the simulator

### Phase 9 - Baseline freeze, Mallala telemetry validation and initial roll-response model

The project approach defines Phase 9 as the Mallala physics/validation milestone used to understand the approximately 1:09 ideal-response simulation versus approximately 1:12 measured R6 laps before materially calibrating uncertain motorcycle parameters.

The implemented sequence covers baseline freeze, telemetry ingestion/map matching, roll-channel assessment, a Level-1 demanded-lean / roll-rate model, fixed-line roll sensitivity, roll-aware re-optimisation, and spatial discrepancy decomposition.

The Level-1 planar demand is based on steady lean

`phi_d = atan(v^2 * kappa / g)`

with a trajectory-driven roll-rate demand. The implemented production sensitivity constraint presently uses the curvature-transition component of demanded roll rate. It is deliberately a simple Level-1 model, not a full steering/rider dynamics model. The constant `max_roll_rate_radps` is a replaceable scenario parameter, not an identified R6/rider constant.

A more complex dynamic lean/roll model is explicitly deferred unless the simpler model leaves a systematic, decision-relevant discrepancy that suitable telemetry can identify.

### Phase 10 - Mallala parameter calibration and hold-out validation

The full Mallala runtime chain has been exercised on current `main` and the result is recorded in [`phase10_mallala_closure.md`](phase10_mallala_closure.md). The engineering decision is to close Phase 10 for the present sequence with **substantial parameter calibration deferred for identifiability reasons**.

The current evidence shows that the frozen ideal-response case is reproducible, the measured-versus-model discrepancy is real and spatially structured, and finite-roll sensitivity explains a substantial physically plausible portion of that discrepancy without changing motorcycle performance parameters. The remaining difference cannot be uniquely assigned to mass, power, drag, grip/utilisation, gearing/radius, rider edge margin, roll response, approximate track geometry, or telemetry uncertainty strongly enough to justify fitting a bounded parameter subset from this one session.

Accordingly:

- the R6 reference model remains provisional rather than calibrated;
- `max_roll_rate_radps` remains a sensitivity/scenario parameter rather than an identified constant;
- defaults are not changed merely to force total lap-time agreement;
- Lap 5 remains the first future calibration/development candidate;
- Lap 4 remains the first untouched hold-out candidate; and
- Laps 1-3 remain additional out-of-fit comparisons.

The correct classification remains **Mallala R6 case validation**, not general motorcycle or track validation.

### Phase 11 - Optimisation assurance and representative line

Phase 11 is closed for the present Mallala sequence; see [`phase11_optimisation_assurance_closure.md`](phase11_optimisation_assurance_closure.md).

The closure used bounded multistart/control-basis/station-relocation evidence and a final common-grid representative-line diagnostic. Four retained candidate lines were re-evaluated at 0.125 m with the authoritative Python backend, 0.250 m margin and 0.8 rad/s finite-roll sensitivity.

The final retained representative is `reduced_reoptimised_51` at 71.396583646 s. The eligible geometric medoid was rejected by the explicit 0.050 s representative lap-time guardrail, causing the selection rule to fall back to the fastest eligible line. The retained line is feasible and optimisation-assured for the present engineering task, but this is not a global-optimality claim.

The optimiser/control-basis spread remains numerical sensitivity. It is not a physical uncertainty interval, rider-variability envelope, run-off corridor, or regulatory criterion.

No further optimiser run is required before Phase 12A unless a later result identifies a specific engineering need. Superseded Phase 11C/11D experiments must not be revived merely because the optimiser is warm-start dependent.

### Phase 12A - Coaching-event extraction and visual review

Phase 12A is the current active development task.

It uses the retained Phase 11 representative controls, reproduces the line on the authoritative scenario/grid, extracts deterministic rider-facing landmarks, writes a machine-readable event table, and adds the coaching marks to a clean racing-line image.

The first map includes rider-facing marks for:

- braking onset;
- brake release;
- turn-in;
- geometric apex;
- positive-drive/throttle pickup; and
- corner exit.

Additional event-table information includes local maximum speed, maximum braking, speed apex, maximum lean, gear shifts and roll-transition landmarks. Every event carries its source rule and a confidence/quality flag.

The coaching image deliberately excludes optimiser control points, optimiser/control-basis spread, margin-corridor lines, centreline diagnostics and convergence information. It is intended to show rider information rather than optimisation diagnostics.

See [`coaching_event_definitions.md`](coaching_event_definitions.md).

Phase 12A is not closed until the generated event locations have been visually inspected around Mallala. The simulator-to-run-off export contract must not be defined before that review.

### Later simulation-facing phases

The roadmap subsequently proposes:

- optional advanced roll dynamics only if evidence requires it;
- a reusable `TrackSurface` / `z(s,n)` LiDAR/DEM interface;
- three-dimensional lap simulation with grade and banking;
- georeferenced GIS output; and
- a separate run-off package consuming a versioned simulator result contract designed after Phase 12A review.

These remain future interfaces, not current capability claims unless repository code and tests say otherwise.

## 5. Current repository status relevant to review

As of the status date, the repository includes:

- a frozen Mallala 52-control numerical baseline and executable regression check;
- telemetry import/quality tooling, rigid registration, map matching, peer/repeatability diagnostics, and speed-comparison utilities;
- a Level-1 demanded-lean/roll-rate implementation and optional finite-roll speed constraint;
- fixed-line finite-roll sensitivity and roll-aware racing-line re-optimisation scripts;
- sector/spatial diagnostics for comparing ideal, finite-roll, and measured behaviour;
- trajectory export with speed, accelerations, lean, roll demand/ceiling, limits, gear and RPM;
- an explicit Phase 10 closure record preserving the runtime validation evidence and calibration-deferral decision;
- completed Phase 11 representative-line / optimiser-spread assurance, retaining `reduced_reoptimised_51`; and
- Phase 12A coaching-event extraction code and rider-facing map generation, pending end-to-end Mallala execution and visual review.

Detailed numerical baseline provenance is in [`phase9_baseline_freeze.md`](phase9_baseline_freeze.md). Telemetry source-quality findings are in [`mallala_r6_telemetry_integrity.md`](mallala_r6_telemetry_integrity.md). Phase 10 closure evidence is in [`phase10_mallala_closure.md`](phase10_mallala_closure.md). Phase 11 closure evidence is in [`phase11_optimisation_assurance_closure.md`](phase11_optimisation_assurance_closure.md). The coaching rules are in [`coaching_event_definitions.md`](coaching_event_definitions.md). The implemented architecture is summarised in [`system_spec.md`](system_spec.md).

## 6. Current evidence and interpretation boundaries

For review, retain these distinctions:

- The frozen ideal-response Mallala result is a numerical/software baseline, not proof of physical accuracy.
- The approximate Mallala track is adequate for present development comparison but is not survey-grade and must not be silently rescaled to force telemetry agreement.
- The provisional R6 configuration is not a fully identified motorcycle/rider model.
- Finite roll response is a sensitivity/model-development feature. A value such as `0.8 rad/s` must be labelled provisional/uncalibrated unless separately supported by evidence.
- Re-optimising with roll active may improve minimum calculated lap time without improving agreement with a particular measured rider line; this is not automatically a failure of the minimum-time model.
- Optimiser/control-basis spread is evidence about numerical search sensitivity and must not be presented as physical model uncertainty.
- The measured rider trajectory and speed are evidence for validation and plausibility, not a target that the optimiser should reproduce exactly.
- Coaching marks are model-derived rider references, not universally safe braking or cornering markers.

## 7. Review and change-control expectations

Reviews should be able to answer four separate questions:

1. **Numerical regression:** with optional new physics disabled, does the frozen baseline remain reproducible?
2. **Physics effect:** with a new model enabled on a fixed path, what changes and where?
3. **Optimisation response:** after re-optimisation, what additional change comes from path adaptation rather than the physics feature itself?
4. **Validation/presentation evidence:** are derived event locations and rider-facing outputs spatially plausible, traceable to explicit rules, and consistent with the solved trajectory?

Do not collapse these questions into one total-lap-time comparison.

Long-running optimisation diagnostics must record initial state, controls/policy, spacing, boundary margin, backend/workers, evaluation/sweep limits, termination reason, final step, and final fixed-grid re-evaluation. Saved controls and configuration hashes are preferable to relying on rerunning a local optimiser to reproduce a historical path.

## 8. Interface boundary before run-off work

The simulator now begins to provide rider-facing outputs such as braking onset/release, turn-in, apex, positive-drive pickup, exit, maximum speed, gear/RPM, lean and roll-transition landmarks.

That does **not** yet freeze the run-off interface. The current event semantics must first be tested numerically and visually on Mallala. Only after that review should the project decide which trajectory fields, event types, provenance, confidence flags, coordinate systems, scenario metadata and later departure seeds belong in a versioned simulator-to-run-off contract.

This sequencing is intentional: the downstream interface should reflect reviewed event semantics rather than make Phase 12A conform to a prematurely fixed schema.
