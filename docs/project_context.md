# Simulation project context and source hierarchy

**Status date:** 2026-08-18  
**Repository:** `rob-graham/motorcycle-lap-sim`

This document brings the simulation-relevant project context from the external project index and roadmap into the repository so that code and pull-request reviews do not depend on chat history or separately supplied files.

It is derived from `PROJECT_INDEX_v1.1.md` and `motorcycle_racing_line_project_approach_v2.1.md`. The repository remains the source of truth for implemented behaviour; the roadmap remains the source of truth for project direction.

## 1. Source hierarchy

When sources overlap, use the following order for simulation work:

1. current project direction and development decisions: `motorcycle_racing_line_project_approach_v2.1.md`;
2. implemented simulator behaviour: current repository code, tests, configuration, cases, and documentation;
3. measured Mallala validation data: `R6 Mallala P4.xlsx`;
4. supplied published standards/reference documents, subject to separate verification of current applicability;
5. published technical precedent such as the supplied Wanneroo safety assessment; and
6. internal run-off calculations, which remain provisional working material rather than accepted external criteria.

The roadmap says where the software should go; this repository says what it actually does.

## 2. Project purpose and constraints

The simulator is being developed as part of a motorcycle track-layout and safety-design workflow. The immediate objective is to develop, test, document, and internally review a technically defensible method before approaching Motorcycling Australia, insurers, certifiers, or other agencies.

Important constraints are:

- SI units are used internally;
- defaults may be used during development but must remain explicit, versioned, traceable, and replaceable;
- Mallala and the available R6 telemetry are the first validation case;
- the R6 motorcycle, rider, logger installation, and session are incompletely characterised;
- measured telemetry is validation evidence, not an optimisation target to be fitted blindly;
- total lap-time agreement is not sufficient validation by itself;
- the simulator remains a minimum-time / high-performance scenario model, not a predictor of one particular rider; and
- no repository output claims regulatory approval, homologation, certification, or insurance acceptance.

## 3. Architecture boundary

Preserve the modular separation of track geometry and boundaries, racing-line/path geometry, motorcycle configuration and limits, fixed-path speed solution, racing-line optimisation, telemetry/validation, coaching/event extraction, and plotting/diagnostics.

Measured data must not become a hidden dependency of the physics solver. Coaching/event extraction consumes solved state and does not alter physics or optimisation.

The wider project also contains future workstreams for three-dimensional track-surface/LiDAR processing, GIS export, and run-off calculations. The physical run-off calculation remains a separate package. Phase 12A is closed; Phase 12B producer export target-machine acceptance succeeded and PR #80 was merged after independent review.

## 4. Closed simulator phases

### Phase 9 - Baseline freeze, Mallala telemetry validation and initial roll-response model

Phase 9 established the reproducible Mallala numerical baseline, telemetry ingestion/map matching, Level-1 demanded-lean / roll-rate sensitivity, fixed-line roll sensitivity, roll-aware re-optimisation, and spatial discrepancy decomposition.

The implemented production finite-roll sensitivity uses the curvature-transition component of demanded roll rate. `max_roll_rate_radps` is a replaceable scenario parameter, not an identified R6/rider constant.

### Phase 10 - Mallala parameter calibration and hold-out validation

Phase 10 is closed for the present sequence; see [`phase10_mallala_closure.md`](phase10_mallala_closure.md). Substantial R6 parameter calibration is deferred for identifiability reasons. The current R6 model remains provisional, and defaults are not changed merely to force lap-time agreement.

Lap 5 remains the first future calibration/development candidate, Lap 4 the first untouched hold-out candidate, and Laps 1-3 additional out-of-fit comparisons.

### Phase 11 - Optimisation assurance and representative line

Phase 11 is closed; see [`phase11_optimisation_assurance_closure.md`](phase11_optimisation_assurance_closure.md).

The retained representative is `reduced_reoptimised_51` at `71.396583646 s`, evaluated on the 0.125 m authoritative Python common grid with 0.250 m margin and 0.8 rad/s finite-roll sensitivity. It is an optimisation-assured feasible local solution for the present engineering task, not a global-optimality claim.

The optimiser/control-basis spread is numerical sensitivity, not a rider-variability envelope, physical uncertainty interval, run-off corridor, or regulatory criterion.

### Phase 12A - Coaching-event extraction and visual review

Phase 12A is closed for the present Mallala engineering sequence; see [`phase12a_coaching_event_closure.md`](phase12a_coaching_event_closure.md) and [`coaching_event_definitions.md`](coaching_event_definitions.md).

The final target-machine acceptance run reproduced the retained Phase 11 line exactly at `71.396583646 s`, retained the generic 12 raw lean regions, consolidated them to nine Mallala nominal corners, and generated the clean rider overview, speed map, regional engineering detail plots, event/provenance CSVs, and longitudinal limit-state diagnostic.

The accepted rider-facing vocabulary is BRK, TURN, APEX, DRIVE, and EXIT. Engineering-only information includes MAX-BRK, REL, VMIN, K-MAX, maximum lean, roll transitions, gear shifts, trail-braking proxy, and model capability limit states.

Important closure caveats are:

- these are prototype engineering/model-derived events, not validated rider instruction or measured controls;
- DRIVE is not throttle position and REL is not measured brake-lever release;
- trail braking is a simulation-derived proxy;
- wheelie/stoppie/traction/power classifications are model capability diagnostics;
- rider-facing use would benefit from review by several experienced riders; and
- the retained Phase 11 trajectory is an engineering simulation path, **not a recommended riding line**.

In particular, the retained closed-loop line bends toward the centre of the start/finish straight. That is a known modelling/optimisation artefact and must not be presented to riders as the recommended line. It does not block the run-off calculation work.

## 5. Current active task - 2D LOWSIDE RIDER workflow through GIS/mapping

Phase 12A's visual gate has passed, and the versioned `0.1.0` in-memory contract and deterministic retained-case producer bundle have passed target-machine acceptance. Wider physical run-off propagation and GIS generation remain downstream work.

The interface is driven by the separate run-off package's engineering needs rather than by rider-facing presentation. The current generic optimise-command task is a release/productisation interface around the existing direct-planar method; it adds no optimisation assurance and does not replace the retained Mallala line.

At minimum, the work should consider:

- path and track chainage conventions;
- local/global coordinates and track-boundary geometry;
- speed and longitudinal/lateral acceleration;
- path curvature and demanded lean/roll state where relevant;
- gear/RPM only where it provides downstream value;
- reviewed corner/event provenance and confidence;
- model capability / limit-state flags where useful;
- scenario/model/configuration identity; and
- explicit separation between simulator results and run-off calculation assumptions.

The run-off contract must not silently convert coaching marks into safety criteria. Departure seeds should be derived and justified for run-off analysis, with coaching events used only where they provide a useful, traceable starting point.

The current Owner decision is to complete the first end-to-end track-layout/run-off workflow using
the existing 2D LOWSIDE RIDER model and GIS/mapping before adding highside, upright overrun,
motorcycle slide, ejection, or other crash models. This repository owns local geometry,
trajectory, boundaries, and the optional authoritative local-to-projected georeference and its
provenance. The downstream package must consume that transform consistently and create GIS
outputs; it must not independently guess, fit, or reconstruct the Mallala transform.

Local coordinates remain authoritative for simulation and georeferencing changes no physics.
The SOURCE-DERIVED EPSG:7854 placement does not make the approximate analytical Mallala track
survey-grade. GIS presentation is not certification, homologation, or external acceptance.

## 6. Current repository status relevant to review

The repository includes:

- a frozen Mallala baseline and executable regression check;
- telemetry ingestion, registration, map matching, repeatability and comparison tools;
- Level-1 demanded-lean/roll-rate sensitivity and optional finite-roll speed constraint;
- fixed-line and re-optimised roll-sensitive comparisons;
- explicit Phase 10 closure with calibration deferral;
- completed Phase 11 representative-line assurance retaining `reduced_reoptimised_51`;
- closed Phase 12A deterministic coaching/event extraction with reviewed visual outputs; and
- engineering limit-state and trail-braking-proxy diagnostics suitable for downstream interpretation with their documented caveats.

## 7. Evidence and interpretation boundaries

Retain these distinctions:

- the frozen ideal-response result is a numerical/software baseline, not proof of physical accuracy;
- the approximate Mallala track is not survey-grade and must not be rescaled to force telemetry agreement;
- the provisional R6 configuration is not a fully identified motorcycle/rider model;
- 0.8 rad/s remains a finite-roll sensitivity scenario, not a calibrated constant;
- optimiser/control-basis spread is numerical search sensitivity, not physical uncertainty;
- measured rider trajectory/speed are validation evidence rather than an optimiser target;
- coaching marks are model-derived references rather than universally safe markers; and
- the retained representative path is an engineering analysis trajectory rather than a recommended rider line.

## 8. Review and change-control expectations

Reviews should continue to separate numerical regression, physics effect, optimisation response, and validation/presentation evidence.

Long-running optimisation diagnostics must record initial state, controls/policy, spacing, boundary margin, backend/workers, evaluation/sweep limits, termination reason, final step, and final fixed-grid re-evaluation. Saved controls and configuration hashes are preferable to relying on rerunning a local optimiser to reproduce a historical path.

The next run-off work must likewise record interface version, simulator scenario/configuration identity, departure-seed rule, coordinate convention, and downstream assumptions so that run-off results remain traceable to the simulation state that generated them.
