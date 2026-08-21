# Current repository state

- **Status date:** 2026-08-21
- **Repository:** `rob-graham/motorcycle-lap-sim`
- **Purpose:** Concise operational context for the next task. Update this file after a material milestone, scope decision, contract change, or change of current priority. Do not turn it into a chronological project history.

## 1. Implemented and retained state

- The repository provides deterministic fixed-path motorcycle simulation and local direct-planar racing-line optimisation in local SI coordinates.
- The authoritative Python speed solver remains the numerical reference; optional Numba acceleration is validated against it for accepted paths.
- The Mallala baseline, telemetry ingestion/map matching, demanded-lean and finite-roll sensitivity work are reproducible.
- Substantial R6 parameter calibration remains deferred for identifiability reasons. The current R6 and rider/session representation is provisional.
- Phase 11 optimisation assurance is closed for the present Mallala sequence. The retained representative is `reduced_reoptimised_51`, re-evaluated at `71.396583646 s` on the 0.125 m authoritative Python common grid with 0.250 m margin and the 0.8 rad/s finite-roll sensitivity scenario.
- Phase 12A coaching/event extraction and visual review are closed as prototype engineering outputs, not validated rider instruction.
- The versioned `0.1.0` simulator-to-run-off interface, deterministic bundle export, optional rigid georeference extension, top-level CLI, and generic direct-planar `optimise` command are implemented.
- A generic controls CSV does not replace the retained Mallala representative or bypass retained-case provenance requirements.

## 2. Current cross-project direction

The immediate project direction is to continue the end-to-end track-layout and run-off workflow through the separate `rob-graham/motorcycle-runoff` repository before adding more simulator-side crash types or reviving superseded optimisation investigations.

This process-documentation change does not start a new physical model or alter simulation behaviour.

## 3. Repository ownership boundary

`motorcycle-lap-sim` owns:

- local track, boundary, racing-line, speed, event, and scenario outputs;
- the canonical producer-side run-off interface and bundle semantics;
- producer contract snapshots and consistency tests;
- optional authoritative local-to-projected rigid georeference and provenance; and
- retained Mallala producer acceptance.

`motorcycle-runoff` owns:

- downstream physical off-track propagation;
- run-off surface/layout scenarios;
- re-entry, feature-intersection, and impact-demand observations;
- run-off reports and GIS presentation; and
- downstream model assumptions that are not simulator outputs.

The consumer must not import optimiser internals or independently guess, fit, or reconstruct the Mallala georeference.

## 4. Interpretation boundaries

- The Mallala analytical geometry is approximate and not survey-grade.
- The retained path is an engineering analysis trajectory, not a recommended rider line.
- `max_roll_rate_radps` remains a replaceable sensitivity/scenario value, not an identified universal R6/rider constant.
- Optimiser/control-basis spread is numerical sensitivity, not physical uncertainty, rider variability, a run-off corridor, or a regulatory criterion.
- Coaching events are model-derived prototype references, not universally safe markers.
- Georeferencing changes presentation coordinates only; it does not improve local geometry accuracy or change physics.
- No result claims regulatory approval, homologation, certification, or insurance acceptance.

## 5. Deferred or conditional re-entry work

Do not restart these without a new, explicit downstream reason and bounded task brief:

- superseded Phase 11C/11D optimiser experiments;
- substantial multi-parameter R6 calibration;
- advanced dynamic lean/steering states beyond the current justified model;
- 3D terrain, grade, and banking in the lap solver;
- changes to the versioned run-off interface not required by a defined consumer need; or
- rider-facing publication/polish without a separate validation purpose.

## 6. Task-start rule

For the next task, read `AGENTS.md`, this file, and only the task-specific documents identified by the reading map. Record the exact starting SHA, task class, roles, required capabilities, acceptance criteria, and any cross-repository dependency before implementation.
