# System specification

## Scope and conventions

`motorcycle-lap-sim` is a clean-sheet minimum-lap-time motorcycle simulator and racing-line optimisation project. Internal calculations use SI units: metres, seconds, kilograms, radians, and derived SI units. Heading is measured counter-clockwise from the positive x-axis. Assumptions and model parameters must be explicit configuration or documented scenario data, not hidden constants.

For project-level intent, evidence hierarchy, validation boundaries, and roadmap context, see [`project_context.md`](project_context.md). This file describes implemented repository architecture and the status of the simulation work.

## Architectural separation

The architecture deliberately separates the following concepts.

1. **Track geometry — what physical region is available to ride on?** Analytic centreline primitives, left/right widths, sampling, differential geometry, and boundaries define the permitted region. Track geometry does not choose a racing line or contain vehicle physics.
2. **Racing-line/path geometry — what path through that region does the motorcycle follow?** The `racing_line` and path layers construct and validate supplied or spline-defined paths. They do not solve vehicle speed.
3. **Motorcycle model — what physical capabilities and limits apply?** Immutable motorcycle configuration and independent force/limit functions define powertrain, resistance, load transfer, tyre/lean constraints, and optional simple handling-response limits.
4. **Fixed-path speed solution — for one supplied path, what is the fastest feasible periodic speed profile?** `speed_solver` computes speed, acceleration, gear/RPM, lap time, active constraints, and diagnostics without choosing the path.
5. **Racing-line optimisation — which permissible path minimises calculated lap time?** `optimisation` varies path controls and calls the fixed-path solver. It must not duplicate vehicle or geometry feasibility logic.
6. **Telemetry/validation — how does the simulation compare with measured data?** `telemetry` imports, quality-checks, registers, map-matches, and compares measured sessions. Measured data remain outside core physics and optimisation.
7. **Coaching/event extraction — which rider-facing landmarks follow from one solved trajectory?** `coaching` derives deterministic events from solved path/speed/state arrays. It does not alter physics, optimise the path, or define run-off trajectories.
8. **Plotting/reporting — how are results inspected?** Plotting and diagnostic scripts consume numerical results but are not dependencies of numerical modules.

The track, path, motorcycle, fixed-path solver, optimiser, telemetry and coaching tools must remain independently testable and deterministic unless an explicitly documented feature requires otherwise.

## Phase 1 - Track geometry

`track.primitives` provides analytic `Straight` and `CircularArc` geometry from a `Pose`. `track.track` composes primitives, retains width and closure intent, and reports closure errors without modifying geometry. `track.sampling` produces immutable samples approximately uniform in centreline arc length. `track.boundaries` offsets samples along normals; the positive normal points left of travel.

Closed-track samples omit the duplicate endpoint by default: `s=0` is present and `s=total_length` is absent. Open tracks include both ends. Primitive joins are represented once where a sample lies exactly on a join.

## Phase 2 - Motorcycle physics

The `motorcycle` package provides immutable YAML configuration, deterministic engine interpolation, gearing, resistance forces, longitudinal axle loads, geometry-derived wheelie/stoppie limits, lateral/lean caps, and combined tyre limits. Physical formulas are independently testable and do not depend on track geometry or plotting.

Optional `handling` configuration may define either or both:

- `max_path_curvature_rate_1pmps` — the earlier curvature-transient proxy; and
- `max_roll_rate_radps` — the later Level-1 finite-roll sensitivity limit.

Omitting handling disables those optional response limits.

## Phase 3 - Fixed-path solver

The fixed-path solver calculates the fastest feasible periodic speed profile on one immutable closed `SampledPath`. It combines local speed ceilings with forward acceleration and backward braking propagation and reports lap time, speed, acceleration, gear/RPM, and constraint diagnostics.

The authoritative implementation is the Python solver. An optional Numba backend accelerates the same fixed-path calculations and is checked against the Python result for accepted paths. Backend choice is a computational setting, not a different physical model.

See [`fixed_path_solver.md`](fixed_path_solver.md).

## Phase 4 - Supplied racing line

The racing-line layer accepts supplied lateral offsets, validates them against track widths, constructs displaced coordinates and path distance, and calculates periodic signed curvature as a generic `SampledPath`. It contains no optimiser or motorcycle-speed logic.

See [`racing_line_representation.md`](racing_line_representation.md).

## Phase 5 - Initial local racing-line optimisation

The original optimisation layer provides a C2 periodic cubic parameterisation, smooth asymmetric boundary-safe mapping, pure lap-time evaluation, and deterministic bounded coordinate search. It returns a locally optimised line and supports finer fixed-path re-evaluation. It makes no global-optimality claim.

See [`racing_line_optimisation.md`](racing_line_optimisation.md).

## Phase 6 - Optional curvature-transient handling proxy

The historical `max_path_curvature_rate_1pmps` feature places a path-curvature-transient speed ceiling in the fixed-path capability layer. It is a simple handling proxy, not validated steering dynamics, and remains optional for regression/sensitivity comparison.

See [`curvature_transient_limit.md`](curvature_transient_limit.md).

## Phase 7 - Smooth planar path geometry

Phase 7 added an alternative C2-periodic Cartesian spline path representation so that analytic track curvature jumps do not have to become motorcycle-path curvature jumps. The fixed-path solver remains unaware of how the supplied `SampledPath` was constructed.

See [`smooth_planar_racing_line.md`](smooth_planar_racing_line.md).

## Phase 7.5 - Mallala reference geometry and variable widths

`Track` supports track-wide default left/right half-widths with optional primitive-specific overrides. The QGIS-derived Mallala reference uses this mechanism and remains an approximate local-coordinate development geometry rather than a survey-grade georeferenced track model.

See [`mallala_reference_track.md`](mallala_reference_track.md).

## Phase 8 - Direct planar racing-line optimisation

Phase 8 places physical lateral controls at geometry-derived stations and builds a non-uniform C2-periodic Cartesian spline through the resulting guide points. Continuous corridor and forward-progress checks are applied before fixed-path sampling. Path-model control resolution is distinct from fixed-path output resolution.

The direct planar optimiser uses deterministic best-improvement polling with coordinate and smooth coupled directions, bounded controls, optional parallel complete-poll evaluation, warm starts, and explicit evaluation/sweep/step termination metadata. More controls do not imply a better solution and the method remains local/warm-start dependent.

See [`direct_planar_racing_line_optimisation.md`](direct_planar_racing_line_optimisation.md).

## Phase 9 - Mallala baseline, telemetry/roll validation and finite-roll sensitivity

The repository implements the roadmap Phase 9 work, although some scripts retain historical `phase10` names from the repository development sequence. Reviewers should follow behaviour and documentation rather than infer roadmap meaning from script numbering alone.

### Frozen numerical baseline

The representative ideal-response baseline is the retained 52-control Mallala line in `cases/mallala_r6/baseline/phase8_reference_controls.csv`. Its identity, canonical input hashes, fixed-geometry regression values, and provenance limitations are documented in [`phase9_baseline_freeze.md`](phase9_baseline_freeze.md).

The executable baseline is reproduced by `scripts/r6_phase9_baseline_check.py`. Change control is based on re-evaluating the saved geometry, not assuming a fresh local optimisation will recover the same path.

### Telemetry subsystem

The `telemetry` package contains AiM/Excel ingestion, source-quality handling, rigid 2D registration, nearest/map matching, cross-lap/peer diagnostics, repeatability measures, and speed comparison utilities. Diagnostic scripts provide Mallala session checks, registration, speed validation, roll-related comparisons, and trajectory exports.

The supplied R6 session is case-specific and incompletely characterised. Raw data are not a hidden simulator dependency and are not treated as universal truth. See [`mallala_r6_telemetry_integrity.md`](mallala_r6_telemetry_integrity.md).

### Level-1 demanded lean and finite roll response

`motorcycle.roll` provides planar steady lean demand

`phi = atan(v^2 * kappa / g)`

and roll-rate diagnostics. The production finite-roll sensitivity ceiling uses the curvature-transition contribution

`phi_dot = (v^3 * kappa' / g) / (1 + (v^2 * kappa / g)^2)`

with an explicit positive `max_roll_rate_radps` scenario parameter. This expression intentionally omits the additional lean-rate contribution caused by longitudinal acceleration/braking. The omission keeps the Level-1 feature simple and trajectory-driven; it must be stated when interpreting results.

The roll-rate parameter is not inferred automatically from telemetry and must not be presented as a calibrated R6/rider constant unless separate evidence establishes that.

### Fixed-line and re-optimised comparisons

Repository diagnostics support:

- ideal-response frozen-line evaluation;
- finite-roll evaluation on the same frozen path, isolating the physics effect;
- roll-aware re-optimisation, isolating path adaptation after the physics change; and
- sector/spatial comparison with measured Mallala telemetry.

A total lap-time match is not an acceptance criterion. Validation should examine where time is gained/lost, local speed and line agreement, transition behaviour, and active constraints.

## Phase 10 - Calibration/hold-out closure

The full Mallala validation/runtime chain has been reproduced on current `main`; see [`phase10_mallala_closure.md`](phase10_mallala_closure.md). The repository records Phase 10 as closed for the present development sequence with substantial R6 parameter calibration **deferred for identifiability reasons**.

The current evidence does not justify fitting a small motorcycle parameter subset from this one incompletely characterised rider/bike/session and approximate track geometry without material compensation/overfitting risk. The provisional motorcycle must therefore not be described as calibrated merely because selected finite-roll cases approach measured lap times.

The closure preserves these boundaries:

- mass, power/torque, drag, grip/utilisation, gearing/radius, edge margin and handling response are not jointly tuned to the Mallala lap;
- `max_roll_rate_radps` remains an uncalibrated sensitivity/scenario parameter;
- Lap 5 remains the first future calibration/development candidate;
- Lap 4 remains the first untouched hold-out candidate;
- Laps 1-3 remain additional out-of-fit comparisons; and
- future calibration requires new evidence that makes a deliberately small bounded parameter subset meaningfully identifiable, followed by hold-out evaluation without further fitting.

The correct claim is **Mallala R6 case validation**, not general validation of all riders, motorcycles, or circuits.

## Phase 11 - Optimisation assurance closure

Phase 11 is closed for the present Mallala development sequence. See [`phase11_optimisation_assurance_closure.md`](phase11_optimisation_assurance_closure.md).

The final bounded diagnostic re-evaluated four retained candidate lines on the authoritative 0.125 m Python common grid. Representative eligibility excluded the fixed-offset relocation sensitivity. The eligible geometric medoid exceeded the explicit 0.050 s lap-time guardrail, so the selection rule retained the fastest eligible candidate:

- representative: `reduced_reoptimised_51`;
- common-grid lap: 71.396583646 s;
- margin: 0.250 m;
- finite-roll sensitivity: 0.8 rad/s; and
- basis: 51 controls after deleting original control 26 and re-optimising the remainder.

The representative is an optimisation-assured feasible local solution for the current engineering task. It is not a global-optimality claim. The reported optimiser/control-basis spread is numerical sensitivity and must not be presented as physical uncertainty or a safety corridor.

The superseded Phase 11C/11D experiments must not be revived simply because the local optimiser is warm-start dependent. Any new optimisation work requires a new stated engineering need and bounded benchmark.

## Phase 12A - Coaching-event extraction and visual review

The `coaching` package now provides deterministic rider-facing event extraction from solved trajectory arrays. `scripts/r6_phase12a_coaching_events.py` is the Mallala integration command.

The command:

1. reconstructs the retained 51-control Phase 11 basis;
2. re-evaluates the supplied representative controls on the 0.125 m authoritative Python grid with the retained 0.250 m margin and 0.8 rad/s finite-roll sensitivity;
3. fails closed if the resulting lap does not reproduce the retained 71.396583646 s Phase 11 reference within the configured tolerance;
4. extracts T1-T9 event landmarks with explicit threshold/rule provenance;
5. writes a full representative trajectory CSV and coaching-event CSV; and
6. writes a clean rider-facing racing-line image containing track edges, the representative line, start/finish, and coaching marks only.

The default map marks braking onset, brake release, turn-in, geometric apex, positive-drive pickup and corner exit. Maximum braking, speed apex, maximum lean, roll transition and gear shifts remain available in the event table without cluttering the map.

No optimiser controls, optimiser envelope, margin corridor, centreline diagnostics, convergence metrics, or other development-only overlays are drawn on the coaching image.

See [`coaching_event_definitions.md`](coaching_event_definitions.md).

Phase 12A is not complete until the generated event locations have been visually inspected around Mallala. The simulator-to-run-off export contract is deliberately deferred until after that review.

## Current module status

- `track`: analytic track geometry, variable widths, sampling, and boundaries.
- `path`: generic immutable path representation.
- `racing_line`: supplied and smooth planar path construction/validation.
- `motorcycle`: immutable configuration, forces, powertrain, limits, and Level-1 roll calculations.
- `speed_solver`: deterministic periodic fixed-path minimum-time solver plus optional validated Numba backend.
- `optimisation`: local deterministic racing-line optimisation, direct planar controls, warm starts, and assurance diagnostics.
- `telemetry`: Mallala data ingestion, quality, registration, map matching, repeatability/peer analysis, and comparison utilities.
- `coaching`: deterministic extraction of rider-facing landmarks from solved trajectory state.
- `plotting` and scripts: reporting, validation, regression, coaching presentation, and engineering diagnostics kept separate from numerical modules.

## Validation and claim boundaries

Repository reviews must preserve the distinction between:

- **numerical verification** — reproducibility of saved geometry and solver outputs;
- **model sensitivity** — effect of enabling a physics/handling feature on the same path;
- **optimisation response** — additional effect after the line is allowed to change;
- **case validation** — comparison with the available Mallala R6 evidence; and
- **general validity** — which is not established by one motorcycle, rider, session, or track.

The measured rider line is validation evidence, not the optimiser objective. The simulator represents a modelled high-performance/minimum-time scenario within stated constraints, not a prediction of a particular rider's exact actions. Coaching landmarks are model-derived reference information, not universally safe physical markers.

## Future interfaces, not current capability claims

The wider project roadmap still proposes GIS/georeferencing, a reusable 3D `TrackSurface`/`z(s,n)` interface, grade/banking in the lap solver, and a separate run-off package consuming versioned trajectory/event/terrain results.

The simulator-to-run-off export contract is specifically **not yet defined**. It will be designed only after Phase 12A event locations have passed the Mallala numerical and visual review so that the downstream interface is based on observed event semantics rather than pre-committing to an unreviewed schema.
