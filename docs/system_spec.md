# System specification

## Scope and conventions

`motorcycle-lap-sim` is a clean-sheet minimum-lap-time motorcycle simulator and racing-line optimisation project. Internal calculations use SI units. Assumptions and model parameters must be explicit configuration or documented scenario data, not hidden constants.

For project intent, evidence hierarchy, validation boundaries, and roadmap context, see [`project_context.md`](project_context.md). This file describes implemented architecture and current simulation status.

## Architectural separation

The architecture deliberately separates:

1. **Track geometry** — physical rideable region, analytic primitives, widths, sampling, and boundaries.
2. **Racing-line/path geometry** — supplied or spline-defined paths through that region.
3. **Motorcycle model** — configuration, powertrain, resistance, load transfer, tyre/lean constraints, and optional simple handling-response limits.
4. **Fixed-path speed solution** — fastest feasible periodic speed profile on one immutable path.
5. **Racing-line optimisation** — path-control search calling the fixed-path solver.
6. **Telemetry/validation** — ingestion, registration, map matching, and comparison with measured sessions.
7. **Coaching/event extraction** — deterministic landmarks derived from solved trajectory state.
8. **Plotting/reporting** — visual and diagnostic consumers of numerical results.

Measured telemetry is not a hidden dependency of core physics. Coaching/event extraction does not alter physics or optimisation.

## Implemented phases

### Track, motorcycle, fixed-path, racing-line and optimisation layers

The repository contains analytic track geometry and variable widths, immutable path representations, motorcycle configuration and capability functions, the authoritative Python fixed-path solver with optional validated Numba acceleration, supplied and smooth planar racing-line construction, and deterministic local direct-planar optimisation with explicit feasibility and termination reporting.

The optimiser is local/warm-start dependent and makes no global-optimality claim.

### Mallala validation and Level-1 roll sensitivity

The Mallala/R6 workflow includes a frozen numerical baseline, telemetry ingestion/registration/map matching, Level-1 demanded lean and roll-rate diagnostics, an optional finite-roll speed ceiling, fixed-line and re-optimised comparisons, and spatial discrepancy diagnostics.

The production finite-roll sensitivity uses the curvature-transition contribution to demanded roll rate. `max_roll_rate_radps` remains a replaceable scenario parameter and is not inferred automatically from telemetry.

Phase 10 is closed for the present development sequence with substantial R6 parameter calibration deferred for identifiability reasons; see [`phase10_mallala_closure.md`](phase10_mallala_closure.md).

### Phase 11 - optimisation assurance closure

Phase 11 is closed; see [`phase11_optimisation_assurance_closure.md`](phase11_optimisation_assurance_closure.md).

The retained representative is:

- `reduced_reoptimised_51`;
- common-grid lap `71.396583646 s`;
- 0.250 m margin;
- 0.8 rad/s finite-roll sensitivity;
- 0.125 m authoritative Python common grid; and
- 51 controls after deleting original control 26 and re-optimising the remainder.

It is an optimisation-assured feasible local solution for the current engineering task, not proof of global optimality. Optimiser/control-basis spread is numerical sensitivity, not physical uncertainty or a safety corridor.

### Phase 12A - coaching-event extraction closure

Phase 12A is closed for the present Mallala engineering sequence; see [`phase12a_coaching_event_closure.md`](phase12a_coaching_event_closure.md) and [`coaching_event_definitions.md`](coaching_event_definitions.md).

`motorcycle_lap_sim.coaching` extracts deterministic event landmarks from solved trajectory arrays. `scripts/r6_phase12a_coaching_events.py` reconstructs the retained Phase 11 basis, re-evaluates the supplied controls on the authoritative scenario/grid, fails closed if the lap does not reproduce the retained reference within tolerance, consolidates Mallala raw lean regions to T1-T9, extracts events, and writes trajectory/event/provenance/limit-state outputs.

The final target-machine acceptance run reproduced `71.396583646 s` exactly, retained 12 generic lean regions, consolidated them to nine nominal corners, and generated the reviewed six-figure visual suite plus CSV diagnostics.

The clean rider overview contains only BRK, TURN, APEX, DRIVE, and EXIT. Engineering detail may additionally contain MAX-BRK, REL, VMIN, K-MAX, maximum lean, roll transitions, gear shifts, trail-braking proxy, and longitudinal capability-limit classifications.

Interpretation boundaries are explicit:

- DRIVE is longitudinal-acceleration-derived and is not throttle position;
- REL is acceleration-derived and is not measured brake-lever release;
- trail braking is a simulation-derived proxy, not measured brake pressure;
- wheelie/stoppie/traction/power states are model capability diagnostics;
- coaching-event locations are prototype engineering outputs rather than validated rider instruction; and
- the retained Phase 11 trajectory is an engineering analysis path, **not a recommended rider line**.

A known retained-path artefact is that the closed-loop racing line bends toward the centre of the start/finish straight. This should not be presented to riders as a recommended line. It does not block downstream run-off calculations.

## Current module status

- `track`: analytic track geometry, variable widths, sampling, and boundaries.
- `path`: generic immutable path representation.
- `racing_line`: supplied and smooth planar path construction/validation.
- `motorcycle`: immutable configuration, forces, powertrain, limits, and Level-1 roll calculations.
- `speed_solver`: deterministic periodic fixed-path minimum-time solver plus optional validated Numba backend.
- `optimisation`: local deterministic racing-line optimisation, direct planar controls, warm starts, and assurance diagnostics.
- `telemetry`: Mallala data ingestion, quality, registration, map matching, repeatability/peer analysis, and comparison utilities.
- `coaching`: deterministic extraction of reviewed rider-facing and engineering landmarks from solved trajectory state.
- plotting/scripts: reporting, validation, regression, coaching presentation, and engineering diagnostics kept separate from numerical modules.

## Validation and claim boundaries

Repository reviews must preserve the distinction between numerical verification, model sensitivity, optimisation response, case validation, and general validity.

The measured rider line is validation evidence, not the optimiser objective. The simulator represents a modelled high-performance/minimum-time scenario within stated constraints, not a prediction of a particular rider's exact actions. Coaching landmarks are model-derived reference information, not universally safe physical markers.

The Mallala geometry is approximate rather than survey-grade, and the provisional R6 model remains incompletely identified.

## Next active interface - simulator to run-off calculations

With Phase 12A's numerical and visual review complete, the next active task is to define the versioned simulator-to-run-off calculation interface.

This interface should be designed around downstream run-off engineering needs and should explicitly define:

- trajectory/path and track chainage conventions;
- local/global coordinates and boundary geometry;
- speed, acceleration, curvature, lean/roll state, and any other required solved fields;
- which reviewed events, provenance and confidence fields are useful downstream;
- candidate departure-point / departure-condition seed rules;
- model/scenario/configuration identity;
- coordinate-system and version metadata; and
- the boundary between simulator-derived quantities and assumptions owned by the separate run-off package.

The interface must not silently convert coaching marks or optimiser outputs into safety criteria. Run-off departure seeds and downstream calculations require their own explicit engineering rules and provenance.

Future roadmap work also includes GIS/georeferencing and a reusable 3D `TrackSurface` / `z(s,n)` interface with grade/banking. Those remain future capability unless current repository code and tests explicitly implement them.
