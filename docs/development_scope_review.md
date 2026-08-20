# Development scope review

**Review date:** 2026-08-18

This review records the bounded development sequence through Phase 12A and the active Phase 12B retained-Mallala export gate.

## Closed decisions retained

- Phase 10 substantial R6 parameter calibration remains deferred for identifiability reasons. The provisional motorcycle/rider/session model is not treated as calibrated merely because selected sensitivity cases approach measured lap times.
- `max_roll_rate_radps` remains a scenario/sensitivity parameter rather than an identified R6/rider constant.
- Lap 5 remains the first future calibration/development candidate, Lap 4 the first untouched hold-out candidate, and Laps 1-3 additional out-of-fit comparisons.
- Phase 11 is closed with `reduced_reoptimised_51` retained as the representative line for downstream engineering work.
- Superseded Phase 11C/11D optimiser experiments are not to be revived merely because the local optimiser is warm-start dependent.
- New optimisation work requires a new downstream engineering reason and a bounded benchmark against the retained method.

## Phase 11 retained representative

The retained representative was re-evaluated on the 0.125 m authoritative Python common grid with 0.250 m margin and the 0.8 rad/s finite-roll sensitivity scenario.

The final retained line is `reduced_reoptimised_51` at `71.396583646 s`.

This is a feasible, optimisation-assured local solution suitable for the present engineering task, not proof of global optimality. Optimiser/control-basis spread remains numerical sensitivity and must not be presented as physical model uncertainty, rider variability, a run-off corridor, or a regulatory criterion.

See [`phase11_optimisation_assurance_closure.md`](phase11_optimisation_assurance_closure.md).

## Phase 12A closure decision

Phase 12A coaching-event extraction and Mallala visual review are now closed for the present engineering sequence; see [`phase12a_coaching_event_closure.md`](phase12a_coaching_event_closure.md).

The final target-machine run reproduced the Phase 11 representative exactly at `71.396583646 s`, retained 12 generic lean regions, consolidated them to nine T1-T9 Mallala corners, and generated the reviewed rider overview, speed map, regional engineering detail plots, event/provenance CSVs, and longitudinal limit-state diagnostic.

The final event semantics include BRK, TURN, APEX, DRIVE and EXIT on the clean overview, with MAX-BRK, REL, VMIN, K-MAX, maximum lean, roll transitions, gear shifts, trail-braking proxy and capability-limit states available for engineering review.

Phase 12A closes with explicit limitations rather than additional polishing requirements:

- coaching marks are model-derived prototype engineering outputs, not validated rider instruction;
- several experienced riders should review the marks before any future rider-facing coaching use;
- DRIVE is not throttle position, REL is not measured brake-lever release, and trail braking is a simulation-derived proxy;
- wheelie/stoppie/traction/power classifications are model capability diagnostics;
- the retained Phase 11 trajectory is an engineering simulation path, **not a recommended rider line**; and
- the bend toward the centre of the start/finish straight is a known modelling/optimisation artefact and must not be presented as a rider recommendation.

These limitations do not block the next run-off stage.

## Current re-entry criteria for optimisation or coaching work

Further optimisation work requires a downstream finding that the retained representative is not fit for the engineering purpose at hand, or another explicit engineering need.

Further coaching/rider-facing refinement requires a separate purpose, such as multi-rider review, improved rider/control modelling, or a need to publish validated rider guidance. It is not required merely to continue the engineering run-off workflow.

## Active bounded task: retained-Mallala integration and serialization

The Phase 12A visual gate has passed, and the Phase 12B `0.1.0` in-memory simulator-to-run-off contract and candidate departure workflow already exist. The active increment integrates the retained Mallala trajectory and reviewed event set and adds a separately versioned deterministic directory bundle.

This integration must validate the retained lap, total-length/wrap semantics, event/trajectory correspondence, provenance, candidate counts and serialization on the Owner's target machine. Phase 12B remains open until that command succeeds.

At minimum, the interface work should define:

- trajectory/path and track chainage conventions;
- coordinate systems and track-boundary geometry;
- speed and longitudinal/lateral acceleration fields;
- curvature and lean/roll-state fields where useful;
- reviewed event provenance/confidence where useful;
- candidate departure-point / departure-condition seed rules;
- scenario/model/configuration identity and versioning; and
- the ownership boundary between simulator output and the separate run-off package.

The downstream interface must not treat coaching marks, optimiser controls, optimiser spread, or capability-limit classifications as safety criteria by default. Each run-off input must have an explicit engineering reason, definition, units, and provenance.

After target-machine acceptance, downstream physical run-off and GIS generation remain separate work. The producer-side optional rigid georeference is the only mapping capability added here; off-track propagation, GIS file output, and 3D terrain remain out of scope.

## Current LOWSIDE-to-GIS sequencing decision

The Owner's current priority is to complete the first end-to-end track-layout/run-off workflow
using the existing 2D LOWSIDE RIDER model and GIS/mapping before implementing further crash
models. Highside, upright overrun, motorcycle slide, ejection, terrain/3D, barriers, and GIS file
presentation are outside the producer-side georeference increment.

This repository owns authoritative local simulation geometry and the optional local-to-projected
rigid transform plus provenance. The downstream run-off repository will consume—not guess, fit,
or reconstruct—that transform and create GIS outputs. Local coordinates remain authoritative and
georeferencing changes no physics. The approximate Mallala analytical geometry remains
non-survey-grade even when positioned in EPSG:7854; mapping is not external acceptance.
