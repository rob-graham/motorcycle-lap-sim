# Phase 12A coaching-event extraction closure

**Closure date:** 2026-08-18  
**Status:** Closed for the present Mallala engineering sequence

## Closure decision

Phase 12A is closed as an engineering/prototype coaching-event extraction and visual-review stage. The retained Mallala representative trajectory, event extraction, engineering detail views, and longitudinal limit-state diagnostic are sufficiently coherent and traceable to support the next bounded task: defining the simulator-to-run-off calculation interface and run-off departure-condition workflow.

This closure is not a claim that the coaching marks are validated rider instruction, that the retained trajectory is a recommended riding line, or that the current motorcycle/rider/track models are sufficiently complete for coaching or regulatory use.

## Target-machine acceptance evidence

The final retained-line target-machine run reproduced the Phase 11 representative exactly:

- representative label: `reduced_reoptimised_51`;
- retained controls SHA-256: `4aa138e5af35e3a9180efc7a79abca7628dac99914ca082019d0140a8dfb02b3`;
- control count: 51;
- deleted original control index: 26;
- margin: 0.250 m;
- finite-roll sensitivity scenario: 0.8 rad/s;
- common spacing: 0.125 m;
- boundary-check spacing: 0.125 m;
- authoritative speed backend: Python;
- lap time: `71.396583646 s`;
- delta from the retained Phase 11 reference: `+0.000000000 s`.

The generic lean detector produced 12 raw regions. Mallala-specific consolidation resolved exactly nine nominal corners, with raw regions 1, 6 and 12 retained explicitly as direction-conflicting setup/transition artefacts rather than silently discarded.

The final run reported:

- braking onset: 9 events;
- maximum braking: 9;
- brake release: 7;
- turn-in: 9;
- geometric apex: 9;
- maximum curvature: 9;
- minimum-speed point: 9;
- maximum lean: 9;
- positive-drive pickup: 7;
- corner exit: 9;
- roll transition: 6;
- gear shift: 34.

The review outputs comprise:

- `phase12a_coaching_overview.png`;
- `phase12a_speed_map.png`;
- `phase12a_T1_T3_detail.png`;
- `phase12a_T4_T6_detail.png`;
- `phase12a_T7_T9_detail.png`;
- `phase12a_limit_state_map.png`;
- `phase12a_coaching_events.csv`;
- `phase12a_corner_regions_review.csv`;
- `phase12a_limit_state.csv`; and
- the representative trajectory CSV.

## Accepted semantics

The final visual-review iteration uses deterministic, model-derived event definitions:

- **BRK**: onset of sustained acceleration-derived braking/deceleration before a meaningful braking episode;
- **MAX-BRK**: maximum modelled deceleration in the accepted braking sequence, engineering-only;
- **REL**: final sustained acceleration-derived departure from the braking regime, not measured brake-lever release;
- **TURN**: onset of the dominant corner-direction demanded-lean / signed-curvature build, with an explicit bounded fallback where needed;
- **APEX**: minimum Euclidean clearance from the representative path to the physical inside track edge;
- **VMIN**: minimum-speed point, engineering-only;
- **K-MAX**: maximum absolute racing-line curvature, engineering-only;
- **DRIVE**: a real transition into a final sustained positive-longitudinal-acceleration regime, not throttle position;
- **EXIT**: substantial post-apex track-out / corner completion after the APEX/VMIN/K-MAX phase;
- **trail-braking proxy**: significant demanded lean combined with braking demand beyond passive drag and rolling resistance; and
- **wheelie/stoppie/traction/power limit states**: model capability diagnostics that are only labelled active when the solved state is close to the corresponding capability.

The clean rider overview remains intentionally limited to BRK, TURN, APEX, DRIVE and EXIT. Engineering-only detail remains separate.

## Known limitations and retained caveats

Phase 12A is closed with the following limitations explicitly retained:

1. The motorcycle, rider/control and Mallala reference-track models are deliberately simplified. Coaching-event locations and limit-state classifications are therefore prototype engineering outputs, not validated rider instruction or measured rider controls.
2. The event set would benefit from review by several experienced riders before any future rider-facing coaching use.
3. DRIVE is not throttle position. REL is not measured brake-lever release. The trail-braking mark is a simulation-derived proxy. Wheelie, stoppie, traction and power classifications are model capability diagnostics.
4. The retained Phase 11 path is an engineering simulation trajectory used for downstream analysis. It is **not a recommended riding line**.
5. In particular, the retained closed-loop trajectory bends toward the centre of the start/finish straight. That behaviour is not considered a desirable rider line and is recorded as a known modelling/optimisation artefact rather than a coaching recommendation.
6. The 0.8 rad/s finite-roll ceiling remains a sensitivity/scenario parameter rather than a calibrated R6/rider constant.
7. The approximate Mallala geometry is not survey-grade.

These caveats do not invalidate the next run-off stage. The retained trajectory, event locations, speed, acceleration, curvature, lean, gear/RPM and limit-state information remain suitable as engineering inputs for developing a versioned simulator-to-run-off interface and departure-condition calculations.

## Follow-on work

The next active task is to design and test the simulator-to-run-off calculation contract now that event semantics have passed the internal numerical and visual gate.

The next stage should define, at minimum:

- which solved trajectory fields are transferred;
- which reviewed event types and provenance/confidence fields are retained;
- candidate departure-point / departure-condition seeds;
- coordinate and chainage conventions;
- scenario and model-version metadata;
- physical-versus-modelled limit-state flags; and
- the boundary between simulator output and the separate run-off package.

Rider-facing polish, multi-rider review, improved rider/control modelling, and correction of the start/finish-line path artefact are deferred unless they become necessary for a later engineering decision. They are not prerequisites for starting run-off calculations.
