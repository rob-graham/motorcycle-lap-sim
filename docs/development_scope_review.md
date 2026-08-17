# Development scope review

**Review date:** 2026-08-17

This review resolves the development detour that followed the Mallala Phase 9
work. The Phase 9/10 validation/runtime workflow has now been reproduced on
current `main`, its decision-relevant validation gaps are recorded, Phase 10 is
closed for the present development sequence with substantial parameter
calibration explicitly deferred for identifiability reasons, and Phase 11 has
now also been closed with one retained optimisation-assured representative line.

## Decisions

- Retain the Phase 11A deterministic multistart diagnostic and Phase 11B
  hierarchical warm-start diagnostic only as bounded evidence of the existing
  optimiser's warm-start dependence.
- Remove the merged Phase 11C latent basin-search script and its unit tests. It
  introduced another search representation but did not change the production
  optimiser, physical model, or Phase 9/10 validation capability.
- Do not merge pull requests #41, #42, or #43. They predate the current project
  documentation and belong to the superseded optimiser-experiment sequence.
  Any independently useful defect fix must be proposed again as a small change
  against current `main`, with an executable current-main failure and regression
  test demonstrating why it is required.
- Do not revive the rejected Phase 11C/11D experiments merely because the local
  optimiser is warm-start dependent.
- Phase 10 substantial R6 parameter calibration is deferred rather than forced
  against one incompletely characterised rider/bike/session and approximate
  track geometry. Existing defaults remain provisional and replaceable.
- Preserve Lap 5 as the first future calibration/development candidate and Lap 4
  as the first untouched hold-out candidate; Laps 1-3 remain additional
  out-of-fit comparisons.
- Close Phase 11 using the merged representative-line / optimiser-spread
  diagnostic. Retain `reduced_reoptimised_51` as the representative line for the
  next bounded task; do not spend further optimisation time seeking small
  improvements without a new engineering reason.
- Make Phase 12A rider-facing event extraction and Mallala visual review the
  next active task.
- Do not define the simulator-to-run-off export contract until the Phase 12A
  event positions have been reviewed on the retained representative line.

## Main-branch review findings

The full test run first exposed a real baseline-workflow defect: the frozen
control CSV identity was being checked with platform-dependent working-tree byte
hashes. On Windows with `core.autocrlf=true`, the repository LF text was
materialised as CRLF and therefore produced different SHA-256 values despite
identical text content and identical numerical results.

That defect was corrected by defining the baseline identities as SHA-256 of
UTF-8 text with newlines canonicalised to LF. The canonical hashes and numerical
baseline remain unchanged. The Windows environment that exposed the defect then
reported `236 passed`, and `python scripts/r6_phase9_baseline_check.py` ended
with `executable_baseline_regression_status=passed`.

The complete Mallala runtime chain was then exercised on current `main` using
the supplied R6 workbook:

- telemetry ingestion reproduced 9,580 samples, five complete laps and the
  incomplete sixth lap;
- rigid registration converged in 66 iterations, with 2.5763 m RMS residual and
  219/256 bins containing all five selected laps;
- the frozen ideal-response line reproduced 69.354897583 s and showed a broad
  +1.9362 m/s mean speed bias relative to the measured median;
- fixed-line finite-roll sensitivity materially reduced the discrepancy without
  changing motorcycle performance parameters, but no single roll-rate value was
  preferred by all comparison metrics.

The detailed evidence and calibration decision are recorded in
[`phase10_mallala_closure.md`](phase10_mallala_closure.md).

## Phase 10 closure decision

The simulator is a minimum-time / high-performance scenario model, while the
available validation evidence is one human rider, one incompletely
characterised motorcycle/session, uncertain logger details, and approximate
track geometry. The remaining discrepancy is not sufficiently identifiable to
justify fitting a small motorcycle parameter subset without material
compensation/overfitting risk.

Phase 10 is therefore closed for the present sequence with substantial
calibration deferred. This is not a claim that the R6 reference is calibrated.
`max_roll_rate_radps` remains a sensitivity/scenario parameter, and the current
mass, power/torque, drag, grip/utilisation, gearing/radius and related defaults
retain their documented provisional status.

Calibration may be reopened when better measured bike/rider/setup inputs,
improved track geometry, clarified logger/roll interpretation, additional
sessions/riders, or a strongly attributable local discrepancy makes a bounded
parameter subset meaningfully identifiable.

## Phase 11 closure decision

The Phase 11 assurance work proceeded only through bounded experiments tied to
an engineering question: whether one credible line could be retained for
rider-facing work despite local optimiser/control-basis sensitivity.

The final representative diagnostic re-evaluated four established feasible
candidates on the same 0.125 m Python common grid with 0.250 m margin and the
0.8 rad/s finite-roll sensitivity scenario. The eligible geometric medoid was
`baseline_restart3_52`, but its 0.063809162 s penalty relative to the fastest
eligible candidate exceeded the explicit 0.050 s representative guardrail.
The deterministic selection rule therefore retained
`reduced_reoptimised_51` at 71.396583646 s.

The retained line is a feasible, optimisation-assured local solution suitable
for the next development task. It is not proof of global optimality. The
reported optimiser/control-basis spread is numerical sensitivity rather than a
physical uncertainty or safety corridor.

The complete closure evidence is in
[`phase11_optimisation_assurance_closure.md`](phase11_optimisation_assurance_closure.md).

## Current re-entry criteria for optimisation work

Phase 11 is closed rather than left open-ended. New optimisation work now
requires a specific downstream finding that the retained representative is not
fit for purpose, or another stated engineering need. Any such change requires a
bounded benchmark against the retained method, common-grid ranking, runtime
reporting, deterministic reproduction, and meaningful tests.

Warm-start dependence by itself is no longer sufficient reason to add another
search representation.

## Next bounded task: Phase 12A

Phase 12A extracts deterministic coaching landmarks from the retained
representative trajectory and presents them on a rider-facing Mallala map. The
machine-readable event table may include more information than is shown on the
image, but the image is intentionally stripped of optimisation controls,
envelopes, corridor diagnostics, and convergence information.

The numerical output and image must be reviewed together before downstream
interfaces are frozen. In particular, braking, turn-in, apex, positive-drive
pickup and exit locations should form a plausible T1-T9 sequence and compound
analytic primitives must not create duplicate rider events.

Only after that review should the simulator-to-run-off contract be designed.
