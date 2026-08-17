# Development scope review

**Review date:** 2026-08-17

This review resolves the development detour that followed the Mallala Phase 9
work. The current priority is the documented Phase 9/10 validation workflow:
reproduce the frozen fixed-path baseline, understand the measured-data and
finite-roll discrepancy, and make the existing runtime workflow reliable.

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
  against current `main`, with an executable Phase 9/10 failure and regression
  test demonstrating why it is required.
- Do not start a new optimiser family, candidate generator, or larger search
  campaign merely because the local optimiser is warm-start dependent.

## Main-branch review finding

The full test run exposed a real baseline-workflow defect: the frozen control
CSV had been replaced with the recovered 69.354897583-second artifact, but the
executable identity constant, baseline manifest, and baseline document still
recorded the earlier artifact hash. The track and motorcycle hashes in the
manifest were also older than the inputs now verified by the executable check.
The numerical regression values themselves still match.

The canonical hashes are now synchronised across the executable check,
manifest, and baseline document. This restores fail-closed identity checking
without weakening tolerances or suppressing a failure.

## Re-entry criteria for optimisation work

Further optimisation-assurance work should wait until the Phase 9/10 runtime
workflow is reproducible and its decision-relevant validation gaps are
recorded. A future optimiser change requires a stated engineering need, a
bounded benchmark against the retained method, common-grid ranking, runtime
reporting, deterministic reproduction, and tests that do more than exercise a
new helper function.
