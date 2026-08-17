# Phase 11 optimisation-assurance closure

**Status date:** 2026-08-17  
**Status:** CLOSED for the present Mallala development sequence  
**Retained representative:** `reduced_reoptimised_51`

## Purpose

Phase 11 was intended to determine whether the Mallala rider-facing line was sufficiently well assured for the next development step, rather than to prove a global optimum. The retained work used bounded multistart, control-basis sensitivity, station-relocation sensitivity, common-grid re-evaluation, and a representative-line / optimiser-spread diagnostic.

The phase is now closed. Further optimiser experiments are not required before coaching-event extraction unless a later result exposes a specific engineering need.

## Closure evidence

The final representative-line diagnostic was merged to `main` in commit `785c8ba7debeb55fe16b85b60473fedfd4fbd824`.

All retained candidates were re-evaluated using:

- 0.250 m usable-corridor margin;
- 0.8 rad/s finite-roll sensitivity scenario;
- authoritative Python fixed-path backend;
- 0.125 m common fixed-path spacing; and
- 0.125 m dense boundary checking.

The four retained comparison lines were:

| Candidate | Representative eligible | Common-grid lap (s) |
|---|---:|---:|
| `baseline_restart3_52` | yes | 71.460392808 |
| `reduced_reoptimised_51` | yes | 71.396583646 |
| `relocated_fixed_offsets_52` | no; spread-only sensitivity | 71.433725922 |
| `relocated_reoptimised_52` | yes | 71.441629806 |

The eligible geometric medoid was `baseline_restart3_52`, but it was 0.063809162 s slower than the fastest eligible line. This exceeded the explicit 0.050 s representative-selection guardrail, so the selection rule fell back to the fastest eligible candidate.

The retained representative is therefore:

`reduced_reoptimised_51` at 71.396583646 s.

This is the 51-control basis formed by deleting original control index 26 at reference-track station 1490.376326042 m and re-optimising the remaining controls. It is a materially different feasible local solution, not proof that the deleted control was redundant and not a claim of global optimality.

The all-candidate optimiser/control-basis spread envelope had:

- maximum width 0.487734648 m;
- RMS width 0.151177197 m; and
- largest local spread around reference-track chainage 1578.21 m.

The spread is numerical optimiser/control-basis sensitivity. It is not a rider-variability envelope, physical uncertainty interval, safety corridor, or regulatory tolerance.

The target-machine verification recorded for the final diagnostic was 288 passing tests, with all four retained candidates feasible and positive forward progress on the common grid.

## Retained artifacts

The Phase 11 diagnostic produces:

- `phase11_robust_line_candidate_summary.csv`;
- `phase11_robust_line_pairwise_spread.csv`;
- `phase11_optimiser_spread_envelope.csv`;
- `phase11_representative_reference_line.csv`; and
- `phase11_representative_line_and_spread.png`.

The representative controls used by the next phase are the `reduced_51_final_controls.csv` output from the reduced-basis re-optimisation.

## Decision

Phase 11 is closed for the current sequence because it has produced a deterministic, feasible, optimisation-assured representative line and quantified the retained numerical spread well enough to support the next bounded task.

The next active task is **Phase 12A: coaching-event extraction on the retained representative line**.

Phase 12A must:

1. reproduce the retained representative line on the same authoritative scenario/grid;
2. extract rider-facing events from solved trajectory state using explicit deterministic rules;
3. export the events with source-rule and confidence fields;
4. draw the rider-facing marks on a clean racing-line image with no optimiser control points, optimiser envelope, corridor diagnostics, or other development-only overlays; and
5. visually inspect the Mallala event positions before any simulator-to-run-off export contract is defined.

The simulator-to-run-off export contract is therefore intentionally **deferred** at this closure point.
