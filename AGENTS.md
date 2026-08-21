# Repository instructions

## Engineering invariants

- This is a clean-sheet implementation.
- Do not import old lap-time simulator source code unless explicitly requested.
- Build and validate fixed-path simulation before racing-line optimisation.
- Use SI units internally.
- Physical formulas belong in clearly identified functions/modules.
- No mutable module-level global state.
- Plotting must be separate from numerical calculations.
- Every new physical or geometric feature requires tests.
- Preserve analytically understandable test cases.
- Never suppress numerical, optimisation, data-quality, or provenance warnings merely to make tests pass.
- Record assumptions explicitly.
- Prefer deterministic calculations.
- Maintain backward-compatible data formats only when deliberately specified.

## Project context and scope

- Before non-trivial work, read `docs/project_context.md`, `docs/system_spec.md`, and `docs/development_scope_review.md`, plus the most relevant phase/method document for the task.
- Repository code, tests, configuration, cases, and current repository documentation define implemented behaviour. External roadmap documents define intended direction, not implemented capability.
- Current `main` repository documentation and `AGENTS.md` govern current implementation state, phase status, and the next repository task. ChatGPT Project snapshot files remain useful for wider roadmap direction unless deliberately superseded by a current project decision.
- Do not infer roadmap phase status from historical script names alone. Some repository `phase10` scripts implement work now grouped under project Phase 9.
- The Phase 9/10 Mallala validation/runtime workflow is reproducible and Phase 10 is closed for the present sequence with substantial calibration explicitly deferred for identifiability reasons; see `docs/phase10_mallala_closure.md`.
- Phase 11 optimisation assurance is closed for the present Mallala sequence. The retained representative is `reduced_reoptimised_51` at 71.396583646 s on the 0.125 m authoritative Python common grid; see `docs/phase11_optimisation_assurance_closure.md`.
- Phase 12A coaching-event extraction and visual review are closed; see `docs/phase12a_coaching_event_closure.md`. Phase 12B producer export target-machine acceptance succeeded and PR #80 was merged after independent review.
- Current development priority is completing the first end-to-end track-layout/run-off workflow using the existing 2D LOWSIDE RIDER model and GIS/mapping before adding further crash models. Physical run-off propagation and GIS generation remain a separate downstream package. The generic optimise command is productisation of the existing direct-planar method, not new optimisation assurance or a replacement retained Mallala line. Do not revive superseded Phase 11C/11D experiments without a new explicit engineering justification and bounded benchmark plan.
- Before changing the simulator-to-run-off interface, read the canonical machine-readable snapshot at `contracts/runoff_interface_0.1.0.json` as well as the human-readable contract.

## Branch and agent workflow

- Start each new task from current `main` unless explicitly instructed otherwise.
- Use one focused branch/PR per task. Do not continue development on stale, superseded, or rejected PR branches.
- Every task must explicitly name one Implementer and one independent Reviewer. Either ChatGPT or Codex may fill either role, but the same participant must not fill both roles for a non-trivial change.
- Use one active Implementer per task branch. Reviewers inspect and comment; they do not make competing edits unless the Owner explicitly transfers the Implementer role.
- For Local implementation, the Owner normally creates and pushes the named task branch before implementation. For Cloud/sandbox implementation, the Owner supplies the repository, base branch/ref, authoritative starting commit SHA, task identity, Implementer, and Reviewer; the service may create the eventual GitHub PR head branch, whose name need not match an internal sandbox branch or a pre-created Owner branch.
- Once a PR exists, record its actual GitHub head branch and exact head SHA. Those identities govern review and merge. The Reviewer must state the exact PR head SHA reviewed, and any later head change requires re-review.
- If an old branch contains a potentially useful fix, first reproduce the failure on current `main`; then re-propose the smallest fix against current `main` with a regression test.
- ChatGPT and Codex may implement code and documentation. When ChatGPT has connected GitHub access, it should inspect the actual repository, pull request, and diff directly; pasted diffs and logs are fallback evidence.
- Review the actual diff and executable behaviour, not only the implementer's explanation. Do not assume a generated change is correct because its unit tests pass.
- If the Implementer's environment cannot push or create a pull request, it must return the verified patch/diff, starting SHA, resulting commit SHA if one exists, and exact test evidence, and state that the Owner must apply/push it. Never fabricate a pull-request URL.
- After a PR is merged or deliberately closed, delete its head branch unless there is a documented reason to retain it.
- Detailed procedures and command templates are in `docs/agent_collaboration_workflow.md`.

## Verification before merge

- A code change is not complete while it has a known runtime error, even if unit tests pass.
- Assess the static/change, targeted, representative runtime, and full regression gates independently. A gate may be recorded as N/A only when the change cannot affect that category, with the reason stated; never infer N/A from another gate passing.
- Run targeted tests for the changed behaviour and run the full test suite with `python -m pytest` before merge. Report the exact pass/fail/skip result; do not hide optional-dependency skips.
- If a change affects a command-line script, long-running workflow, import path, file-loading path, or user-facing runtime path, execute the affected entry point on the smallest representative case available. Unit tests alone are not sufficient runtime verification.
- For changes affecting the frozen Mallala baseline, its inputs, or baseline provenance, run `python scripts/r6_phase9_baseline_check.py` and require the fail-closed regression checks to pass.
- For long optimisation workflows, use a bounded smoke/diagnostic run when a full optimisation is unnecessary. Record settings, termination reason, and any warnings.
- If required proprietary/external data are unavailable in the execution environment, say so explicitly. Use synthetic or repository-contained smoke coverage where possible, and do not claim the unavailable end-to-end path was runtime-validated.
- Do not weaken tolerances, disable checks, catch-and-ignore exceptions, or convert failures into warnings merely to obtain a green test run.
- Immediately before merge, the Owner must verify that the locally tested `HEAD` and the GitHub PR head both equal the exact head SHA stated in the final review.

## Pull-request evidence and review

- PR descriptions for code changes should state scope, important assumptions, tests run, representative runtime commands run, skipped/unavailable checks, and known limitations.
- Preserve the distinction between numerical regression, physics sensitivity, optimisation response, and validation evidence. Do not reduce validation to total lap time alone.
- For long-running optimisation results, record initial state, control policy, sampling/grid settings, boundary margin, backend/workers, evaluation/sweep limits, termination reason, final step, and common-grid re-evaluation where applicable.
- Resolve substantive code-review findings before merge, or document why a finding is not applicable.
- Do not mark a roadmap phase complete merely because related code exists. Phase completion should be tied to the documented deliverables/gates or to an explicit decision that a remaining item is deferred with rationale.
