# Repository instructions

This file contains stable rules for the repository. Volatile phase status, current priorities, retained results, and deferred work belong in [`CURRENT_STATE.md`](CURRENT_STATE.md).

## 1. Normative terms

- **MUST** and **MUST NOT** are mandatory unless the Owner explicitly changes the rule for a named task before implementation begins.
- **SHOULD** describes the normal path. A deviation requires a short recorded reason.
- **MAY** describes an optional practice.

Do not turn every preference into a merge blocker. Correctness, provenance, numerical integrity, contract compatibility, and truthful evidence remain mandatory.

## 2. Source hierarchy

Use sources according to their purpose:

1. `AGENTS.md` governs repository process and stable implementation invariants.
2. `CURRENT_STATE.md` records the current operational state and near-term direction.
3. Current code, tests, cases, configuration, contracts, and repository documentation define implemented behaviour.
4. Task-specific method and closure documents define the accepted method or evidence for their subject.
5. External project roadmap files define intended direction, not implemented capability.
6. A task brief may narrow scope but MUST NOT silently override an invariant, frozen result, or versioned contract.

When sources conflict, stop and ask the Owner or record the conflict in the task brief rather than choosing silently.

## 3. Engineering invariants

- This is a clean-sheet implementation. Do not import old lap-time simulator source code unless explicitly requested.
- Use SI units internally.
- Physical formulas belong in clearly identified functions or modules.
- Do not use mutable module-level global state.
- Keep plotting and presentation separate from numerical calculations.
- Every new physical or geometric feature requires meaningful tests, including analytically understandable limiting cases where practical.
- Preserve deterministic calculations unless nondeterminism is an explicit, tested requirement.
- Record assumptions and provenance explicitly.
- Never suppress numerical, optimisation, data-quality, contract, or provenance warnings merely to make tests pass.
- Do not weaken tolerances, disable checks, catch-and-ignore exceptions, or convert failures into warnings merely to obtain a green result.
- Maintain backward-compatible data formats only when deliberately specified.
- Preserve the distinction between numerical regression, physics sensitivity, optimisation response, telemetry validation, and presentation evidence.
- No repository output claims regulatory approval, homologation, certification, insurance acceptance, or universally safe rider guidance.

## 4. Required reading by task

Every task MUST read this file and `CURRENT_STATE.md`. Additional reading is task-specific:

| Task | Additional required reading |
| --- | --- |
| Documentation-only | The target document and any document it supersedes |
| Localised CLI, loader, exporter, or utility | The affected module, user guide, and focused tests |
| Motorcycle physics or speed solver | `docs/system_spec.md`, the relevant model/method document, and analytical tests |
| Racing-line optimisation | `docs/direct_planar_racing_line_optimisation.md` and the relevant closure/method document |
| Mallala retained workflow or provenance | The relevant case manifest, closure document, and executable acceptance script |
| Simulator-to-run-off interface | `contracts/runoff_interface_0.1.0.json`, `docs/runoff_input_contract.md`, and georeference contract when applicable |
| Architecture, roadmap, or phase planning | `docs/project_context.md`, `docs/system_spec.md`, and the current external roadmap/index supplied by the Owner |

Do not require broad historical reading for a minor change when the task-specific sources are sufficient.

## 5. Roles and capability declaration

Every Standard or High-assurance task MUST name:

- **Owner** — priority, technical decisions, target-machine evidence, and merge authority;
- **Implementer** — the one active author of the change set;
- **Reviewer** — independent review of the exact proposed result; and
- **GitHub Integrator** — branch/PR identity and merge-order verification, which may be performed by the Owner or ChatGPT.

Before editing, the task packet MUST state which capabilities are required and which are available:

- GitHub read and write access;
- repository execution and dependency installation;
- Windows or other target environment;
- proprietary Mallala data or retained artefacts;
- GIS/QGIS/GDAL;
- long-running optimisation capacity;
- visual inspection of plots or reports; and
- access to the other repository for cross-repository work.

Do not assign an unavailable gate to an agent. Split it into an explicit Owner or target-machine hand-off.

Use one active Implementer per change set. The Reviewer does not make competing edits unless the Owner explicitly transfers the Implementer role.

## 6. Change classes

Classify the task before implementation:

- **E — Experiment/spike:** uncertain method or bounded investigation. Time-box it, define a hypothesis and comparison, and finish with `adopt`, `revise`, `discard`, or `defer`. Production merge is not presumed.
- **L — Low risk:** prose correction, comments, narrow test maintenance, or similarly non-executable change. Independent review MAY be waived by the Owner.
- **S — Standard:** localised bug fix, CLI or serializer change, contained feature, or user-facing workflow change. Independent review is normally required.
- **H — High assurance:** physical model, numerical baseline, optimisation semantics, provenance, public/versioned contract, cross-repository interface, or change with material interpretation consequences. Design review before implementation and independent final review are required.

If the risk becomes higher during implementation, stop and reclassify before broadening the change.

## 7. Branch and pull-request rules

- Start from current `main` unless an explicit dependency requires another base.
- Record the repository, base branch, and full starting SHA before editing.
- Use one focused branch and PR per independently reviewable increment.
- Do not continue development on a merged, rejected, superseded, or stale branch.
- Keep unrelated fixes separate.
- Once a PR exists, its actual GitHub base branch, head branch, and exact head SHA are authoritative.
- The Reviewer MUST state the exact head SHA reviewed. Any later head change requires review of the new delta and any affected behaviour.
- Delete merged or deliberately abandoned head branches unless there is a recorded reason to retain them.

### Codex Cloud correction PRs

Updating an existing PR branch is preferred only when the task can do so unambiguously. If Codex Cloud cannot reliably update the parent PR, one temporary child correction PR is permitted:

```text
main
└── parent feature PR
    └── temporary correction PR
```

The GitHub Integrator MUST verify before coding:

- the parent PR's actual head branch and full head SHA;
- the branch selected in Codex;
- the child PR's intended base; and
- the merge order.

After the child PR is created, verify that its base equals the parent head branch and that its diff contains only the correction. Merge the child into the parent branch, delete the child branch, then re-review the resulting exact parent head before merging to `main`.

Only one active child correction PR is allowed per parent. Avoid stacks deeper than two PR levels. For several planned increments, use a named integration branch instead of a deep chain.

Detailed procedures and prompt templates are in [`docs/agent_collaboration_workflow.md`](docs/agent_collaboration_workflow.md).

## 8. Verification scaled to risk

Assess these gates independently as **Pass**, **Fail**, or justified **N/A**:

1. **Static/change gate:** inspect the complete diff and run `git diff --check` or equivalent.
2. **Targeted gate:** run focused tests for changed behaviour.
3. **Representative runtime gate:** execute the affected CLI, import, loader, exporter, optimisation path, plotting path, or other real entry point on the smallest representative case.
4. **Full regression gate:** run `python -m pytest` for executable Standard and High-assurance changes unless the task brief records a justified alternative.
5. **Owner/target-machine gate:** run checks requiring proprietary data, Windows, GIS, long optimisation, or visual judgement.

Typical expectations:

| Class | Minimum evidence |
| --- | --- |
| E | Static check, experiment-specific comparison, limitations, and decision record |
| L | Static check and any directly affected documentation/example validation |
| S | Static, targeted, representative runtime, full regression, and independent review |
| H | S gates plus design review, provenance/contract evidence, target-machine or retained-case evidence where applicable, and final exact-head review |

A code change is not complete while it has a known runtime error, even if unit tests pass. Unit tests do not replace execution of an affected user-facing path.

For frozen Mallala baseline changes, run `python scripts/r6_phase9_baseline_check.py`. For long optimisation work, record controls/policy, grid and boundary settings, backend/workers, limits, termination reason, warnings, and authoritative common-grid re-evaluation.

## 9. Owner draft gate

For plots, reports, GIS output, CLI interaction, authoring workflows, or other user-visible behaviour, produce the smallest working output early and obtain one consolidated Owner review before extensive polish or formal final review.

A Draft PR MAY be used during this stage. Owner acceptance of usefulness does not replace numerical, runtime, or contract verification.

## 10. Review findings and scope control

Classify every review finding:

- **Blocker:** incorrect result, broken runtime path, data loss, false provenance, contract incompatibility, or material numerical/physical defect. Must resolve.
- **Significant:** an agreed acceptance criterion is not met or a credible material regression remains. Normally resolve before merge.
- **Follow-up:** useful hardening, maintainability, broader coverage, or future resilience outside the bounded increment. Does not block merge.
- **Nit:** style or preference. Never blocks merge.

The Reviewer assesses the agreed task and normal correctness; review MUST NOT silently expand the increment. A desirable enhancement may block only when it reveals a Blocker or Significant defect in the proposed change.

After two correction cycles, the Owner chooses: merge with recorded follow-up, split/rescope, redesign, or abandon. Do not review indefinitely by repeatedly converting optional hardening into new acceptance criteria.

## 11. Cross-repository changes

`motorcycle-lap-sim` owns the canonical simulator output and versioned producer contracts. `motorcycle-runoff` consumes released bundle/contracts without importing simulator internals.

A cross-repository task MUST record:

- a shared task identity;
- the starting SHA and PR in each repository;
- which repository owns each contract or semantic decision;
- compatibility and merge order;
- the exact producer and consumer commits used for end-to-end evidence; and
- whether either PR can merge independently.

Each repository receives its own branch, PR, tests, and exact-head review. Do not hide a two-repository change inside one repository's chat history.

## 12. Merge gate and evidence

Before merge:

- the final Reviewer states the exact PR head SHA reviewed;
- the Owner or Integrator verifies the local tested head and GitHub PR head match that SHA;
- required checks and target-machine evidence are attached or linked;
- substantive findings are resolved or explicitly dispositioned; and
- the PR description identifies assumptions, warnings, unavailable checks, and known limitations.

If an environment cannot push or create a PR, return the verified patch/diff, starting SHA, resulting commit SHA if any, exact commands/results, warnings/skips, and a clear Owner hand-off. Never fabricate a PR URL or claim unperformed verification.