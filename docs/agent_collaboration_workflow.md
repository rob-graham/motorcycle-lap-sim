# Owner, ChatGPT and Codex collaboration workflow

**Workflow version:** 1.0  
**Status:** recommended repository workflow  
**Audience:** Owner, ChatGPT, Codex Cloud, Codex desktop/local, and independent reviewers

This document expands the stable rules in `AGENTS.md`. It is a working procedure, not a substitute for task-specific technical judgement.

## 1. Objectives

The workflow is designed to:

- keep one unambiguous active change set and Implementer;
- scale assurance to the consequence of the change;
- expose real runtime and user-visible behaviour early;
- let intermediate results change the plan without losing provenance;
- prevent one participant from assigning another a gate it cannot perform;
- keep chat history from becoming the only record of a decision; and
- make two-repository work and Codex Cloud branch limitations explicit.

The default sequence is **brief -> thin vertical slice -> Owner draft gate -> hardening -> independent review -> exact-head merge gate**. Experiments may stop before production hardening.

## 2. Roles

### Owner

The Owner:

- selects the objective and priority;
- decides interpretation and scope trade-offs;
- approves or rejects user-visible outputs;
- controls proprietary data, credentials, and target-machine access;
- decides disputed review severity;
- authorises merge; and
- conducts or delegates milestone closure review.

The Owner should not need to infer the correct GitHub branch graph. The GitHub Integrator prepares that information.

### ChatGPT

ChatGPT may be Planner, Implementer, Reviewer, or GitHub Integrator according to the tools and environment available. It is normally well suited to:

- reading across roadmap, repository state, prior evidence, and PR history;
- turning the objective into a bounded task brief;
- identifying acceptance criteria, assumptions, and likely failure modes;
- inspecting actual GitHub PR metadata, diffs, and comments;
- preparing Codex Cloud preflight and postflight instructions;
- interpreting numerical, visual, and cross-repository results; and
- maintaining concise current-state and decision records.

Code authored without executable verification remains an unverified patch, not a completed implementation.

### Codex Cloud

Codex Cloud is normally a bounded repository implementation or review worker. Treat it as branch-bound:

- give it one repository, selected base branch, full starting SHA, task identity, scope, and acceptance criteria;
- do not expect it to understand the complete cross-repository or PR-stack state from chat history;
- let it create a result PR when that is the reliable path; and
- verify the actual GitHub base/head relationship after PR creation.

### Codex desktop/local or Owner-machine implementation

Use a persistent local environment when the task needs:

- repeated updates to one branch;
- proprietary retained controls or telemetry;
- Windows launcher behaviour;
- GIS/QGIS/GDAL;
- long-running optimisation;
- target-machine performance; or
- iterative visual review with the Owner.

### Independent Reviewer

The Reviewer inspects the exact proposed result and evidence. Reviewers do not make competing edits unless authorship is explicitly transferred.

### GitHub Integrator

The Integrator verifies:

- starting base and SHA;
- actual PR base/head branches and head SHA;
- temporary child-PR relationships;
- cross-repository merge order; and
- the exact reviewed head immediately before merge.

ChatGPT or the Owner normally fills this role.

## 3. Task packet

Every Standard or High-assurance task should begin with a compact packet:

```markdown
# Task <ID> - <title>

Class: E / L / S / H
Repository:
Base branch and full starting SHA:
Owner:
Implementer:
Reviewer:
GitHub Integrator:
Required capabilities and available environment:

## Objective
One bounded outcome and the downstream reason it is needed now.

## In scope
- ...

## Out of scope
- ...

## Done when
1. Observable success behaviour
2. Required failure behaviour
3. Targeted test
4. Representative runtime path
5. Full-suite or justified alternative
6. Owner/target-machine check, if any

## Required reading
- AGENTS.md
- CURRENT_STATE.md
- one or two task-specific sources

## Preserved behaviour and contracts
- ...

## Stop or pivot conditions
- base SHA differs
- required capability is unavailable
- task cannot be completed without broadening scope
- first result invalidates the planned method

## Evidence to return
- actual PR/base/head/SHA
- changed-file summary
- exact commands/results
- warnings, skips, unavailable checks, limitations
- any recommendation to revise the next increment
```

Do not front-load a complete multi-stage design when the first output is expected to change the plan. Define the next decision and one or two increments in detail; keep later work directional.

## 4. Capability preflight

Before implementation, mark each requirement as available, unavailable, or Owner-only:

| Capability | Status |
| --- | --- |
| GitHub read | |
| GitHub branch/PR write | |
| Repository execution | |
| Dependency installation | |
| Windows target environment | |
| Proprietary Mallala data/retained artefacts | |
| GIS/QGIS/GDAL | |
| Long-running optimisation | |
| Plot/report visual inspection | |
| Other repository access | |

If a mandatory capability is unavailable, change the task plan before code is written. Prefer a repository-contained synthetic or smoke case, followed by a clearly assigned target-machine gate, rather than pretending unavailable end-to-end evidence exists.

## 5. Risk classes and staging

### Class E - experiment/spike

Use when the preferred method is uncertain.

Required before work:

- question and hypothesis;
- baseline/comparison;
- decision metric;
- time, sweep, or evaluation budget;
- production work deliberately excluded; and
- possible outcomes: `adopt`, `revise`, `discard`, or `defer`.

An experiment does not need production API polish, broad compatibility, or complete documentation. Do not merge it merely to preserve effort. If adopted, start a new production task from current `main` or a named integration branch and carry forward only the justified result.

### Class L - low risk

Typical work: prose, comments, narrow test maintenance, or non-executable examples.

Use one Implementer. Run static checks and any directly affected validation. Independent review may be waived by the Owner.

### Class S - standard

Typical work: localised bug fix, CLI/loader/exporter change, contained feature, or report/GIS interaction change.

Use one Implementer, a representative runtime path, targeted tests, full regression, and independent review. Use an Owner draft gate when behaviour is user-visible.

### Class H - high assurance

Typical work: physical model, numerical baseline, optimiser semantics, public/versioned schema, provenance, producer/consumer contract, or consequential interpretation change.

Add:

- short design review before implementation;
- explicit preserved baselines/contracts;
- analytical and failure-mode tests;
- retained-case or target-machine evidence where applicable;
- cross-repository compatibility evidence if relevant; and
- final exact-head independent review.

## 6. Major-task stages

### Stage 0 - decision brief

Define the question, downstream decision, risk class, unknowns, evidence needed, non-goals, required environment, and pivot conditions.

### Stage 1 - thin vertical slice

Exercise the real boundary early. Examples:

- installed console command rather than only a handler unit test;
- one retained artifact rather than only synthetic arrays;
- one producer bundle read by the real consumer;
- one corner through the full run-off/GIS path; or
- one generated report inspected by the Owner.

### Stage 2 - Owner draft gate

For visual, CLI, GIS, report, or authoring work, give the Owner a rough but functional output. Collect one consolidated feedback set and freeze the intended behaviour before extensive polish.

### Stage 3 - production hardening

Generalise only the accepted approach. Add failure handling, necessary tests, stable contracts, and relevant user/method documentation.

### Stage 4 - formal review

Review the frozen increment at an exact head. Avoid repeatedly performing milestone-level review during exploratory iterations.

### Stage 5 - closure review

At a meaningful milestone, record accepted evidence, rejected/deferred approaches, known limitations, document pruning, roadmap impact, and the next one or two increments. Update `CURRENT_STATE.md` once.

## 7. Branch patterns

### Normal task

```text
main
└── task branch -> PR to main
```

Start from current `main`. Record the full starting SHA. One active Implementer owns the branch.

### Codex Cloud temporary correction

Use only when updating the existing parent PR is not reliable:

```text
main
└── parent PR branch
    └── temporary correction PR branch
```

Preflight packet:

```text
Repository:
Parent PR:
Codex branch to select:
Expected full starting SHA:
Child PR must target:
Purpose: correction only
Merge order: child -> parent branch -> re-review parent -> main
```

Postflight checks:

1. child base equals parent head branch;
2. child started from the expected parent head;
3. diff contains only the correction;
4. no sibling child is active;
5. child is merged into parent, then deleted; and
6. parent receives a new exact-head review.

Do not build a chain deeper than a parent and one active child. If several increments are planned, use an integration branch.

### Integration branch

```text
main
└── integration/<milestone> -> final PR to main
    ├── task A PR
    ├── task B PR
    └── task C PR
```

Merge one reviewed child at a time into the integration branch. Every later child starts from the updated integration head. Use the final PR for milestone-level cumulative review.

### Cross-repository task

Use separate branches and PRs in each repository. Never treat matching branch names as proof of compatibility. Record both full SHAs and explicit merge order.

## 8. Cross-repository contract procedure

For `motorcycle-lap-sim` and `motorcycle-runoff`:

1. Assign one shared task ID.
2. Identify producer-owned and consumer-owned decisions.
3. Record producer and consumer starting SHAs.
4. Decide whether the change is additive/backward-compatible or version-breaking.
5. Change the canonical producer contract first when producer semantics change.
6. Update the consumer snapshot with exact source provenance; do not paraphrase a canonical machine-readable contract.
7. Test old/new compatibility as specified by the task.
8. Run one end-to-end bundle using exact producer and consumer commits.
9. Link the paired PRs and state merge order in both descriptions.
10. Merge independently only when the task brief proves that is safe.

The simulator owns output facts and contract semantics. The run-off package owns downstream physical assumptions and presentation. Neither repository silently compensates for a defect in the other.

## 9. Verification evidence

Record each gate independently:

```text
Static/change: Pass / Fail / N/A - evidence
Targeted: Pass / Fail / N/A - commands and results
Representative runtime: Pass / Fail / N/A - real entry point and result
Full regression: Pass / Fail / N/A - exact pass/fail/skip count
Owner/target-machine: Pass / Fail / Pending / N/A - evidence and assignee
```

Rules:

- A known runtime error blocks executable work even when tests pass.
- Help output alone does not prove a successful command path.
- Do not hide optional-dependency skips.
- Do not claim proprietary or GIS paths were tested when unavailable.
- For long optimisation, record policy, controls, grid, boundary margin, backend/workers, limits, termination reason, warnings, and final authoritative evaluation.
- The Owner's visual acceptance validates usefulness/presentation, not numerical correctness.

## 10. Review protocol

A review prompt should say:

```markdown
Review exact PR head <SHA> against <base> and the existing task brief.
Inspect the actual diff, affected runtime path, tests/evidence, and agreed assumptions.
Do not expand the increment merely to add desirable future hardening.
Classify each finding as Blocker, Significant, Follow-up, or Nit.
State the exact SHA reviewed and return merge/request-changes.
```

Severity meanings are defined in `AGENTS.md`.

First review: inspect the full change. Later reviews: inspect the delta plus affected regression risk. A head change always invalidates an earlier exact-head approval, but it does not require re-reading unrelated unchanged history.

After two correction cycles, the Owner explicitly chooses merge-with-follow-up, split/rescope, redesign, or abandon.

## 11. Chat and hand-off discipline

Use separate chats for:

- milestone planning;
- one PR's implementation/review; and
- milestone closure/retrospective.

Start a new chat after merge, objective change, experiment-to-production transition, more than two review cycles, or when multiple PRs/branches make the current state ambiguous.

Close a chat with a short durable hand-off:

```text
Objective:
Repository / PR / exact SHA:
Decisions made:
Implemented result:
Verification completed:
Known limitations:
Open Owner decision:
Next single action:
Required sources:
```

Important decisions must also be recorded in the task/PR, `CURRENT_STATE.md`, a method document, or a decision log. Chat memory alone is not project state.

## 12. Pull-request evidence

A PR should include:

- task ID and class;
- base branch and starting SHA;
- roles;
- bounded objective, in-scope and out-of-scope work;
- important assumptions and preserved behaviour;
- all verification gates;
- Owner draft/target-machine status;
- cross-repository links and merge order, if any;
- warnings, skips, unavailable checks, and known limitations; and
- actual GitHub head branch and the final reviewed head SHA in the final review comment.

Do not continually edit a PR body to claim a mutable "current head". GitHub metadata is authoritative. Use the starting SHA in the body and the exact final reviewed head in the review record.

## 13. Merge and cleanup

Immediately before merge, the Owner or Integrator verifies:

- PR base/head identity;
- GitHub head equals the reviewed SHA;
- locally tested head equals the reviewed SHA when local checks are required;
- required evidence is attached;
- no unresolved Blocker or Significant finding remains; and
- any Follow-up is clearly non-blocking and recorded only if worth retaining.

After merge, update local `main`, delete the actual PR head branch, prune stale branches, and start the next task from the new `main`.

## 14. Retrospective cadence

After a milestone or several consequential PRs, review:

- review/correction cycles;
- post-merge fixes;
- whether the real runtime path was exercised early;
- abandoned/superseded work;
- mandatory reading burden;
- findings later downgraded to optional;
- documentation duplication; and
- one process improvement to pilot next.

Metrics are diagnostic, not performance targets.