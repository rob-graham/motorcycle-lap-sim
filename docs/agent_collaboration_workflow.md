# Owner, ChatGPT and Codex collaboration workflow

**Status:** recommended repository workflow
**Audience:** repository Owner, ChatGPT, Codex, and external reviewers

## 1. Objective

The workflow is designed to prevent two common failure modes:

1. a change is accepted because unit tests pass even though its real entry point fails; and
2. two people or agents edit the same branch or files and create avoidable merge conflicts.

The central rule is **one task, one branch, one active Implementer**. Every task must explicitly
name its **Implementer** and **Reviewer**. Either ChatGPT or Codex may fill either role, depending
on access and capability, but the same participant should not fill both roles for a non-trivial
change. Other participants may reproduce or advise, but they do not push competing edits to the
task branch unless the Owner explicitly transfers the Implementer role.

### Source precedence

Current `main` repository documentation and `AGENTS.md` govern current implementation state,
phase status, and the next repository task. ChatGPT Project snapshot files remain useful for the
wider roadmap direction unless a current project decision deliberately supersedes them. A task
prompt may narrow a change, but should not silently override repository state or phase status.

## 2. Recommended responsibilities

### Owner: integrator and release gate

The Owner should:

- choose the task and acceptance criteria;
- explicitly name the Implementer and independent Reviewer;
- create or select the correct branch and state its exact name in every hand-off;
- control access to GitHub, external data, and secrets;
- run checks that are unavailable to an agent, especially Windows- or proprietary-data checks;
- review the actual diff and test evidence before merging; and
- merge only the pull request whose head branch was named in the task.

The Owner should normally avoid editing an agent's in-progress branch. If an urgent Owner edit
is necessary, stop the agent first, commit the Owner edit, and tell the agent the new commit SHA.

### ChatGPT: eligible Implementer, planner, or Reviewer

When ChatGPT has connected GitHub access, it should inspect the actual repository, pull request,
and diff directly. Pasted diffs, logs, plots, and file excerpts are fallback evidence when direct
access is unavailable. Depending on the task and available tools, ChatGPT may be the named
Implementer or Reviewer.

When ChatGPT cannot execute the repository, use it primarily to:

- turn the engineering objective into bounded acceptance criteria;
- identify affected files, physical assumptions, tests, and runtime checks;
- prepare copy-and-paste command blocks for the Owner;
- review a pasted diff, pull-request URL, logs, plots, or failure output; and
- diagnose failures without proposing broad unrelated rewrites.

If ChatGPT authors code that it cannot execute, treat the result as an **unverified patch**, not
as a completed change. The Owner should apply it on a dedicated branch and return exact command
output before further development or merge.

### Codex: eligible Implementer or Reviewer with executable verification

Use Codex as Implementer when its environment contains the needed repository, dependencies, and
data; it may instead be the independent Reviewer when ChatGPT implements. In either role, Codex
should:

- inspect repository instructions and current documentation before editing;
- verify the supplied starting commit SHA before editing;
- report missing dependencies or data rather than hiding skips or warnings;
- inspect the actual diff and executable behaviour rather than relying on a summary; and
- provide the exact commands, results, assumptions, and limitations.

As Implementer, Codex should make the smallest coherent change with tests, run targeted tests,
the affected runtime entry point, and the full test suite, then commit and open a pull request
when access permits. As Reviewer, it should not make competing edits; it should independently
inspect the change and rerun appropriate checks when its environment permits.

An inability to import an optional library does not justify changing production behaviour or
weakening a test. Use the supported non-accelerated path for coverage when appropriate, then
record the unavailable accelerated check for the Owner to run on the target machine.

### External reviewers and clients: evidence review

Give reviewers a pull-request link or immutable commit SHA, scope statement, assumptions, and
reproduction commands. Ask them to comment on the pull request rather than sending parallel
edited copies. Client approval of an output is not a substitute for numerical or runtime
verification.

## 3. Branch protocol

### Start a task

Only the Owner (or one explicitly delegated integrator) should allocate branch names. Before
starting Codex, the Owner can run this uninterrupted command block in the VS Code terminal:

```bash
git switch main
git pull --ff-only
git status --short
git switch -c task/<short-description>
git branch --show-current
git rev-parse HEAD
git push -u origin task/<short-description>
```

Do not proceed unless `git status --short` is empty. Send the agent both the branch name and the
starting SHA. For a locally attached agent, verify the checked-out task branch. For a cloud or
sandbox agent, the supplied starting SHA is the authoritative identity: its internal sandbox
branch name need not exactly match the GitHub branch name. Stop if its starting commit differs
from the supplied SHA unless the Owner deliberately refreshes the hand-off.

### While work is in progress

- Do not run a second Implementer on the same task branch.
- Do not reuse a branch from a merged, rejected, or superseded pull request.
- Keep unrelated fixes in separate branches and pull requests.
- Prefer review comments over direct reviewer commits.
- If authorship must transfer, the first author commits and pushes, then stops. The next author
  fetches the branch, verifies the expected SHA, and becomes the sole active author.
- Never resolve a conflict by accepting an entire side without reviewing the resulting diff and
  rerunning affected checks.

### Before merge

The Owner should fetch the proposed head and verify it explicitly:

```bash
git fetch origin
git switch task/<short-description>
git pull --ff-only
git status --short
git log -1 --oneline
git diff --check origin/main...HEAD
git diff --stat origin/main...HEAD
python -m pytest
```

Also run every task-specific command listed in the pull request. For a command-line or
user-facing path, run the smallest representative invocation; unit tests alone are insufficient.
For frozen Mallala baseline changes, also run `python scripts/r6_phase9_baseline_check.py`.

Merge through GitHub only after required review and checks pass. Then clean up locally:

```bash
git switch main
git pull --ff-only
git branch -d task/<short-description>
git remote prune origin
```

## 4. Standard hand-off packet

Every coding prompt should contain:

1. **Repository and branch:** exact repository, branch name, and starting SHA.
2. **Roles:** the named Implementer and independent Reviewer.
3. **Objective:** one bounded outcome.
4. **Out of scope:** nearby work that must not be changed.
5. **Acceptance criteria:** observable behaviour and failure behaviour.
6. **Required reading:** `AGENTS.md`, the three repository context documents, and the relevant
   phase or method document.
7. **Verification:** exact targeted test, runtime command, full-suite command, and any target-
   machine-only check.
8. **Evidence required:** diff summary, assumptions, warnings/skips, exact results, commit SHA,
   and pull-request link.

A useful prompt footer is:

> You are the `<Implementer|Reviewer>`; `<name>` holds the other role. Use `<sha>` as the
> authoritative starting identity. Work only on this task; an internal sandbox branch name may
> differ from the GitHub branch name. Do not merge, rebase, or edit unrelated files.
> Run the required targeted test, smallest representative runtime command, and
> `python -m pytest`. Preserve all warnings and skips in the report. Commit the change and open
> one pull request when the environment permits. Stop and report if the starting SHA differs.

### Push/PR fallback

If an environment cannot push or create a pull request, it must return:

- the verified patch or complete diff;
- the supplied starting SHA;
- the resulting commit SHA, if the environment created a commit;
- exact commands and test results, including warnings and skips; and
- an explicit statement that the Owner must apply and/or push the change and create the PR.

Never fabricate a PR URL or imply that a local commit is available on GitHub. The Owner should
verify the starting SHA before applying the patch, rerun the required checks, and then push the
named GitHub task branch.

## 5. Runtime-error prevention gates

Use all four gates; no participant should infer one from another:

1. **Static/change gate:** inspect `git diff --check` and the complete diff.
2. **Targeted gate:** run tests for the changed feature, including new physical or geometric
   behaviour.
3. **Runtime gate:** execute the actual CLI, import, loader, workflow, or plotting path on the
   smallest representative case.
4. **Regression gate:** run `python -m pytest` and report the exact pass/fail/skip result.

Where the agent environment lacks Numba or another optional dependency, establish whether the
repository's declared base dependencies installed correctly before diagnosing the code. Do not
silently install unrecorded packages and then claim the repository is reproducible. Record the
environment, Python version, install command, missing check, and a command for the Owner's target
machine. Proprietary data should remain outside Git; use documented hashes/manifests and a
repository-contained synthetic case whenever possible.

## 6. Access the Owner can safely provide

The most useful additional access is reproducible, least-privilege access rather than broad
desktop control:

- grant the coding service access only to the required GitHub repository or organization;
- provide a repository bootstrap script or documented environment file that installs the same
  Python version and dependencies as the target machine;
- make non-secret test fixtures available in the repository or an approved artifact store;
- configure required secrets in the coding environment's secret store, scoped to the task and
  never pasted into prompts, commits, logs, or pull requests;
- provide a Windows CI runner or Owner-run command block for checks that genuinely require
  Windows, licensed software, hardware, or proprietary data; and
- use branch protection, pull-request review, and required CI checks so access cannot bypass the
  integration gate.

Do not give an agent the Owner's personal browser session, personal access token, SSH private key,
or unrestricted machine access merely to avoid a hand-off. Prefer a dedicated account or app
installation with read/write access to this repository and no permission to merge protected
branches.

## 7. Windows application troubleshooting

A blank desktop-app window is an access/support problem, not a reason to change repository or
branch policy. Product availability, supported Windows versions, account entitlements, and app
requirements can change, so confirm them against current official OpenAI documentation or
Support before reinstalling repeatedly.

For a useful support report, record:

- the exact product name and installer source (ChatGPT and Codex are distinct surfaces);
- Windows edition, version, and build;
- app version, account/workspace type, and whether browser sign-in succeeds;
- whether the blank window persists after a full exit/restart and after disabling VPN/proxy;
- relevant timestamp, screenshot, and Windows Event Viewer entry; and
- whether the browser and terminal/IDE alternatives still work.

Use only an official installer. Do not disable security software permanently or delete unrelated
application data. If the basic checks do not resolve the problem, send the recorded evidence to
OpenAI Support and continue with the browser plus VS Code workflow; the collaboration protocol
above does not depend on the desktop app.

## 8. Recommended default sequence

For most tasks, use this order:

1. Owner defines scope, names the Implementer and Reviewer, creates and pushes the branch, and
   sends the hand-off packet with the authoritative starting SHA.
2. The named Reviewer helps make the plan and acceptance criteria analytically testable.
3. The named Implementer (ChatGPT or Codex) implements, tests, runs the real entry point, commits,
   and opens the pull request, or returns the complete fallback evidence if it cannot push.
4. The named Reviewer (Codex or ChatGPT) examines the actual repository, PR, diff, and evidence
   directly when access permits, without editing the branch; pasted material is the fallback.
5. The Implementer addresses substantive review findings on the same task branch and reruns checks.
6. Owner runs any unavailable Windows/proprietary-data checks, reviews the final diff, and merges.
7. Owner deletes the task branch and starts the next task from updated `main`.

This sequence allocates roles by actual access rather than product name and leaves the Owner with
one clear integration decision rather than several competing code copies.
