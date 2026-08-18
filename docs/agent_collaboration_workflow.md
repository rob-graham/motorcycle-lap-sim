# Owner, ChatGPT and Codex collaboration workflow

**Status:** recommended repository workflow
**Audience:** repository Owner, ChatGPT, Codex, and external reviewers

## 1. Objective

The workflow is designed to prevent two common failure modes:

1. a change is accepted because unit tests pass even though its real entry point fails; and
2. two people or agents edit the same branch or files and create avoidable merge conflicts.

The central rule is **one task, one change set, one active Implementer**. Every task must
explicitly name its **Implementer** and **Reviewer**. Either ChatGPT or Codex may fill either role,
depending on access and capability, but the same participant should not fill both roles for a
non-trivial change. Other participants may reproduce or advise, but they do not push competing
edits to the change set unless the Owner explicitly transfers the Implementer role.

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
- for Local implementation, normally create and push the named task branch before implementation;
- for Cloud/sandbox implementation, supply the repository, base branch/ref, authoritative starting
  commit SHA, and task identity, without requiring a pre-created Owner branch;
- once a pull request exists, record its actual GitHub head branch and exact head SHA;
- control access to GitHub, external data, and secrets;
- run checks that are unavailable to an agent, especially Windows- or proprietary-data checks;
- review the actual diff and test evidence before merging; and
- merge only after the locally tested `HEAD` and GitHub PR head both equal the exact SHA approved
  in the final review.

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

### Identity terms

The **starting/base SHA** identifies the exact code from which implementation began. It anchors
the task and its base state; it does not identify the proposed result.

The **PR head SHA** identifies the exact result proposed by a pull request. Once the PR exists,
its actual GitHub head branch and exact head SHA are authoritative for review and merge, even if
the branch was generated by a cloud service and was not named before implementation.

### Local implementation

For a Local implementation, the Owner (or one explicitly delegated integrator) normally creates
and pushes the named task branch before starting the Implementer. The primary Owner command path
is Windows PowerShell:

```powershell
$TaskBranch = (Read-Host "Task branch (for example, task/short-description)").Trim()
if (-not $TaskBranch) { throw "A task branch is required." }

git switch main
if ($LASTEXITCODE -ne 0) { throw "git switch main failed." }

git pull --ff-only
if ($LASTEXITCODE -ne 0) { throw "git pull --ff-only failed." }

$WorkingTreeState = git status --porcelain
if ($LASTEXITCODE -ne 0) { throw "git status failed." }
if ($WorkingTreeState) { throw "The working tree is not clean." }

git switch -c $TaskBranch
if ($LASTEXITCODE -ne 0) { throw "Task branch creation failed." }

$CurrentBranch = (git branch --show-current).Trim()
if ($LASTEXITCODE -ne 0) { throw "Reading the task branch failed." }
$StartingSha = (git rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw "Reading the starting SHA failed." }

git push -u origin $TaskBranch
if ($LASTEXITCODE -ne 0) { throw "Task branch push failed." }

[pscustomobject]@{
    Branch = $CurrentBranch
    StartingSha = $StartingSha
}
```

Do not proceed unless `git status --porcelain` is empty. Send the Local Implementer the repository,
base branch/ref, named task branch, starting/base SHA, task identity, Implementer, and Reviewer.
The Implementer verifies both the checked-out branch and starting/base SHA before editing.

### Cloud or sandbox implementation

For Cloud/sandbox implementation, the Owner supplies:

- the repository and base branch/ref;
- the authoritative starting/base SHA;
- the task identity and acceptance criteria; and
- the explicitly named Implementer and independent Reviewer.

The supplied starting/base SHA is the authoritative identity for the code from which work begins.
The service may use an internal sandbox branch and may create the eventual GitHub PR head branch
itself. Neither generated name needs to equal a pre-created Owner branch. Stop if the starting
commit differs from the supplied SHA unless the Owner deliberately refreshes the hand-off.

### Once a pull request exists

Record the PR number/URL, its actual GitHub head branch, and its exact PR head SHA. These become
the authoritative identities for review and merge. The Reviewer must state the exact PR head SHA
reviewed. If the PR head changes for any reason after that review, the new head requires re-review;
an approval or review statement for an earlier SHA does not carry forward.

### While work is in progress

- Do not run a second Implementer on the same task/change set.
- Do not reuse a branch from a merged, rejected, or superseded pull request.
- Keep unrelated fixes in separate branches and pull requests.
- Prefer review comments over direct reviewer commits.
- If authorship must transfer, the first author commits and pushes, then stops. The next author
  fetches the actual PR head, verifies the expected SHA, and becomes the sole active author.
- Never resolve a conflict by accepting an entire side without reviewing the resulting diff and
  rerunning affected checks.

### Before merge: exact reviewed-head gate

The final Reviewer must state the exact PR head SHA reviewed. The Owner then checks out the actual
PR head rather than assuming that a local `task/<short-description>` branch exists. Where GitHub
CLI is available, use this Windows PowerShell procedure with the full reviewed SHA:

```powershell
$PrNumberText = (Read-Host "Pull request number").Trim()
$PrNumber = 0
if (-not [int]::TryParse($PrNumberText, [ref]$PrNumber)) {
    throw "The pull request number must be an integer."
}

$ReviewedHeadSha = (Read-Host "Exact full reviewed PR head SHA").Trim()
if ($ReviewedHeadSha -notmatch '^[0-9a-fA-F]{40}$') {
    throw "The reviewed head SHA must contain exactly 40 hexadecimal characters."
}

$WorkingTreeState = git status --porcelain
if ($LASTEXITCODE -ne 0) { throw "git status failed." }
if ($WorkingTreeState) { throw "The working tree is not clean." }

gh pr checkout $PrNumber
if ($LASTEXITCODE -ne 0) { throw "gh pr checkout failed." }

$WorkingTreeState = git status --porcelain
if ($LASTEXITCODE -ne 0) { throw "git status failed after PR checkout." }
if ($WorkingTreeState) { throw "The checked-out PR working tree is not clean." }

$LocalHeadSha = (git rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw "Reading the local HEAD failed." }
$PrJson = gh pr view $PrNumber --json headRefName,headRefOid,baseRefName
if ($LASTEXITCODE -ne 0) { throw "gh pr view failed." }
$Pr = $PrJson | ConvertFrom-Json
$Pr | Select-Object headRefName, headRefOid, baseRefName

if ($LocalHeadSha -ne $ReviewedHeadSha) {
    throw "Local HEAD does not equal the reviewed head SHA."
}
if ($Pr.headRefOid -ne $ReviewedHeadSha) {
    throw "The GitHub PR head does not equal the reviewed head SHA."
}

git fetch origin $Pr.baseRefName
if ($LASTEXITCODE -ne 0) { throw "Fetching the PR base branch failed." }
$BaseComparison = "origin/$($Pr.baseRefName)...HEAD"
git diff --check $BaseComparison
if ($LASTEXITCODE -ne 0) { throw "git diff --check failed." }
git diff --stat $BaseComparison
if ($LASTEXITCODE -ne 0) { throw "git diff --stat failed." }

python -m pytest
if ($LASTEXITCODE -ne 0) { throw "The full regression suite failed." }

$PostTestWorkingTreeState = git status --porcelain
if ($LASTEXITCODE -ne 0) { throw "git status failed after testing." }
if ($PostTestWorkingTreeState) { throw "Testing changed the working tree." }

$PostTestLocalHeadSha = (git rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw "Reading the post-test local HEAD failed." }
$PostTestPrJson = gh pr view $PrNumber --json headRefName,headRefOid,baseRefName
if ($LASTEXITCODE -ne 0) { throw "The post-test gh pr view failed." }
$PostTestPr = $PostTestPrJson | ConvertFrom-Json

if ($PostTestLocalHeadSha -ne $ReviewedHeadSha) {
    throw "The tested local HEAD no longer equals the reviewed head SHA."
}
if ($PostTestPr.headRefOid -ne $ReviewedHeadSha) {
    throw "The GitHub PR head changed during testing and requires re-review."
}

[pscustomobject]@{
    LocalTestedHeadSha = $PostTestLocalHeadSha
    GitHubPrHeadBranch = $PostTestPr.headRefName
    GitHubPrHeadSha = $PostTestPr.headRefOid
    ReviewedHeadSha = $ReviewedHeadSha
}
```

Stop if either `git status --porcelain` is non-empty, either SHA comparison fails, or any required
gate fails. If GitHub CLI is unavailable, fetch the PR head by its actual GitHub branch or pull
request ref, check out that commit, and perform the same local-HEAD and remote-PR-head comparisons
through the available GitHub UI or API.

Also run every task-specific command listed in the pull request and independently assess all four
completion gates in Section 5. For frozen Mallala baseline changes, also run
`python scripts/r6_phase9_baseline_check.py`.

Merge through GitHub only after required review and checks pass. Then clean up locally:

```powershell
$LocalPrBranch = (git branch --show-current).Trim()
if ($LASTEXITCODE -ne 0) { throw "Reading the local PR branch failed." }
if (-not $LocalPrBranch) { throw "The local PR checkout is detached." }
if ($LocalPrBranch -in @("main", "master")) {
    throw "Check out the local PR branch before running cleanup."
}

git switch main
if ($LASTEXITCODE -ne 0) { throw "git switch main failed." }
git pull --ff-only
if ($LASTEXITCODE -ne 0) { throw "git pull --ff-only failed." }
git branch -d $LocalPrBranch
if ($LASTEXITCODE -ne 0) { throw "Deleting the local PR branch failed." }
git remote prune origin
if ($LASTEXITCODE -ne 0) { throw "git remote prune origin failed." }
```

Delete the actual GitHub PR head branch after merge unless there is a documented reason to retain
it. The local branch name created by `gh pr checkout` need not match an Owner-created task branch.

## 4. Standard hand-off packet

Every coding prompt should contain:

1. **Repository and base identity:** exact repository, base branch/ref, and starting/base SHA.
2. **Branch handling:** for Local work, the named pushed task branch; for Cloud/sandbox work, the
   task identity and permission for the service to create the eventual PR head branch.
3. **Roles:** the named Implementer and independent Reviewer.
4. **Objective:** one bounded outcome.
5. **Out of scope:** nearby work that must not be changed.
6. **Acceptance criteria:** observable behaviour and failure behaviour.
7. **Required reading:** `AGENTS.md`, the three repository context documents, and the relevant
   phase or method document.
8. **Verification:** exact targeted test, runtime command, full-suite command, and any target-
   machine-only check.
9. **Evidence required:** diff summary, assumptions, warnings/skips, exact results, and, once a PR
   exists, its link, actual GitHub head branch, and exact PR head SHA.

A useful prompt footer is:

> You are the `<Implementer|Reviewer>`; `<name>` holds the other role. Use `<starting-sha>` as the
> authoritative starting/base identity. For Local work, use `<task-branch>`; for Cloud/sandbox
> work, the service may create the eventual GitHub PR head branch. Work only on this task; an
> internal sandbox branch name may differ from the GitHub branch name. Do not merge, rebase, or
> edit unrelated files. Once a PR exists, report its actual GitHub head branch and exact head SHA.
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
appropriate branch or update the existing PR head.

## 5. Runtime-error prevention gates

Assess all four gates independently and record each as **Pass**, **Fail**, or **N/A** with evidence
or rationale. A gate may be N/A only when the change cannot affect that category. Never infer N/A
merely because another gate passed. For example, a documentation-only change may record the
representative runtime gate as N/A because no executable path changed.

1. **Static/change gate:** run `git diff --check` and inspect the complete diff.
2. **Targeted gate:** run tests specific to the changed feature, including new physical or
   geometric behaviour where relevant.
3. **Representative runtime gate:** execute the affected CLI, import, loader, workflow, plotting,
   or other user-facing path on the smallest representative case.
4. **Full regression gate:** run `python -m pytest` and report the exact pass/fail/skip result.

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

## 7. Recommended default sequence

For most tasks, use this order:

1. Owner defines scope, task identity, acceptance criteria, starting/base SHA, and explicitly names
   the Implementer and independent Reviewer.
2. For Local work, Owner normally creates and pushes the named task branch. For Cloud/sandbox
   work, Owner supplies the repository and base identity and allows the service to create the
   eventual GitHub PR head branch.
3. The named Reviewer helps make the plan and acceptance criteria analytically testable.
4. The named Implementer (ChatGPT or Codex) implements, assesses all four gates, commits, and
   opens or updates one pull request, or returns the complete fallback evidence if it cannot push.
5. Once the PR exists, record its actual GitHub head branch and exact head SHA.
6. The named Reviewer (Codex or ChatGPT) examines the actual repository, PR, diff, and evidence
   directly when access permits, without editing the branch; pasted material is the fallback. The
   Reviewer states the exact PR head SHA reviewed.
7. The Implementer addresses substantive findings on the same change set and reruns checks. Any
   changed PR head SHA requires re-review.
8. Owner runs any unavailable target-machine/proprietary-data checks, verifies the locally tested
   `HEAD` and GitHub PR head both equal the exact reviewed SHA, reviews the final diff, and merges.
9. Owner deletes the actual PR head branch and starts the next task from updated `main`.

This sequence allocates roles by actual access rather than product name and leaves the Owner with
one clear integration decision rather than several competing code copies.
