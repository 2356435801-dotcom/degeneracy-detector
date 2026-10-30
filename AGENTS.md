# Project Development Rules

These instructions apply to work in this repository. Follow any more specific user instruction when it narrows or overrides these defaults.

## Project Scope

- This repository supports the Cartographer 2D SLAM degeneracy-detection research project on ROS 2 Humble.
- Preserve user changes and inspect `git status` before editing, staging, committing, or pushing.
- Do not modify `ros1_original/` unless the user explicitly authorizes it.
- Keep Cartographer and detector changes within the user's requested scope. Do not change frontend matching, Ceres weights, or PoseGraph behavior unless specifically requested.
- Do not clean or remove `build/`, `install/`, or `log/` as a routine step.

## Required End-of-Task Workflow

After completing each full development task or read-only analysis task:

1. Summarize the requested outcome and inspect every changed file.
2. Review `git diff` and `git diff --check`; distinguish changes that existed before this task from changes made for it.
3. Run the relevant build and tests. Report exactly what ran and its result. Do not claim unrun checks passed.
4. Add a task entry to `docs/development_log/YYYY-MM-DD.md` using the system's current local date and update `docs/development_log/README.md`.
5. Review the proposed commit for credentials, private keys, `.env` files, personal data, real user datasets, large rosbag files, temporary logs, build outputs, caches, and unrelated changes. Never stage these files.
6. Stage only files belonging to this task. Inspect `git diff --cached` before committing. Keep one complete task in one commit when practical, with its development log in that commit.
7. Before pushing, confirm the current branch and upstream, inspect whether the remote branch has advanced, and confirm the staged/committed task scope. Use the configured GitHub remote and the current branch only.
8. Push with a normal, non-forcing push. If authentication, branch protection, remote divergence, or another server condition prevents a safe push, stop and report it. Do not change the remote, rewrite remote history, delete branches, create a pull request, or bypass protection.
9. Report the commit, branch, remote, push result, tests, and outstanding issues.

When the relevant build and tests pass, carry out the scoped commit and normal push automatically as part of this workflow; do not wait for a separate user request. Read-only analysis tasks still receive a log entry, with build or test execution recorded as not applicable when appropriate.

Do not include pre-existing or unrelated work in a task commit. Do not use `git add -A` or `git add .` when unrelated work is present. If a task changes a file that also contains earlier user changes, inspect and separate the task's hunks carefully; if they cannot be separated reliably, leave the commit/push pending and report why.

## Commit Messages

Use one of these concise prefixes and describe the task:

- `feat:` new functionality
- `fix:` bug fix
- `refactor:` structural change without intended behavior change
- `test:` test additions or changes
- `docs:` documentation changes
- `build:` build-system changes

## Validation and Failures

- Choose targeted build and tests that match the change; avoid full-workspace builds unless required.
- If a change causes a build or test failure, do not report it as validated. Try to fix the introduced issue and record actual failures and logs/results.
- Distinguish pre-existing environment failures from regressions caused by the task.
- A local checkpoint commit may be used when necessary to preserve incomplete work, but do not push code with a known build/test failure unless the user explicitly requests that exact state be pushed.

## Credentials and Data

- Never print, commit, or document passwords, GitHub tokens, SSH private keys, credential-bearing URLs, or other secrets.
- Treat `.env` files, personal credentials, real user data, large rosbags, temporary logs, build products, and caches as excluded unless the user explicitly gives a safe, task-specific reason to include an item.
- Do not add ignore rules merely to hide uncertain files. Explain the reason and get user direction before changing ignore behavior when needed.

## Development Records

- Keep the dated task record in `docs/development_log/YYYY-MM-DD.md`; append a new section when that date already has entries.
- Keep the index in `docs/development_log/README.md` in reverse chronological order.
- Each entry records local date/time, task/stage/goal, changed files and reasons, implementation, actual build/test commands and outcomes, runtime checks, commit message/id/branch, remote push status and ordinary remote URL, and remaining issues/risks/next steps.
- Never describe a planned or unexecuted build/test as completed.
- The initialization baseline recorded in the first development log is not task work and must not be swept into a later commit.
