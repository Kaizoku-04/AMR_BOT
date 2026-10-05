# Contributing to AMR_BOT (OpenAMR Swarm)

This repo holds the ROS 2 packages of **OpenAMR Swarm**. The team's workflow is defined once, in the companion repo
**`Kaizoku-04/openamr-swarm`**: [`CONTRIBUTING.md`](https://github.com/Kaizoku-04/openamr-swarm/blob/main/CONTRIBUTING.md)
(branches, pull requests, reviews, tests, ROS domains) and `notes/Team.md` (who owns what). In short:

- Branch `<person>/<topic>` -> pull request -> the owner reviews (`.github/CODEOWNERS`) -> squash-merge. No direct or
  force pushes to `main`.
- Changes to `open_amr_msgs` or to how the `/swarm/*` topics are used are **shared-interface changes**: both developers
  review, and `notes/WMS Interface.md` in the companion repo is updated in a linked PR.
- Every change gets an author-tagged bullet in the companion repo's `DEVELOPMENT.md`.
- Tests: CI builds the workspace and runs the unit tests (blocking) on every PR; run them locally first —
  `colcon test --packages-select open_amr_swarm && colcon test-result --verbose` and
  `python3 -m pytest open_amr_docking/test open_amr_arm_cell/test`.
