# CLAUDE.md — AMR_BOT (ROS 2 packages of OpenAMR Swarm)

The ROS 2 Jazzy packages of **OpenAMR Swarm**: a decentralized fleet of OpenAMR robots running a warehouse's box flow,
simulated in Isaac Sim. The project's documentation, rules and roadmap live in the companion repo
**`Kaizoku-04/openamr-swarm`** (usually `~/Robotics/OpenAMR`; `$OPENAMR_ROOT` after `source
$OPENAMR_ROOT/tools/setup/openamr_env.sh`). Read its `CLAUDE.md` first — the rules there apply here too — then
`notes/Team.md` (ownership) and, for anything touching tasks or the WMS, `notes/WMS Interface.md`.
Machine-specific context of the developer you are working for, if any: `CLAUDE.local.md` here (git-ignored).

## Where this repo lives
**`~/ros2_lab/open_amr_ws/src/open_amr`** on every machine (the companion repo's scripts depend on it). Build:
`cd ~/ros2_lab/open_amr_ws && colcon build --symlink-install` (the WMS layer only needs `--packages-up-to
open_amr_swarm`).

## Packages and owners (details: `notes/ROS2 Workspace.md` in the companion repo)
| Package | Owner |
|---|---|
| `open_amr_swarm`: `swarm_agent`, `station_agent`, `battery_sim`, `traffic`/`liveness`/`energy`/`lane_graph` | Mohannad (`Kaizoku-04`) |
| `open_amr_swarm/mission_generator.py` — the WMS stand-in | **Laith** (`xlaithx`) |
| `open_amr_msgs` — the interface between the fleet and the WMS | **shared**: both review |
| `open_amr_arm_cell`, `open_amr_docking`, `open_amr_navigation`, `open_amr_localization`, `open_amr_description`, `open_amr_isaac_hardware`, `open_amr_bringup`, `open_amr_gazebo`, `open_amr_system_tests` | Mohannad |
New WMS / operations packages (e.g. `open_amr_wms`) belong to Laith; add them to `.github/CODEOWNERS`.

## Rules (same as the companion repo)
- Branch `<person>/<topic>` + pull request; the owner reviews (`.github/CODEOWNERS`); interface changes need both.
- Log every change in the companion repo's `DEVELOPMENT.md` (author-tagged bullet under today's date).
- Changing `open_amr_msgs` or topic semantics: update `notes/WMS Interface.md` in the same change (PR in each repo,
  linked); prefer adding fields over changing meanings.
- **The WMS offers tasks and never assigns robots** — no robot field in `Task`, no "send robot X".
- Tests before a PR: `colcon test --packages-select open_amr_swarm && colcon test-result --verbose`;
  `python3 -m pytest open_amr_docking/test open_amr_arm_cell/test` (CI runs both, blocking). Whole-system runs need
  Isaac (Mohannad's Dell): say "needs an Isaac run" in the PR when robots could be affected.
- QoS profiles for `/swarm/*`: import `STATE_QOS` / `EVENT_QOS` / `OPERATOR_QOS` from `open_amr_swarm.agent`.
- Never commit secrets; don't delete files (move to `_archive/` in the companion repo, tell the owner).

## Original upstream content
`README.md` below its first section describes the original single-robot Gazebo platform (still builds; Gazebo is
used for single-robot checks). The open audit file `open_amr_analysis.md` is resolved (git-ignored).
