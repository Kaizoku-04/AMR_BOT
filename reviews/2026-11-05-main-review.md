# Review of AMR_BOT `main` 77e7263..4206a0a (27 commits) and draft PR #5 — 2026-11-05

Reviewer: Claude (cloud session, for Mohannad). Scope: correctness bugs in the robot-side work merged on 2026-11-05
(lost-message guards, controller_starter, Pilz LIN joint goal, docking TF tolerance, station lost-goal guards,
re-localization at the bay, bay-exit back-up, WMS kind share / stale-station log, detector stamp fix, EKF
`predict_to_current_time`, standstill dock check, 905 budget) and `mohannad/site-adr-030`. Line numbers are of
`4206a0a`. Style is out of scope. Reproducers for findings 1 and 2 are in this folder (`repro_test_*.py`); each fails
on `4206a0a`.

## Findings, by severity

### 1. HIGH — A task dropped while its robot docks leaves the DockRobot goal running
- **Where:** `open_amr_swarm/open_amr_swarm/agent.py:852` `abort_task()` → `stop_following()` (`agent.py:1506`) cancels
  only the FollowPath goal; `dock_handle`, `backout_handle` and `exitbk_handle` (added by `9ef84a7`, `e0c9f87`) are
  neither cancelled nor cleared, and `dock_state` stays.
- **Commit:** gap present since docking was added; `9ef84a7` introduced the handles that would make it fixable.
- **Why it is wrong:** every path that drops a task can run during the pickup dock at an arm bay (`loaded` is still
  false there): the WMS's CANCEL (`agent.py:355-360`, the "claimed in the same instant it was withdrawn" race the WMS
  itself handles), a peer's outranking claim or heartbeat (`agent.py:305-312`, `351-354`), a release on my behalf
  (`agent.py:368-375`), `give_back()` on an arm out of service (`agent.py:1689`).
- **Failure scenario:** amr_2 is docking at `receiving_bay` for t0042; the WMS withdraws t0042 (CANCEL). The agent goes
  IDLE (reserves only its last node), may win another task and send a FollowPath while the docking server is still
  driving the robot into the bay; two action servers command the base. If it doesn't win one, the dock completes,
  `on_dock_done` runs the settle / check / re-localize path for a task that no longer exists and reports `docked`
  with no task: the station never serves it (`station_agent.at_bay` needs a `task_id`), and the robot stands nose-in at
  the pedestal, where a forward-only path cannot leave (the 2026-10-03 deadlock note at `agent.py:384`).
- **Evidence:** `reviews/repro_test_abort_while_docking.py` fails on `4206a0a`: after a WMS CANCEL during docking the
  task is gone and the dock handle was not cancelled.
- **Suggested fix:** in `abort_task()`, cancel and clear `dock_handle` / `backout_handle` / `exitbk_handle`, clear their
  `Pending`s (a late acceptance is then cancelled on arrival), reset `dock_state`, and if `in_bay()` back out to the
  staging node (`back_out()` without a task, then idle). Needs an Isaac check (abort mid-dock).

### 2. HIGH — Station: a resent transfer goal rejected by the busy cell is booked as a failed job while the first goal runs
- **Where:** `open_amr_swarm/open_amr_swarm/station_agent.py:142-150` `guard_lost()` resends after `ack_timeout`;
  `station_agent.py:105-117` `on_accepted()` books a rejection as `finished(False, C_NONE)`; `station.py`
  `CellStation.finished()` then puts the carton back on the pallet count (`_put`) and sets FAULT; the next tick clears
  FAULT and the station starts another job.
- **Commit:** `40439c9` (station lost-goal guards).
- **Why it is wrong:** the guard assumes a resent goal replaces the first. The cell does not allow that: "One goal at
  a time: a goal sent while another runs ... is rejected" (`open_amr_msgs/action/ArmTransfer.action`), implemented in
  `open_amr_arm_cell/cell_controller.py:237-247` (`busy`). If the cell accepted the first goal and only the
  acceptance reply was lost — the loss the guard exists for — the resend is rejected, the station concludes the job
  failed with nothing picked, and the first goal keeps running with no handle (its acceptance never arrives, so the
  "cancel a late acceptance" path never runs).
- **Failure scenario:** PREPARE pallet 0 slot 23 is accepted, its reply lost; 3 s later the resend is rejected
  (busy). The station counts the carton back on pallet 0 and sends a new PREPARE, which the busy cell rejects again,
  until the orphan PREPARE ends with a carton on the tool that the station doesn't know about; the stock is off by
  one and the next PREPARE starts with the tool full. With a DEPALLETIZE (robot at the bay) the orphan puts the carton
  on the deck while the station books "rejected", never marks the robot served, and later sends another DEPALLETIZE
  for the same robot: a second carton onto an occupied deck.
- **Evidence:** `reviews/repro_test_station_resend_busy.py::test_resend_rejected_by_the_busy_cell_while_the_first_goal_runs`
  fails on `4206a0a`: `carton booked back on the pallet: [24, 24]` (expected `[23, 24]`).
  The existing `test_late_acceptance_of_an_abandoned_goal_is_cancelled` accepts the resend while the first goal is
  accepted too — a case the real cell rejects, so the test passes on a sequence that cannot happen.
- **Suggested fix:** state over events: a rejected **resend** (`req.tries > 1`) is not a failure; keep the job in flight
  and settle it from the cell's heartbeat (`ArmCellState`: `state` BUSY with `phase`, then `cycles` incremented and
  `carton_on_tool`, back to READY / HOLDING), or have the cell accept an identical duplicate goal (same `task_id`,
  pallet, slot) as the running one. Add the reproducer as a test.

### 3. MEDIUM — Re-localization at the bay never checks or corrects the heading
- **Where:** `agent.py:1342-1352` `bay_pose_from_dock()` returns `yaw = self.pose[2]` (AMCL's own heading);
  `agent.py:1358-1365` decides only on the position error; `agent.py:1372-1378` resets AMCL with that heading and a
  2° sigma.
- **Commits:** `1e78fd4`, `3dd3ddd`.
- **Why it is wrong:** the bay's surveyed yaw is known (`sim/worlds/warehouse_mission_graph.yaml`: `receiving_bay` yaw
  180°) and the marker gives the dock's yaw error (`dock_error()[2]`), but neither is used. The cell's interlock checks
  the reported heading too: `open_amr_arm_cell/cell_controller.py:80,207` (`bay_tol_deg` 10°).
- **Failure scenario:** AMCL's heading is 12° off at a bay with the position within 0.10 m (Results has a 37.9° AMCL
  outlier, c45_truth): the agent reports docked at once (no reset), the cell's interlock never sees the robot at its
  bay, and the arm waits — the c41_base symptom, for heading instead of position. Below 10° the reset locks AMCL's
  heading error in with a 2° covariance.
- **Suggested fix:** give the agent the bays' yaws (graph yaml or the dock database), set the reset heading to
  `bay_yaw + err[2]`, rotate `err` by the bay yaw (not AMCL's), and include the heading in the "off" check.

### 4. MEDIUM — The 905 budget treats every FAILED_TO_CONTROL as a late transform
- **Where:** `agent.py:1451-1463` (`dock_failed`): any `DockRobot.Result.FAILED_TO_CONTROL` uses the 5-per-visit budget.
- **Why it matters:** the evidence (49 × 905, all with "Could not get transform") holds for the runs seen; a 905 from
  a real control failure (an obstacle, a stuck caster, a deck collision) now costs up to 5 extra dock attempts at the
  bay before it counts, with the queue behind it waiting.
- **Suggested fix:** count against the budget only when `error_msg` names the transform failure (the server's message
  is in the result); others count as attempts as before. Log both.

### 5. LOW — The standstill dock check accepts a dock it couldn't judge
- **Where:** `agent.py:1315-1340` `dock_error_from()` returns None on a TF exception; `check_dock()`
  (`agent.py:1267`) treats `err is None` as accurate and goes on to re-localize / mark docked; `bay_pose_from_dock()`
  then returns None and `localize_at_bay()` marks docked without a reset (`agent.py:1364`).
- **Commit:** `c9cc562` (the same behaviour existed for the server pose).
- **Failure scenario:** `lookup_transform(odom, base_footprint, latest)` fails once (TF buffer not filled after a
  restart of the localization stack): the dock is reported without any accuracy check. The cell's dock sensor still
  guards the arm (ADR-018), so the cost is a cell stop, not a collision.
- **Suggested fix:** an unknown error retries the check on the next ticks (bounded), then falls back to the server
  pose, then counts as an inaccurate dock.

### 6. LOW — The bay-exit back-up measures its distance in AMCL's pose
- **Where:** `agent.py:1552-1567` `leave_bay()`: `exitbk_from` and `moved_since()` use `self.pose` (map →
  base_footprint, AMCL).
- **Commit:** `dfa94fb`. The docstring says the distance never comes from AMCL's pose of the bay node, which holds,
  but AMCL resumes updating once the robot moves and can jump by centimetres; on a resend (`retry=True`) the remaining
  distance is computed across that jump.
- **Suggested fix:** measure in the odom frame (`odom → base_footprint`).

### 7. LOW — Three negative controls fail only by import error
- `7ea3439` (`test_traffic.py`), `38737e8` (`test_agent_dock.py`) and `1e78fd4` (`test_relocalize_at_bay.py`): with
  the commit's own code reverted the tests error at import (a new name is missing), so they show the code is there,
  not that the behaviour is right. The later fixes in the same files have behavioural tests (below), so the risk is
  small.

### Draft PR #5 (`mohannad/site-adr-030`, `f48a0a1`: carrying PTP moves at 0.75× speed and acceleration)
No correctness defect found. Checked: the slow profile applies only to PTP with a carton (`cell_motion.py:208`
`self.carrying is not None`; `carrying` is the carton's source string or None, `cell_motion.py:131,287,298`);
`(True, True)` shares the LIN profile object, so the `vel_scale` / `acc_scale` overrides reach every profile once;
`lin_place` is not scaled by the override (as before). Not verifiable here (no MoveIt): the 36.0 N / 2.2× shear figure
and the cycle-time cost — the arm cycle grows, so `station_dwell_s` / `arm_cycle_s` (8 s) and the fleet baseline
(76/h) should be re-measured in the Isaac run the PR already waits for.

## Negative controls (each fix reverted, its tests run)

Method: for each commit that adds or changes tests, its non-test changes were reverse-applied on top of `4206a0a`
and its tests run; where that didn't apply cleanly, the tests were run at the commit with its parent's code.

| Commit | Tests | With the fix | Fix reverted |
|---|---|---|---|
| 7ea3439 traffic rule 6 | test_traffic.py | 14 passed | import error (finding 7) |
| 38737e8 back out to staging | test_agent_dock.py | 9 passed (at commit) | import error (finding 7) |
| 76e95d4 unsendable segment | test_agent_dock.py | 15 passed | 1 failed |
| 9ef84a7 dock / back-out guards | test_agent_dock.py | 13 passed (at commit) | 3 failed |
| ee8af1c lane closures resent | test_agent_dock.py | 15 passed | 2 failed |
| 40439c9 station lost goal | test_station_lost_goal.py | 4 passed | 4 failed |
| 1e78fd4 re-localize at bay | test_relocalize_at_bay.py | 5 passed (at commit) | import error (finding 7) |
| 3dd3ddd re-localize before docked | test_relocalize_at_bay.py | 5 passed | 3 failed |
| e0c9f87 bay-exit back-up | test_bay_exit_backup.py | 8 passed (at commit) | 8 failed |
| dfa94fb back-up from docked pose | test_bay_exit_backup.py | 10 passed | 2 failed |
| e3af4ea stale-station log | test_station_stale_log.py | 1 passed | 1 failed |
| e9fce24 WMS kind share | test_wms_kind_share.py | 2 passed | 1 failed |
| 449ff90 detector repeated stamp | test_detector_odom.py | 2 passed | 1 failed |
| c9cc562 standstill dock check | test_dock_check_standstill.py | 4 passed | 3 failed |
| 4206a0a 905 budget | test_dock_905_budget.py | 3 passed | 3 failed |
| 0ec5f63, 99aacd8, 317f29c (arm cell) | test_cell_geometry.py, test_controller_starter.py, test_vacuum_io.py | not collected here | — |

## What ran and what didn't
- Environment: cloud container, Ubuntu 24.04, ROS 2 Jazzy from RoboStack (packages.ros.org is blocked by the
  proxy), `setuptools<72` (newer breaks `colcon build --symlink-install` and `colcon test`).
- `colcon build --symlink-install --packages-up-to open_amr_swarm`: ok. Built only the packages the swarm tests need;
  not built: `open_amr_arm_cell` runtime deps (MoveIt 2, UR driver), Nav2 stack, `open_amr_gazebo`,
  `open_amr_isaac_hardware`, `open_amr_description` checks (xacro/urdf) — Isaac / Gazebo / MoveIt only.
- `colcon test --packages-select open_amr_swarm`: **113 tests, 0 failures** on `4206a0a` (same as the HP).
- `python3 -m pytest open_amr_docking/test open_amr_arm_cell/test`: **9 passed, 1 skipped** (not 36):
  `test_reach_study.py` skips (no MoveIt), and pytest collects no tests from the other `open_amr_arm_cell/test` files in
  this environment ("found no collectors", although each file imports and its functions exist; not resolved). So the
  arm-cell fixes (`0ec5f63`, `99aacd8`, `317f29c`, `69b2913`) were reviewed by reading only.
- Not run: Isaac (all whole-system behaviour), MoveIt / Pilz (`69b2913`), the Nav2 docking server (`c893f97`), the EKF
  (`65fdc85`: read only; its interaction with the detector's repeated-stamp guard is consistent).
