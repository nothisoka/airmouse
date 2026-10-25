# AirMouse TASK QUEUE

Project: autonomous indoor drone simulation. With no prior knowledge of the building it must explore,
map, find up to 6 survivors, return to its launch point and land.
Allowed sensors only: MPU6050 IMU, monocular camera, ultrasonic rangers (4 horizontal + 1 down).
No GPS, no LiDAR, no ground-truth pose in `core/`. Layout: `core/` (pure Python), `backends/`
(pygame_sim, gazebo_ros2), `generate_maps.py`, `run.py`, `tools/benchmark_suite.py`, `config/*.yaml`,
`tests/`. Environment: Windows + WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz sim 8).

Tick a box `[x]` in this file when a task is DONE, and commit the tick with the task.

---------------------------------------------------------------------------------------------------
## PROTOCOL (binding for every task)

1. ONE task at a time, in order. Read only the current task. Do not look ahead or start the next one
   until the current one's DONE WHEN is met, recorded and committed.
2. ATTEMPT LIMIT: at most 3 attempts per task. An attempt = one change + one verification run.
   If DONE WHEN is not met after 3 attempts, OR the same error appears twice in a row: STOP.
   Write under "BLOCKED" in docs/PROGRESS.md: task id, what you tried, evidence, your best guess.
   Commit it and report to me. Do NOT try a fourth approach and do NOT widen the scope.
3. SCOPE: change only what the task names. No refactors, renames, extra features, or tuning of
   unrelated parameters. If you notice another bug, add one line under "FOUND ISSUES" in
   docs/PROGRESS.md and continue with your task.
4. COMMANDS: foreground only, one per command, never background tasks, no interactive programs, no GUI
   windows (headless only). Every simulation:
       timeout 300 <command> > generated_maps/<label>.log 2>&1; echo "exit=$?"
   then `tail -40 generated_maps/<label>.log` and read the metrics JSON. If something runs longer than
   6 minutes, stop it and treat it as a failed attempt.
5. EVIDENCE: a task is DONE only when its CHECK output exists (paste the key lines in
   docs/PROGRESS.md). "It should work" is not done.
6. BEFORE EVERY COMMIT: `source .venv/bin/activate && pytest -q` must pass (from the repo root, no
   environment variables). Then: `git add -A && git commit -m "<TASK-ID>: <summary>"`.
7. GIT: branch `autonomy-work`, local only. Never push, add remotes, `git reset --hard`, `git clean`.
   Never modify ~/slam_ws.
8. `core/` never reads ground truth. A debug true-pose flag may exist ONLY in the simulator backend.
   The zero-ground-truth test must keep passing.
9. REPORTING: every results table has these columns: layout, took_off, pose_valid (mean drift < 1.0 m),
   coverage %, survivors found/total, false positives, collision_events, home_error_m, flight_time_s,
   path_length_m, exit_reason, plus MEAN and WORST rows. Always show results that got WORSE next to
   ones that improved, and always compare with the previous recorded baseline. Never call mixed results a
   success. Claims of improvement need at least 10 layouts (20 for final claims), not 5.
10. CHECKPOINT lines: when you reach one, STOP, print the table, `git log --oneline | head -10`, the
    next task id, and wait for me to say "go".
11. If anything in a task is unclear or a command does not exist, say so and stop. Do not guess.

STANDARD COMMANDS
    cd ~/airmouse && source .venv/bin/activate
    scripts/run_headless.sh <map.json> <label>       # one headless run, one summary line
    scripts/run_batch.sh <start_seed> <count> <house|maze>   # generate layouts + run + markdown table
    (both scripts are created in task A4/A8; the oracle/true-pose flag is located in task A3)

---------------------------------------------------------------------------------------------------
## PHASE A — Verify state, metrics, tooling

- [x] A1 Environment check
  DO: `pwd; git branch --show-current; git status --short; which python3; pip list | grep -i airmouse; pytest -q`
  DONE WHEN: branch is autonomy-work; python3 is ~/airmouse/.venv/bin/python3; pytest passes.
  If the tree is dirty, commit it first as "A1: save pending work". Paste the outputs in PROGRESS.md.

- [x] A2 Repository hygiene
  DO: `git ls-files | wc -l; git ls-files | grep -E "generated_maps|\.venv|__pycache__|\.log$|Zone|scratch"`
  DONE WHEN: the grep prints nothing; .gitignore covers those; maps/maze_00200.* and
  maps/house_00042.* are tracked. Fix .gitignore only by adding entries.

- [x] A3 Code map (NO code changes)
  DO: use `grep -rn` / `ls` to find where each of these lives, and write docs/CODEMAP.md with
  file path + function/class name + one line each: mission loop, state machine, frontier selection,
  planner (A*), obstacle inflation, controller, reactive avoidance, collision counting, coverage
  computation, exit reason, metrics JSON writer, the debug true-pose flag (`grep -rni "true.pose\|oracle\|debug_true"`),
  config keys for flight time / speed / return-home, scan-to-map correction, visual odometry.
  DONE WHEN: docs/CODEMAP.md has an entry for every item above (write "NOT FOUND" if missing).

- [x] A4 scripts/run_headless.sh
  DO: create scripts/run_headless.sh (chmod +x) if missing. It runs run.py headless on a map under
  `timeout 300`, writes generated_maps/metrics_<label>.json and run_<label>.log, prints ONE summary line
  with the protocol columns, and returns the simulation's exit code.
  CHECK: `scripts/run_headless.sh maps/house_00042.json a4_test`
  DONE WHEN: the summary line prints with all columns (use "n/a" only for values the code truly lacks).

- [x] A5 Metrics fields
  DO: compare the metrics JSON against protocol column list. For each missing field, add it in its own
  small commit: took_off, pose_valid, coverage_pct (of reachable free cells), survivors_found,
  survivors_total, false_positives, collision_events, home_error_m, flight_time_s, path_length_m,
  exit_reason, frontier_clusters_remaining (replaces "frontiers discovered").
  DONE WHEN: one run's JSON contains every field and the summary line shows them.

- [x] A6 Collision events
  DO: count a collision when contact STARTS (transition free -> in-contact), not every step in contact.
  Add a unit test: 100 consecutive in-contact steps = 1 event; leave and re-enter = 2 events.
  DONE WHEN: test passes; one run's collision_events looks plausible (write the number).

- [x] A7 exit_reason rules
  DO: COMPLETE only if (coverage target met OR no reachable frontiers) AND home_error_m < 1.0.
  Landed but not meeting that = INCOMPLETE. Others: TIMEOUT, FAILSAFE, CRASH. Make the coverage
  target a config value (default 0.90). Unit tests for all five outcomes.
  DONE WHEN: tests pass.

- [x] A8 scripts/run_batch.sh
  DO: create it (chmod +x). It generates layouts with fixed seeds into generated_maps/ using
  generate_maps.py, runs run_headless.sh on each sequentially (foreground), and prints a markdown table
  with the protocol columns and MEAN / WORST rows. A failing run must be recorded as a row, never abort
  the batch. Support a flag or env var `POSE_MODE=true|estimated` that selects the debug true-pose flag
  found in A3.
  CHECK: `scripts/run_batch.sh 100 3 house`
  DONE WHEN: a 3-row table with MEAN/WORST prints.

- [x] A9 Baselines (no code changes)
  DO: run `scripts/run_batch.sh 100 5 house` and `scripts/run_batch.sh 200 5 maze` twice: once with
  POSE_MODE=true and once with POSE_MODE=estimated. If one layout takes longer than 5 minutes, say so
  and reduce to 3+3. Save the four tables to docs/baselines/A9_true_house.md, A9_true_maze.md,
  A9_est_house.md, A9_est_maze.md. These are the numbers every later change is compared against.
  DONE WHEN: the four files exist and are committed.

CHECKPOINT A — stop and report the four baseline tables.

---------------------------------------------------------------------------------------------------
## PHASE B — Exploration works with TRUE pose (upper bound, independent of drift)

Always use POSE_MODE=true in this phase. Always compare with the A9 tables.

- [x] B1 Configurable budget
  DO: move flight-time limit, hard time limit, cruise speed and return-home margin into
  config/mission.yaml and make the code read them (no hardcoded values; use the CODEMAP). Defaults:
  max_flight_time_s 600, hard_time_limit_s 660, cruise speed unchanged.
  CHECK: change the yaml value and show that a run log prints the new limit.
  DONE WHEN: the check passes; pytest passes.

- [x] B2 Smarter return-home trigger
  DO: trigger return-home when remaining_time < est_time_home * 1.5 + 30 s, where est_time_home =
  planned path length home / cruise speed. Log the trigger time and the margin.
  DONE WHEN: unit test (synthetic numbers) passes; one run log shows the trigger line.

- [x] B3 Diagnose coverage (NO fixes)
  DO: on 3 layouts with POSE_MODE=true, add temporary debug logging if needed: why the mission ended,
  coverage over time (every 10 s), speed over time, number of replans, number of blacklisted goals,
  time spent per state. Write docs/diagnosis_B.md ranking the top 3 causes of low coverage with
  evidence (log lines or numbers) for each.
  DONE WHEN: the file exists, each cause has evidence. No behaviour changes in this task.

- [x] B4 Fix cause #1 from diagnosis_B.md only
  DO: smallest possible change. Re-run the same 3 layouts. Record before/after rows.
  DONE WHEN: coverage on those 3 layouts improved or the cause is shown not to apply (record why);
  pytest passes.

- [x] B5 Fix cause #2 (same method as B4)
- [x] B6 Fix cause #3 (same method as B4)

- [x] B7 Doorway clearance
  DO: check drone radius vs obstacle inflation vs door width (cells are 1 m; doors are 1-2 cells wide).
  Add a unit test: planner finds a path through a 1-cell doorway and the planned path keeps the
  configured clearance; no path is returned if the doorway is truly too small.
  DONE WHEN: test passes.

- [x] B8 Stall / oscillation watchdog
  DO: if the drone stays within 0.5 m for 15 s, or alternates between the same two goals 3 times,
  blacklist the current goal and replan. Unit test with a scripted stall.
  DONE WHEN: test passes; a run log shows the watchdog never fires in normal runs (or explains each firing).

- [x] B9 Exploration-finished detection
  DO: finish exploring when no reachable frontier cluster larger than a configurable minimum remains.
  Unit test on a fully known map.
  DONE WHEN: test passes.

- [x] B10 Residual collisions
  DO: with POSE_MODE=true run 5 layouts, list the collision events (position, speed, nearest wall
  distance, state). Fix the single most common pattern (for example: slow down within X m of walls).
  DONE WHEN: collision_events is near 0 on those layouts or the cause is documented.

- [x] B11 Phase B batch
  DO: `scripts/run_batch.sh 100 5 house` and `... 200 5 maze` with POSE_MODE=true. Compare with A9.
  GATE (mean): coverage >= 90%, collision_events ~0, home_error_m < 1.0, no TIMEOUT, no stall.
  If the gate is not met: you may do at most TWO more rounds of (B3-style diagnosis -> one fix ->
  rerun). Then report honestly with the table.

CHECKPOINT B — stop and report the table with worsened cases.

---------------------------------------------------------------------------------------------------
## PHASE C — Survivors (true pose first)

- [x] C1 Survivors in every table (if A5 did not already): survivors_found/total and false_positives.

- [x] C2 Detection check on simulated camera frames
  DO: write tools/check_yolo.py: run a headless mission, save 10 camera frames in which a person is in
  view to generated_maps/frames/, run YOLO (weights path from config) on them, print per-frame
  detections and confidence. If YOLO cannot load, say so (do not fake).
  DONE WHEN: script output pasted: detection rate on frames that contain a person.

- [x] C3 Improve person visibility (only if detection rate < 80%)
  DO: change ONLY the backend renderer: person size, contrast, textured body, shadow. Re-run C2.
  DONE WHEN: detection rate >= 80% on the test frames, or documented why not.

- [x] C4 Localisation unit test
  DO: bounding box + camera intrinsics + estimated pose + altitude -> map coordinates. Unit test with
  synthetic boxes at known positions.
  DONE WHEN: error < 0.5 m on all test cases.

- [x] C5 Tracker rules
  DO: confirm a survivor after N=3 consistent detections, dedupe within 1.0 m, cap at 6, ignore detections
  inside walls. Extend the existing dedupe tests.
  DONE WHEN: tests pass.

- [ ] C6 End-to-end survivors, 5 layouts, POSE_MODE=true
  DO: run the batch; if survivors found < 90% or false positives > 0, fix ONE cause at a time
  (record before/after). Maximum 2 rounds.
  DONE WHEN: table shows survivors found/total.

- [ ] C7 Phase C batch (10 layouts, true pose). GATE: survivors found >= 90%, false positives low.

CHECKPOINT C.

---------------------------------------------------------------------------------------------------
## PHASE D — Estimation (estimated pose, only after Phases B and C pass)

Always POSE_MODE=estimated here. Compare with A9 estimated tables.

- [ ] D1 Error over time
  DO: for 2 layouts write a CSV of estimated vs true x, y, yaw per second (true values only in the
  backend/logging, never in core/). Plot to docs/drift_<layout>.png.
  DONE WHEN: files exist.

- [ ] D2 Drift attribution (NO fixes)
  DO: ablation runs: (a) gyro-only yaw error over time, (b) visual-odometry velocity scale error on
  straight flight, (c) accelerometer bias effect. Write docs/diagnosis_D.md ranking the causes.

- [ ] D3 Camera texture for odometry
  DO: give floor and walls texture in the simulated camera so VO has features.
  CHECK: unit test: straight 10 m flight, VO distance error < 5%.

- [ ] D4 VO scale and outliers
  DO: scale from the downward sonar, reject outlier flow vectors. Test as D3.

- [ ] D5 Yaw correction by wall alignment
  DO: the MPU6050 has no magnetometer, so yaw drifts. Indoor walls are axis-aligned, so use the dominant
  wall direction in sonar/scan data to correct yaw. Unit test: injected yaw bias stays bounded.

- [ ] D6 Gate scan-to-map updates
  DO: use scan-to-map only when at least 2 sonar returns are in range, reject mismatches over a
  threshold, use measurement covariance. Re-run 5 layouts; table vs A9 estimated, worsened cases shown.

- [ ] D7 Pose-quality behaviour
  DO: pose-quality flag from the EKF covariance; below threshold, slow down and prefer routes near walls;
  if very low, hold and re-localise. Test with a scripted quality drop.

- [ ] D8 Phase D batch (10 layouts). GATE: mean drift < 1.0 m, coverage >= 90%, survivors >= 90%,
  home_error < 1 m, collision_events ~0.

- [ ] D9 (only if the D8 gate fails) keyframe / loop-closure correction, max 2 rounds. If still failing,
  write docs/HARDWARE_NOTES.md with evidence and the sensors that would close the gap (for example an
  optical-flow module such as PMW3901 plus a ToF ranger).

CHECKPOINT D.

---------------------------------------------------------------------------------------------------
## PHASE E — Gazebo + RViz integration

The first three tasks are headless. The RViz task needs me to look at the screen.

- [ ] E1 `timeout 60 gz sim -s -r maps/house_00042.sdf > generated_maps/e1.log 2>&1; echo "exit=$?"`
  DONE WHEN: the log shows the world loaded with no plugin errors (list any warnings).

- [ ] E2 Bridge: write the ros_gz_bridge config (IMU, ranges, camera, cmd_vel, clock).
  CHECK: with the world running, `ros2 topic list` shows the bridged topics.

- [ ] E3 Node: backends/gazebo_ros2 node runs the core stack on Gazebo topics and publishes
  /map, /odom, /drone/path, /survivors, /mission/state (check with `ros2 topic echo --once`).

- [ ] E4 Launch file: `ros2 launch airmouse mission.launch.py map:=<name>` starts Gazebo, bridge, node,
  and RViz with maps/rescue_mapping.rviz. Write the exact commands in README.

- [ ] E5 Consistency: assert in a test that the survivor world coordinates in the JSON equal those in the
  SDF for 5 generated layouts.

- [ ] E6 HUMAN CHECK: ask me to run the launch and send a screenshot of RViz. Do not claim it works
  without it.

CHECKPOINT E.

---------------------------------------------------------------------------------------------------
## PHASE F — Hardening and final report

- [ ] F1 Failsafe tests: pose-quality loss, sensor timeout, imminent collision, low battery.
- [ ] F2 Loop-rate profile (target >= 10 Hz autonomy loop); list the slowest 3 functions.
- [ ] F3 Hardware abstraction stubs: MPU6050 (I2C), HC-SR04 x5, camera, MAVLink — interfaces only,
  documented in README.
- [ ] F4 Final 20-layout report (10 house + 10 maze, estimated pose) in docs/FINAL_REPORT.md: table,
  mean/worst, what passes, what does not, and remaining risks.
- [ ] F5 README final: install, run (pygame, Gazebo, RViz), add a layout, move to the companion computer.

CHECKPOINT F — final report.
