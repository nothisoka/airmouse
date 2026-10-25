# AirMouse TASK QUEUE (v2: sensor profiles, lidar, memory, second camera)

Project: autonomous indoor drone simulation. With no prior knowledge of the building it must explore,
map, find up to 6 survivors, return to its launch point and land.
Sensor profiles (selected in config/sensors.yaml, `profile:` key):
  baseline = MPU6050 IMU + 5 ultrasonic rangers (4 horizontal, 1 down) + front camera
  flow     = baseline + downward camera (feature / optical-flow odometry)
  full     = flow + 360-degree 2D lidar
No GPS, no ground-truth pose in `core/`. Layout: `core/` (pure Python), `backends/` (pygame_sim,
gazebo_ros2), `generate_maps.py`, `run.py`, `tools/benchmark_suite.py`, `config/*.yaml`, `tests/`.
Environment: Windows + WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz sim 8).

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
8. `core/` never reads ground truth. Simulated sensors (lidar, cameras, sonar, IMU) live in
   `backends/` and only hand NOISY MEASUREMENTS to `core/` through the HAL. A debug true-pose flag may
   exist ONLY in the simulator backend. The zero-ground-truth test must keep passing.
9. REPORTING: every results table has these columns: layout, profile, took_off, pose_valid (mean drift
   < 1.0 m), coverage %, survivors found/total, false positives, collision_events, home_error_m,
   flight_time_s, path_length_m, exit_reason, plus MEAN and WORST rows. Always show results that got WORSE
   next to ones that improved, and always compare with the previous recorded baseline. Never call mixed
   results a success. Claims of improvement need at least 10 layouts (20 for final claims), not 5.
10. CHECKPOINT lines: when you reach one, STOP, print the table, `git log --oneline | head -10`, the
    next task id, and wait for me to say "go".
11. If anything in a task is unclear or a command does not exist, say so and stop. Do not guess.
12. REGRESSION GUARD: after any change in core/ or backends/, `pytest -q` plus one headless run of
    maps/house_00042.json in the profile you touched must still complete without crashing. If the new
    result is clearly worse than the last recorded one for that layout, say so in PROGRESS.md.

STANDARD COMMANDS
    cd ~/airmouse && source .venv/bin/activate
    scripts/run_headless.sh <map.json> <label>
    scripts/run_batch.sh <start_seed> <count> <house|maze>
    Environment variables for both scripts:  POSE_MODE=true|estimated   PROFILE=baseline|flow|full
    (the scripts are created in A4/A8, the POSE_MODE flag is located in A3, PROFILE is added in D2)

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
  DO: compare the metrics JSON against the protocol column list. For each missing field, add it in its
  own small commit: took_off, pose_valid, coverage_pct (of reachable free cells), survivors_found,
  survivors_total, false_positives, collision_events, home_error_m, flight_time_s, path_length_m,
  exit_reason, frontier_clusters_remaining (replaces "frontiers discovered"), profile.
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
  the batch. Support `POSE_MODE=true|estimated` selecting the debug true-pose flag found in A3.
  CHECK: `scripts/run_batch.sh 100 3 house`
  DONE WHEN: a 3-row table with MEAN/WORST prints.

- [x] A9 Baselines (no code changes)
  DO: run `scripts/run_batch.sh 100 5 house` and `scripts/run_batch.sh 200 5 maze` twice: once with
  POSE_MODE=true and once with POSE_MODE=estimated. If one layout takes longer than 5 minutes, say so
  and reduce to 3+3. Save the four tables to docs/baselines/A9_true_house.md, A9_true_maze.md,
  A9_est_house.md, A9_est_maze.md.
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
  If the gate is not met: at most TWO more rounds of (B3-style diagnosis -> one fix -> rerun).
  Then report honestly with the table.

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

- [x] C6 End-to-end survivors, 5 layouts, POSE_MODE=true
  DO: run the batch; if survivors found < 90% or false positives > 0, fix ONE cause at a time
  (record before/after). Maximum 2 rounds.
  DONE WHEN: table shows survivors found/total.

- [x] C7 Phase C batch (10 layouts, true pose). GATE: survivors found >= 90%, false positives low.

CHECKPOINT C.

---------------------------------------------------------------------------------------------------
## PHASE D — Sensor profiles, lidar, memory, second camera (estimated pose from here on)

Always POSE_MODE=estimated in this phase. The goal is to measure what each sensor adds, so keep the
`baseline` profile working throughout and compare every new result with it.

### D-I  Foundations

- [x] D1 Baseline re-measure (NO code changes)
  DO: with the Phase B/C system, PROFILE=baseline (the only profile that exists yet), POSE_MODE=estimated:
  `scripts/run_batch.sh 100 5 house` and `scripts/run_batch.sh 200 5 maze`. Save to
  docs/baselines/D1_baseline_house.md and D1_baseline_maze.md.
  DONE WHEN: both files exist and are committed. All later tables are compared with these.

- [x] D2 Sensor profiles + HAL interfaces
  DO: (a) add `profile: baseline|flow|full` and per-sensor `enabled:` flags in config/sensors.yaml.
  (b) In core/hal.py add abstract interfaces: `Lidar2D.read()` -> ranges[], angle_min, angle_increment,
  range_min, range_max, timestamp; `Camera.read()` -> frame, timestamp, name ("front" or "down").
  (c) Make the profile selectable in run.py (`--profile`) and via the PROFILE env var in both scripts.
  (d) Make sure core/ asks the HAL which sensors are present and works with any subset.
  CHECK: `PROFILE=baseline scripts/run_headless.sh maps/house_00042.json d2_base` gives the same result
  as before (same exit reason, coverage within 2 points).
  DONE WHEN: check passes; a test confirms baseline has no lidar object and core still runs.

### D-II  360-degree lidar

- [x] D3 Lidar simulation (backend only)
  DO: in backends/pygame_sim/sensor_sim.py add a 2D lidar: 360 rays (1 degree), range 0.15 to 12 m,
  Gaussian range noise sigma 0.02 m, 2% random dropouts (return range_max), 8 Hz, ray-cast against the
  true grid from the true pose at flight altitude. Config keys: lidar_rate_hz, lidar_range_max,
  lidar_noise_sigma, lidar_dropout, lidar_sees_furniture (default true, matching the simulator's obstacle
  model; document that a real lidar would miss objects below its plane). Hand only the noisy ranges to
  core/ through the HAL.
  CHECK: unit test: drone in an empty 10x10 m room returns distances equal to the true wall distances
  within 3 sigma.
  DONE WHEN: test passes; the zero-ground-truth test still passes.

- [x] D4 Lidar occupancy mapping
  DO: add lidar rays to the occupancy grid update in core/mapping (log-odds: free along the ray, occupied
  at the hit, skip max-range returns for the occupied update). Uses the ESTIMATED pose only.
  CHECK: unit test with a known scene and the true pose: >= 95% of cells classified correctly.
  DONE WHEN: test passes.

- [x] D5 Scan matcher
  DO: write core/estimation/scan_matcher.py: match the current scan to the current map (correlative scan
  matching or point-to-grid ICP), return a pose correction (dx, dy, dyaw), a match score and a covariance.
  Reject matches below a score threshold or in featureless corridors (degenerate geometry).
  CHECK: unit test: start with a known scan and map, add a 0.30 m / 5 degree offset; the matcher recovers it
  to within 0.05 m / 1 degree. A second test with a bad scan must be rejected.
  DONE WHEN: both tests pass.

- [x] D6 Fuse scan matching into the EKF
  DO: use the matcher output as an (x, y, yaw) measurement update with its covariance, gated by score.
  CHECK: synthetic test: IMU with injected bias, straight 20 m flight; drift stays below 0.3 m with the
  matcher and clearly above that without it.
  DONE WHEN: test passes.

- [ ] D7 Lidar vs baseline batch (10 layouts)
  DO: `PROFILE=full scripts/run_batch.sh 100 5 house` and `... 200 5 maze`. Table next to D1.
  GATE (mean, estimated pose): drift < 0.5 m, coverage >= 90%, survivors >= 90%, home_error < 1 m,
  collision_events ~0. Show worsened cases. If not met: at most 2 rounds of (diagnose -> one fix -> rerun).

CHECKPOINT D1 — lidar results.

### D-III  Memory component (map + path + keyframes)

- [ ] D8 Design note (NO code)
  DO: write docs/MEMORY.md (one page): what is stored — occupancy grid, trajectory (timestamped poses,
  decimated to >= 0.2 m spacing), keyframes (pose + downsampled scan + camera features when available,
  one per 1 m or 20 degrees travelled), confirmed survivors, home pose, mission state; file format
  (npz + json, versioned); autosave every 5 s; size bounds.
  DONE WHEN: file exists and covers every item.

- [ ] D9 MapMemory implementation
  DO: core/memory/map_memory.py with add_pose, add_keyframe, set_home, add_survivor, save(path), load(path).
  CHECK: unit tests: save then load gives identical contents; trajectory decimation keeps spacing; size
  stays below a configured limit on a long synthetic run.
  DONE WHEN: tests pass.

- [ ] D10 Wire memory into the mission and test crash recovery
  DO: the mission loop writes to MapMemory and autosaves to generated_maps/<run>/memory.npz. Add a
  `--resume <memory.npz>` option.
  CHECK: run headless for 60 s, stop it (timeout), restart with --resume: the log shows the map, home pose
  and survivors restored and exploration continues from the stored state.
  DONE WHEN: check passes; test added.

- [ ] D11 Return home using memory
  DO: plan the way home with A* over the stored map; fall back to following the stored trajectory in
  reverse if no path exists; log which one was used.
  CHECK: unit test for both cases.
  DONE WHEN: test passes.

- [ ] D12 Home relocalisation (loop closure at home)
  DO: store a keyframe at takeoff. When within 3 m of the estimated home, match the current scan (lidar, or
  sonar/camera features if the profile has no lidar) against the home keyframe and correct the pose.
  CHECK: unit test: inject 2 m of drift, home error after relocalisation < 0.5 m; and with no match the
  mission continues without crashing.
  DONE WHEN: both tests pass.

- [ ] D13 Return-home batch (10 layouts, PROFILE=full)
  DO: run with relocalisation off, then on. Table with both.
  GATE: home_error_m < 1.0 on all layouts (target 0.5); show worsened cases.

CHECKPOINT D2 — memory and return home.

### D-IV  Second camera (downward) and corner / edge features

- [ ] D14 Downward camera simulation (backend only)
  DO: render a downward-looking frame (320x240) of a textured floor from the true pose. Add a random
  texture to the floor (backend renderer only). Expose it as Camera "down" through the HAL; the front camera
  is unchanged; the enable flag is in config/sensors.yaml.
  CHECK: unit test: move the drone 1.0 m forward at altitude 1.2 m; the texture shifts in the image by the
  expected number of pixels (focal length x distance / altitude) within 5%.
  DONE WHEN: test passes.

- [ ] D15 Feature pipeline
  DO: core/perception/features.py: Gaussian blur (sigma from config) -> Canny edges -> Shi-Tomasi corners
  (max count, quality level, min distance from config). Return corner coordinates and an edge map.
  CHECK: unit test on a synthetic checkerboard: corners found within 2 px of the true corners; blur does not
  remove them.
  DONE WHEN: test passes.

- [ ] D16 Optical-flow odometry
  DO: Lucas-Kanade tracking of the corners between consecutive down frames; metric scale = altitude from the
  downward sonar / focal length; outlier rejection (median or RANSAC); output velocity (vx, vy) and
  covariance; skip the update when too few features are tracked.
  CHECK: synthetic straight 10 m flight: distance error < 5%; a textureless frame is skipped cleanly.
  DONE WHEN: both checks pass.

- [ ] D17 Fuse flow into the EKF and run the flow profile
  DO: velocity measurement update in the EKF. `PROFILE=flow scripts/run_batch.sh 100 5 house` and
  `... 200 5 maze`. Table next to D1 (baseline) and D7 (full).
  DONE WHEN: table recorded; worsened cases listed.

- [ ] D18 Landmark memory (camera features in keyframes)
  DO: store ORB descriptors and corner positions in keyframes; relocalise at home from features when no
  lidar is present.
  CHECK: unit test: inject drift, home error after feature relocalisation < 1.0 m on a textured test world;
  on a textureless world the code reports "no match" and does not corrupt the pose.
  DONE WHEN: both tests pass.

- [ ] D19 Final profile comparison
  DO: 10 layouts each for baseline, flow, full (estimated pose). Write docs/SENSOR_PROFILES.md: one table
  (drift, coverage, survivors, home error, collisions per profile), which profile meets the gate, and a
  short hardware note per profile: what is needed (lidar model class, downward camera, ToF/optical-flow
  option) and a blank line for weight and cost that I will fill in. Do not invent weight or cost numbers.
  DONE WHEN: the file exists with real numbers from the batches.

CHECKPOINT D3 — sensor profile comparison. I will decide which profile(s) go forward.

### D-baseline (optional — only if I say baseline-only operation must also meet the gate)

- [ ] DB1 Wall-alignment yaw correction (no magnetometer on the MPU6050; indoor walls are axis-aligned);
  unit test: injected yaw bias stays bounded.
- [ ] DB2 Gate the sonar scan-to-map updates: only with >= 2 returns in range, reject large mismatches,
  use measurement covariance. Table vs D1 with worsened cases.
- [ ] DB3 Pose-quality behaviour: below a covariance threshold slow down and prefer routes near walls; hold
  and re-localise if very low. Test with a scripted quality drop.
- [ ] DB4 Baseline batch (10 layouts). GATE: mean drift < 1.0 m, coverage >= 90%, survivors >= 90%,
  home_error < 1 m, collision_events ~0. If it fails after 2 rounds, say so with evidence.

---------------------------------------------------------------------------------------------------
## PHASE E — Gazebo + RViz integration (for the profile(s) I choose at Checkpoint D3)

The first tasks are headless. The RViz task needs me to look at the screen.

- [ ] E1 World loads
  DO: `timeout 60 gz sim -s -r maps/house_00042.sdf > generated_maps/e1.log 2>&1; echo "exit=$?"`
  DONE WHEN: the log shows the world loaded with no plugin errors (list any warnings).

- [ ] E2 Drone model with sensors
  DO: create a drone model (SDF) with: IMU, 5 range sensors (narrow-cone gpu_lidar, matching the
  simulated sonars), a 360-degree gpu_lidar (matching the D3 parameters), and two cameras (front, down).
  Add it to the world at the start cell from the JSON. Sensor noise parameters must match config/sensors.yaml.
  DONE WHEN: `gz topic -l` lists all sensor topics.

- [ ] E3 Bridge
  DO: ros_gz_bridge config for IMU, ranges, scan, both camera images, camera info, cmd_vel, clock.
  CHECK: `ros2 topic list` shows /imu/data, /range/*, /scan, /camera/front/image_raw, /camera/down/image_raw.

- [ ] E4 Node
  DO: backends/gazebo_ros2 node runs the same core stack through the HAL on those topics and publishes
  /map, /odom, /drone/path, /drone/plan, /survivors, /camera/image_annotated, /mission/state, TF
  map -> odom -> base_link -> sensor frames.
  CHECK: `ros2 topic echo --once` on each.

- [ ] E5 Launch file
  DO: `ros2 launch airmouse mission.launch.py map:=<name> profile:=<profile>` starts Gazebo, bridge, node and
  RViz with maps/rescue_mapping.rviz. Add LaserScan and both Image displays to the RViz config. Write the
  exact commands in README.

- [ ] E6 Consistency test
  DO: a test that the survivor world coordinates in the JSON equal those in the SDF for 5 generated layouts,
  and that the drone start pose matches.
  DONE WHEN: test passes.

- [ ] E7 HUMAN CHECK: ask me to run the launch and send a screenshot of RViz and Gazebo. Do not claim it
  works without it.

CHECKPOINT E.

---------------------------------------------------------------------------------------------------
## PHASE F — Hardening and final report

- [ ] F1 Failsafe tests: pose-quality loss, sensor timeout (including lidar and camera loss), imminent
  collision, low battery, memory file corruption (load must fail safely).
- [ ] F2 Loop-rate profile (target >= 10 Hz autonomy loop); list the slowest 3 functions and the combined
  cost of lidar scan matching plus feature tracking plus YOLO.
- [ ] F3 Hardware abstraction stubs: MPU6050 (I2C), HC-SR04 x5, 2D lidar (serial), two cameras, MAVLink —
  interfaces only, documented in README.
- [ ] F4 Final 20-layout report (10 house + 10 maze, estimated pose, chosen profile) in
  docs/FINAL_REPORT.md: table, mean/worst, what passes, what does not, remaining risks.
- [ ] F5 README final: install, run (pygame, Gazebo, RViz), add a layout, switch profile, move to the
  companion computer.

CHECKPOINT F — final report.
