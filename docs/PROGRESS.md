# NIDAR AirMouse: Autonomy Progress & Benchmark Log

**Branch:** `autonomy-work`  
**Date:** October 20, 2026  
**Execution Environment:** WSL2 Ubuntu 24.04, Python 3.12 (`~/airmouse_venv`), ROS 2 Jazzy, Gazebo Harmonic (gz sim 8.15.0).

---

## 1. Batch Benchmark Evaluation (5 Procedural Layouts)

All simulations executed headless, wrapped in `timeout 300`, using procedural runtime map generation into `generated_maps/` with fixed seeds. Hard mission limits enforced via `config/mission.yaml` (`max_flight_time_s: 240.0`, `hard_time_limit_s: 280.0`, `max_sim_steps: 28000`).

| # | Layout | Mode | Took Off? | Pose Valid? | Frontiers Discovered | Distance (m) | Time (s) | Exit Reason | Exit Code | Coverage (%) | Survivors | Collisions | Mean Drift (m) | Peak Drift (m) | RTH Error (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | `house_00101` | House | **YES** | **NO** | 1,376 | 146.78 | 280.0 | **TIMEOUT** | 1 | 31.4% | 0 / 2 | 36 | 8.50 | 14.82 | 11.67 |
| 2 | `house_00102` | House | **YES** | **NO** | 1,227 | 147.04 | 280.0 | **TIMEOUT** | 1 | 25.2% | 0 / 6 | 61 | 8.91 | 14.13 | 18.01 |
| 3 | `house_00103` | House | **YES** | **NO** | 997 | 103.54 | 200.5 | **COMPLETE** | 0 | 19.5% | 0 / 2 | 5 | 9.66 | 14.01 | 5.35 |
| 4 | `maze_00201` | Maze | **YES** | **NO** | 1,186 | 126.06 | 280.0 | **TIMEOUT** | 1 | 34.8% | 0 / 1 | 198 | 4.72 | 8.70 | 11.46 |
| 5 | `maze_00202` | Maze | **YES** | **NO** | 782 | 115.04 | 280.0 | **TIMEOUT** | 1 | 19.2% | 0 / 6 | 621 | 7.21 | 12.75 | 10.39 |

---

## 2. Verification of User Directives

1. **Did the drone take off?**
   - **YES** across all 5 layouts ($5/5$, 100%). In every run, the drone ascended cleanly from the floor to hover altitude ($z \approx 1.15 - 1.25\text{ m}$) within the first 3.0 seconds.
2. **Was the pose estimate valid?**
   - **NO** ($0/5$). While vertical altitude ($z$) is strictly constrained by the downward ultrasonic sensor ($1.15-1.25\text{ m}$), horizontal dead-reckoning drift accumulated to an average mean drift of $7.80\text{ m}$ (peaking at $8.7-14.8\text{ m}$). This horizontal drift causes the estimated map to smear and the estimated return-to-home waypoint to deviate significantly from the true physical launch coordinates.
3. **How many frontiers did it see?**
   - Discovered **1,114 frontiers on average** per layout (ranging from 782 in `maze_00202` to 1,376 in `house_00101`). Frontier clustering successfully drove continuous exploration of unmapped space.
4. **How far did it move?**
   - Traveled an average path length of **$127.69\text{ m}$** per layout ($103.54\text{ m}$ to $147.04\text{ m}$).
5. **Why did it end?**
   - `house_00103`: Ended as **`COMPLETE`** (Exit Code: 0). The drone completed room exploration, transitioned to `RETURN_HOME`, navigated back along breadcrumbs, and executed `LAND` at $t = 200.5\text{ s}$.
   - `house_00101`, `house_00102`, `maze_00201`, `maze_00202`: Ended as **`TIMEOUT`** (Exit Code: 1) upon hitting the hard simulation budget limit ($t = 280.0\text{ s}$, 28,000 steps). The drone transitioned to `RETURN_HOME` at the $70\%$ budget threshold ($t = 168\text{ s}$), but could not reach within $0.8\text{ m}$ of the true physical start point before the hard cutoff because horizontal estimation drift placed its perceived home offset by $10-18\text{ m}$.

---

## 3. Key Decisions Made

1. **Packaging & Clean Virtualenv Integration:**
   - Authored `pyproject.toml` supporting `pip install -e .` with direct module mapping for `airmouse`, `airmouse.core`, `airmouse.backends`, and `airmouse.tools`.
   - Verified that `pytest` runs from repo root with **zero environment variables** (10/10 tests passing in $<7\text{ s}$).
2. **Procedural Map Isolation:**
   - Kept only sample maps (`maps/house_00042.*` and `maps/maze_00200.*`) in git.
   - All runtime test maps and benchmark maps are generated on the fly into `generated_maps/` (git-ignored) with fixed seeds for deterministic reproducibility.
3. **Hard Mission Limits & Exit Codes:**
   - Centralized `max_flight_time_s: 240.0`, `hard_time_limit_s: 280.0`, and `max_sim_steps: 28000` in `config/mission.yaml`.
   - Added `--hard-limit`, `--max-steps`, and `--output-json` to `run.py`.
   - Configured non-zero exit code (`sys.exit(1)`) on `TIMEOUT` or `FAILSAFE`, and `sys.exit(0)` on `COMPLETE`.

---

## 4. Open Problems & Root Cause Analysis

1. **Optical Flow Scale & Floor Feature Drift:**
   - In open rooms and corridors where floor texture is repetitive or featureless, Lucas-Kanade optical flow suffers from scale underestimation and feature dropouts.
   - Without global loop closure or reliable scan-matching, gyro and accelerometer integration slowly drifts ($1-2\text{ cm/s}$ velocity bias drift), compounding to $>8\text{ m}$ over 200 seconds of flight.
2. **Aperture Problem in Scan-to-Map Matching:**
   - HC-SR04 sonars provide only 4 horizontal readings (front, left, right, rear) at 20 Hz. When flying down a corridor, only lateral walls are visible (1 degree of freedom constrained); the longitudinal axis remains unconstrained by sonars.
3. **Maze Obstacle Proximity & Reactive Avoidance:**
   - In narrow 1-meter maze passages (`maze_00201`, `maze_00202`), drone radius plus safety margin ($0.35\text{ m}$) leaves very tight clearance ($0.15\text{ m}$). Combined with pose drift, the drone clips corners, leading to high collision counts.
4. **Survivor Detection Throughput:**
   - Ground-truth survivors were located in the layouts, but because the drone was forced into early RTH or suffered significant pose smearing, monocular back-projection did not register $\ge 3$ consistent clustered detections needed for confirmation.
