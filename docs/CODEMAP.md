# AirMouse Code Map

This document maps key autonomy components to their locations in the codebase (file path, function/class name, and summary description).

| Component | File Path | Class / Function Name | Summary Description |
|---|---|---|---|
| **Mission Loop** | `run.py` | `run_pygame_sim` | Main 100 Hz simulation loop (`while running and mission.state != MissionState.COMPLETE:`) driving physics, sensors, estimation, and mission coordinator. |
| **State Machine** | `core/mission/state_machine.py` | `MissionCoordinator.update` | Finite state machine managing transitions across `PREFLIGHT`, `TAKEOFF`, `EXPLORE`, `RETURN_HOME`, `LAND`, and `COMPLETE`. |
| **Frontier Selection** | `core/exploration/frontier_detector.py` | `FrontierDetector.find_best_frontier` | Identifies candidate frontier clusters, filters blacklisted targets, and selects the lowest-cost reachable frontier. |
| **Planner (A\*)** | `core/planning/astar.py` | `AStarPlanner.plan` | 8-connected grid A* path planner on inflated costmaps with Euclidean heuristic and line-of-sight path smoothing. |
| **Obstacle Inflation** | `core/mapping/occupancy_grid.py` | `OccupancyGrid.get_inflated_costmap` | Performs binary dilation by safety radius $r_{safe}$ and generates soft buffer penalty costmaps around occupied cells. |
| **Controller** | `core/control/pid.py` | `WaypointController.compute_command` | Proportional waypoint-following controller generating 3D velocity (`vx`, `vy`, `vz`) and yaw rate commands clamped to velocity limits. |
| **Reactive Avoidance** | `core/planning/reactive_avoidance.py` | `ReactiveAvoidance.filter_command` | Applies repulsive artificial potential field vectors from ultrasonic range sensors to deflect commanded velocities away from obstacles. |
| **Collision Counting** | `core/mission/mission_logger.py` & `backends/pygame_sim/drone_dynamics.py` | `MissionLogger.log_step` / `DroneDynamics.step` | `DroneDynamics.step` detects physical hull intersection with obstacle cells; `MissionLogger.log_step` debounces and counts collision start events. |
| **Coverage Computation** | `run.py` & `core/mission/mission_logger.py` | `compute_true_room_coverage` / `MissionLogger.finalize` | `compute_true_room_coverage` calculates ratio of ground-truth reachable free cells mapped as free in the occupancy grid; stored by `MissionLogger`. |
| **Exit Reason** | `run.py` & `core/mission/mission_logger.py` | `run_pygame_sim` / `MissionLogger.finalize` | Evaluates termination conditions (`COMPLETE`, `TIMEOUT`, `FAILSAFE`, `INTERRUPTED`) and assigns exit reason and exit code (0 for COMPLETE, 1 otherwise). |
| **Metrics JSON Writer** | `core/mission/mission_logger.py` | `MissionMetrics.save_json` | Serializes mission metrics (flight time, coverage, drift, collisions, survivors, exit reason) to JSON; invoked by `run.py`. |
| **Debug True-Pose Flag** | `run.py` | `run_pygame_sim(debug_true_pose=...)` | CLI flag `--debug-true-pose` in `run.py` bypassing EKF state with ground-truth physical pose for upper-bound exploration testing. |
| **Config Keys (Flight / Speed / Return-Home)** | `config/mission.yaml` & `core/mission/state_machine.py` | `config/mission.yaml` keys | `mission.max_flight_time_s`, `mission.hard_time_limit_s`, `mission.nominal_cruise_speed_mps`, `mission.max_cruise_speed_mps`; return-home trigger margin is hardcoded at `elapsed >= max_flight_time * 0.70` in `core/mission/state_machine.py`. |
| **Scan-to-Map Correction** | `core/estimation/ekf.py` | `SensorFusionEKF.update_scan_to_map` | Projects ultrasonic ranger endpoints onto occupancy grid and applies 2D spatial Kalman correction against established wall obstacles. |
| **Visual Odometry** | `core/estimation/visual_odometry.py` | `VisualOdometry.estimate_displacement` | Computes inter-frame camera displacement and planar velocity using Lucas-Kanade optical flow, fed into the EKF. |
