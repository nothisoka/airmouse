# NIDAR AirMouse: Autonomous Exploration, Sensor Fusion & Unified Simulation
## Implementation Plan & Architectural Specification

**Target System:** NIDAR AirMouse Autonomous Indoor Drone  
**Environment:** Ubuntu 24.04 (WSL2 / WSLg), ROS 2 Jazzy, Gazebo Harmonic (8.15.0), Python 3.12 (venv with `numpy 1.26.4`, `cv_bridge`, `ultralytics`, `pygame`)  
**Mission:** Autonomous indoor exploration of unknown layouts, log-odds occupancy mapping, survivor (1–6 people) localization via monocular camera + YOLO, and return-to-home landing with zero GPS/LiDAR and zero ground-truth knowledge.

---

## 1. Audit Findings: Current Workspace State

### 1.1 Existing `main.py` Analysis
The audit of `/home/sriram/airmouse/main.py` confirms all 7 defects identified in Section 2:
1. **Direct Access to Ground Truth Map:** `GridPlanner.next_coverage_target()` queries `self.grid` directly, searching for unvisited cells in the true building layout. The autonomy logic has perfect omniscient knowledge of free space and obstacles.
2. **Hardcoded Building & Survivors:** `build_house()` generates an identical layout every run, and `self.people` hardcodes three static coordinates `[(6, 8), (22, 7), (19, 16)]`.
3. **Discrete Integer Dynamics & Zero Noise:** The drone pose is a discrete integer grid cell `(x, y)` without physical dynamics, inertia, acceleration limits, or motion noise.
4. **$O(N^2)$ Frontier/Coverage Search:** `next_coverage_target()` computes an $A^*$ path from the drone to *every* unvisited free cell in the map on every replan tick, which fails to scale to larger layouts.
5. **Silhouette Rendering with Coordinate Association Cheating:** `observe_camera()` renders stylized 2D silhouettes at screen coordinates calculated from hidden ground-truth offsets (`ox = p.x - dx`), and matches YOLO bounding boxes to people by checking if the detected bounding box center is within 40 pixels of the expected hidden person position.
6. **No Mission State Machine or Failsafes:** Lacks Return-to-Home (RTH), battery/time management, pose quality metrics, and failsafe modes.
7. **Monolithic Architecture:** Simulation physics, rendering, sensing, planning, and perception are intertwined in `RescueSimulation`.

### 1.2 Existing Gazebo/RViz Workspace (`~/slam_ws`)
The existing ROS 2 workspace at `~/slam_ws/src/my_slam_robot` contains:
- **`robot.urdf`:** 4-wheeled differential drive ground robot model equipped with an IMU (`/imu`) and 360° 2D LiDAR (`/scan`).
- **`sim.launch.py` / `autonomous_exploration.launch.py`:** Launches `gz_sim` (Gazebo Harmonic), `robot_state_publisher`, `ros_gz_bridge`, `odom_to_tf`, and RViz2.
- **`slam_toolbox.yaml`:** Pre-configured for laser-scan based SLAM.
- **`generate_maps.py`:** Generates synchronized multi-format map assets (`.json`, `.sdf`, `.pgm`, `.yaml`, `.rviz`) from a single procedural seed.

---

## 2. Architectural Blueprint & Target Structure

The codebase is split strictly into pure Python autonomy algorithms (`core/`), platform adapters (`backends/`), shared configurations (`config/`), and simulation launchers (`run.py`):

```
airmouse/
├── run.py                          # Unified CLI entry point for pygame and Gazebo modes
├── config/
│   ├── sensors.yaml                # Noise, bias, frequencies, and beam cone angles
│   ├── estimation.yaml             # EKF covariances, process noise, Mahony filter gains
│   ├── mapping.yaml                # Cell resolution (0.05m or 1.0m), log-odds clamps, ray step
│   ├── planning.yaml               # Obstacle inflation radius, safety margins, replan rate
│   └── mission.yaml                # Max flight duration, battery discharge model, RTH thresholds
├── core/                           # Pure Python 3.10+ (NO pygame, NO ROS 2 / rclpy imports)
│   ├── __init__.py
│   ├── hal.py                      # Abstract SensorHub, FlightInterface, DroneState dataclasses
│   ├── estimation/
│   │   ├── __init__.py
│   │   ├── ekf.py                  # 15-state Extended Kalman Filter (pos, vel, attitude, biases)
│   │   ├── visual_odometry.py      # Lucas-Kanade optical flow / FAST features + altitude scale
│   │   └── altitude_filter.py      # 1D Kalman filter fusing downward ultrasonic + baro/accel
│   ├── mapping/
│   │   ├── __init__.py
│   │   ├── occupancy_grid.py       # 2D Log-Odds occupancy grid with Bresenham / cone raycasting
│   │   └── map_io.py               # Serializers / deserializers for ROS and JSON map formats
│   ├── exploration/
│   │   ├── __init__.py
│   │   ├── frontier_detector.py    # Frontier edge extraction (free-to-unknown boundary detection)
│   │   └── frontier_evaluator.py   # Information gain vs travel cost scoring, blacklist cache
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── astar.py                # 8-connected grid A* / Theta* with diagonal cost and inflation
│   │   └── reactive_avoidance.py   # Safety reflex layer from raw ultrasonic ranges (virtual force field)
│   ├── control/
│   │   ├── __init__.py
│   │   └── pid.py                  # Position, velocity, and yaw heading PID controllers
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── yolo_detector.py        # Ultralytics YOLO wrapper (person class filter, conf thresh)
│   │   └── ray_projector.py        # Monocular pinhole 3D back-projection onto ground plane
│   │   └── survivor_tracker.py     # N-detection verification, Euclidean distance gating, deduplication
│   └── mission/
│       ├── __init__.py
│       ├── state_machine.py        # States: PREFLIGHT, TAKEOFF, EXPLORE, CONFIRM, RTH, LAND, FAILSAFE
│       └── mission_logger.py       # Metrics recorder (drift, coverage, survivors, flight time)
├── backends/
│   ├── __init__.py
│   ├── pygame_sim/                 # High-fidelity 2D sim preserving visual style + simulated physics
│   │   ├── __init__.py
│   │   ├── drone_dynamics.py       # Continuous 2D/3D quadrotor flight dynamics with drag and disturbance
│   │   ├── sensor_sim.py           # Realistic MPU6050 (bias drift), ultrasonic (15° cone), camera renderer
│   │   ├── world_renderer.py       # Textured 2.5D perspective camera generation for realistic YOLO detection
│   │   └── pygame_viewer.py        # Refactored UI dashboard (top-down map, camera inset, telemetry panel)
│   ├── gazebo_ros2/                # ROS 2 adapter nodes connecting Gazebo Harmonic to core/
│   │   ├── __init__.py
│   │   ├── airmouse_node.py        # ROS 2 Node bridging HAL to ROS topics (/imu, /range/*, /cmd_vel)
│   │   └── tf_broadcaster.py       # Broadcasts map -> odom -> base_link -> sensor frames
│   └── hardware/                   # Stubs for real hardware deployment
│       └── companion_hal.py        # MPU6050 I2C, HC-SR04 GPIO, V4L2 camera, MAVLink flight controller
├── tools/
│   └── generate_maps.py            # Unified procedural layout generator (.json, .sdf, .pgm, .rviz)
├── tests/
│   ├── test_zero_ground_truth.py   # Static inspection: fails if core/ imports backend or ground-truth
│   ├── test_ekf_convergence.py     # Verifies state estimator bounds under sensor noise
│   ├── test_mapping.py             # Occupancy grid log-odds updates and obstacle inflation tests
│   ├── test_frontier_planner.py    # Frontier detection & A* replanning benchmark
│   ├── test_perception_dedupe.py   # Multi-view survivor confirmation and clustering
│   └── test_state_machine.py       # Mission state transitions and failsafe triggers
└── README.md
```

---

## 3. Mathematical & Algorithmic Formulations

### 3.1 State Estimation: 15-State Sensor Fusion EKF
The drone cannot rely on IMU dead-reckoning alone, as double-integration of accelerometer bias leads to quadratic position error within seconds.
- **State Vector:**
  $$\mathbf{x} = \begin{bmatrix} \mathbf{p}^W & \mathbf{v}^W & \mathbf{q}_{WB} & \mathbf{b}_a & \mathbf{b}_g \end{bmatrix}^T \in \mathbb{R}^{16} \text{ (or error-state } \delta\mathbf{x} \in \mathbb{R}^{15}\text{)}$$
  Where $\mathbf{p}^W$ is position $(x, y, z)$, $\mathbf{v}^W$ is velocity $(\dot{x}, \dot{y}, \dot{z})$, $\mathbf{q}_{WB}$ is attitude quaternion, and $\mathbf{b}_a, \mathbf{b}_g$ are accelerometer and gyroscope biases.
- **Prediction Step (100–200 Hz):**
  Driven by MPU6050 specific force $\tilde{\mathbf{f}}_b$ and angular rates $\tilde{\boldsymbol{\omega}}_b$:
  $$\dot{\mathbf{p}}^W = \mathbf{v}^W, \quad \dot{\mathbf{v}}^W = \mathbf{R}(\mathbf{q})(\tilde{\mathbf{f}}_b - \mathbf{b}_a - \boldsymbol{\eta}_a) + \mathbf{g}^W$$
  $$\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \begin{bmatrix} 0 \\ \tilde{\boldsymbol{\omega}}_b - \mathbf{b}_g - \boldsymbol{\eta}_g \end{bmatrix}, \quad \dot{\mathbf{b}}_a = \boldsymbol{\eta}_{ba}, \quad \dot{\mathbf{b}}_g = \boldsymbol{\eta}_{bg}$$
- **Measurement Update 1: Visual Odometry / Optical Flow (15–30 Hz):**
  Sparse Lucas-Kanade optical flow on corner features across consecutive camera frames yields apparent optical flow $(\Delta u, \Delta v)$. Given altitude $z$ from the downward ultrasonic ranger:
  $$\mathbf{v}_{xy}^B = \begin{bmatrix} \frac{\Delta u \cdot z}{f_x \cdot \Delta t} \\ \frac{\Delta v \cdot z}{f_y \cdot \Delta t} \end{bmatrix} + \text{rotation compensation}$$
  This directly constrains velocity drift.
- **Measurement Update 2: Downward Ultrasonic Ranger (20 Hz):**
  Updates $z$ altitude: $z_{meas} = \mathbf{p}_z / (\mathbf{R}_{33}) + \nu_{alt}$.
- **Measurement Update 3: Scan-to-Map Ultrasonic Consistency (10 Hz):**
  Rays from the 4 horizontal ultrasonic sensors are compared with the current occupancy map to provide bounded position corrections.

### 3.2 Mapping: Inverse Sensor Model with Beam Cones
- **Cell Representation:** Log-odds $L(m_i) = \log\left(\frac{P(m_i=1)}{1 - P(m_i=1)}\right)$.
- **Ultrasonic Beam Cone Update:**
  Ultrasonic sensors have a ~15° beam divergence angle $\theta_{beam}$. For each ray inside the cone out to range $R$:
  - Along the cone interior ($r < R - \epsilon$): $L(m_i) \leftarrow \max(L(m_i) + l_{free}, L_{min})$ where $l_{free} = \log(0.35 / 0.65)$.
  - At the echo surface ($|r - R| \le \epsilon$): $L(m_i) \leftarrow \min(L(m_i) + l_{occ}, L_{max})$ where $l_{occ} = \log(0.85 / 0.15)$.
  - Outside the cone or beyond $R_{max}$: Unchanged.
- **Obstacle Inflation:** A circular dilation kernel with radius $r_{safe} = r_{drone} + \text{margin}$ (e.g., 0.35m) ensures planned paths maintain physical clearance.

### 3.3 Perception: 3D Camera Back-Projection & Tracking
- **Bounding Box to Ground Projection:**
  For any detection classified as `person` with confidence $\ge c_{thresh}$:
  1. Extract the bottom-center coordinate $(u_b, v_b) = ((x_1+x_2)/2, y_2)$ representing ground contact.
  2. Compute normalized ray in camera frame:
     $$\mathbf{v}_c = \mathbf{K}^{-1} \begin{bmatrix} u_b \\ v_b \\ 1 \end{bmatrix}$$
  3. Transform ray to world frame using estimated drone attitude $\mathbf{R}_{WB}$ and camera tilt: $\mathbf{v}_W = \mathbf{R}_{WC} \mathbf{v}_c$.
  4. Intersect ray with the ground plane $z = 0$ given drone altitude $z_d$:
     $$\mathbf{p}_{survivor}^W = \mathbf{p}_{drone}^W - \frac{z_d}{\mathbf{v}_{W, z}} \mathbf{v}_W$$
- **Survivor Deduplication & Confirmation:**
  - Track candidates across frames using a Kalman filter or running spatial cluster with gate distance $d_{gate} = 1.2\text{ m}$.
  - A survivor is marked **confirmed** only when detected in $\ge N_{confirm}$ (e.g., 4) distinct frames from different viewpoints.
  - Hard limit of 6 survivors tracked and recorded.

---

## 4. Phase-by-Phase Execution Plan

### Phase 1: Audit, Planning & Architecture Setup (Current Phase)
- [x] Complete environment verification (ROS 2 Jazzy, Gazebo Harmonic 8.15.0, WSLg, Python venv with `numpy 1.26.4`).
- [x] Audit `main.py`, `generate_maps.py`, and `my_slam_robot` URDF/launch files.
- [x] Author formal Implementation Plan artifact (`airmouse_implementation_plan.md`).
- [x] Define YAML configuration schema (`sensors.yaml`, `estimation.yaml`, `mapping.yaml`, `mission.yaml`).

### Phase 2: Codebase Refactoring & Unified Map Loading
- [ ] Construct the directory layout (`core/`, `backends/`, `config/`, `tests/`).
- [ ] Define the Hardware Abstraction Layer in `core/hal.py` (`SensorHub`, `FlightInterface`, `ImuData`, `RangeData`, `CameraFrame`).
- [ ] Implement `map_io.py` integrating `generate_maps.py:load_map` to consume procedural JSON maps without hardcoded houses.
- [ ] Migrate `main.py` UI rendering into `backends/pygame_sim/pygame_viewer.py` while ensuring zero autonomy imports of simulator internals.
- [ ] Verify unit test: `test_zero_ground_truth.py` confirms no circular or illegal imports.

### Phase 3: Sensor Simulation & Realistic Physics
- [ ] Implement `drone_dynamics.py` with continuous 3D velocity/attitude states, drag, and motor delay.
- [ ] Implement `sensor_sim.py`:
  - **MPU6050:** Gyro bias drift (random walk), white noise, accelerometer bias, gravity.
  - **Ultrasonics (4+1):** 15° cone raycasting against world geometry, Gaussian measurement noise, miss dropouts (reading $4.0\text{ m}$ on no return).
  - **Camera:** 2.5D / perspective rendering showing walls, doorways, furniture, and recognizable human figures (not flat single-color lines).
- [ ] Verify sensor outputs match hardware specifications through visualization and tests.

### Phase 4: State Estimation & Sensor Fusion
- [ ] Implement 15-state error-state EKF (`core/estimation/ekf.py`).
- [ ] Implement optical flow velocity estimator (`core/estimation/visual_odometry.py`) with ultrasonic altitude scaling.
- [ ] Implement Mahony/complementary filter for pitch/roll reference updates from accelerometer gravity vector.
- [ ] Log and plot estimation drift: $|\mathbf{p}_{est} - \mathbf{p}_{true}|$ over time to verify error boundedness.

### Phase 5: Online Mapping, Frontier Exploration & RTH
- [ ] Implement log-odds occupancy grid (`core/mapping/occupancy_grid.py`) updated solely from ultrasonic readings + estimated pose.
- [ ] Implement obstacle inflation and safety margins.
- [ ] Implement frontier detection (`core/exploration/frontier_detector.py`) extracting boundary points between free and unknown space.
- [ ] Implement goal selection based on utility: $U(f) = \text{InfoGain}(f) - \lambda \cdot \text{Cost}(f)$ with failure blacklisting.
- [ ] Implement A* pathfinder and local reactive ultrasonic avoidance reflex.
- [ ] Implement Return-to-Home (RTH) path planning back to launch coordinate.

### Phase 6: Perception & Survivor Localization
- [ ] Integrate Ultralytics YOLO (`yolo11n.pt`) wrapped in `core/perception/yolo_detector.py`.
- [ ] Implement geometric 3D ray back-projection from bounding-box contact point to ground coordinates.
- [ ] Implement multi-frame confirmation and spatial deduplication for up to 6 survivors.
- [ ] Test on generated house/maze layouts with 1 to 6 survivors; report localization precision and recall.

### Phase 7: Gazebo Harmonic, ROS 2 Jazzy & RViz Integration
- [ ] Update `my_slam_robot` or create `airmouse_gazebo` ROS 2 package.
- [ ] Configure Gazebo drone SDF model with IMU plugin, 5 ultrasonic/lidar sensor range plugins, and monocular camera.
- [ ] Configure `ros_gz_bridge` parameter bridge for bidirectional sensor/control streaming.
- [ ] Create `airmouse_node.py` implementing `SensorHub` and `FlightInterface` over ROS 2 topics (`/imu/data`, `/range/*`, `/camera/image_raw`, `/cmd_vel`).
- [ ] Configure RViz2 visualization displaying `/map` (drone SLAM) overlaid on `/ground_truth_map` (from map generator), with path history and survivor markers.

### Phase 8: Hardening, Benchmarking & Acceptance Testing
- [ ] Implement failsafes (pose quality degradation hold, sensor loss RTH, low battery threshold).
- [ ] Benchmark autonomy stack across $\ge 20$ random layouts (seeds 1001–1020) across house and maze modes.
- [ ] Generate comprehensive evaluation report covering:
  - Coverage percentage ($\ge 90\%$ of reachable free space).
  - Survivors detected vs total (precision / recall).
  - Final return-to-home landing error ($< 1.0\text{ m}$).
  - Localization drift (mean and peak error).
- [ ] Write complete `README.md` with instructions for running pygame sim, Gazebo + RViz sim, running tests, and porting to companion computers.

