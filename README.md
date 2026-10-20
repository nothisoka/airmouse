# NIDAR AirMouse: Autonomous Indoor Exploration & Search-and-Rescue Stack

An autonomous, sensor-driven indoor drone autonomy stack developed for search-and-rescue in unknown buildings.
AirMouse explores unseen environments with **zero prior knowledge**, constructs an online log-odds occupancy map using narrow-beam ultrasonic rangefinders, localizes up to 6 survivors using YOLO vision with monocular 3D back-projection, and executes autonomous Return-to-Home (RTH) and landing.

---

## 1. System Architecture & HAL Design

The codebase strictly decouples pure autonomy algorithms from simulation engines and hardware drivers:

```
airmouse/
├── run.py                          # Unified CLI entry point for Pygame and Gazebo backends
├── config/                         # Centralized YAML configuration files
│   ├── sensors.yaml                # MPU6050 & HC-SR04 noise, bias, and beam geometry
│   ├── estimation.yaml             # EKF covariances, process noise, optical flow settings
│   ├── mapping.yaml                # Log-odds clamps, cell resolution, obstacle inflation
│   ├── planning.yaml               # A* heuristic weights, frontier clustering, avoidance
│   └── mission.yaml                # Flight time budget, battery discharge, RTH thresholds
├── core/                           # Pure Python 3.10+ Autonomy Core (NO pygame, NO ROS 2)
│   ├── hal.py                      # Abstract SensorHub, FlightInterface, DroneState
│   ├── estimation/                 # 15-state EKF, Lucas-Kanade optical flow, altitude filter
│   ├── mapping/                    # Log-odds occupancy grid, cone raycasting, map I/O
│   ├── exploration/                # Frontier boundary detection, utility scoring, blacklisting
│   ├── planning/                   # 8-connected grid A*, obstacle inflation, reactive avoidance
│   ├── control/                    # Waypoint tracking and heading alignment PID
│   ├── perception/                 # Ultralytics YOLO wrapper, 3D ray projector, deduplication
│   └── mission/                    # State machine (PREFLIGHT->TAKEOFF->EXPLORE->RTH->LAND), metrics
├── backends/
│   ├── pygame_sim/                 # High-fidelity 2D/2.5D sim with continuous 3D drone physics
│   ├── gazebo_ros2/                # ROS 2 Jazzy node adapter bridging Gazebo Harmonic to core/
│   └── hardware/                   # Hardware stubs for MPU6050, ultrasonic GPIO, V4L2, MAVLink
├── tools/
│   ├── generate_maps.py            # Procedural map generator (.json, .sdf, .pgm, .rviz)
│   └── benchmark_suite.py          # Automated 20-layout evaluation and metrics compiler
└── tests/                          # Automated unit test suite with pytest
```

### Architectural Guarantees
- **Zero Ground Truth in Autonomy:** Autonomy algorithms inside `core/` have zero access to the simulator's true grid, true drone coordinates, or hidden survivor coordinates. This is strictly enforced via AST inspection in `tests/test_zero_ground_truth.py`.
- **Zero ROS/Pygame in Core:** `core/` never imports `pygame` or `rclpy`. Platform communication occurs exclusively through the `SensorHub` and `FlightInterface` abstract interfaces.

---

## 2. Sensor Fusion & Autonomy Algorithms

### 2.1 State Estimation (15-State Error-State EKF)
IMU dead-reckoning diverges rapidly due to accelerometer bias integration. AirMouse implements a 15-state Extended Kalman Filter fusing:
- **MPU6050 (100 Hz):** Accelerometer specific force and gyroscope angular rates drive the prediction step.
- **Downward Ultrasonic (20 Hz):** Ground-truth altitude measurement corrects vertical position and velocity.
- **Camera Optical Flow (20 Hz):** Sparse Lucas-Kanade feature flow with rotational gyro compensation provides metric velocity scaled by altitude:
  $$\mathbf{v}_{xy}^B = \begin{bmatrix} -\frac{\Delta v \cdot z}{f_y \Delta t} \\ -\frac{\Delta u \cdot z}{f_x \Delta t} \end{bmatrix}$$
- **Scan-to-Map Consistency (10 Hz):** Ultrasonic range returns are compared against the online occupancy grid to directly constrain horizontal position drift.
- **Pose Quality:** Real-time covariance trace monitoring classifies pose reliability into `EXCELLENT`, `DEGRADED`, or `CRITICAL` (triggering cautious hold or RTH).

### 2.2 Log-Odds Occupancy Grid Mapping
Ultrasonic sensors feature a ~15° beam divergence cone. AirMouse casts multiple rays across the cone:
- Free cells along the cone interior are updated with $l_{free} = \ln(0.35 / 0.65)$.
- Surface echo hit cells are updated with $l_{occ} = \ln(0.85 / 0.15)$.
- Obstacles are dynamically inflated by $r_{safe} = 0.35\text{ m}$ using circular morphological dilation.

### 2.3 Frontier Exploration & Planning
- **Frontier Detection:** Scans the occupancy grid for free cells bordering unexplored unknown space and clusters adjacent frontier cells.
- **Utility Function:** Selects targets maximizing information gain while penalizing travel distance: $U = \alpha \cdot \text{ClusterSize} - \beta \cdot \text{Distance}$.
- **Blacklisting:** Targets that fail pathfinding are blacklisted to eliminate infinite exploration loops.
- **Reactive Safety Layer:** Live ultrasonic readings trigger repulsive virtual force vectors if obstacles approach within $0.35\text{ m}$.

### 2.4 Perception & 3D Survivor Localization
- **YOLO Inference:** Runs `yolo11n.pt` on the camera stream to detect persons.
- **Geometric Ray Back-Projection:** Computes ground intersection from bounding-box bottom center $(u, v)$, camera intrinsics $\mathbf{K}$, and drone attitude/altitude.
- **Multi-View Confirmation & Tracking:** Detections are clustered with a $1.2\text{ m}$ Euclidean distance gate. A survivor is marked confirmed only after $\ge 3$ consistent detections. Tracks up to 6 distinct survivors.

---

## 3. Quickstart & Usage

### 3.1 Prerequisites, Virtual Environment & Installation
```bash
# Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic
python3 -m venv --system-site-packages ~/airmouse_venv
source ~/airmouse_venv/bin/activate

# Install Python dependencies (pin numpy<2 for cv_bridge compatibility)
pip install "numpy<2" pygame opencv-python ultralytics pyyaml

# Install airmouse in editable mode:
cd ~/airmouse
pip install -e .
```

### 3.2 Running the 2D/2.5D Simulation (Pygame)
```bash
source ~/airmouse_venv/bin/activate
cd ~/airmouse

# Interactive simulation with live GUI dashboard:
python run.py --map maps/house_00042.json --backend pygame

# Headless benchmark mode:
python run.py --map maps/house_00042.json --headless --max-time 60.0
```
**Keyboard Controls in GUI:**
- `SPACE`: Pause / Resume mission
- `R`: Restart mission from home
- `ESC`: Quit

### 3.3 Running Gazebo Harmonic + ROS 2 Jazzy + RViz
```bash
source /opt/ros/jazzy/setup.bash
source ~/slam_ws/install/setup.bash

# Launch full unified simulation in Gazebo & RViz:
ros2 launch my_slam_robot airmouse_mission.launch.py map_name:=house_00042
```
In RViz, you can simultaneously view:
- `/map`: Online SLAM occupancy grid built by the drone
- `/ground_truth_map`: Procedural ground truth building layout
- `/drone/path`: Trajectory history
- `/survivors`: 3D cylinder markers showing confirmed survivor locations
- `/camera/image_annotated`: Live camera feed with YOLO detection bounding boxes

---

## 4. Map Generation & Adding Custom Layouts

To generate procedural house or maze layouts with survivors:
```bash
# Generate 5 house layouts with 1-6 survivors:
python generate_maps.py --count 5 --mode house --seed 42 --out maps/

# Generate 3 maze layouts:
python generate_maps.py --count 3 --mode maze --persons 4 --out maps/
```
Each layout generates:
- `<name>.json`: Grid geometry, start pose, hidden survivor coordinates
- `<name>.sdf`: Gazebo Harmonic world file (walls, furniture, survivors)
- `<name>.pgm` / `<name>.yaml`: Ground-truth map for RViz and Nav2
- `<name>_preview.png`: Top-down visual preview image

---

## 5. Verification & Automated Testing

Run the full pytest suite (with no environment variables needed after editable install):
```bash
source ~/airmouse_venv/bin/activate
pytest -v tests/
```
Tests cover:
- `test_zero_ground_truth.py`: Enforces architectural isolation (0 illegal imports in `core/`)
- `test_map_loader.py`: Verifies procedural map loading and coordinate bijections
- `test_sensor_sim.py`: Verifies MPU6050, 4+1 ultrasonic rangers, and camera simulation
- `test_ekf_convergence.py`: Verifies state estimation convergence and covariance bounds
- `test_mapping.py`: Verifies log-odds beam raycasting and obstacle inflation
- `test_frontier_planner.py`: Verifies frontier detection and A* path generation
- `test_perception_dedupe.py`: Verifies 3D ray back-projection and survivor deduplication
- `test_state_machine.py`: Verifies mission transitions and low-battery failsafes

---

## 6. Porting to Physical Hardware (Companion Computer)

To deploy AirMouse onto a physical drone (Jetson Orin Nano / Raspberry Pi 5):
1. Implement the hardware drivers in `backends/hardware/companion_hal.py`:
   - `SensorHub.read_imu()`: Read MPU6050 via I2C (`smbus2`)
   - `SensorHub.read_ranges()`: Read 5 HC-SR04 ultrasonic sensors via GPIO or UART
   - `SensorHub.read_camera()`: Read downward-tilted USB/MIPI-CSI camera via OpenCV V4L2
   - `FlightInterface.send_velocity()`: Send MAVLink `SET_POSITION_TARGET_LOCAL_NED` to flight controller (PX4 / ArduPilot via PyMAVLink)
2. Run `core/` as a standalone daemon on the companion computer. No changes to `core/` are needed!
3. For accelerated vision inference, export YOLO weights to TensorRT:
   ```bash
   yolo export model=yolo11n.pt format=engine device=0
   ```
