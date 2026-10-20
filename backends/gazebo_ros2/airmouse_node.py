"""ROS 2 Node Adapter connecting Gazebo Harmonic to the NIDAR AirMouse Core.

Implements the SensorHub and FlightInterface ABCs over ROS 2 topics and bridges
to the pure Python autonomy core.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu, Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid as RosOccupancyGrid, Path as RosPath
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
from cv_bridge import CvBridge

from airmouse.core.hal import SensorHub, FlightInterface, ImuData, RangeData, CameraFrame, VelocityCommand, DroneState
from airmouse.core.mapping.map_io import load_map_file
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.estimation.ekf import SensorFusionEKF
from airmouse.core.perception.yolo_detector import YoloDetector
from airmouse.core.perception.ray_projector import GroundRayProjector
from airmouse.core.mission.state_machine import MissionCoordinator, MissionState


class Ros2SensorHub(SensorHub):
    """Bridges ROS 2 incoming sensor topics to SensorHub interface."""
    def __init__(self, node: Node):
        self.node = node
        self.cv_bridge = CvBridge()

        self.latest_imu: Optional[ImuData] = None
        self.latest_ranges: Dict[str, RangeData] = {}
        self.latest_camera: Optional[CameraFrame] = None
        self.lock = threading.Lock()

        # Subscribers
        self.sub_imu = node.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.sub_cam = node.create_subscription(Image, '/camera/image_raw', self._cam_cb, 10)
        # LaserScan / Range subscriptions
        self.sub_scan = node.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        with self.lock:
            self.latest_imu = ImuData(timestamp_s=t, accel_mps2=accel, gyro_radps=gyro)

    def _cam_cb(self, msg: Image):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            h, w = cv_img.shape[:2]
            with self.lock:
                self.latest_camera = CameraFrame(
                    timestamp_s=t,
                    image_bgr=cv_img,
                    width=w,
                    height=h,
                    fov_h_rad=math.radians(70.0),
                    fov_v_rad=math.radians(55.0),
                )
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan):
        """Extract front, left, right, rear ranges from 360 scan."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        def get_dist(angle_rad: float) -> float:
            idx = int((angle_rad - msg.angle_min) / msg.angle_increment) % num_readings
            val = msg.ranges[idx]
            if math.isnan(val) or val > msg.range_max or val < msg.range_min:
                return 4.0
            return float(val)

        r_front = get_dist(0.0)
        r_left = get_dist(math.pi / 2.0)
        r_right = get_dist(-math.pi / 2.0)
        r_rear = get_dist(math.pi)

        with self.lock:
            self.latest_ranges = {
                "front": RangeData("front", r_front, r_front < 3.9, t),
                "left": RangeData("left", r_left, r_left < 3.9, t),
                "right": RangeData("right", r_right, r_right < 3.9, t),
                "rear": RangeData("rear", r_rear, r_rear < 3.9, t),
                "down": RangeData("down", 1.2, True, t),  # default Gazebo height
            }

    def read_imu(self) -> Optional[ImuData]:
        with self.lock:
            return self.latest_imu

    def read_ranges(self) -> Dict[str, RangeData]:
        with self.lock:
            return dict(self.latest_ranges)

    def read_camera(self) -> Optional[CameraFrame]:
        with self.lock:
            return self.latest_camera

    def get_time(self) -> float:
        now = self.node.get_clock().now()
        return now.nanoseconds * 1e-9


class Ros2FlightInterface(FlightInterface):
    """Bridges FlightInterface commands to /cmd_vel topic."""
    def __init__(self, node: Node):
        self.node = node
        self.pub_cmd = node.create_publisher(Twist, '/cmd_vel', 10)
        self.battery = 98.0
        self.is_armed = False

    def send_velocity(self, cmd: VelocityCommand) -> None:
        msg = Twist()
        msg.linear.x = float(cmd.vx)
        msg.linear.y = float(cmd.vy)
        msg.linear.z = float(cmd.vz)
        msg.angular.z = float(cmd.yaw_rate_radps)
        self.pub_cmd.publish(msg)

    def takeoff(self, target_altitude_m: float = 1.2) -> bool:
        self.is_armed = True
        return True

    def land(self) -> bool:
        msg = Twist()
        self.pub_cmd.publish(msg)
        self.is_armed = False
        return True

    def get_battery_percentage(self) -> float:
        return self.battery


class AirMouseNode(Node):
    def __init__(self, map_path: str):
        super().__init__('airmouse_mission_node')
        self.get_logger().info(f"Initializing AirMouse node with map: {map_path}")

        # 1. Load map
        self.true_grid, self.meta = load_map_file(map_path)
        start_wx, start_wy = self.meta.start_world

        # 2. HAL Adapters
        self.sensor_hub = Ros2SensorHub(self)
        self.flight = Ros2FlightInterface(self)
        self.cv_bridge = CvBridge()

        # 3. Autonomy Core
        init_pos = np.array([start_wx, start_wy, 1.2])
        self.ekf = SensorFusionEKF(init_pos=init_pos, init_yaw=0.0)
        self.grid_map = OccupancyGrid(
            width_m=self.meta.cols * self.meta.cell_size_m,
            height_m=self.meta.rows * self.meta.cell_size_m,
            resolution_m=0.25,
        )
        self.yolo = YoloDetector()
        self.ray_projector = GroundRayProjector()
        self.mission = MissionCoordinator(
            start_world=(start_wx, start_wy),
            grid_map=self.grid_map,
            flight=self.flight,
        )

        # 4. ROS 2 Publishers
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(RosOccupancyGrid, '/map', latching_qos)
        self.pub_gt_map = self.create_publisher(RosOccupancyGrid, '/ground_truth_map', latching_qos)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_path = self.create_publisher(RosPath, '/drone/path', 10)
        self.pub_plan = self.create_publisher(RosPath, '/drone/plan', 10)
        self.pub_survivors = self.create_publisher(MarkerArray, '/survivors', 10)
        self.pub_cam_annotated = self.create_publisher(Image, '/camera/image_annotated', 10)
        self.pub_mission_state = self.create_publisher(String, '/mission/state', 10)

        # Publish ground truth map once
        self._publish_ground_truth_map()

        # Main timer loop at 20 Hz
        self.path_history: List[Tuple[float, float]] = []
        self.timer = self.create_timer(0.05, self._tick)

    def _tick(self):
        now_s = self.sensor_hub.get_time()

        # Step EKF with IMU
        imu = self.sensor_hub.read_imu()
        if imu is not None:
            self.ekf.predict(imu)

        # Update Sonar
        ranges = self.sensor_hub.read_ranges()
        if ranges:
            self.ekf.update_sonar(ranges)
            drone_state = self.ekf.get_state(now_s)
            est_yaw = drone_state.attitude_rpy[2]
            est_x, est_y = drone_state.position[0], drone_state.position[1]
            yaw_offsets = {"front": 0.0, "left": math.pi / 2.0, "right": -math.pi / 2.0, "rear": math.pi}
            for sid, offset in yaw_offsets.items():
                if sid in ranges:
                    self.grid_map.update_beam(
                        start_wx=est_x,
                        start_wy=est_y,
                        angle_rad=est_yaw + offset,
                        range_m=ranges[sid].range_m,
                        valid_hit=ranges[sid].valid,
                    )
            self.ekf.update_scan_to_map(ranges, self.grid_map)

        # Update Camera & YOLO
        cam = self.sensor_hub.read_camera()
        latest_detections = []
        if cam is not None:
            drone_state = self.ekf.get_state(now_s)
            gyro = imu.gyro_radps if imu is not None else np.zeros(3)
            self.ekf.update_camera(cam, gyro)

            latest_detections = self.yolo.detect(cam.image_bgr)
            for det in latest_detections:
                u_b, v_b = det.bottom_center
                pt_ground = self.ray_projector.project_pixel_to_ground(
                    u_b, v_b, drone_state.position, drone_state.attitude_rpy[2],
                )
                if pt_ground is not None:
                    self.mission.tracker.update(pt_ground, det.confidence)

            # Publish annotated camera
            annotated = cam.image_bgr.copy()
            for b in latest_detections:
                cv2.rectangle(annotated, (int(b.x1), int(b.y1)), (int(b.x2), int(b.y2)), (0, 255, 0), 2)
            img_msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"
            self.pub_cam_annotated.publish(img_msg)

        # Update Mission
        drone_state = self.ekf.get_state(now_s)
        cmd = self.mission.update(now_s, drone_state, ranges)
        self.flight.send_velocity(cmd)

        # Publish telemetry and ROS messages
        self.path_history.append((float(drone_state.position[0]), float(drone_state.position[1])))
        self._publish_ros_state(drone_state)

    def _publish_ros_state(self, state: DroneState):
        now = self.get_clock().now().to_msg()

        # 1. State string
        str_msg = String()
        str_msg.data = self.mission.state.name
        self.pub_mission_state.publish(str_msg)

        # 2. Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(state.position[0])
        odom.pose.pose.position.y = float(state.position[1])
        odom.pose.pose.position.z = float(state.position[2])
        odom.pose.pose.orientation.w = float(state.quaternion[0])
        odom.pose.pose.orientation.x = float(state.quaternion[1])
        odom.pose.pose.orientation.y = float(state.quaternion[2])
        odom.pose.pose.orientation.z = float(state.quaternion[3])
        self.pub_odom.publish(odom)

        # 3. Path
        path_msg = RosPath()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "map"
        for x, y in self.path_history[-100:]:  # last 100 points
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = float(state.position[2])
            path_msg.poses.append(ps)
        self.pub_path.publish(path_msg)

        # 4. Survivors MarkerArray
        ma = MarkerArray()
        for s in self.mission.tracker.confirmed_survivors:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = "map"
            m.ns = "survivors"
            m.id = s.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            sx, sy = s.estimated_pos
            m.pose.position.x = sx
            m.pose.position.y = sy
            m.pose.position.z = 0.8
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 1.6
            m.color.r = 0.9
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 0.95
            ma.markers.append(m)
        self.pub_survivors.publish(ma)

        # 5. Occupancy Grid
        ros_map = RosOccupancyGrid()
        ros_map.header.stamp = now
        ros_map.header.frame_id = "map"
        ros_map.info.resolution = self.grid_map.res
        ros_map.info.width = self.grid_map.cols
        ros_map.info.height = self.grid_map.rows
        ros_map.info.origin.position.x = self.grid_map.origin_x
        ros_map.info.origin.position.y = self.grid_map.origin_y

        prob = 1.0 / (1.0 + np.exp(-self.grid_map.log_odds))
        grid_data = np.full((self.grid_map.rows, self.grid_map.cols), -1, dtype=np.int8)
        grid_data[prob < self.grid_map.free_thresh] = 0
        grid_data[prob > self.grid_map.occ_thresh] = 100
        ros_map.data = grid_data.flatten().tolist()
        self.pub_map.publish(ros_map)

    def _publish_ground_truth_map(self):
        now = self.get_clock().now().to_msg()
        gt_map = RosOccupancyGrid()
        gt_map.header.stamp = now
        gt_map.header.frame_id = "map"
        gt_map.info.resolution = self.meta.cell_size_m
        gt_map.info.width = self.meta.cols
        gt_map.info.height = self.meta.rows
        gt_map.info.origin.position.x = 0.0
        gt_map.info.origin.position.y = 0.0

        # Flip so row 0 corresponds to max Y
        flipped = np.flipud(self.true_grid)
        data = np.where(flipped == 0, 0, 100).astype(np.int8)
        gt_map.data = data.flatten().tolist()
        self.pub_gt_map.publish(gt_map)


def launch_ros2_node(map_path: str):
    rclpy.init()
    node = AirMouseNode(map_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    map_p = sys.argv[1] if len(sys.argv) > 1 else "maps/house_00042.json"
    launch_ros2_node(map_p)

