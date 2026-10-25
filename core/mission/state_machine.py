"""Mission State Machine for NIDAR AirMouse."""
from __future__ import annotations

import math
from enum import Enum, auto
from typing import List, Optional, Tuple
import numpy as np

from airmouse.core.hal import DroneState, PoseQuality, VelocityCommand, FlightInterface
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.planning.astar import AStarPlanner
from airmouse.core.exploration.frontier_detector import FrontierDetector
from airmouse.core.control.pid import WaypointController
from airmouse.core.planning.reactive_avoidance import ReactiveAvoidance
from airmouse.core.perception.survivor_tracker import SurvivorTracker


class MissionState(Enum):
    PREFLIGHT = auto()
    TAKEOFF = auto()
    EXPLORE = auto()
    RETURN_HOME = auto()
    LAND = auto()
    COMPLETE = auto()
    FAILSAFE = auto()


def _get_default_mission_config() -> dict:
    try:
        from pathlib import Path
        import yaml
        cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "mission.yaml"
        if cfg_path.exists():
            with open(cfg_path, "r") as f:
                data = yaml.safe_load(f)
                if isinstance(data, dict):
                    return data.get("mission", {})
    except Exception:
        pass
    return {}


class MissionCoordinator:
    def __init__(
        self,
        start_world: Tuple[float, float],
        grid_map: OccupancyGrid,
        flight: FlightInterface,
        max_flight_time_s: Optional[float] = None,
        low_battery_pct: Optional[float] = None,
        target_altitude_m: Optional[float] = None,
        cruise_speed_mps: Optional[float] = None,
        return_home_margin: Optional[float] = None,
    ):
        cfg = _get_default_mission_config()
        self.start_world = start_world
        self.grid_map = grid_map
        self.flight = flight
        self.max_flight_time = float(max_flight_time_s if max_flight_time_s is not None else cfg.get("max_flight_time_s", 600.0))
        self.low_battery_pct = float(low_battery_pct if low_battery_pct is not None else cfg.get("low_battery_threshold_pct", 20.0))
        self.target_alt = float(target_altitude_m if target_altitude_m is not None else cfg.get("takeoff_altitude_m", 1.2))
        self.cruise_speed = float(cruise_speed_mps if cruise_speed_mps is not None else cfg.get("nominal_cruise_speed_mps", 0.6))
        self.return_home_margin = float(return_home_margin if return_home_margin is not None else cfg.get("return_home_margin", 0.30))

        self.state = MissionState.PREFLIGHT
        self.start_time = 0.0

        # Subsystems
        self.planner = AStarPlanner()
        self.frontier_detector = FrontierDetector()
        self.controller = WaypointController(max_vel_xy=self.cruise_speed)
        self.avoidance = ReactiveAvoidance()
        self.tracker = SurvivorTracker()

        # Path tracking
        self.current_path: List[Tuple[float, float]] = []
        self.current_target: Optional[Tuple[float, float]] = None
        self.last_replan_time = -1.0
        self.replan_interval = 2.0  # seconds
        self.no_frontier_count = 0
        self.breadcrumbs: List[Tuple[float, float]] = [self.start_world]
        self.last_breadcrumb_pos: Tuple[float, float] = self.start_world

    def update(
        self,
        current_time_s: float,
        drone_state: DroneState,
        ranges: dict,
    ) -> VelocityCommand:
        """Run one step of the mission state machine and return velocity command."""
        battery = self.flight.get_battery_percentage()
        elapsed = current_time_s - self.start_time if self.start_time > 0 else 0.0

        # Global failsafe checks during flight
        if self.state == MissionState.EXPLORE:
            # Reserve return_home_margin of battery/time budget for return-to-home navigation and landing
            margin_fraction = self.return_home_margin if self.return_home_margin <= 0.5 else (1.0 - self.return_home_margin)
            rth_trigger_time = self.max_flight_time * (1.0 - margin_fraction)
            if battery <= self.low_battery_pct or elapsed >= rth_trigger_time:
                self.state = MissionState.RETURN_HOME
                self._plan_return_home(drone_state)
            elif drone_state.quality == PoseQuality.CRITICAL:
                # High uncertainty: slow down and return home
                self.state = MissionState.RETURN_HOME
                self._plan_return_home(drone_state)

        # State Machine Transitions
        if self.state == MissionState.PREFLIGHT:
            self.start_time = current_time_s
            self.state = MissionState.TAKEOFF
            return VelocityCommand(vx=0.0, vy=0.0, vz=0.4, yaw_rate_radps=0.0)

        elif self.state == MissionState.TAKEOFF:
            reached = self.flight.takeoff(target_altitude_m=self.target_alt)
            if reached or drone_state.position[2] >= self.target_alt - 0.1:
                self.state = MissionState.EXPLORE
                self._plan_next_frontier(drone_state)
            return VelocityCommand(vx=0.0, vy=0.0, vz=0.3, yaw_rate_radps=0.0)

        elif self.state == MissionState.EXPLORE:
            # Check if all 6 survivors are found
            if self.tracker.all_found:
                self.state = MissionState.RETURN_HOME
                self._plan_return_home(drone_state)
                return VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.0)

            # Replan periodically or when path is empty (gated by minimum replan cooldown)
            should_replan = (current_time_s - self.last_replan_time >= self.replan_interval) or (
                not self.current_path and current_time_s - self.last_replan_time >= 0.5
            )
            if should_replan:
                self.last_replan_time = current_time_s
                self._plan_next_frontier(drone_state)
                if not self.current_path:
                    self.no_frontier_count += 1
                else:
                    self.no_frontier_count = 0

            if not self.current_path:
                # If we recently started or have only tried a few replans, rotate slowly to scan the room with sonars
                if self.no_frontier_count < 10:
                    return VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.6)
                else:
                    # Genuinely exhausted frontiers: exploration completed!
                    self.state = MissionState.RETURN_HOME
                    self._plan_return_home(drone_state)
                    return VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.0)

            # Record safe traversed breadcrumbs
            drone_xy = (float(drone_state.position[0]), float(drone_state.position[1]))
            if math.hypot(drone_xy[0] - self.last_breadcrumb_pos[0], drone_xy[1] - self.last_breadcrumb_pos[1]) >= 0.5:
                self.breadcrumbs.append(drone_xy)
                self.last_breadcrumb_pos = drone_xy

            # Follow current path
            target_pt = self.current_path[0]
            target_3d = (target_pt[0], target_pt[1], self.target_alt)
            cmd, reached = self.controller.compute_command(drone_state, target_3d)

            if reached:
                self.current_path.pop(0)

            # Apply reactive sonar avoidance filter
            safe_cmd, override = self.avoidance.filter_command(cmd, ranges)
            return safe_cmd

        elif self.state == MissionState.RETURN_HOME:
            dist_to_home = math.hypot(drone_state.position[0] - self.start_world[0],
                                      drone_state.position[1] - self.start_world[1])
            # Only land if close to home AND has completed flight exploration (prevents instant landing at takeoff)
            if dist_to_home < 0.8 and elapsed > 15.0:
                self.state = MissionState.LAND
                return VelocityCommand(vx=0.0, vy=0.0, vz=-0.4, yaw_rate_radps=0.0)

            # Replan path periodically to adapt to newly mapped obstacles or deviations
            if not self.current_path or (current_time_s - self.last_replan_time > self.replan_interval):
                self._plan_return_home(drone_state)
                self.last_replan_time = current_time_s

            if not self.current_path:
                if dist_to_home < 1.0:
                    self.state = MissionState.LAND
                    return VelocityCommand(vx=0.0, vy=0.0, vz=-0.4, yaw_rate_radps=0.0)
                rev_crumbs = list(reversed(self.breadcrumbs))
                self.current_path = rev_crumbs + [self.start_world]

            target_pt = self.current_path[0]
            target_3d = (target_pt[0], target_pt[1], self.target_alt)
            cmd, reached = self.controller.compute_command(drone_state, target_3d)

            if reached:
                self.current_path.pop(0)
                if not self.current_path:
                    self.state = MissionState.LAND

            safe_cmd, override = self.avoidance.filter_command(cmd, ranges)
            return safe_cmd

        elif self.state == MissionState.LAND:
            landed = self.flight.land()
            if landed or drone_state.position[2] <= 0.2:
                self.state = MissionState.COMPLETE
            return VelocityCommand(vx=0.0, vy=0.0, vz=-0.4, yaw_rate_radps=0.0)

        elif self.state == MissionState.COMPLETE:
            return VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.0)

        return VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate_radps=0.0)

    def _plan_next_frontier(self, state: DroneState):
        drone_xy = (float(state.position[0]), float(state.position[1]))
        candidates = self.frontier_detector.find_candidate_frontiers(self.grid_map, drone_xy, top_k=5)
        for frontier in candidates:
            path = self.planner.plan(self.grid_map, drone_xy, frontier, allow_unknown=True)
            if path:
                self.current_target = frontier
                self.current_path = path
                return
            else:
                self.frontier_detector.report_failure(frontier, self.grid_map)

        self.current_path = []

    def _plan_return_home(self, state: DroneState):
        drone_xy = (float(state.position[0]), float(state.position[1]))
        # First try planning strictly through KNOWN FREE space (never through unmapped walls)
        path = self.planner.plan(self.grid_map, drone_xy, self.start_world, allow_unknown=False)
        if path and len(path) >= 2:
            self.current_path = path
        else:
            # Fallback to reverse traversed breadcrumbs (guaranteed collision-free verified path)
            if self.breadcrumbs:
                dists = [math.hypot(b[0] - drone_xy[0], b[1] - drone_xy[1]) for b in self.breadcrumbs]
                closest_idx = int(np.argmin(dists))
                rev_crumbs = list(reversed(self.breadcrumbs[:closest_idx + 1]))
                self.current_path = rev_crumbs + [self.start_world]
            else:
                self.current_path = [self.start_world]
