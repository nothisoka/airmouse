"""Pygame Dashboard Viewer for NIDAR AirMouse.

Renders real-time exploration map, telemetry dashboard, camera view with YOLO
bounding box annotations, and ground truth comparison in simulation.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple
import cv2
import numpy as np

try:
    import pygame
except ImportError:
    pygame = None

from airmouse.core.hal import DroneState, PoseQuality
from airmouse.core.mapping.occupancy_grid import OccupancyGrid
from airmouse.core.mission.state_machine import MissionState
from airmouse.core.perception.survivor_tracker import SurvivorTrack
from airmouse.core.perception.yolo_detector import DetectionBox


class PygameViewer:
    def __init__(
        self,
        grid_rows: int,
        grid_cols: int,
        cell_px: int = 28,
        panel_w: int = 340,
        top_h: int = 58,
        fps: int = 30,
        headless: bool = False,
    ):
        self.rows = grid_rows
        self.cols = grid_cols
        self.cell_px = cell_px
        self.panel_w = panel_w
        self.top_h = top_h
        self.fps = fps
        self.headless = headless

        self.width = self.cols * self.cell_px + self.panel_w
        self.height = self.rows * self.cell_px + self.top_h

        self.screen = None
        self.font = None
        self.small_font = None
        self.clock = None

        if not self.headless and pygame is not None:
            pygame.init()
            pygame.display.set_caption("NIDAR AirMouse — Autonomous Exploration & Rescue Sim")
            self.screen = pygame.display.set_mode((self.width, self.height))
            self.font = pygame.font.SysFont("consolas", 20, bold=True)
            self.small_font = pygame.font.SysFont("consolas", 13)
            self.clock = pygame.time.Clock()

    def process_events(self) -> Tuple[bool, bool, bool]:
        """Handle keyboard input: returns (running, paused_toggled, restart_requested)."""
        if self.headless or pygame is None:
            return True, False, False

        running = True
        paused_toggled = False
        restart_requested = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused_toggled = True
                elif event.key == pygame.K_r:
                    restart_requested = True

        return running, paused_toggled, restart_requested

    def render(
        self,
        true_grid: np.ndarray,
        occupancy_grid: OccupancyGrid,
        drone_est_state: DroneState,
        drone_true_pos: np.ndarray,
        drone_true_yaw: float,
        planned_path: List[Tuple[float, float]],
        confirmed_survivors: List[SurvivorTrack],
        camera_bgr: np.ndarray,
        detections: List[DetectionBox],
        mission_state: MissionState,
        flight_time_s: float,
        battery_pct: float,
        ranges: dict,
        coverage_pct: float,
        drift_m: float,
    ):
        if self.headless or self.screen is None:
            return

        screen = self.screen
        screen.fill((21, 24, 30))

        # Top banner
        pygame.draw.rect(screen, (30, 35, 44), (0, 0, self.width, self.top_h))
        title = self.font.render("NIDAR AIRMOUSE  |  AUTONOMOUS RESCUE MISSION", True, (240, 244, 250))
        screen.blit(title, (16, 16))

        # Render Map Grid
        ox, oy = 0, self.top_h
        for y in range(self.rows):
            for x in range(self.cols):
                rect = pygame.Rect(ox + x * self.cell_px, oy + y * self.cell_px, self.cell_px - 1, self.cell_px - 1)
                val = true_grid[y, x]
                if val == 1:
                    color = (55, 62, 72)     # Wall
                elif val == 2:
                    color = (120, 108, 92)   # Furniture
                else:
                    color = (215, 220, 225)  # Free floor
                pygame.draw.rect(screen, color, rect)

                # Overlay online estimated occupancy grid:
                # Find corresponding cells in occupancy grid
                wx = (x + 0.5) * 1.0
                wy = (self.rows - y - 0.5) * 1.0
                gx, gy = occupancy_grid.world_to_grid(wx, wy)
                if occupancy_grid.is_free(gx, gy):
                    # Soft green tint for explored free space
                    tint = pygame.Surface((self.cell_px - 1, self.cell_px - 1), pygame.SRCALPHA)
                    tint.fill((140, 220, 160, 95))
                    screen.blit(tint, rect)

        # Draw planned path
        if planned_path:
            pts = []
            for wx, wy in planned_path:
                px = ox + int(wx * self.cell_px)
                py = oy + int((self.rows - wy) * self.cell_px)
                pts.append((px, py))
            if len(pts) > 1:
                pygame.draw.lines(screen, (40, 130, 250), False, pts, 3)

        # Draw confirmed survivors
        for s in confirmed_survivors:
            sx, sy = s.estimated_pos
            px = ox + int(sx * self.cell_px)
            py = oy + int((self.rows - sy) * self.cell_px)
            pygame.draw.circle(screen, (230, 60, 60), (px, py), 10)
            pygame.draw.circle(screen, (255, 255, 255), (px, py), 10, 2)
            lbl = self.small_font.render(f"P{s.id}", True, (255, 255, 255))
            screen.blit(lbl, (px - 7, py - 6))

        # Draw Drone
        # Estimated pose (blue)
        est_px = ox + int(drone_est_state.position[0] * self.cell_px)
        est_py = oy + int((self.rows - drone_est_state.position[1]) * self.cell_px)
        pygame.draw.circle(screen, (30, 110, 245), (est_px, est_py), 10)
        pygame.draw.circle(screen, (255, 255, 255), (est_px, est_py), 10, 2)

        # Heading vector arrow
        est_yaw = drone_est_state.attitude_rpy[2]
        arrow_len = 16
        end_x = est_px + int(arrow_len * math.cos(est_yaw))
        end_y = est_py - int(arrow_len * math.sin(est_yaw))
        pygame.draw.line(screen, (255, 230, 80), (est_px, est_py), (end_x, end_y), 3)

        # Telemetry Panel
        panel_x = self.cols * self.cell_px + 14
        panel_y = self.top_h + 12
        pygame.draw.rect(screen, (26, 30, 38), (self.cols * self.cell_px, self.top_h, self.panel_w, self.height - self.top_h))

        state_colors = {
            MissionState.PREFLIGHT: (200, 200, 200),
            MissionState.TAKEOFF: (240, 200, 70),
            MissionState.EXPLORE: (80, 220, 140),
            MissionState.RETURN_HOME: (245, 140, 50),
            MissionState.LAND: (240, 200, 70),
            MissionState.COMPLETE: (80, 220, 140),
            MissionState.FAILSAFE: (240, 60, 60),
        }

        q_color = (80, 220, 140) if drone_est_state.quality == PoseQuality.EXCELLENT else (
            (240, 200, 70) if drone_est_state.quality == PoseQuality.DEGRADED else (240, 60, 60)
        )

        front_dist = f"{ranges['front'].range_m:.2f}m" if 'front' in ranges else "--"
        left_dist = f"{ranges['left'].range_m:.2f}m" if 'left' in ranges else "--"
        right_dist = f"{ranges['right'].range_m:.2f}m" if 'right' in ranges else "--"
        alt_dist = f"{ranges['down'].range_m:.2f}m" if 'down' in ranges else "--"

        telemetry_lines = [
            ("MISSION STATUS", (210, 215, 225)),
            (mission_state.name, state_colors.get(mission_state, (200, 200, 200))),
            ("", (0, 0, 0)),
            (f"Flight Time: {flight_time_s:.1f}s", (230, 235, 240)),
            (f"Battery: {battery_pct:.1f}%", (100, 220, 140) if battery_pct > 25 else (240, 70, 70)),
            (f"Altitude: {drone_est_state.position[2]:.2f}m (Sonar: {alt_dist})", (230, 235, 240)),
            (f"Est Pos: ({drone_est_state.position[0]:.2f}, {drone_est_state.position[1]:.2f})", (230, 235, 240)),
            (f"Pose Quality: {drone_est_state.quality.name}", q_color),
            (f"Drift Error: {drift_m:.2f}m", (240, 200, 70) if drift_m > 0.5 else (140, 220, 160)),
            (f"Coverage: {coverage_pct:.1f}%", (230, 235, 240)),
            (f"Survivors: {len(confirmed_survivors)}/6", (245, 120, 120)),
            ("", (0, 0, 0)),
            (f"Sonar: F:{front_dist} L:{left_dist} R:{right_dist}", (180, 190, 205)),
            ("", (0, 0, 0)),
            ("CONTROLS:", (210, 215, 225)),
            ("SPACE: Pause  |  R: Restart  |  ESC: Quit", (160, 170, 185)),
        ]

        ty = panel_y
        for text, col in telemetry_lines:
            if text:
                screen.blit(self.small_font.render(text, True, col), (panel_x, ty))
            ty += 18

        # Camera Inset (bottom right) with YOLO annotations
        cam_annotated = camera_bgr.copy()
        for box in detections:
            cv2.rectangle(
                cam_annotated,
                (int(box.x1), int(box.y1)),
                (int(box.x2), int(box.y2)),
                (0, 220, 0),
                2,
            )
            cv2.putText(
                cam_annotated,
                f"person {box.confidence:.0%}",
                (int(box.x1), max(14, int(box.y1) - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 0),
                1,
            )

        cam_rgb = cv2.cvtColor(cam_annotated, cv2.COLOR_BGR2RGB)
        cam_surf = pygame.surfarray.make_surface(np.transpose(cam_rgb, (1, 0, 2)))
        inset_w, inset_h = self.panel_w - 24, 160
        cam_scaled = pygame.transform.scale(cam_surf, (inset_w, inset_h))
        screen.blit(cam_scaled, (self.cols * self.cell_px + 12, self.height - inset_h - 12))
        pygame.draw.rect(screen, (160, 170, 185), (self.cols * self.cell_px + 12, self.height - inset_h - 12, inset_w, inset_h), 1)

        pygame.display.flip()
        if self.clock:
            self.clock.tick(self.fps)

