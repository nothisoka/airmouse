"""3D Pinhole World Renderer for Camera Synthesis and Top-Down UI.

Synthesizes realistic 320x240 camera views from the drone's actual 6-DOF pose.
Renders textured walls, floors, furniture blocks, and 3D human figures into
the camera frame using perspective projection.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import List, Tuple
import cv2
import numpy as np
from airmouse.core.mapping.map_io import cell_to_world


class WorldRenderer:
    def __init__(
        self,
        grid: np.ndarray,
        cell_size_m: float = 1.0,
        img_w: int = 320,
        img_h: int = 240,
        fov_h_deg: float = 70.0,
        camera_pitch_down_deg: float = 12.0,
    ):
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.cell_size = cell_size_m
        self.w = img_w
        self.h = img_h
        self.fov_h = math.radians(fov_h_deg)
        self.fx = (self.w / 2.0) / math.tan(self.fov_h / 2.0)
        self.fy = self.fx
        self.cx = self.w / 2.0
        self.cy = self.h / 2.0
        self.cam_pitch_down = math.radians(camera_pitch_down_deg)
        # Load photorealistic person sprite for camera view survivor rendering
        sprite_path = Path("/home/sriram/airmouse/assets/person_sprite.png")
        if sprite_path.exists():
            self.person_sprite = cv2.imread(str(sprite_path), cv2.IMREAD_UNCHANGED)
        else:
            self.person_sprite = None

        # Precompute static procedural floor texture (wood/concrete/tile grain)
        rng = np.random.RandomState(42)
        tex = np.full((512, 512, 3), (135, 130, 120), dtype=np.uint8)
        noise1 = cv2.resize(rng.randint(0, 45, (64, 64, 3), dtype=np.uint8), (512, 512))
        noise2 = cv2.resize(rng.randint(0, 25, (128, 128, 3), dtype=np.uint8), (512, 512))
        noise3 = rng.randint(0, 15, (512, 512, 3), dtype=np.uint8)
        tex = np.clip(tex.astype(np.int16) + noise1 + noise2 + noise3 - 35, 0, 255).astype(np.uint8)
        # 1m tile seams (128 px per meter)
        tex[::128, :, :] = (65, 60, 55)
        tex[:, ::128, :] = (65, 60, 55)
        self.floor_texture = tex

        # Precompute camera ray geometry for ground rendering (rows 125..235, cols 0..320)
        v_indices, u_indices = np.indices((110, self.w))
        self.ground_v = v_indices + 125
        self.ground_u = u_indices
        self.ground_ky = (self.ground_v - self.cy) / self.fy
        self.ground_kx = (self.ground_u - self.cx) / self.fx

        # Precompute obstacle 3D bounding boxes [xmin, xmax, ymin, ymax, zmin, zmax, color, type]
        self.boxes = []
        for y in range(self.rows):
            for x in range(self.cols):
                val = self.grid[y, x]
                if val != 0:
                    x0 = x * self.cell_size
                    x1 = (x + 1) * self.cell_size
                    # In world coords, row 0 has max Y
                    y0 = (self.rows - y - 1) * self.cell_size
                    y1 = (self.rows - y) * self.cell_size
                    z0 = 0.0
                    if val == 1:  # Wall
                        z1 = 2.4
                        color = (80, 85, 95)
                    else:         # Furniture
                        z1 = 0.8
                        color = (120, 110, 90)
                    self.boxes.append((x0, x1, y0, y1, z0, z1, color, val))

    def render_camera_frame(
        self,
        drone_pos: np.ndarray,
        drone_yaw: float,
        drone_pitch: float,
        drone_roll: float,
        survivors: List[Tuple[float, float]],
    ) -> np.ndarray:
        """Render a synthetic camera frame from continuous drone pose."""
        frame = np.full((self.h, self.w, 3), (210, 205, 195), dtype=np.uint8)  # warm indoor ceiling
        # Draw floor
        cv2.rectangle(frame, (0, int(self.cy)), (self.w, self.h), (145, 140, 130), -1)

        # Draw grid floor tiles for optical flow feature tracking
        tile_size = 0.5
        cos_yaw = math.cos(drone_yaw)
        sin_yaw = math.sin(drone_yaw)

        # Camera rotation matrix: Drone Yaw + Cam Pitch
        # Camera frame: +Z forward, +X right, +Y down
        total_pitch = drone_pitch + self.cam_pitch_down
        cp, sp = math.cos(total_pitch), math.sin(total_pitch)
        cr, sr = math.cos(drone_roll), math.sin(drone_roll)

        # Drone heading unit vector
        fwd_x, fwd_y = cos_yaw, sin_yaw
        right_x, right_y = sin_yaw, -cos_yaw

        # Render continuous textured floor with ground-plane inverse projection
        alt = max(0.15, float(drone_pos[2]))
        denom = self.ground_ky * cp + sp
        valid_ground = denom > 0.05
        x_rot = alt * (cp - self.ground_ky * sp) / np.maximum(0.05, denom)
        z_cam = x_rot * cp + alt * sp
        y_rot = - self.ground_kx * z_cam

        # Rotate by drone yaw to get world coordinates
        dx = x_rot * cos_yaw - y_rot * sin_yaw
        dy = x_rot * sin_yaw + y_rot * cos_yaw
        wx = drone_pos[0] + dx
        wy = drone_pos[1] + dy

        map_x = ((wx * 128.0) % 512).astype(np.float32)
        map_y = ((wy * 128.0) % 512).astype(np.float32)
        frame[125:235, :, :] = cv2.remap(self.floor_texture, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)

        # Collect objects to render, sort back-to-front (Painter's algorithm)
        render_items = []
        for box in self.boxes:
            bx_center = (box[0] + box[1]) / 2.0
            by_center = (box[2] + box[3]) / 2.0
            bz_center = (box[4] + box[5]) / 2.0
            dist_sq = (bx_center - drone_pos[0])**2 + (by_center - drone_pos[1])**2
            if dist_sq < 36.0:  # within 6 metres
                render_items.append(('box', dist_sq, box))

        for idx, (sx, sy) in enumerate(survivors):
            dist_sq = (sx - drone_pos[0])**2 + (sy - drone_pos[1])**2
            if dist_sq < 36.0:
                render_items.append(('survivor', dist_sq, (sx, sy, idx)))

        # Sort descending by distance
        render_items.sort(key=lambda item: item[1], reverse=True)

        for item_type, _, data in render_items:
            if item_type == 'box':
                self._draw_3d_box(frame, data, drone_pos, drone_yaw, total_pitch)
            elif item_type == 'survivor':
                sx, sy, s_idx = data
                self._draw_human_figure(frame, sx, sy, drone_pos, drone_yaw, total_pitch)

        return frame

    def _project_world_to_cam(
        self,
        wx: float,
        wy: float,
        wz: float,
        drone_pos: np.ndarray,
        yaw: float,
        pitch: float,
    ) -> Tuple[int, int] | None:
        """Project world coordinate (wx, wy, wz) into camera pixel (u, v)."""
        # Translation relative to camera
        dx = wx - drone_pos[0]
        dy = wy - drone_pos[1]
        dz = wz - drone_pos[2]

        # Rotate by yaw around world Z
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        x_rot = dx * cos_y - dy * sin_y
        y_rot = dx * sin_y + dy * cos_y

        # Camera frame:
        # Forward axis = body +X tilted down by pitch
        # Right axis = body -Y
        # Down axis = body -Z tilted
        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)

        z_cam = x_rot * cos_p - dz * sin_p
        x_cam = -y_rot
        y_cam = -x_rot * sin_p - dz * cos_p

        if z_cam <= 0.2:  # Behind camera or near clipping plane
            return None

        u = int(self.cx + (self.fx * x_cam) / z_cam)
        v = int(self.cy + (self.fy * y_cam) / z_cam)

        if -150 <= u <= self.w + 150 and -150 <= v <= self.h + 150:
            return (u, v)
        return None

    def _draw_3d_box(self, frame: np.ndarray, box, drone_pos, yaw, pitch):
        x0, x1, y0, y1, z0, z1, color, btype = box
        # 8 box corners
        corners_3d = [
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)
        ]
        pts_2d = [self._project_world_to_cam(c[0], c[1], c[2], drone_pos, yaw, pitch) for c in corners_3d]
        if all(p is None for p in pts_2d):
            return

        # Faces: front, back, left, right, top
        faces = [
            ([0, 1, 5, 4], 0.85),
            ([1, 2, 6, 5], 0.70),
            ([2, 3, 7, 6], 0.85),
            ([3, 0, 4, 7], 0.70),
            ([4, 5, 6, 7], 1.00),
        ]
        for indices, shade in faces:
            poly = [pts_2d[i] for i in indices]
            if any(p is None for p in poly):
                continue
            pts = np.array(poly, dtype=np.int32)
            shaded_color = tuple(int(c * shade) for c in color)
            cv2.fillConvexPoly(frame, pts, shaded_color)
            cv2.polylines(frame, [pts], True, (40, 40, 45), 1)

    def _draw_human_figure(self, frame: np.ndarray, sx: float, sy: float, drone_pos, yaw, pitch):
        """Draw a realistic human figure (survivor) using photorealistic sprite or fallback."""
        # Survivor stands on ground at (sx, sy, 0)
        p_feet = self._project_world_to_cam(sx, sy, 0.0, drone_pos, yaw, pitch)
        p_head = self._project_world_to_cam(sx, sy, 1.7, drone_pos, yaw, pitch)

        if p_feet is None or p_head is None:
            return

        uf, vf = p_feet
        uh, vh = p_head
        height_px = max(16, vf - vh)

        if self.person_sprite is not None and height_px >= 16:
            aspect = self.person_sprite.shape[1] / self.person_sprite.shape[0]
            width_px = max(8, int(height_px * aspect))
            resized = cv2.resize(self.person_sprite, (width_px, height_px), interpolation=cv2.INTER_AREA)

            x1 = uh - width_px // 2
            y1 = vf - height_px
            x2 = x1 + width_px
            y2 = vf

            # Clip to frame dimensions
            src_x1 = max(0, -x1)
            src_y1 = max(0, -y1)
            src_x2 = width_px - max(0, x2 - self.w)
            src_y2 = height_px - max(0, y2 - self.h)

            dst_x1 = max(0, x1)
            dst_y1 = max(0, y1)
            dst_x2 = min(self.w, x2)
            dst_y2 = min(self.h, y2)

            if dst_x2 > dst_x1 and dst_y2 > dst_y1 and src_x2 > src_x1 and src_y2 > src_y1:
                sprite_crop = resized[src_y1:src_y2, src_x1:src_x2]
                alpha = (sprite_crop[:, :, 3] / 255.0)[:, :, np.newaxis]
                frame[dst_y1:dst_y2, dst_x1:dst_x2] = (
                    (1.0 - alpha) * frame[dst_y1:dst_y2, dst_x1:dst_x2] + alpha * sprite_crop[:, :, :3]
                ).astype(np.uint8)
            return

        # Fallback geometric drawing if sprite is unavailable
        p_torso = self._project_world_to_cam(sx, sy, 1.1, drone_pos, yaw, pitch)
        if p_torso is None:
            return
        width_px = max(6, int(height_px * 0.4))
        shirt_color = (40, 60, 210)
        pants_color = (160, 80, 40)
        skin_color = (150, 185, 230)
        cv2.rectangle(frame, (uh - width_px // 2, vh + int(height_px * 0.5)),
                      (uh + width_px // 2, vf), pants_color, -1)
        cv2.rectangle(frame, (uh - width_px // 2, vh + int(height_px * 0.2)),
                      (uh + width_px // 2, vh + int(height_px * 0.55)), shirt_color, -1)
        head_radius = max(3, int(height_px * 0.12))
        cv2.circle(frame, (uh, vh + head_radius), head_radius, skin_color, -1)
